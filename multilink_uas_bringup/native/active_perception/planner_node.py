#!/usr/bin/env python3
"""ROS1 planner: observed ROVIO landmarks + read-only DRPM telemetry -> gimbal."""
import argparse
import json
from pathlib import Path
import threading
import time
from collections import deque
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2,Image,Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,Float64MultiArray,String
from scipy.spatial.transform import Rotation
import yaml
from core import Fisher,VisualMap,Planner,lidar_model,adjoint

class Active:
    def __init__(self,args):
        self.args=args;self.cfg=yaml.safe_load(args.config.read_text());self.cfg['mode']=args.mode
        if self.cfg.get('fixation_live_only',False) and not all(self.cfg.get(k,False) for k in ('fixation','direct_track_input','fixation_bearing_motion','image_geometry_map')):
            raise ValueError('Live fixation requires direct bearings and gyro alignment')
        self.fixed_angles=None if args.fixed_angles_deg is None else np.deg2rad(args.fixed_angles_deg)
        if self.fixed_angles is not None:
            if args.mode!='fixed' or not np.isfinite(self.fixed_angles).all():raise ValueError('Fixed angles require fixed mode and finite values')
            if np.any(self.fixed_angles<np.array(self.cfg['limits_min'])-1e-12) or np.any(self.fixed_angles>np.array(self.cfg['limits_max'])+1e-12):raise ValueError('Fixed angles exceed gimbal limits')
        self.map=VisualMap(self.cfg['map_max_age'],self.cfg['map_max_points'])
        self.planner=Planner(Fisher(args.library),self.cfg)
        if self.cfg.get('fixation',False):
            if self.cfg.get('fixation_retinal_candidates',False):
                if not all(self.cfg.get(k,False) for k in ('fixation_predictive_geometry','fixation_lidar_conditioned_velocity','fixation_correlated_tracking','fixation_joint_velocity','match_actuator_reachability')):
                    raise ValueError('Retinal candidates require predictive geometry and actual actuator limits')
                from retinal_tracking_view import RetinalTrackingView as Fixation
            elif self.cfg.get('fixation_predictive_geometry',False):
                if not all(self.cfg.get(k,False) for k in ('fixation_lidar_conditioned_velocity','fixation_correlated_tracking','fixation_joint_velocity')):
                    raise ValueError('Geometry forecast requires calibrated correlated tracking')
                from predictive_tracking_view import PredictiveTrackingView as Fixation
            elif self.cfg.get('fixation_lidar_conditioned_velocity',False):
                if not self.cfg.get('fixation_correlated_tracking',False):raise ValueError('LiDAR conditioned tracking requires correlated tracking')
                from lidar_conditioned_tracking_view import LidarConditionedTrackingView as Fixation
            elif self.cfg.get('fixation_correlated_tracking',False):
                if not self.cfg.get('fixation_joint_velocity',False):raise ValueError('Correlated tracking requires joint ROVIO covariance')
                from correlated_tracking_view import CorrelatedTrackingView as Fixation
            elif self.cfg.get('fixation_joint_velocity',False):
                from joint_velocity_view import JointVelocityView as Fixation
            elif self.cfg.get('fixation_stable_view',False):
                from stable_view import StableView as Fixation
            elif self.cfg.get('fixation_joint_information',False):
                from joint_information_view import JointInformationView as Fixation
            elif self.cfg.get('fixation_spatial_parallax',False):
                from spatial_parallax_fixation import SpatialParallaxFixation as Fixation
            elif self.cfg.get('fixation_parallax_fallback',False):
                from parallax_fixation import ParallaxFixation as Fixation
            else:
                from fixation import Fixation
            self.fixation=Fixation(self.planner.fisher,self.cfg)
            self.track_history={};self.fixation_record=None
            if self.cfg.get('fixation_bearing_motion',False):
                from bearing_motion import BearingMotion
                self.bearing_motion=BearingMotion(self.cfg.get('motion_average_s',.3))
        self.lock=threading.RLock();self.odoms={};self.clouds={};self.last_odom=None
        self.current_tracks=None;self.tracking_lock=threading.Lock();self.tracking_input=None
        self.joint_inputs={};self.geometry_inputs={}
        self.geometry=None;self.scene_health={}
        if self.cfg.get('image_geometry_map',False):
            from image_geometry_map import ImageGeometryMap
            from view_motion import RotationHistory
            calibration=yaml.safe_load((args.output/'cam0.yaml').read_text())
            self.K=np.array(calibration['camera_matrix']['data']).reshape(3,3)
            self.geometry=ImageGeometryMap(self.K,self.cfg.get('image_map_features',180))
            self.motion=RotationHistory();self.encoder_history=deque(maxlen=1000);self.encoder_lock=threading.Lock()
            self.images={};self.extrinsics={};self.scans={}
            self.geometry_pending=None;self.geometry_coalesced=0;self.geometry_failure=None
            self.geometry_event=threading.Event();self.geometry_stop=threading.Event();self.geometry_thread=None
        self.lidar=None;self.encoder=None;self.scan=None;self.last_plan=None
        self.health={'info_received':0,'info_accepted':0,'stale':0,'planned':0}
        cam=json.loads((args.output/'camera_manifest.json').read_text())['cameras'][0]
        T=np.array(cam['T_FC_camera']);self.home=T[:3,:3];self.tbc=T[:3,3]
        fusion=yaml.safe_load((args.output/'mimosa.yaml').read_text())
        p=fusion['lidar']['T_B_S'];self.TBL=np.eye(4);self.TBL[:3,:3]=Rotation.from_quat(p[3:]).as_matrix();self.TBL[:3,3]=p[:3]
        args.output.mkdir(exist_ok=True,parents=True)
        self.log=(args.output/'decisions.jsonl').open('w',buffering=1)
        self.pub=[rospy.Publisher('/active/gimbal/'+a,Float64,queue_size=1) for a in ('yaw','pitch')]
        self.status=rospy.Publisher('/active/status',String,queue_size=1)
        self.sub=[rospy.Subscriber('/slam/odometry' if self.geometry else '/rovio/odometry',Odometry,self.odom,queue_size=30),
            rospy.Subscriber('/rovio/pcl',PointCloud2,self.features,
                queue_size=1 if self.cfg.get('fixation_live_only',False) else 30,
                tcp_nodelay=self.cfg.get('fixation_live_only',False)),
            rospy.Subscriber('/active/lidar_information',Float64MultiArray,self.info,queue_size=5),
            rospy.Subscriber('/active/encoder',Float64MultiArray,self.enc,queue_size=5),
            rospy.Subscriber(args.lidar_topic,PointCloud2,self.obstacles,queue_size=1,buff_size=4000000)]
        if self.geometry:
            if not self.cfg.get('fixation_live_only',False):
                self.sub.extend([rospy.Subscriber('/active/image',Image,self.image,queue_size=10,buff_size=2000000),
                    rospy.Subscriber('/active/extrinsic',PoseStamped,self.extrinsic,queue_size=30)])
            self.sub.append(rospy.Subscriber('/slam/input/imu',Imu,self.imu,queue_size=2000,tcp_nodelay=True))
        if self.cfg.get('fixation_joint_velocity',False):
            if not self.cfg.get('fixation_live_only',False):raise ValueError('Joint velocity scoring requires live tracked features')
            self.sub.append(rospy.Subscriber('/rovio_gimbal/joint_information',Float64MultiArray,self.joint_information,queue_size=5,tcp_nodelay=True))
        if self.cfg.get('fixation_predictive_geometry',False):
            self.sub.append(rospy.Subscriber('/rovio_gimbal/joint_geometry',Float64MultiArray,self.joint_geometry,queue_size=5,tcp_nodelay=True))
        self.timer=None;self.planning_stop=threading.Event();self.planning_thread=None
        if self.cfg.get('pace_planner',False):
            # /clock can be paused specifically to await this planner. Retry
            # pending work on wall time, while limiting successes by sim time.
            self.planning_thread=threading.Thread(target=self.planning_worker,name='view_planning',daemon=True)
        else:self.timer=rospy.Timer(rospy.Duration(1/self.cfg['planning_rate']),self.tick)
        self.health_timer=rospy.Timer(rospy.Duration(1),self.report_health)
        if self.geometry and self.cfg.get('geometry_async',False) and not self.cfg.get('fixation_live_only',False):
            self.geometry_thread=threading.Thread(target=self.geometry_worker,name='planning_image_map',daemon=True);self.geometry_thread.start()
        rospy.on_shutdown(self.finish)
        if self.planning_thread is not None:self.planning_thread.start()

    def finish(self):
        if self.timer is not None:self.timer.shutdown()
        self.health_timer.shutdown();self.planning_stop.set()
        if self.planning_thread is not None and threading.current_thread()!=self.planning_thread:self.planning_thread.join(timeout=5.)
        if self.geometry and self.geometry_thread is not None:
            self.geometry_stop.set();self.geometry_event.set();self.geometry_thread.join(timeout=5.)
        with self.lock:self.map.save(self.args.output/'visual_map.npz');self.log.close()

    def planning_worker(self):
        try:
            while not self.planning_stop.wait(.005) and not rospy.is_shutdown():
                now=rospy.Time.now().to_sec()
                if self.last_plan is None or now-self.last_plan>=1/self.cfg['planning_rate']-1e-9:
                    self.tick(None)
        except Exception as exc:
            rospy.logerr('View planning failed: %s',exc)
            rospy.signal_shutdown('View planning worker failed')

    def enc(self,m):
        if self.geometry:
            # 1000 Hz input must not wait for image geometry or view search.
            if len(m.data)==3 and np.isfinite(m.data).all():
                with self.encoder_lock:
                    self.encoder=(m.data[0],np.array(m.data[1:]));self.encoder_history.append(self.encoder)
            return
        with self.lock:
            if len(m.data)==3 and np.isfinite(m.data).all():
                self.encoder=(m.data[0],np.array(m.data[1:]))

    def key(self,m):
        ns=m.header.stamp.to_nsec()
        # Association only: account for <=2 ns ROS floating-point conversion.
        return int(round(ns/100))*100 if self.geometry else ns

    def image(self,m):
        with self.lock:self.images[self.key(m)]=m;self.join()

    def extrinsic(self,m):
        with self.lock:self.extrinsics[self.key(m)]=m;self.join()

    def imu(self,m):
        # RotationHistory owns a short lock solely for copying/appending samples.
        w=m.angular_velocity;self.motion.append(m.header.stamp.to_sec(),[w.x,w.y,w.z])

    def info(self,m):
        with self.lock:
            self.health['info_received']+=1
            if len(m.data)!=96 or m.data[0]!=1 or not np.isfinite(m.data).all():return
            a=np.array(m.data);H=a[18:54].reshape(6,6);U=a[54:90].reshape(6,6);p=a[90:]
            if np.linalg.eigvalsh((H+H.T)/2).min()<-1e-5:return
            if not np.allclose(U.T@U,np.eye(6),atol=1e-6) or np.any((p<0)|(p>1)):return
            self.lidar=(a[1],*lidar_model(H,U,p,self.TBL))
            self.health['info_accepted']+=1

    def report_health(self,event):
        with self.lock:
            now=rospy.Time.now().to_sec()
            report=dict(self.health,sim_time=now,lidar_age=now-self.lidar[0] if self.lidar else None,
                odom_age=now-self.last_odom.header.stamp.to_sec() if self.last_odom else None,
                encoder_age=now-self.encoder[0] if self.encoder else None)
            if self.geometry:report.update(geometry_coalesced=self.geometry_coalesced,geometry_failure=self.geometry_failure)
            tmp=self.args.output/'planner_health.tmp';tmp.write_text(json.dumps(report));tmp.replace(self.args.output/'planner_health.json')

    def odom(self,m):
        with self.lock:
            self.last_odom=m
            if not self.cfg.get('fixation_live_only',False):
                self.odoms[self.key(m)]=m;self.join()

    def features(self,m):
        if self.cfg.get('direct_track_input',False):
            stamp=m.header.stamp.to_sec()
            if self.cfg.get('fixation',False):
                rows=[r for r in pc2.read_points(m,field_names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22'],skip_nans=True) if r[1]==4]
                ids=np.array([r[0] for r in rows],dtype=np.int64)
                points=np.array([r[2:5] for r in rows]).reshape(-1,3)
                covs=np.array([[[r[5],r[6],r[7]],[r[6],r[8],r[9]],[r[7],r[9],r[10]]] for r in rows]).reshape(-1,3,3)
                with self.tracking_lock:
                    for ident in ids:
                        born,last=self.track_history.get(int(ident),(stamp,stamp))
                        self.track_history[int(ident)]=(stamp if stamp-last>.25 else born,stamp)
                    self.track_history={i:v for i,v in self.track_history.items() if stamp-v[1]<=.25}
                    ages=np.array([stamp-self.track_history[int(i)][0] for i in ids])
                    metadata=(ids,covs,ages)
                    if self.cfg.get('fixation_bearing_motion',False):
                        rates=self.bearing_motion.update(stamp,ids,points-self.tbc,self.motion.rotation)
                        metadata=(*metadata,rates)
                    self.tracking_input=(stamp,points,metadata)
            else:
                rows=list(pc2.read_points(m,field_names=['status','x','y','z'],skip_nans=True))
                points=np.array([[x,y,z] for status,x,y,z in rows if status==4]).reshape(-1,3)
                rays=points-self.tbc;norm=np.linalg.norm(rays,axis=1);good=norm>.1
                # Visibility rays only, not metric landmarks or estimator factors.
                points=self.tbc+rays[good]/norm[good,None]*self.cfg.get('track_visibility_range',8.)
                with self.tracking_lock:self.tracking_input=(stamp,points)
        # The fixation controller only consumes current ROVIO bearings. Do not
        # make the next feature callback wait for an unused image-map join.
        if self.cfg.get('fixation_live_only',False):return
        with self.lock:self.clouds[self.key(m)]=m;self.join()

    def joint_information(self,m):
        from joint_velocity_view import decode
        stamp,ids,points,covariance=decode(m.data)
        with self.tracking_lock:
            self.joint_inputs[stamp]=(ids,points,covariance)
            while len(self.joint_inputs)>8:self.joint_inputs.pop(min(self.joint_inputs))

    def joint_geometry(self,m):
        from joint_geometry import decode
        stamp,ids,points,velocity,covariance=decode(m.data)
        with self.tracking_lock:
            self.geometry_inputs[stamp]=(ids,points,velocity,covariance)
            while len(self.geometry_inputs)>8:self.geometry_inputs.pop(min(self.geometry_inputs))

    def direct_tracks(self,plan_time,old_to_now):
        with self.tracking_lock:
            record=self.tracking_input
            joint=(None if record is None else self.joint_inputs.get(int(round(record[0]*1e9)))) if self.cfg.get('fixation_joint_velocity',False) else None
            geometry=(None if record is None else self.geometry_inputs.get(int(round(record[0]*1e9)))) if self.cfg.get('fixation_predictive_geometry',False) else None
        if record is None:return None
        if self.cfg.get('fixation_joint_velocity',False) and joint is None:return None
        if self.cfg.get('fixation_predictive_geometry',False) and geometry is None:return None
        stamp,points,*metadata=record;age=plan_time-stamp
        if age<-2e-9 or age>self.cfg['max_sensor_age']:return None
        relative=old_to_now(min(stamp,plan_time))
        if relative is None:return None
        aligned=(points-relative[:3,3])@relative[:3,:3]
        if metadata:
            ids,covs,ages,*extra=metadata[0];R=relative[:3,:3]
            if self.cfg.get('fixation_joint_velocity',False):
                from joint_velocity_view import subset
                joint_ids,joint_points,joint_covariance=joint
                if not np.array_equal(ids,joint_ids) or not np.allclose(points,joint_points,atol=2e-5,rtol=2e-6):
                    raise ValueError('Joint covariance and tracked point frame/identity mismatch')
                self.fixation.set_joint_information(ids,subset(joint_ids,joint_covariance,ids,R))
            if self.cfg.get('fixation_predictive_geometry',False):
                from joint_geometry import advance
                geometry_ids,geometry_points,geometry_velocity,geometry_covariance=geometry
                if not np.array_equal(ids,geometry_ids) or not np.allclose(points,geometry_points,atol=2e-5,rtol=2e-6):
                    raise ValueError('Geometry covariance and tracked point frame/identity mismatch')
                predicted_points,predicted_velocity,predicted_covariance=advance(
                    geometry_points,geometry_velocity,geometry_covariance,R,max(age,0.))
                self.fixation.set_joint_geometry(ids,predicted_points,predicted_velocity,predicted_covariance)
            if extra:
                from bearing_motion import advance_bearings,unit_rows
                ray=points-self.tbc;distance=np.linalg.norm(ray,axis=1)
                predicted,rates=advance_bearings(unit_rows(ray),extra[0],R,age)
                aligned=self.tbc+distance[:,None]*predicted
                self.fixation_record=(ids,aligned,R.T@covs@R,ages+age,rates)
            else:
                self.fixation_record=(ids,aligned,R.T@covs@R,ages+age)
        return aligned,dict(stamp=stamp,age_s=age,count=len(points),source='latest_rovio_bearings')

    def join(self):
        if getattr(self,'geometry',None):return self.geometry_join()
        for stamp in sorted(self.odoms.keys()&self.clouds.keys()):
            od=self.odoms.pop(stamp);msg=self.clouds.pop(stamp)
            if stamp<=self.map.last_stamp:continue
            p=od.pose.pose.position;q=od.pose.pose.orientation
            R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();t=np.array([p.x,p.y,p.z])
            # ROS1 read_points returns fields in their binary offset order,
            # irrespective of the order supplied in field_names.
            rows=list(pc2.read_points(msg,field_names=['id','status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22'],skip_nans=True))
            ids=[];points=[];covs=[]
            for ident,status,x,y,z,xx,xy,xz,yy,yz,zz in rows:
                if self.cfg.get('tracking_aware',False) and status!=4:continue  # ROVIO TRACKED
                ids.append(ident);points.append(R@np.array([x,y,z])+t)
                C=np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]);covs.append(R@C@R.T)
            self.map.update(stamp,ids,points,covs,R@self.tbc+t)
            # Successful image tracks can have uncertain depth. Keep them for
            # visibility/retention without claiming they are mature landmarks.
            self.current_tracks=(stamp,np.asarray(points).reshape(-1,3))
        for cache in (self.odoms,self.clouds):
            while len(cache)>40:cache.pop(min(cache))

    def geometry_join(self):
        caches=(self.odoms,self.clouds,self.images,self.extrinsics,self.scans)
        common=set(caches[0]).intersection(*(set(c) for c in caches[1:]))
        for stamp in sorted(common):
            od,msg,im,extr,scan=[c.pop(stamp) for c in caches]
            if stamp<=self.map.last_stamp:continue
            frame=(stamp,od,msg,im,extr,scan)
            if self.cfg.get('geometry_async',False):
                if self.geometry_pending is not None:self.geometry_coalesced+=1
                self.geometry_pending=frame;self.geometry_event.set()
            else:self.accept_geometry(self.process_geometry(frame))
        for cache in caches:
            while len(cache)>60:cache.pop(min(cache))

    def geometry_worker(self):
        try:
            while not self.geometry_stop.is_set():
                self.geometry_event.wait(.1)
                with self.lock:
                    frame=self.geometry_pending;self.geometry_pending=None;self.geometry_event.clear()
                if frame is None:continue
                result=self.process_geometry(frame)
                with self.lock:self.accept_geometry(result)
        except Exception as exc:
            with self.lock:self.geometry_failure=str(exc)
            rospy.logerr('Planning geometry failed: %s',exc)

    def process_geometry(self,frame):
        from image_geometry_map import depth_at_pixels
        start=time.monotonic();stamp,od,msg,im,extr,scan=frame
        if max(x.header.stamp.to_nsec() for x in (od,msg,im,extr))-min(x.header.stamp.to_nsec() for x in (od,msg,im,extr))>2:
            raise ValueError('Image/geometry acquisition stamps do not match')
        if im.encoding!='mono8':raise ValueError('Planning map expects unchanged processed mono image')
        gray=np.ndarray((im.height,im.width),np.uint8,buffer=im.data,strides=(im.step,1))
        p=od.pose.pose.position;q=od.pose.pose.orientation
        R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();t=np.array([p.x,p.y,p.z])
        q=extr.pose.orientation;p=extr.pose.position
        Rbc=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();tbc=np.array([p.x,p.y,p.z])
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
        ids,points,C,health=self.geometry.update(gray,scan,Rbc,tbc,self.cfg['range_max'],T)
        world=points@R.T+t;cov=R@C@R.T
        # Preserve ROVIO's actual tracked rays, independently of the larger
        # planning map. Depth here affects view prediction only.
        rows=list(pc2.read_points(msg,field_names=['status','x','y','z'],skip_nans=True))
        tracked=np.array([[x,y,z] for status,x,y,z in rows if status==4]).reshape(-1,3)
        pc=(tracked-tbc)@Rbc;good=pc[:,2]>.1;pc=pc[good]
        pixels=pc[:,:2]/pc[:,2,None]*[self.K[0,0],self.K[1,1]]+self.K[:2,2]
        track_points,unused=depth_at_pixels(pixels,scan,Rbc,tbc,self.K,self.cfg['range_max'])
        valid=np.isfinite(track_points).all(axis=1)
        # Unmatched bearings remain image tracks, not metric map landmarks.
        track_points[~valid]=(pc[~valid]/np.linalg.norm(pc[~valid],axis=1)[:,None]*8.)@Rbc.T+tbc
        report=dict(health,stamp_ns=stamp,rovio_tracks=len(tracked),matched_rovio_depth=int(valid.sum()),compute_ms=1000*(time.monotonic()-start))
        return stamp,ids,world,cov,R@tbc+t,track_points@R.T+t,report

    def accept_geometry(self,result):
        stamp,ids,world,cov,camera,tracks,report=result
        self.map.update(stamp,ids,world,cov,camera)
        self.current_tracks=(stamp,tracks);self.scene_health=report

    def obstacles(self,m):
        # Same points and order, without constructing 24,000 Python tuples.
        fields={f.name:f for f in m.fields};endian='>' if m.is_bigendian else '<'
        if all(fields[k].datatype==7 and fields[k].count==1 for k in ('x','y','z')):
            dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':m.point_step})
            raw=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
            pts=np.column_stack([raw[k].reshape(-1) for k in ('x','y','z')]).astype(float)
        else:
            pts=np.asarray(list(pc2.read_points(m,field_names=['x','y','z'],skip_nans=True)),dtype=float).reshape(-1,3)
        pts=pts[np.isfinite(pts).all(axis=1)&(np.linalg.norm(pts,axis=1)>.3)]
        with self.lock:
            self.scan=(m.header.stamp.to_sec(),pts@self.TBL[:3,:3].T+self.TBL[:3,3])
            if self.geometry and not self.cfg.get('fixation_live_only',False):
                self.scans[self.key(m)]=self.scan[1];self.join()

    def tick(self,event):
        if getattr(self,'geometry_failure',None):
            rospy.signal_shutdown('Planning geometry worker failed');return
        with self.lock:
            now=rospy.Time.now().to_sec()
            if self.fixed_angles is not None:
                # Set the physical view before VIO initialization. This command
                # must not wait for odometry, landmarks or LiDAR diagnostics.
                for pub,a in zip(self.pub,self.fixed_angles):pub.publish(Float64(data=float(a)))
                self.log.write(json.dumps(dict(stamp_ns=rospy.Time.now().to_nsec(),reason='fixed_angle',target=self.fixed_angles.tolist(),encoder=None if self.encoder is None else self.encoder[1].tolist(),mode='fixed'))+'\n')
                self.last_plan=now
                return
            if not self.last_odom or self.encoder is None or self.lidar is None:return
            stamps=[self.last_odom.header.stamp.to_sec(),self.encoder[0],self.lidar[0]]
            if min(now-t for t in stamps)<-.01 or max(now-t for t in stamps)>self.cfg['max_sensor_age']:
                self.health['stale']+=1
                self.status.publish(String(data='hold: stale or inconsistent timestamps'));return
            if self.last_plan is not None and now<=self.last_plan:return
            dt=min(1/self.cfg['planning_rate'] if self.last_plan is None else now-self.last_plan,self.cfg['max_dt'])
            od=self.last_odom;p=od.pose.pose.position;q=od.pose.pose.orientation
            T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[p.x,p.y,p.z]
            _,H,U,weak=self.lidar;cur=self.encoder[1]
            v=od.twist.twist.linear;velocity=np.array([v.x,v.y,v.z])
            w=od.twist.twist.angular;angular_velocity=np.array([w.x,w.y,w.z])
            scan=self.scan[1] if self.scan is not None and abs(now-self.scan[0])<.15 else None
            motion_report={}
            if self.geometry:
                latest=self.motion.latest()
                if latest is None:return
                with self.encoder_lock:enc=next((e for e in reversed(self.encoder_history) if e[0]<=latest),None)
                if enc is None or enc[0]<max(stamps[0],stamps[2]):return
                plan_time,cur=enc
                D=self.motion.rotation(stamps[0],plan_time)
                angular_velocity=self.motion.average_rate(plan_time,self.cfg.get('motion_average_s',.3))
                if D is None or angular_velocity is None:return
                # MIMOSA publishes its linear velocity in the world frame.
                world_velocity=velocity.copy();T[:3,3]+=world_velocity*(plan_time-stamps[0]);T[:3,:3]=T[:3,:3]@D
                velocity=T[:3,:3].T@world_velocity
                def old_to_now(stamp):
                    d=self.motion.rotation(stamp,plan_time)
                    if d is None:return None
                    tr=np.eye(4);tr[:3,:3]=d;tr[:3,3]=d@velocity*(plan_time-stamp);return tr
                relative=old_to_now(stamps[2])
                if relative is None:return
                A=adjoint(relative);H=A.T@H@A;U=A.T@U
                scan=None
                if self.scan is not None and 0<=plan_time-self.scan[0]<.5:
                    relative=old_to_now(self.scan[0])
                    if relative is not None:scan=(self.scan[1]-relative[:3,3])@relative[:3,:3]
                motion_report=dict(planning_time=plan_time,pose_age_s=plan_time-stamps[0],scene=self.scene_health)
            points,covs,views,ages=self.map.body_points(T,metadata=True,min_observations=self.cfg.get('min_observations',3))
            tracking_points=None
            if self.cfg.get('separate_image_tracks',False):
                tracking_points=np.empty((0,3))
                if self.cfg.get('direct_track_input',False):
                    tracks=self.direct_tracks(plan_time,old_to_now)
                    if tracks is None:
                        self.health['tracking_input_wait']=self.health.get('tracking_input_wait',0)+1
                        return
                    tracking_points,motion_report['tracks']=tracks
                elif self.current_tracks is not None:
                    stamp,world=self.current_tracks
                    if -2e-9<=(od.header.stamp.to_nsec()-stamp)*1e-9<=self.cfg['current_track_age']:
                        tracking_points=(world-T[:3,3])@T[:3,:3]
            start=time.monotonic()
            waiting=False
            if (self.args.output/'route').exists():
                try:waiting=json.loads((self.args.output/'route_status.json').read_text())['stage'] not in ('tracking','finish_hold','landing','complete')
                except (FileNotFoundError,json.JSONDecodeError):waiting=True
            if self.args.mode=='fixed' or waiting:target=np.zeros(2);report=dict(reason='fixed_baseline')
            else:
                try:
                    if self.cfg.get('fixation',False):
                        target,report=self.fixation.choose(self.fixation_record,H,U,weak,self.home,self.tbc,cur,velocity,dt,angular_velocity,plan_time)
                    else:
                        target,report=self.planner.choose(points,covs,views,H,U,weak,self.home,self.tbc,cur,velocity,dt,scan,ages,angular_velocity,tracking_points)
                except (ValueError,np.linalg.LinAlgError) as exc:
                    self.status.publish(String(data='hold: invalid information matrix: '+str(exc)))
                    rospy.logerr_throttle(5,'View evaluation failed: %s',exc)
                    return
            report.update(stamp_ns=int(rospy.Time.now().to_nsec()),input_stamps=stamps,encoder=cur.tolist(),
                          target=target.tolist(),map_points=len(points),image_tracks=None if tracking_points is None else len(tracking_points),body_velocity=velocity.tolist(),body_angular_velocity=angular_velocity.tolist(),lidar_weakness=weak.tolist(),compute_ms=1000*(time.monotonic()-start),mode=self.args.mode)
            if motion_report:report['motion_alignment']=motion_report
            for pub,a in zip(self.pub,target):pub.publish(Float64(data=float(a)))
            if not self.cfg.get('log_candidates',False):report.pop('candidates',None)
            self.log.write(json.dumps(report,allow_nan=False)+'\n')
            self.status.publish(String(data=json.dumps({k:report[k] for k in ('stamp_ns','reason','map_points','target','compute_ms')})))
            self.last_plan=now;self.health['planned']+=1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('config.yaml'))
    p.add_argument('--lidar-topic',default='/dragon/lidar/points')
    p.add_argument('--library',type=Path,required=True);p.add_argument('--mode',choices=['active','fixed'],default='active')
    p.add_argument('--fixed-angles-deg',nargs=2,type=float,metavar=('YAW','PITCH'))
    a=p.parse_args(rospy.myargv()[1:]);rospy.init_node('active_view_planner');node=Active(a);rospy.spin()
