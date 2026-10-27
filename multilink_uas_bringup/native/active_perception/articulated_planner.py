"""Global observed-map view search with coordinated, committed joint paths."""
import argparse,json,sys,time
from collections import deque
from pathlib import Path
import numpy as np
import rospy,yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState,Imu,PointCloud2,Image
from sensor_msgs import point_cloud2
from std_msgs.msg import Float64MultiArray,Bool
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
from joint_geometry import decode,advance
from predictive_bearing_covariance import forecast
from lidar_conditioned_velocity_gain import fused_position_gain
from core import lidar_model,VisualMap,adjoint
from image_geometry_map import ImageGeometryMap
from articulated_search import Roadmap,view_weights,information_gain,interpolate_path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES

class Planner:
    def __init__(self,out,mode):
        self.out=out;self.kin=Kinematics(out/'dragon_slam.urdf');self.mode=mode
        self.names=NAMES;self.gimbal=len(self.kin.names)>6
        if self.gimbal:
            from dragon_gimbal import GimbalKinematics,GIMBAL_NAMES
            self.kin=GimbalKinematics(out/'dragon_slam.urdf');self.names=GIMBAL_NAMES
        self.cfg=yaml.safe_load((out/'planner.yaml').read_text());self.q=None;self.geo=None;self.info=None;self.cloud=None
        self.omega=np.zeros(3);self.enabled=False;self.poses=deque(maxlen=300);self.joint_history=deque(maxlen=2000);self.map=VisualMap(30.,400);self.image_map=VisualMap(30.,3000);self.image=None;self.image_stamp=-1
        cam=yaml.safe_load((out/'cam0.yaml').read_text());K=np.array([[cam['camera_matrix']['data'][0],0,cam['camera_matrix']['data'][2]],[0,cam['camera_matrix']['data'][4],cam['camera_matrix']['data'][5]],[0,0,1]])
        self.image_geometry=ImageGeometryMap(K,180)
        self.map_stamp=-1;self.goal=None;self.path=[];self.goal_index=None;self.last_plan=-1e9;self.last_goal=-1e9
        self.log=(out/'joint_decisions.jsonl').open('w',buffering=1)
        cfg=yaml.safe_load((out/'mimosa.yaml').read_text());p=cfg['lidar']['T_B_S']
        self.velocity_scale=cfg['odometry']['manager']['velocity_covariance_scale']
        self.TBL=np.eye(4);self.TBL[:3,:3]=Rotation.from_quat(p[3:]).as_matrix();self.TBL[:3,3]=p[:3]
        self.pub=rospy.Publisher('/articulated/target',JointState,queue_size=1)
        self.subs=[rospy.Subscriber('/active/image',Image,lambda m:setattr(self,'image',m),queue_size=1),rospy.Subscriber('/dragon/articulated/states',JointState,self.joints,queue_size=10),
                   rospy.Subscriber('/rovio_gimbal/joint_geometry',Float64MultiArray,self.geometry,queue_size=1),
                   rospy.Subscriber('/rovio/odometry',Odometry,self.pose,queue_size=100),
                   rospy.Subscriber('/active/lidar_information',Float64MultiArray,self.lidar,queue_size=1),
                   rospy.Subscriber('/dragon/lidar/points',PointCloud2,self.cloud_callback,queue_size=1),
                   rospy.Subscriber('/slam/input/imu',Imu,self.imu,queue_size=10),
                   rospy.Subscriber('/articulated/enable',Bool,lambda m:setattr(self,'enabled',m.data),queue_size=1)]
        if self.gimbal:
            from dragon_gimbal import GimbalRoadmap
            self.road=GimbalRoadmap(self.kin)
        else:self.road=Roadmap(self.kin)
        (out/'reachability.json').write_text(json.dumps(self.road.audit(),indent=2))
        self.timer=rospy.Timer(rospy.Duration(.2),self.checked_tick)

    def joints(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in self.names+NAMES):
            if self.gimbal:self.kin.body=np.array([d[n] for n in NAMES])
            self.q=(m.header.stamp.to_sec(),np.array([d[n] for n in self.names]));self.joint_history.append(self.q)
    def geometry(self,m):self.geo=decode(m.data)
    def pose(self,m):
        p=m.pose.pose;t=m.header.stamp.to_sec()
        if self.poses and t<=self.poses[-1][0]:return
        T=np.eye(4);T[:3,3]=[p.position.x,p.position.y,p.position.z]
        T[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
        self.poses.append((t,T))
    def pose_at(self,t):
        a=list(self.poses)
        if len(a)<2 or t<a[0][0] or t>a[-1][0]+.5:return None
        ts=np.array([v[0] for v in a]);i=np.searchsorted(ts,t)
        if i==0:return a[0][1].copy()
        if i==len(a):
            dt=t-a[-1][0];T=a[-1][1].copy()
            if self.geo is None:return None
            T[:3,3]+=T[:3,:3]@self.geo[3]*dt
            T[:3,:3]=T[:3,:3]@Rotation.from_rotvec(self.omega*dt).as_matrix()
            return T
        if ts[i]-ts[i-1]>.15:return None
        f=(t-ts[i-1])/(ts[i]-ts[i-1]);T=np.eye(4)
        T[:3,3]=(1-f)*a[i-1][1][:3,3]+f*a[i][1][:3,3]
        T[:3,:3]=Slerp(ts[i-1:i+1],Rotation.from_matrix([a[i-1][1][:3,:3],a[i][1][:3,:3]]))([t]).as_matrix()[0]
        return T
    def imu(self,m):self.omega=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
    def lidar(self,m):
        a=np.asarray(m.data)
        if len(a)==96 and a[0]==1 and np.isfinite(a).all():self.info=(a[1],*lidar_model(a[18:54].reshape(6,6),a[54:90].reshape(6,6),a[90:],self.TBL))
    def cloud_callback(self,m):
        # Planning only, one scan per second, no recording and no truth.
        t=m.header.stamp.to_sec()
        if self.cloud is not None and t-self.cloud[0]<1:return
        p=np.array(list(point_cloud2.read_points(m,field_names=('x','y','z'),skip_nans=True)))[::2]
        if p.ndim==2 and len(p):
            p=p[np.isfinite(p).all(axis=1)];p=p@self.TBL[:3,:3].T+self.TBL[:3,3];p=p[np.linalg.norm(p,axis=1)>.8]
            self.cloud=(t,p)

    def camera_at(self,t):
        history=list(self.joint_history);ts=np.array([a[0] for a in history]);i=np.searchsorted(ts,t)
        if i==0 or i==len(history):return None
        f=(t-ts[i-1])/max(ts[i]-ts[i-1],1e-12)
        return self.kin.camera(history[i-1][1]*(1-f)+history[i][1]*f)

    def update_image_map(self):
        m=self.image;cloud=self.cloud
        if m is None or cloud is None:return
        stamp=m.header.stamp.to_nsec();t=stamp*1e-9
        if stamp<=self.image_stamp:return
        T=self.pose_at(t);Tc=self.pose_at(cloud[0]);camera=self.camera_at(t)
        if T is None or Tc is None or camera is None or not 0<=t-cloud[0]<1.2:return
        gray=np.ndarray((m.height,m.width),np.uint8,buffer=m.data,strides=(m.step,1))
        points=(cloud[1]@Tc[:3,:3].T+Tc[:3,3]-T[:3,3])@T[:3,:3]
        ids,p,C,diagnostic=self.image_geometry.update(gray,points,camera[:3,:3],camera[:3,3],30.,T)
        self.image_map.update(stamp,ids+1000000000,p@T[:3,:3].T+T[:3,3],T[:3,:3]@C@T[:3,:3].T,T[:3,:3]@camera[:3,3]+T[:3,3])
        self.image_stamp=stamp

    def update_map(self,geo,q):
        stamp,ids,p,v,C=geo
        if stamp<=self.map_stamp:return
        T=self.pose_at(stamp*1e-9)
        if T is None:return
        cov=np.array([C[3+3*i:6+3*i,3+3*i:6+3*i] for i in range(len(ids))]).reshape(-1,3,3)
        camera=self.camera_at(stamp*1e-9)
        if camera is None:return
        self.map.update(stamp,ids,p@T[:3,:3].T+T[:3,3],T[:3,:3]@cov@T[:3,:3].T,T[:3,:3]@camera[:3,3]+T[:3,3])
        self.map_stamp=stamp

    def safe_path(self,path,points,views,ages,tree):
        # Check the entire coordinated path, not just its endpoints. Visibility
        # can hand over between observed landmarks instead of retaining the
        # same 90% of tracks forever. Unknown regions receive no invented score.
        counts=[]
        for a,b in zip(path,path[1:]):
            for q in interpolate_path(a,b,np.deg2rad(5.)):
                if not self.kin.valid(q):return False,0.
                T=self.kin.camera(q);w=view_weights(points,T,self.cfg,views,ages)*self.kin.visible(q,points)
                counts.append(float(w.sum()))
                if tree is not None:
                    centers,radii=self.kin.collision_samples(q)
                    if np.any(tree.query(centers)[0]<radii+.10):return False,0.
        minimum=min(counts,default=0.)
        return minimum>=min(4.,counts[0]*.6) if counts else False,minimum

    def checked_tick(self,event):
        try:self.tick(event)
        except Exception as error:
            rospy.logerr('Articulated planner stopped: %s',error)
            (self.out/'planner_failure.json').write_text(json.dumps({'error':repr(error)}))
            rospy.signal_shutdown('View planner failed')
            raise

    def tick(self,event):
        if self.q is None:return
        t,q=self.q;now=rospy.Time.now().to_sec()
        if now-t>.25:return
        geo=self.geo;info=self.info
        if geo is not None:self.update_map(geo,q)
        self.update_image_map()
        if not self.enabled:return
        reason='hold_missing_inputs';scores=[];target=q.copy();minimum=None
        Tnow=self.pose_at(t)
        if self.mode=='fixed':reason='fixed'
        elif geo is not None and Tnow is not None and 0<=t-geo[0]*1e-9<.5:
            parts=[m.body_points(Tnow,True,min_observations=2) for m in (self.map,self.image_map)]
            points,C,views,ages=[np.concatenate(v) for v in zip(*parts)]
            # One representative per spatial cell, prioritizing recent evidence.
            if len(points):
                cells={}
                for i in np.argsort(ages):cells.setdefault(tuple(np.floor(points[i]/.2).astype(int)),int(i))
                buckets={}
                for i in cells.values():
                    d=points[i]/max(np.linalg.norm(points[i]),1e-9)
                    key=(int(np.floor(np.arctan2(d[1],d[0])/np.deg2rad(30))),int(np.floor(np.arcsin(np.clip(d[2],-1,1))/np.deg2rad(30))))
                    buckets.setdefault(key,[]).append(i)
                ii=[]
                while len(ii)<180 and any(buckets.values()):
                    for v in buckets.values():
                        if v and len(ii)<180:ii.append(v.pop(0))
                points,C,views,ages=points[ii],C[ii],views[ii],ages[ii]
            points=points-geo[3]*.8
            if len(points)>=4:
                # Keep current information in the current body frame.
                H=np.diag(1/np.square(self.cfg['prior_std']))
                lidar_current=info is not None and 0<=t-info[0]<.5
                if lidar_current:
                    relative=np.eye(4);relative[:3,:3]=Rotation.from_rotvec(self.omega*(t-info[0])).as_matrix()
                    relative[:3,3]=geo[3]*(t-info[0]);A=adjoint(relative);H+=A.T@info[1]@A
                # Missing scan-to-map matches cannot supply a fresh direction.
                # Use zero LiDAR information for VIEW SELECTION ONLY, retaining
                # the finite prior and measured visual information. Do not
                # modify or manufacture a factor in the fusion estimator.
                def map_score(candidate):
                    T=self.kin.camera(candidate)
                    weights=view_weights(points,T,self.cfg,views,ages)*self.kin.visible(candidate,points)
                    if weights.sum()<4:return 0.
                    return information_gain(H,points,C,T,weights,self.cfg)
                reason='follow_joint_path' if self.path else 'hold_best_view'
                if now-self.last_plan>=1.:
                    self.last_plan=now
                    pp,vv,CC=advance(geo[2],geo[3],geo[4],Rotation.from_rotvec(self.omega*(t-geo[0]*1e-9)).as_matrix(),t-geo[0]*1e-9)
                    T0=self.kin.camera(q);initial=view_weights(pp,T0,self.cfg)
                    def live_score(candidate):
                        T=self.kin.camera(candidate)
                        predicted,rays,valid,_=forecast(pp,vv,CC,T[:3,3],self.omega,.8)
                        weights=np.minimum(initial,view_weights(rays+T[:3,3],T,self.cfg))*valid*self.kin.visible(candidate,rays+T[:3,3])
                        if weights.sum()<4:return 0.
                        scale=np.ones(len(predicted));scale[:3]=np.sqrt(self.velocity_scale)
                        return float(fused_position_gain(H,scale[:,None]*predicted*scale[None,:],weights[None],self.cfg['bearing_sigma'],1.)[0])
                    hold_map=map_score(q);hold_live=live_score(q)
                    def score(candidate):
                        return float(np.log1p(max(0.,live_score(candidate))/max(hold_live,1e-6))+.1*np.log1p(max(0.,map_score(candidate))/max(hold_map,1e-6)))
                    hold=score(q)
                    coarse=sorted([(map_score(c),i) for i,c in enumerate(self.road.q)],reverse=True)[:24]
                    candidate_q={i:self.road.q[i] for _,i in coarse}
                    # Fine coordinated motions join the global roadmap; goals
                    # can combine all three yaw/pitch joints, not one +/-1deg.
                    patterns=list(np.eye(len(q)))
                    for inds in ([] if self.gimbal else [[1,3,5],[0,2,4],[1,3],[3,5],[0,2],[2,4]]):
                        d=np.zeros(6);d[inds]=1.;patterns.append(d)
                    for inds in ([] if self.gimbal else [[0,2],[2,4],[0,4],[1,3],[3,5]]):
                        d=np.zeros(6);d[inds]=[1.,-1.];patterns.append(d)
                    for d in patterns:
                        for sign in (-1,1):
                            c=np.clip(q+sign*np.deg2rad(10.)*d,self.kin.lower,self.kin.upper)
                            if self.kin.valid(c):candidate_q[len(self.road.q)+len(candidate_q)]=c
                    ranked=[]
                    for i,candidate in candidate_q.items():
                        raw=score(candidate);cost=raw-.002*np.max(abs(candidate-q))/self.kin.speed
                        ranked.append((cost,i,raw))
                    tree=None;cloud=self.cloud
                    if cloud is not None and 0<=t-cloud[0]<2:
                        Tc=self.pose_at(cloud[0])
                        if Tc is not None:
                            p=(cloud[1]@Tc[:3,:3].T+Tc[:3,3]-Tnow[:3,3])@Tnow[:3,:3];tree=cKDTree(p)
                    candidates=sorted(ranked,reverse=True)[:12]
                    current_score=score(self.goal) if self.goal is not None else hold
                    # Do not abandon a multi-second transition for tiny changes.
                    threshold=max(hold,current_score if self.path else hold)*1.03
                    if candidates and candidates[0][0]>threshold and (not self.path or now-self.last_goal>=3.):
                        paths=self.road.paths(q)
                        for value,i,raw in candidates:
                            if value<=threshold:continue
                            if i>=len(self.road.q):
                                path=[q,candidate_q[i]]
                                if not self.kin.path_valid(*path):continue
                                duration=float(np.max(abs(candidate_q[i]-q))/self.kin.speed)
                            elif i in paths:duration,path=paths[i]
                            else:continue
                            valid,minimum=self.safe_path(path,points,views,ages,tree)
                            if not valid:continue
                            self.goal=candidate_q[i].copy();self.path=[v.copy() for v in path[1:] if np.max(abs(v-q))>np.deg2rad(1.)]
                            self.goal_index=i;self.last_goal=now;reason='select_global_view';break
                    # Recheck the short next edge against observations/scene.
                    if self.path:
                        valid,minimum=self.safe_path([q,self.path[0]],points,views,ages,tree)
                        if not valid:self.path=[];self.goal=None;reason='stop_path_obstruction_or_lost_support'
                    scores=[dict(index=i,score=v,raw=raw) for v,i,raw in candidates[:3]]
                while self.path and np.max(abs(self.path[0]-q))<np.deg2rad(1.):self.path.pop(0)
                if self.path:target=self.path[0]
        msg=JointState();msg.header.stamp=rospy.Time.now();msg.name=self.names;msg.position=target.tolist();self.pub.publish(msg)
        self.log.write(json.dumps(dict(time=now,measured=q.tolist(),target=target.tolist(),reason=reason,scores=scores,
            goal=None if self.goal is None else self.goal.tolist(),selected=self.goal_index,map_points=len(self.map.points),image_map_points=len(self.image_map.points),
            path_remaining=len(self.path),minimum_visible_weight=minimum,
            geometry_age_s=None if geo is None else t-geo[0]*1e-9, lidar_age_s=None if info is None else t-info[0],pose_available=Tnow is not None,
            lidar_information_state='current' if info is not None and 0<=t-info[0]<.5 else 'unavailable_visual_recovery'))+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',default='active',choices=['active','fixed']);a=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('articulated_view_planner');node=Planner(a.output,a.mode)
    try:rospy.spin()
    finally:
        node.map.save(a.output/'visual_map.npz');node.image_map.save(a.output/'image_visual_map.npz')
