#!/usr/bin/env python3
"""Read-only depth consistency where current camera tracks overlap the LiDAR.

Exact acquisition times are joined. Unsupported rays stay absent. Only aggregate
numbers are stored; no images, point coordinates, or cloud streams are written.
"""
import argparse,csv,json,os,time,threading
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from compare_fusion_sidecar import owner_environment
from image_geometry_map import depth_at_pixels


def main():
    p=argparse.ArgumentParser();p.add_argument('--flight',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,required=True)
    p.add_argument('--max-wall-s',type=float,default=2800.);a=p.parse_args();start=time.monotonic()
    env=owner_environment(a.port)
    if not env:raise RuntimeError('No live owner')
    os.environ.update(env)
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    cfg=yaml.safe_load((a.flight/'mimosa.yaml').read_text());extr=np.asarray(cfg['lidar']['T_B_S']);RBL=Rotation.from_quat(extr[3:]).as_matrix()
    K=np.array(yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3,3)
    lock=threading.RLock();clouds={};tracks={'active':{},'fixed':{}};poses={'active':{},'fixed':{}}
    completed={'active':set(),'fixed':set()};last_cloud=-np.inf;counts={'active':0,'fixed':0};errors=[]
    def key(m):return int(round(m.header.stamp.to_nsec()/100))*100
    with a.output.open('x') as file:
        writer=csv.writer(file);writer.writerow(['stamp_ns','side','tracks','lidar_supported','depth_ratio_median','absolute_log_depth_ratio_median','relative_depth_error_median','reported_depth_sigma_median','absolute_depth_error_median_m']);file.flush()
        def join(side):
            common=set(clouds)&set(tracks[side])&set(poses[side])-completed[side]
            for t in sorted(common):
                try:
                    pts,cov=tracks[side][t];Rbc,tbc=poses[side][t];rays=pts-tbc;pc=rays@Rbc
                    good=(pc[:,2]>.1)&(np.linalg.norm(rays,axis=1)>.3)
                    points=pts[good];rays=rays[good];cov=cov[good];pc=pc[good]
                    pixels=pc[:,:2]/pc[:,2,None]*np.array([K[0,0],K[1,1]])+K[:2,2]
                    lidar,unused_cov=depth_at_pixels(pixels,clouds[t],Rbc,tbc,K,30.)
                    supported=np.isfinite(lidar).all(axis=1);values=[np.nan]*5
                    if supported.any():
                        pred=np.linalg.norm(rays[supported],axis=1);measured=np.linalg.norm(lidar[supported]-tbc,axis=1);ratio=pred/measured
                        unit=rays[supported]/pred[:,None];sigma=np.sqrt(np.maximum(0.,np.einsum('ni,nij,nj->n',unit,cov[supported],unit)))
                        values=[np.median(ratio),np.median(abs(np.log(ratio))),np.median(abs(ratio-1.)),np.median(sigma),np.median(abs(pred-measured))]
                    writer.writerow([t,side,len(pts),int(supported.sum()),*values]);file.flush();counts[side]+=1
                except Exception as exc:errors.append(str(exc))
                completed[side].add(t)
            cutoff=max(clouds,default=0)-4000000000
            for cache in (tracks[side],poses[side]):
                for t in list(cache):
                    if t<cutoff:del cache[t]
            completed[side]={t for t in completed[side] if t>=cutoff}
        def cloud(m):
            nonlocal last_cloud
            t=key(m)
            with lock:
                if t-last_cloud<1000000000:return
                fields={f.name:f for f in m.fields};endian='>' if m.is_bigendian else '<'
                if not all(fields[k].datatype==7 and fields[k].count==1 for k in 'xyz'):raise ValueError('Unexpected cloud format')
                dtype=np.dtype({'names':list('xyz'),'formats':[endian+'f4']*3,'offsets':[fields[k].offset for k in 'xyz'],'itemsize':m.point_step})
                raw=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
                pts=np.column_stack([raw[k].reshape(-1) for k in 'xyz']).astype(float);pts=pts[np.isfinite(pts).all(axis=1)&(np.linalg.norm(pts,axis=1)>.3)]
                clouds[t]=pts@RBL.T+extr[:3];last_cloud=t
                for old in list(clouds):
                    if old<t-4000000000:del clouds[old]
                for side in tracks:join(side)
        def feature(m,side):
            rows=list(pc2.read_points(m,field_names=['status','x','y','z','c_00','c_01','c_02','c_11','c_12','c_22'],skip_nans=True));rows=[r for r in rows if r[0]==4]
            points=np.array([r[1:4] for r in rows]).reshape(-1,3);cov=np.array([[[r[4],r[5],r[6]],[r[5],r[7],r[8]],[r[6],r[8],r[9]]] for r in rows]).reshape(-1,3,3)
            with lock:tracks[side][key(m)]=(points,cov);join(side)
        def pose(m,side):
            q=m.pose.orientation;t=m.pose.position;R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            with lock:poses[side][key(m)]=(R,np.array([t.x,t.y,t.z]));join(side)
        rospy.init_node('visual_depth_observer',anonymous=True,disable_signals=True)
        subs=[rospy.Subscriber('/rmf_unipilot/lidar/points',PointCloud2,cloud,queue_size=2,buff_size=4000000)]
        for side,prefix in [('active',''),('fixed','/fixed')]:
            subs.extend([rospy.Subscriber(prefix+'/rovio/pcl',PointCloud2,lambda m,s=side:feature(m,s),queue_size=10),
                         rospy.Subscriber(prefix+'/active/extrinsic',PoseStamped,lambda m,s=side:pose(m,s),queue_size=10)])
        try:
            while not (a.flight/'runtime.json').exists() and time.monotonic()-start<a.max_wall_s:time.sleep(.2)
        finally:
            for s in subs:s.unregister()
            rospy.signal_shutdown('Observation finished')
    a.output.with_suffix('.json').write_text(json.dumps(dict(scope=__doc__,counts=counts,errors=errors,
        finished_reference=(a.flight/'runtime.json').exists(),limitation='LiDAR-supported planar rays only; depth matching is a diagnostic approximation, not a truth oracle or estimator input.'),indent=2)+'\n')
    print(counts,errors,flush=True)


if __name__=='__main__':main()
