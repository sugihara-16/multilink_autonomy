#!/usr/bin/env python3
"""Read-only comparison of VIO landmark range with same-stamp LiDAR support."""
import argparse,csv,json,time,threading
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rospy,yaml
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from core import camera_rotation
from image_geometry_map import depth_at_pixels

def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--max-wall-s',type=float,default=1800.);a=p.parse_args()
    rospy.init_node('planning_depth_probe',anonymous=True)
    K=np.array(yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data']).reshape(3,3)
    camera=json.loads((a.flight/'camera_manifest.json').read_text())['cameras'][0]
    T=np.array(camera['T_FC_camera']);home,tbc=T[:3,:3],T[:3,3]
    l=yaml.safe_load((a.flight/'mimosa.yaml').read_text())['lidar']['T_B_S'];Rbl=Rotation.from_quat(l[3:]).as_matrix();tbl=np.array(l[:3])
    scans={};encoders={};features={m:{} for m in ['active','fixed']};lock=threading.Lock();counts={};last={m:-10**18 for m in features}
    f=(a.flight/'planning_depth_probe.csv').open('x');writer=csv.writer(f)
    writer.writerow(['stamp_ns','mode','tracked','lidar_supported','vio_range_p50','lidar_range_p50','range_ratio_p10','range_ratio_p50','range_ratio_p90'])
    def key(t):return int(round(t/1000))*1000
    def trim(cache):
        while len(cache)>30:cache.pop(min(cache))
    def scan(m):
        t=key(m.header.stamp.to_nsec())
        if t%1000000000: return
        points=np.array(list(pc2.read_points(m,field_names=['x','y','z'],skip_nans=True))).reshape(-1,3)
        with lock:scans[t]=points@Rbl.T+tbl;trim(scans)
    def enc(m):
        t=key(m.data[0]*1e9)
        if t%1000000000:return
        with lock:encoders[t]=np.array(m.data[1:]);trim(encoders)
    def feature(m,mode):
        t=key(m.header.stamp.to_nsec())
        if t%1000000000:return
        points=np.array([r[1:] for r in pc2.read_points(m,field_names=['status','x','y','z'],skip_nans=True) if r[0]==4]).reshape(-1,3)
        with lock:features[mode][t]=points;trim(features[mode])
    subs=[rospy.Subscriber('/slam/input/points',PointCloud2,scan,queue_size=2,buff_size=4000000),rospy.Subscriber('/active/encoder',Float64MultiArray,enc,queue_size=2000)]
    for mode,topic in [('active','/rovio/pcl'),('fixed','/fixed/rovio/pcl')]:subs.append(rospy.Subscriber(topic,PointCloud2,lambda m,mode=mode:feature(m,mode),queue_size=2,buff_size=1000000))
    start=time.monotonic()
    try:
        while not rospy.is_shutdown() and not (a.flight/'runtime.json').exists() and time.monotonic()-start<a.max_wall_s:
            with lock:
                work=[]
                for mode in features:
                    common=sorted(set(features[mode])&set(scans)&set(encoders))
                    for t in common:
                        points=features[mode].pop(t)
                        if t<=last[mode]:continue
                        last[mode]=t;work.append((mode,t,points,scans[t],encoders[t] if mode=='active' else np.zeros(2)))
            for mode,t,points,cloud,angles in work:
                R=camera_rotation(home,*angles);pc=(points-tbc)@R;valid=pc[:,2]>.1
                pc,points=pc[valid],points[valid]
                pixels=pc[:,:2]/pc[:,2,None]*np.array([K[0,0],K[1,1]])+K[:2,2]
                hits,_=depth_at_pixels(pixels,cloud,R,tbc,K,30.)
                good=np.isfinite(hits).all(axis=1);rv=np.linalg.norm(points[good]-tbc,axis=1);rl=np.linalg.norm(hits[good]-tbc,axis=1)
                values=[np.median(rv),np.median(rl),*np.percentile(rv/rl,[10,50,90])] if len(rv) else ['']*5
                writer.writerow([t,mode,len(points),len(rv),*values]);f.flush();counts[mode]=counts.get(mode,0)+1
            time.sleep(.1)
    finally:
        rospy.signal_shutdown('Owner finished or probe deadline')
        for s in subs:s.unregister()
        f.close();(a.flight/'planning_depth_probe.json').write_text(json.dumps(dict(counts=counts,scope='Read-only 1 Hz same-stamp comparison. LiDAR planar support is an independent geometric reference, not truth. No estimator or planner inputs changed; no images/clouds retained.'),indent=2)+'\n')
if __name__=='__main__':main()
