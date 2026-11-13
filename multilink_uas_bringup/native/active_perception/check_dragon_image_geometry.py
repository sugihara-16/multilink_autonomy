#!/usr/bin/env python3
"""Read-only rendered-image / encoder timing check. Never publishes to control.

Only numeric residuals are retained; images live in a small memory buffer.
Ground truth is used solely by this independent diagnostic observer.
"""
import argparse,json,time
from collections import deque
from pathlib import Path
import cv2,numpy as np,rospy,yaml
from scipy.spatial.transform import Rotation,Slerp
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from audit_camera_motion import epipolar_pixels

def pose(m,odom=False):
    p=m.pose.pose if odom else m.pose
    return (m.header.stamp.to_sec(),np.array([p.position.x,p.position.y,p.position.z]),np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]))

def interp(history,t):
    d=sorted({x[0]:x for x in history}.values(),key=lambda x:x[0]);ts=np.array([x[0] for x in d]);i=np.searchsorted(ts,t)
    if i==0 or i==len(d):return None
    a,b=d[i-1],d[i];f=(t-a[0])/(b[0]-a[0])
    if b[0]-a[0]>.1:return None
    return (1-f)*a[1]+f*b[1],Slerp(ts[i-1:i+1],Rotation.from_quat([a[2],b[2]]))([t])[0]

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--wall-seconds',type=float,default=300);a=p.parse_args(rospy.myargv()[1:])
    cv2.setNumThreads(1);rospy.init_node('read_only_image_geometry_check',anonymous=True)
    truth=deque(maxlen=3000);extrinsics=deque(maxlen=600);images=deque(maxlen=80)
    cfg=yaml.safe_load((a.trial/'cam0.yaml').read_text());K=np.array(cfg['camera_matrix']['data']).reshape(3,3)
    subs=[rospy.Subscriber('/dragon/ground_truth',Odometry,lambda m:truth.append(pose(m,True)),queue_size=300),
          rospy.Subscriber('/active/extrinsic',PoseStamped,lambda m:extrinsics.append(pose(m)),queue_size=100),
          rospy.Subscriber('/active/image',Image,lambda m:images.append(m),queue_size=40)]
    rows=[];last=-1;beg=time.monotonic();offsets=[-.05,-.02,0,.02,.05]
    def gray(m):return np.ndarray((m.height,m.width),np.uint8,buffer=m.data,strides=(m.step,1)).copy()
    try:
        while not rospy.is_shutdown() and time.monotonic()-beg<a.wall_seconds:
            time.sleep(.1);ims=list(images)
            if len(ims)<6:continue
            m0,m1=ims[-5],ims[-3];t0=m0.header.stamp.to_sec();t1=m1.header.stamp.to_sec()
            if t0-last<.5:continue
            poses=[]
            for offset in offsets:
                pair=[]
                for t in (t0+offset,t1+offset):
                    b=interp(list(truth),t);e=interp(list(extrinsics),t)
                    if b is None or e is None:break
                    pair.append((b[0]+b[1].apply(e[0]),b[1]*e[1],e[1]))
                if len(pair)!=2:break
                poses.append(pair)
            if len(poses)!=len(offsets):continue
            last=t0;im0,im1=gray(m0),gray(m1)
            corners=cv2.goodFeaturesToTrack(im0,180,.01,10,blockSize=7)
            if corners is None:continue
            x=corners.reshape(-1,2);y,ok,_=cv2.calcOpticalFlowPyrLK(im0,im1,x,None,winSize=(21,21),maxLevel=3)
            back,ok2,_=cv2.calcOpticalFlowPyrLK(im1,im0,y,None,winSize=(21,21),maxLevel=3)
            valid=ok.ravel().astype(bool)&ok2.ravel().astype(bool)&(np.linalg.norm(back-x,axis=1)<.75)
            x,y=x[valid],y[valid]
            if len(x)<12:continue
            error=[]
            for pair in poses:
                c0,c1=pair;R=(c1[1].inv()*c0[1]).as_matrix();t=c1[1].inv().apply(c0[0]-c1[0])
                error.append(float(np.median(epipolar_pixels(x,y,R,t,K))))
            c0,c1=poses[2]
            rows.append(dict(time=t0,tracks=len(x),median_epipolar_error_px=error,
                camera_relative_rotation_deg=float(np.rad2deg((c1[2].inv()*c0[2]).magnitude())),baseline_m=float(np.linalg.norm(c1[0]-c0[0]))))
    finally:
        for sub in subs:sub.unregister()
        groups={}
        for name,rs in [('all',[r for r in rows if r['baseline_m']>.01]),('moving_camera',[r for r in rows if r['baseline_m']>.01 and r['camera_relative_rotation_deg']>.1])]:
            if rs:
                median=np.median([r['median_epipolar_error_px'] for r in rs],axis=0)
                groups[name]=dict(pairs=len(rs),median_errors_px=median.tolist(),best_offset_s=offsets[int(np.argmin(median))])
        (a.trial/'independent_image_geometry.json').write_text(json.dumps(dict(offsets_s=offsets,groups=groups,rows=rows,
            scope='Read-only actual rendered image optical flow against independently measured vehicle truth and encoded camera pose. No images retained; no diagnostic data published.'),indent=2)+'\n')
        print(json.dumps(groups),flush=True)

if __name__=='__main__':main()
