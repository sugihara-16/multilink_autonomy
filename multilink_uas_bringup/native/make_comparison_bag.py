#!/usr/bin/env python3
"""Lossless ROS1 input adapter, with exact integer source header timestamps."""
import csv,json
from pathlib import Path
import numpy as np
import rosbag,rospy,yaml
from sensor_msgs.msg import Imu,Image,PointCloud2,PointField
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/livo_comparison'
def stamp(t): return rospy.Time(*divmod(int(t),10**9))
def main(label, include_images=True):
    inp=OUT/'inputs'/label;manifest=json.loads((inp/'manifest.json').read_text())
    odoms=list(csv.DictReader((OUT/'rovio'/label/'result_vio.csv').open()));oi=0
    bagpath=OUT/'inputs'/f'{label}.bag';truth=[];counts={}
    dtype=np.dtype({'names':['x','y','z','intensity','t','line','ring','reflectivity','ambient','tag','range'],
        'formats':['<f4']*4+['<u4','u1','u1','<u2','<u2','u1','<u4'],
        'offsets':[0,4,8,12,16,20,21,22,24,26,28],'itemsize':32})
    fields=[PointField(n,dtype.fields[n][1],PointField.FLOAT32 if n in ('x','y','z','intensity') else PointField.UINT32 if n in ('t','range') else PointField.UINT16 if n in ('reflectivity','ambient') else PointField.UINT8,1) for n in ('x','y','z','intensity','t','reflectivity','ring')]
    def odom(row):
        m=Odometry();m.header.stamp=stamp(row['stamp_ns']);m.header.frame_id='vio_world';m.child_frame_id='slam_imu'
        p=m.pose.pose.position;q=m.pose.pose.orientation
        p.x,p.y,p.z=[float(row[k]) for k in ('x','y','z')];q.x,q.y,q.z,q.w=[float(row[k]) for k in ('qx','qy','qz','qw')]
        m.pose.covariance=[float(row['cov'+str(i)]) for i in range(36)];return m
    with rosbag.Bag(str(bagpath),'w',compression='lz4') as bag:
        for chunk in manifest['chunks']:
            with np.load(inp/chunk) as archive:
                a={k:archive[k] for k in archive.files}
                for t,v in zip(a['truth_t'],a['truth']):truth.append([int(t),*v])
                for t,kind,i,stored in a['order'][np.argsort(a['order'][:,0],kind='stable')]:
                    if t<2003000000:continue
                    while oi<len(odoms) and int(odoms[oi]['stamp_ns'])<int(t):
                        m=odom(odoms[oi]);bag.write('/comparison/vio',m,m.header.stamp);oi+=1
                    if kind==0:
                        m=Imu();m.header.frame_id='slam_imu';v=a['imu'][i];m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=v[:3];m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=v[3:];m.orientation_covariance[0]=-1;topic='/comparison/imu'
                    elif kind==2:
                        if not include_images:continue
                        m=Image();m.header.frame_id='cam_link3_optical';m.width=480;m.height=360;m.encoding='bgr8';m.step=1440;m.data=np.repeat(a['cam1_im'][i][:,:,None],3,axis=2).tobytes();topic='/comparison/image'
                    elif kind==4:
                        v=a['cloud'][i];r=np.linalg.norm(v[:,:3],axis=1);v=v[np.isfinite(v[:,:3]).all(axis=1)&(r>=0.8)&(r<=30)]
                        cloud=np.zeros(len(v),dtype=dtype)
                        for j,k in enumerate(('x','y','z','intensity')):cloud[k]=v[:,j]
                        cloud['line']=v[:,4];cloud['ring']=v[:,4]
                        cloud['range']=(np.linalg.norm(v[:,:3],axis=1)*1000).astype(np.uint32)
                        m=PointCloud2();m.header.frame_id='slam_lidar';m.height=1;m.width=len(v);m.fields=fields;m.point_step=32;m.row_step=len(v)*32;m.is_dense=True;m.data=cloud.tobytes();topic='/comparison/points'
                    else:continue
                    m.header.stamp=stamp(t);bag.write(topic,m,m.header.stamp);counts[topic]=counts.get(topic,0)+1
        while oi<len(odoms):m=odom(odoms[oi]);bag.write('/comparison/vio',m,m.header.stamp);oi+=1
    with (inp/'truth.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz']);w.writerows(truth)
    (inp/'bag_summary.json').write_text(json.dumps(dict(counts=counts,vio_count=oi,bytes=bagpath.stat().st_size,zero_point_time=True),indent=2))
if __name__=='__main__':
    import sys
    for label in sys.argv[1:]:main(label)
