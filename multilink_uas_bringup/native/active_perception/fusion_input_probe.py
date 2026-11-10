#!/usr/bin/env python3
"""Read-only exact input/geometry diagnostics, finite under the flight owner."""
import argparse,csv,json,threading
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import Imu,PointCloud2
from std_msgs.msg import Float64MultiArray
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(exist_ok=True);rospy.init_node('fusion_input_probe',anonymous=True)
lock=threading.Lock();last={'cloud':-1e9,'info':-1e9}
f=(a.output/'imu.csv').open('w');imu=csv.writer(f);imu.writerow(['t','ax','ay','az','wx','wy','wz'])
g=(a.output/'lidar_information.jsonl').open('w')
def inertial(m):
 v=m.linear_acceleration;w=m.angular_velocity
 imu.writerow([m.header.stamp.to_sec(),v.x,v.y,v.z,w.x,w.y,w.z])
def cloud(m):
 t=m.header.stamp.to_sec()
 if t-last['cloud']<10:return
 last['cloud']=t
 offsets={x.name:x.offset for x in m.fields}
 dt=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[offsets[k] for k in ['x','y','z']],'itemsize':m.point_step})
 rows=np.frombuffer(m.data,dtype=dt);xyz=np.column_stack([rows[k] for k in ['x','y','z']]);xyz=xyz[np.isfinite(xyz).all(axis=1)]
 np.savez_compressed(a.output/f'cloud_{t:.3f}.npz',t=t,xyz=xyz)
def info(m):
 x=list(m.data)
 if len(x)<2 or x[1]-last['info']<1:return
 last['info']=x[1];g.write(json.dumps(x)+'\n');g.flush()
subs=[rospy.Subscriber('/slam/input/imu',Imu,inertial,queue_size=10000),rospy.Subscriber('/slam/input/points',PointCloud2,cloud,queue_size=10),rospy.Subscriber('/active/lidar_information',Float64MultiArray,info,queue_size=50)]
try:rospy.spin()
finally:
 for s in subs:s.unregister()
 f.close();g.close()
