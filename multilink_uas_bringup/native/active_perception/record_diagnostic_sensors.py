#!/usr/bin/env python3
"""Record reference sensors only; never publish them to flight control or estimators."""
import argparse,json,struct,signal
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu,Image
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output/'diagnostic_sensors';out.mkdir()
# Do not destroy the ROS context asynchronously while a callback is taking a
# message. Finish the callback, flush files, then shut down from this thread.
stop_signal=0
def stop(signum,frame):
 global stop_signal
 stop_signal=signum
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('diagnostic_sensor_recorder');files={k:(out/(k+('.csv' if k=='imu' else '.bin'))).open('w' if k=='imu' else 'wb') for k in ['imu','clean','wide','far']};counts={k:0 for k in files}
files['imu'].write('stamp_ns,gx,gy,gz,ax,ay,az\n')
files['imu_fast']=(out/'imu_fast.csv').open('w');files['imu_fast'].write('stamp_ns,gx,gy,gz,ax,ay,az\n');counts['imu_fast']=0
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def imu(m,key='imu'):
 w=m.angular_velocity;v=m.linear_acceleration;files[key].write(str(stamp(m))+','+','.join(format(x,'.17g') for x in [w.x,w.y,w.z,v.x,v.y,v.z])+'\n');counts[key]+=1
def image(m,key):
 assert m.encoding in ['rgb8','bgr8']
 raw=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1));gray=cv2.cvtColor(raw,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
 files[key].write(struct.pack('<QII',stamp(m),m.width,m.height));files[key].write(gray.tobytes());counts[key]+=1
def report():
 for f in files.values():f.flush()
 (out/'counts.json').write_text(json.dumps(counts,indent=2)+'\n')
node.create_subscription(Imu,'/diagnostic/imu_clean',imu,2000)
node.create_subscription(Imu,'/diagnostic/imu_fast',lambda m:imu(m,'imu_fast'),5000)
for key in ['clean','wide','far']:node.create_subscription(Image,'/diagnostic/image_'+key,lambda m,key=key:image(m,key),30)
node.create_timer(2.,report)
try:
 while not stop_signal and rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
except (KeyboardInterrupt,ExternalShutdownException):pass
finally:
 report()
 for f in files.values():f.close()
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 (out/'shutdown.json').write_text(json.dumps(dict(stop_signal=stop_signal,files_closed=True,counts=counts),indent=2)+'\n')
