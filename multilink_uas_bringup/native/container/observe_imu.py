"""Independent timestamp audit at Gazebo and ROS2 sides of the IMU bridge."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time
import rclpy
from sensor_msgs.msg import Imu
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--duration',type=float,default=180)
args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=True)
data={'gazebo':[],'ros2':[]}
lock=threading.Lock()
def stamp(name,value):
    with lock:data[name].append(value)
transport=Node()
def gz_callback(msg):stamp('gazebo',msg.header.stamp.sec*1000000000+msg.header.stamp.nsec)
transport.subscribe(IMU,'/dragon/imu/raw',gz_callback)
rclpy.init();node=rclpy.create_node('imu_path_audit')
sub=node.create_subscription(Imu,'/dragon/imu/raw',lambda m:stamp('ros2',m.header.stamp.sec*1000000000+m.header.stamp.nanosec),2000)
deadline=time.monotonic()+args.duration
try:
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
except KeyboardInterrupt:
    pass
finally:
    with lock:
        result={}
        for name,values in data.items():
            result[name]=dict(count=len(values),first=values[0] if values else None,last=values[-1] if values else None,
                gaps=[[a,b] for a,b in zip(values,values[1:]) if b-a!=1000000])
            (args.output/(name+'.json')).write_text(json.dumps(values))
        (args.output/'summary.json').write_text(json.dumps(result,indent=2))
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
