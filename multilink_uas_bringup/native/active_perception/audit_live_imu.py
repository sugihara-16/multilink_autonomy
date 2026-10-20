#!/usr/bin/env python3
"""Observe source/ROS2 IMU continuity; records timestamps and gaps, no sensor data."""
import argparse,json,time,threading
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu
from gz.transport13 import Node as GzNode
from gz.msgs10.imu_pb2 import IMU
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--duration',type=float,default=300);a=p.parse_args()
rclpy.init();node=rclpy.create_node('independent_imu_continuity_observer');gz=GzNode();lock=threading.Lock();streams={k:[] for k in ('gz','ros2')};sequences={}
def add(name,t):
    with lock:streams[name].append(t)
def source(m):
    t=m.header.stamp.sec*10**9+m.header.stamp.nsec
    with lock:
        streams['gz'].append(t)
        for item in m.header.data:
            if item.key=='seq':sequences[t]=int(item.value[0])
gz.subscribe(IMU,'/rmf_unipilot/imu',source)
sub=node.create_subscription(Imu,'/rmf_unipilot/imu',lambda m:add('ros2',m.header.stamp.sec*10**9+m.header.stamp.nanosec),QoSProfile(depth=5000,reliability=ReliabilityPolicy.RELIABLE))
start=time.monotonic();last=start
def save():
    with lock:data={k:list(v) for k,v in streams.items()};seq=dict(sequences)
    result={}
    for k,v in data.items():
        unique=sorted(set(v));gaps=[[x,y] for x,y in zip(unique,unique[1:]) if y-x>1000000]
        result[k]=dict(count=len(v),duplicates=len(v)-len(unique),first=v[0] if v else None,last=v[-1] if v else None,gaps=gaps[:1000],total_gaps=len(gaps))
        if k=='gz':result[k]['gap_sequences']=[[seq.get(x),seq.get(y)] for x,y in gaps[:1000]]
    if all(data.values()):
        lo=max(v[0] for v in data.values());hi=min(v[-1] for v in data.values());s={k:set(t for t in v if lo<=t<=hi) for k,v in data.items()}
        result['source_only']=sorted(s['gz']-s['ros2']);result['ros2_only']=sorted(s['ros2']-s['gz'])
    a.output.write_text(json.dumps(result,indent=2)+'\n')
try:
    while time.monotonic()-start<a.duration:
        rclpy.spin_once(node,timeout_sec=.02)
        if time.monotonic()-last>5:save();last=time.monotonic()
except KeyboardInterrupt:
    pass
finally:
    save();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
