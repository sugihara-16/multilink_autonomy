#!/usr/bin/env python3
"""Measure simulated throughput and received sensor timestamps without changing quality."""
import argparse,json,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image,Imu,PointCloud2,JointState
from nav_msgs.msg import Odometry

class Probe(Node):
    def __init__(self):
        super().__init__('active_speed_probe');self.clock=None;self.rows={};self.subs=[]
        self.subs.append(self.create_subscription(Clock,'/clock',self.tick,qos_profile_sensor_data))
        for name,topic,typ in [('imu','/rmf_unipilot/imu',Imu),('image','/rmf_unipilot/cam_gimbal/image',Image),('fixed_image','/rmf_unipilot/cam_fixed/image',Image),
            ('lidar','/rmf_unipilot/lidar/points',PointCloud2),('truth','/rmf_unipilot/odom',Odometry),
            ('encoder','/active/gimbal/states',JointState),('vio','/rovio/odometry',Odometry),('fusion','/slam/odometry',Odometry)]:
            qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE) if name=='encoder' else qos_profile_sensor_data
            self.subs.append(self.create_subscription(typ,topic,lambda m,k=name:self.message(k,m),qos))
    def tick(self,m):self.clock=m.clock.sec+m.clock.nanosec*1e-9
    def message(self,key,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.clock is not None:self.rows.setdefault(key,[]).append((t,self.clock-t))
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)

def factor(value,world):
    p=subprocess.run(['gz','service','-s',f'/world/{world}/set_physics','--reqtype','gz.msgs.Physics','--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'real_time_factor: {value}'],capture_output=True,text=True,timeout=10,check=True)
    if 'true' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--world',default='zab')
    p.add_argument('--factors',nargs='+',type=float,default=[1,5,100]);p.add_argument('--seconds',type=float,default=15);a=p.parse_args()
    rclpy.init();n=Probe();result=[]
    try:
        deadline=time.monotonic()+60
        while n.clock is None and time.monotonic()<deadline:n.spin(1)
        n.spin(2)
        if n.clock is None:raise RuntimeError('No simulation clock')
        for target in a.factors:
            factor(target,a.world);n.spin(2);n.rows={};t=time.monotonic();s=n.clock;n.spin(a.seconds)
            wall=time.monotonic()-t;sim=n.clock-s;channels={}
            for k,v in n.rows.items():
                x=np.array(v);g=np.diff(x[:,0]);channels[k]=dict(count=len(x),hz_sim=len(x)/sim,
                    gap_max_s=float(g.max()) if len(g) else None,lag_p95_s=float(np.quantile(x[:,1],.95)),lag_end_s=float(x[-1,1]))
            row=dict(target_factor=target,actual_factor=sim/wall,wall_s=wall,sim_s=sim,channels=channels)
            result.append(row);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(row),flush=True)
    finally:
        factor(1,a.world);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
