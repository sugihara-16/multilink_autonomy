#!/usr/bin/env python3
"""Short hover throughput sweep. Changes only Gazebo's target real-time factor."""
import argparse,json,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image,Imu,PointCloud2
from nav_msgs.msg import Odometry

class Monitor(Node):
    def __init__(self):
        super().__init__('sim_speed_probe');self.clock=None;self.data={};self.pose={};self.subs=[]
        self.subs.append(self.create_subscription(Clock,'/clock',self.tick,qos_profile_sensor_data))
        channels=[('imu','/dragon/imu/raw',Imu),('lidar','/dragon/lidar/points',PointCloud2),
                  ('vio','/rovio/odometry',Odometry),('mimosa','/slam/odometry',Odometry),
                  ('gt','/dragon/ground_truth',Odometry)]
        channels += [(f'cam{i}',f'/dragon/cam_link{i}/image',Image) for i in (1,3,4)]
        for name,topic,kind in channels:
            self.subs.append(self.create_subscription(kind,topic,lambda m,n=name:self.message(n,m),qos_profile_sensor_data))
    def tick(self,m):self.clock=m.clock.sec+m.clock.nanosec*1e-9
    def message(self,name,m):
        if self.clock is None:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.data.setdefault(name,[]).append((stamp,self.clock-stamp))
        if hasattr(m,'pose'):
            p=m.pose.pose.position;self.pose.setdefault(name,[]).append((p.x,p.y,p.z))
    def run(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)

def factor(value):
    p=subprocess.run(['gz','service','-s','/world/map/set_physics','--reqtype','gz.msgs.Physics','--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'real_time_factor: {value}'],capture_output=True,text=True,check=True)
    if 'true' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--flight-log',type=Path,required=True);args=p.parse_args()
    rclpy.init();n=Monitor();reports=[]
    try:
        deadline=time.monotonic()+240
        while not (args.flight_log/'flight_slam_result.json').exists():
            n.run(1);n.data={};n.pose={}
            if time.monotonic()>deadline:raise RuntimeError('Stable hover not confirmed')
        flight=json.loads((args.flight_log/'flight_slam_result.json').read_text())
        if flight.get('failure') or flight.get('failed'):raise RuntimeError(str(flight))
        print('Hover ready; starting multiplier sweep',flush=True)
        for target in (1.,1.5,2.,4.,1.):
            factor(target);n.run(5);n.data={};n.pose={};t=time.monotonic();s=n.clock;n.run(20)
            wall=time.monotonic()-t;sim=n.clock-s;channels={}
            for key,values in n.data.items():
                a=np.array(values);g=np.diff(a[:,0]);lag=a[:,1]
                channels[key]=dict(count=len(a),hz_sim=len(a)/sim,hz_wall=len(a)/wall,
                    stamp_gap_p99_s=float(np.quantile(g,.99)) if len(g) else None,stamp_gap_max_s=float(g.max()) if len(g) else None,
                    lag_p50_sim_s=float(np.median(lag)),lag_p95_sim_s=float(np.quantile(lag,.95)),lag_end_sim_s=float(lag[-1]))
            poses={k:dict(min_xyz=np.min(v,axis=0).tolist(),max_xyz=np.max(v,axis=0).tolist()) for k,v in n.pose.items()}
            report=dict(target_factor=target,actual_factor=sim/wall,wall_s=wall,sim_s=sim,channels=channels,positions=poses)
            reports.append(report);args.output.write_text(json.dumps(dict(physics_step_s=.001,flight_log=str(args.flight_log),warmup_wall_s=5,windows=reports),indent=2)+'\n')
            print(json.dumps(report),flush=True)
    finally:
        factor(1.);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
