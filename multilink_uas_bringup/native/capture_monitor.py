#!/usr/bin/env python3
"""Raw sensor health and throughput only; no LIO/VIO or image processing."""
import argparse,csv,json,time,subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,Imu,PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class Capture(Node):
 def __init__(self,out):
  super().__init__('capture_monitor');self.out=out;self.sim=0.;self.start=time.monotonic();self.base_wall=self.start;self.base_sim=0.;self.samples={};self.counts={};self.last={};self.gaps={};self.windows=[];self.target=1.;self.phase=0;self.ready=False
  self.f=(out/'truth_raw.csv').open('w',buffering=1);self.writer=csv.writer(self.f);self.writer.writerow(['t','x','y','z','qx','qy','qz','qw'])
  self.sensor_subs=[]
  channels=[('imu','/dragon/imu/raw',Imu),('lidar','/dragon/lidar/points',PointCloud2),('gt','/dragon/ground_truth',Odometry)]+[(f'cam{i}',f'/dragon/cam_link{i}/image',Image) for i in (1,3,4)]
  for name,topic,cls in channels:
   sub=self.create_subscription(cls,topic,lambda m,n=name:self.message(n,m),qos_profile_sensor_data)
   if name!='gt':self.sensor_subs.append(sub)
  self.create_timer(1.,self.status)
 def message(self,n,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  self.counts[n]=self.counts.get(n,0)+1;self.samples[n]=self.samples.get(n,0)+1
  if n in self.last:self.gaps[n]=max(self.gaps.get(n,0),t-self.last[n])
  self.last[n]=t
  if n=='gt':
   self.sim=t
   p=m.pose.pose.position;q=m.pose.pose.orientation;self.writer.writerow([t,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
 def factor(self,value):
  world=json.loads((self.out/'world.json').read_text())['world_name']
  p=subprocess.run(['gz','service','-s',f'/world/{world}/set_physics','--reqtype','gz.msgs.Physics','--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'real_time_factor: {value}'],capture_output=True,text=True,timeout=8)
  if p.returncode or 'true' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)
  self.target=value;self.get_logger().info(f'Gazebo target factor {value}; physical step and sensors unchanged')
 def status(self):
  now=time.monotonic();delta=now-self.base_wall;ds=self.sim-self.base_sim
  if len(self.counts)==6 and min(self.counts.values())>=30 and self.phase==0:
   self.factor(2.);self.phase=1;self.base_wall=now;self.base_sim=self.sim;self.samples={};self.gaps={}
  elif self.phase and delta>=12 and ds>0:
   rates={k:v/ds for k,v in self.samples.items()};bad=any(rates.get(k,0)<rate*.97 for k,rate in [('imu',1000),('lidar',10),('cam1',20),('cam3',20),('cam4',20)])
   window=dict(target_factor=self.target,actual_factor=ds/delta,rates_sim_hz=rates,max_stamp_gaps=self.gaps.copy(),bad=bad,sim_time=self.sim)
   self.windows.append(window)
   if self.phase<3:
    self.phase+=1;self.factor(4. if self.phase==2 else 0.)
   elif self.phase==3:
    candidates=[w for w in self.windows[-3:] if not w['bad']]
    best=max(candidates,key=lambda x:x['actual_factor']) if candidates else {'target_factor':1.}
    self.factor(best['target_factor']);self.phase=4;self.ready=True
    for sub in self.sensor_subs:self.destroy_subscription(sub)
    self.sensor_subs=[]
   elif self.phase==4:
    window['bad']=None;window['note']='Rate probe subscriptions removed after selection; recorder remains subscribed to every raw sensor'
   self.base_wall=now;self.base_sim=self.sim;self.samples={};self.gaps={}
  report=dict(counts=self.counts if self.ready else {},sensor_counts=self.counts,ready=self.ready,sim_time=self.sim,wall_s=now-self.start,target_factor=self.target,windows=self.windows)
  tmp=self.out/'capture_status.tmp';tmp.write_text(json.dumps(report,indent=2));tmp.replace(self.out/'capture_status.json')

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();rclpy.init();n=Capture(args.output)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.f.close();n.destroy_node()
