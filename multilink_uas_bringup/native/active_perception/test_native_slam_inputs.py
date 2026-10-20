#!/usr/bin/env python3
"""Compare original and native sensor adapter output bytes over actual ROS topics."""
import io,json,os,signal,subprocess,time,sys
gated="--gate-slam" in sys.argv
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import Imu,PointCloud2,PointField
from std_msgs.msg import Bool
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
children=[];result={}
def wait(predicate,timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if predicate():return
  time.sleep(.01)
 raise RuntimeError('Sensor adapter timeout')
def serialized(m):
 # ROS publishers assign transport sequence numbers independently.
 m.header.seq=0
 f=io.BytesIO();m.serialize(f);return f.getvalue()
try:
 children.append(subprocess.Popen(['roscore','-p','11490'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True));time.sleep(2)
 rospy.init_node('sensor_adapter_comparison')
 commands=[['python3',str(HERE.parent/'slam_inputs.py')],[str(WS/'build/native-adapters/native_slam_inputs'),
   '__name:=native_slam_inputs','/slam/input/imu:=/native/imu','/slam/input/points:=/native/points']]
 if gated:
  for cmd in commands:cmd.append('_gate_slam:=true')
 children.extend(subprocess.Popen(cmd,stdout=subprocess.DEVNULL,start_new_session=True) for cmd in commands)
 messages={k:{} for k in ['old_imu','new_imu','old_points','new_points','gated_imu']}
 subscribers=[]
 for key,topic,typ in [('old_imu','/slam/input/imu',Imu),('new_imu','/native/imu',Imu),('old_points','/slam/input/points',PointCloud2),('new_points','/native/points',PointCloud2),('gated_imu','/vio/initialized_imu',Imu)]:
  subscribers.append(rospy.Subscriber(topic,typ,lambda m,k=key:messages[k].__setitem__(m.header.stamp.to_nsec(),serialized(m)),queue_size=1000))
 imu=rospy.Publisher('/rmf_unipilot/imu',Imu,queue_size=1000);cloud=rospy.Publisher('/rmf_unipilot/lidar/points',PointCloud2,queue_size=50);gate=rospy.Publisher('/active/initialize_vio',Bool,queue_size=1)
 wait(lambda:imu.get_num_connections()==2 and cloud.get_num_connections()==2 and gate.get_num_connections()==2)
 if gated:
  for t in [0.,2.5,3.]:
   m=Imu();m.header.stamp=rospy.Time.from_sec(t);imu.publish(m);time.sleep(.05)
  m=PointCloud2();m.header.stamp=rospy.Time(3);cloud.publish(m);time.sleep(.2)
  assert not any(messages.values()), 'Messages leaked before hover gate'
 gate.publish(Bool(True));time.sleep(.1)
 for i in range(650):
  m=Imu();m.header.stamp=rospy.Time.from_sec(i*.005+(4. if gated else 0.));m.header.frame_id='raw';m.orientation.w=.5;m.orientation.x=.5;m.orientation.y=.5;m.orientation.z=.5
  m.linear_acceleration.x=i*.001;m.linear_acceleration.z=9.8;m.angular_velocity.y=-.03;m.orientation_covariance=list(range(9));imu.publish(m);time.sleep(.001)
 expected=650 if gated else 250
 wait(lambda:all(len(messages[k])==expected for k in ['old_imu','new_imu','gated_imu']))
 for name in ['old_imu','new_imu','gated_imu']:
  print(name,len(messages[name]),sorted(messages[name])[:2],flush=True)
 for t in messages['old_imu']:
  vals=[messages[k].get(t,b'') for k in ['old_imu','new_imu','gated_imu']]
  if len(set(vals))>1:print('First differing IMU',t,[(len(v),list(v[:16])) for v in vals],flush=True);break
 assert messages['old_imu']==messages['new_imu']==messages['gated_imu'],'IMU output mismatch'
 rng=np.random.default_rng(329)
 for i in range(12):
  width,height=500,48;step=24;row_step=step*width+16
  raw=np.zeros((height,row_step),np.uint8)
  dtype=np.dtype(dict(names=['x','y','z','intensity','ring'],formats=['<f4']*4+['<u2'],offsets=[0,4,8,16,20],itemsize=step))
  a=np.ndarray((height,width),dtype=dtype,buffer=raw,strides=(row_step,step))
  for k in ['x','y','z','intensity']:a[k]=rng.normal(size=(height,width))
  a['ring']=np.arange(height)[:,None];a['x'][0,0]=np.nan;a['z'][0,1]=np.inf
  m=PointCloud2();m.header.stamp=rospy.Time(20,i*1000000);m.header.frame_id='raw';m.height=height;m.width=width;m.point_step=step;m.row_step=row_step;m.data=raw.tobytes()
  m.fields=[PointField(k,offset,datatype,1) for k,offset,datatype in [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('ring',20,4)]];cloud.publish(m)
  wait(lambda:len(messages['old_points'])==i+1 and len(messages['new_points'])==i+1)
 assert messages['old_points']==messages['new_points'],'PointCloud2 output mismatch'
 result=dict(imu_messages=len(messages['old_imu']),clouds=len(messages['old_points']),messages_identical_except_transport_sequence=True,gated_imu_identical=True,slam_gate_enabled=gated)
finally:
 for proc in reversed(children):
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
out=WS/('build/native-adapters/sensor_gate_validation.json' if gated else 'build/native-adapters/sensor_validation.json');out.write_text(json.dumps(result,indent=2)+'\n');print(out.read_text())
