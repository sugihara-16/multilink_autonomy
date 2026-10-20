#!/usr/bin/env python3
"""Feed identical synthetic exposures/encoders to Python and C++ adapters."""
import argparse,json,os,signal,subprocess,time
import yaml
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image,JointState
from geometry_msgs.msg import PoseStamped
from array import array
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
parser=argparse.ArgumentParser();parser.add_argument('--calibration',type=Path,default=HERE.parent/'logs/speed6-baseline/run1_tail_a1_active');parser.add_argument('--active-only',action='store_true');parser.add_argument('--output',type=Path,default=WS/'build/native-adapters/image_validation.json');args=parser.parse_args();cal=args.calibration
rclpy.init();node=Node('native_encoder_comparison');result={};processes=[]
def spin_until(predicate,timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.02)
  if predicate():return
 raise RuntimeError('Adapter comparison timeout')
try:
 for fixed in ([False] if args.active_only else [False,True]):
  folder=cal/'fixed' if fixed else cal;prefix='/fixed' if fixed else '';suffix=['--fixed'] if fixed else []
  raw=yaml.safe_load((folder/'raw_cam0.yaml').read_text());width=raw['image_width'];height=raw['image_height']
  commands=[['python3',str(HERE/'image_encoder.py'),'--calibration',str(folder),'--reliable',*suffix],
   [str(WS/'build/native-adapters/native_image_encoder'),'--calibration',str(folder),*suffix,'--ros-args',
    '-r',prefix+'/active/image:=/native/image','-r',prefix+'/active/extrinsic:=/native/extrinsic','-r',prefix+'/active/encoder:=/native/encoder']]
  children=[subprocess.Popen(cmd,stdout=subprocess.DEVNULL,start_new_session=True) for cmd in commands];processes+=children
  data={k:{} for k in ['python_image','native_image','python_pose','native_pose']};subs=[]
  for key,topic,kind in [('python_image',prefix+'/active/image',Image),('native_image','/native/image',Image),('python_pose',prefix+'/active/extrinsic',PoseStamped),('native_pose','/native/extrinsic',PoseStamped)]:
   subs.append(node.create_subscription(kind,topic,lambda m,key=key:data[key].__setitem__(m.header.stamp.sec*10**9+m.header.stamp.nanosec,m),30))
  topic=json.loads((folder/'camera_manifest.json').read_text())['cameras'][0]['topic']
  pub=node.create_publisher(Image,topic,30);joints=node.create_publisher(JointState,'/active/gimbal/states',30)
  spin_until(lambda:pub.get_subscription_count()>=2 and (fixed or joints.get_subscription_count()>=2))
  rng=np.random.default_rng(905);max_rotation=0.
  for i in range(30):
   t=10*10**9+i*50000000
   def joint(stamp,y,p):
    m=JointState();m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9);m.name=['active_camera_yaw_joint','active_camera_pitch_joint'];m.position=[float(y),float(p)];joints.publish(m)
   if not fixed:joint(t-5000000,.8*np.sin(i),1.5*np.cos(i))
   m=Image();m.header.stamp.sec,m.header.stamp.nanosec=divmod(t,10**9);m.header.frame_id='camera';m.height=height;m.width=width;m.step=width*3;m.encoding='rgb8' if i%2 else 'bgr8';m.data=array('B',rng.integers(0,256,height*width*3,dtype=np.uint8).tobytes());pub.publish(m)
   if not fixed:joint(t+5000000,.8*np.sin(i+.02),1.5*np.cos(i+.02))
   spin_until(lambda:all(t in d for d in data.values()))
   x=data['python_image'][t];y=data['native_image'][t]
   fields=['header','height','width','encoding','is_bigendian','step','data']
   assert all(getattr(x,k)==getattr(y,k) for k in fields),('image mismatch',i,{k:(str(getattr(x,k))[:100],str(getattr(y,k))[:100]) for k in fields if getattr(x,k)!=getattr(y,k)})
   a=data['python_pose'][t];b=data['native_pose'][t]
   assert a.header==b.header and a.pose.position==b.pose.position
   q=lambda m:np.array([getattr(m.pose.orientation,k) for k in ['x','y','z','w']])
   x=q(a);y=q(b);err=min(np.linalg.norm(x-y),np.linalg.norm(x+y));max_rotation=max(max_rotation,float(err));assert err<1e-12
  result['fixed' if fixed else 'active']=dict(exposures=30,image_fields_and_pixels_identical=True,max_quaternion_difference=max_rotation)
  for proc in children:os.killpg(proc.pid,signal.SIGINT)
  for proc in children:proc.wait(timeout=10);processes.remove(proc)
  for sub in subs:node.destroy_subscription(sub)
  node.destroy_publisher(pub);node.destroy_publisher(joints)
finally:
 for proc in processes:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 node.destroy_node();rclpy.shutdown()
out=args.output;out.write_text(json.dumps(result,indent=2)+'\n');print(out.read_text())
