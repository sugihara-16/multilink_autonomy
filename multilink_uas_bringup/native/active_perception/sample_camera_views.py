#!/usr/bin/env python3
"""Small diagnostic contact sheet inputs: selected unmodified camera frames."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--times',type=float,nargs='+',default=[35,75,120,180,230]);a=p.parse_args()
out=a.flight/'sample_views';out.mkdir(exist_ok=True);rclpy.init();node=rclpy.create_node('camera_diagnostic_sampler');done={k:set() for k in ['active','fixed']};enc=None;records=[]
def angles(m):
 global enc
 enc=list(m.data)
angle_sub=node.create_subscription(Float64MultiArray,'/active/encoder',angles,2000)
def frame(m,mode):
 try:r=json.loads((a.flight/'route_status.json').read_text())
 except (FileNotFoundError,json.JSONDecodeError):return
 t=r['route_time_s'];pending=[x for x in a.times if x<=t and x not in done[mode]]
 if not pending:return
 target=max(pending);done[mode].update(pending)
 if m.encoding!='mono8':raise ValueError(m.encoding)
 im=np.ndarray((m.height,m.width),dtype=np.uint8,buffer=m.data,strides=(m.step,1))
 name=f'{mode}_{target:g}.png';assert cv2.imwrite(str(out/name),im)
 records.append(dict(file=name,mode=mode,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,route_time_s=t,encoder=enc if mode=='active' else None));(out/'manifest.json').write_text(json.dumps(records,indent=2)+'\n')
subs=[node.create_subscription(Image,topic,lambda m,k=mode:frame(m,k),10) for mode,topic in [('active','/active/image'),('fixed','/fixed/active/image')]]
start=time.monotonic()
try:
 while rclpy.ok() and time.monotonic()-start<700 and not all(len(v)==len(a.times) for v in done.values()):
  rclpy.spin_once(node,timeout_sec=.1)
  if (a.flight/'runtime.json').exists():break
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
