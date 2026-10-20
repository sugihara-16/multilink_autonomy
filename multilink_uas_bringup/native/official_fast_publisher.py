#!/usr/bin/env python3
"""External ROS publisher: never calls estimator internals or alters image pixels."""
import sys,os,struct,time,json,subprocess,signal,csv
from pathlib import Path
import yaml,rospy,rosnode
from sensor_msgs.msg import Imu,Image,PointCloud2
ROOT=Path('/root/multilink_autonomy_ws/fast_livo2_official_ws')
d=Path(sys.argv[1]);cfg=yaml.safe_load((d/'config.yaml').read_text())
rospy.init_node('official_input_replay',anonymous=True,disable_signals=True)
for k,v in cfg['parameters'].items():rospy.set_param('/'+k,v)
rospy.set_param('/laserMapping',cfg['camera']);rospy.set_param('/use_sim_time',False)
types=[Imu,PointCloud2,Image];topics=[cfg['parameters']['common'][k] for k in ['imu_topic','lid_topic','img_topic']]
pubs=[rospy.Publisher(t,c,queue_size=None) for t,c in zip(topics,types)]
log=(d/'estimator.log').open('w');node=subprocess.Popen([str(ROOT/'devel/lib/fast_livo/fastlivo_mapping')],stdout=log,stderr=subprocess.STDOUT)
pose=ROOT/'src/FAST-LIVO2/Log/result'/ (cfg['parameters']['evo']['seq_name']+'.txt')
counts=[0,0,0];first=[None]*3;last=[None]*3
try:
 deadline=time.monotonic()+30
 while not all(p.get_num_connections() for p in pubs):
  assert node.poll() is None,'Official node exited at startup'
  assert time.monotonic()<deadline,'Input connections timeout'
  time.sleep(.05)
 time.sleep(.5);prev=None;due=time.monotonic()
 while True:
  raw=sys.stdin.buffer.read(8)
  if not raw:break
  assert len(raw)==8
  idx,size=struct.unpack('<II',raw);b=sys.stdin.buffer.read(size);assert len(b)==size
  m=types[idx]();m.deserialize(b);t=m.header.stamp.to_nsec()
  if prev is not None and t>prev:
   due+=(t-prev)*1e-9
   now=time.monotonic()
   if due<now-.02:due=now
   if due>now:time.sleep(due-now)
  assert prev is None or t>=prev
  prev=t
  assert node.poll() is None,'Official node terminated'
  pubs[idx].publish(m);counts[idx]+=1
  if first[idx] is None:first[idx]=t
  last[idx]=t
  if idx==2 and counts[idx]%20==0:
   (d/'publisher_progress.json').write_text(json.dumps(dict(counts=counts,last=last)))
 # Leave ample time for the normal callback queues and final state updates.
 time.sleep(5)
finally:
 if node.poll() is None:
  (d/'node_status_before_shutdown.txt').write_text(Path(f'/proc/{node.pid}/status').read_text())
  try:rosnode.kill_nodes(['/laserMapping'])
  except Exception:node.send_signal(signal.SIGINT)
 try:rc=node.wait(timeout=30)
 except subprocess.TimeoutExpired:
  (d/'node_status_shutdown_timeout.txt').write_text(Path(f'/proc/{node.pid}/status').read_text())
  subprocess.run(['gdb','-batch','-ex','thread apply all bt','-p',str(node.pid)],stdout=(d/'shutdown_backtrace.txt').open('w'),stderr=subprocess.STDOUT,timeout=15)
  node.kill();rc=node.wait()
 log.close()
 (d/'publisher_audit.json').write_text(json.dumps(dict(counts=counts,first=first,last=last,node_returncode=rc),indent=2))
 if pose.exists():
  (d/'official_pose.txt').write_bytes(pose.read_bytes())
 for name in ['mat_pre.txt','mat_out.txt']:
  p=ROOT/'src/FAST-LIVO2/Log'/name
  if p.exists():(d/name).write_bytes(p.read_bytes())
if rc:sys.exit(rc if rc>0 else 128-rc)
