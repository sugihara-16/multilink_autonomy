#!/usr/bin/env python3
"""Check streamed ROS1 serialization against established bag messages, then replay."""
import io,json,hashlib,subprocess,struct,sys
from pathlib import Path
import numpy as np,rosbag
import fast_stream_transport as w
R=Path(__file__).resolve().parent;d=R/'logs/zab-offline/fast_multicamera_all/transport_check';d.mkdir(exist_ok=True)
old=R/'logs/zab-offline/fast_multicamera/link1_link3_link4/run2a_end/trial1'
p=subprocess.Popen(['python3',str(R/'run_fast_stream_child.py'),str(old/'config.yaml'),str(d/'odometry.csv')],stdin=subprocess.PIPE,stdout=(d/'run.log').open('w'),stderr=subprocess.STDOUT)
c=[0]*5
try:
 with rosbag.Bag(str(R/'logs/zab-offline/fast_multicamera/inputs/run2a_end.bag')) as bag:
  for topic,m,t in bag.read_messages():
   idx=0 if topic.endswith('/imu') else 1 if topic.endswith('/points') else 2+w.CAMERAS.index(topic.split('/')[-1]);ns=m.header.stamp.to_nsec()
   buf=io.BytesIO();m.serialize(buf);original=buf.getvalue()
   if idx==0:b=w.imu(ns,[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
   elif idx==1:
    a=np.frombuffer(m.data,w.DTYPE);v=np.stack([a[k] for k in ['x','y','z','intensity','ring']],axis=-1);b=w.cloud(ns,v)
   else:b=w.image(ns,idx-2,np.frombuffer(m.data,np.uint8).reshape(360,480,3)[:,:,0])
   assert original==b,(topic,ns,len(original),len(b));c[idx]+=1;p.stdin.write(w.packet(idx,b))
 p.stdin.close();assert p.wait(timeout=300)==0
 (d/'verification.json').write_text(json.dumps(dict(serialized_byte_identical=True,counts=c)))
 print('PASS: original bag and streamed serialized payloads byte-identical',c)
finally:
 if p.poll() is None:p.terminate();p.wait()
