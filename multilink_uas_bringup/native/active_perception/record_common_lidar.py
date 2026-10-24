#!/usr/bin/env python3
"""Read-only pure-LIO geometry recorder, never publishes estimator inputs."""
import argparse,gzip,json,signal,threading,time
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 rospy.init_node('common_lidar_geometry_recorder',disable_signals=True)
 stop=threading.Event();lock=threading.Lock();count=0
 def finish(*_):stop.set()
 signal.signal(signal.SIGINT,finish);signal.signal(signal.SIGTERM,finish)
 with gzip.open(a.output,'xt',compresslevel=2) as f:
  def receive(m):
   nonlocal count
   x=np.asarray(m.data)
   if x.shape!=(96,) or x[0]!=1 or not np.isfinite(x).all():raise ValueError('Unexpected LiDAR diagnostic')
   with lock:
    f.write(json.dumps(x.tolist(),separators=(',',':'))+'\n');count+=1
  sub=rospy.Subscriber('/lio_ablation/pure_lio/active/lidar_information',Float64MultiArray,receive,queue_size=500)
  while not stop.wait(.2):pass
  sub.unregister()
  with lock:f.flush()
 rospy.signal_shutdown('owner stopped')
 a.output.with_suffix('.status.json').write_text(json.dumps(dict(samples=count,stopped=True,topic='/lio_ablation/pure_lio/active/lidar_information'))+'\n')
if __name__=='__main__':main()
