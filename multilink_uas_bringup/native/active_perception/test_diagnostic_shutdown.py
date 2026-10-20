#!/usr/bin/env python3
"""Stop a busy diagnostic subscriber without tearing down its active callback."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parents[3]/'build/diagnostic-shutdown'
ROOT.mkdir(exist_ok=True)
out=Path(tempfile.mkdtemp(dir=ROOT))
rclpy.init();node=Node('diagnostic_shutdown_test')
pubs=[node.create_publisher(Imu,t,2000) for t in ['/diagnostic/imu_clean','/diagnostic/imu_fast']]
results=[]
try:
 for n,sig in enumerate([signal.SIGINT,signal.SIGTERM,signal.SIGINT]):
  d=out/str(n);d.mkdir()
  with (d/'recorder.log').open('w') as log:
   proc=subprocess.Popen(['python3',str(HERE/'record_diagnostic_sensors.py'),'--output',str(d)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   end=time.monotonic()+10
   while not all(p.get_subscription_count() for p in pubs):
    assert proc.poll() is None and time.monotonic()<end
    rclpy.spin_once(node,timeout_sec=.01)
   time.sleep(.2)
   for i in range(2000):
    m=Imu();m.header.stamp.sec,m.header.stamp.nanosec=divmod(i*1000000,1000000000)
    for p in pubs:p.publish(m)
    time.sleep(.0005)
   os.killpg(proc.pid,sig)
   assert proc.wait(timeout=10)==0,(d/'recorder.log').read_text()
   status=json.loads((d/'diagnostic_sensors/shutdown.json').read_text())
   assert status['stop_signal']==sig and status['files_closed']
   for name in ['imu','imu_fast']:
    count=len((d/'diagnostic_sensors'/(name+'.csv')).read_text().splitlines())-1
    assert count==status['counts'][name] and count>=1900,(name,count,status)
   results.append(status)
  finally:
   if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  time.sleep(.2)
finally:node.destroy_node();rclpy.shutdown()
(ROOT/'result.json').write_text(json.dumps(dict(passed=True,trials=results),indent=2)+'\n')
print('Busy diagnostic recorder: 3 clean signal shutdowns, recorded row counts verified')
