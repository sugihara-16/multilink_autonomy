#!/usr/bin/env python3
"""Verify precise clock values, pause deduplication and reset propagation."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as RosClock
from gz.transport13 import Node as GzNode
from gz.msgs10.clock_pb2 import Clock
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
rclpy.init();node=Node('clock_bridge_test');gz=GzNode();pub=gz.advertise('/clock',Clock);results={}
for hz in [1000,200]:
 received=[];sub=node.create_subscription(RosClock,'/clock',lambda m:received.append(m.clock.sec*1000000000+m.clock.nanosec),1000)
 proc=subprocess.Popen([str(WS/'build/native-adapters/native_clock_bridge'),'--hz',str(hz)],start_new_session=True,stdout=subprocess.DEVNULL)
 try:
  end=time.monotonic()+10
  while not pub.has_connections() or node.count_publishers('/clock')!=1:
   if time.monotonic()>end:raise RuntimeError('Discovery timeout')
   rclpy.spin_once(node,timeout_sec=.01)
  time.sleep(.5)
  stamps=[0]+[x*1000000 for x in range(101) for _ in range(2)]+[100000000]*30+[99000000,99000000,100000000,0,1000000,5000000]
  expected=[];last=-1;bucket=-1;period=1000000000//hz
  for stamp in stamps:
   reset=last>=0 and stamp<last
   if stamp!=last and (reset or stamp//period!=bucket):expected.append(stamp);bucket=stamp//period
   last=stamp
   m=Clock();m.sim.sec=stamp//1000000000;m.sim.nsec=stamp%1000000000;pub.publish(m)
   until=time.monotonic()+.004
   while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
  end=time.monotonic()+3
  while len(received)<len(expected) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
  assert received==expected,(hz,len(received),len(expected),received,expected)
  results[hz]=dict(input_messages=len(stamps),output_messages=len(received),exact_timestamps=True,backward_resets_preserved=True)
 finally:
  os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=10);node.destroy_subscription(sub);time.sleep(.3)
node.destroy_node();rclpy.shutdown();(WS/'build/native-adapters/clock_validation.json').write_text(json.dumps(results,indent=2)+'\n');print(results)
