#!/usr/bin/env python3
"""Bounded ROS1 topic-path smoke test; always terminate owned launch/play processes."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rospy,rosbag
from nav_msgs.msg import Odometry
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_multicamera';bag=OUT/'inputs/run2a_end.bag';config=OUT/'link1_link3_link4/run2a_end/trial1/ros_parameters.yaml'
def main():
 rospy.init_node('multicamera_ros_smoke',anonymous=True,disable_signals=True);seen=[]
 sub=rospy.Subscriber('/multicamera/odometry',Odometry,lambda m:seen.append(m),queue_size=1000)
 launch=None;player=None
 try:
  with (OUT/'ros_topic_test.log').open('w') as log:
   launch=subprocess.Popen(['roslaunch',str(ROOT/'comparison/fastlivo_multicamera.launch'),'config:='+str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   time.sleep(2);assert launch.poll() is None
   player=subprocess.Popen(['rosbag','play','--clock','--duration','10',str(bag)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   player.wait(timeout=40);assert player.returncode==0;time.sleep(1);assert launch.poll() is None
  assert len(seen)>20,len(seen)
  stamps=[m.header.stamp.to_nsec() for m in seen];assert all(b>a for a,b in zip(stamps,stamps[1:]))
  with rosbag.Bag(str(bag)) as b:source={m.header.stamp.to_nsec() for _,m,_ in b.read_messages(topics=['/comparison/image/link1'])}
  assert set(stamps)<=source
  for m in seen:
   q=m.pose.pose.orientation;p=m.pose.pose.position
   assert all(math.isfinite(x) for x in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]) and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-8
  (OUT/'ros_topic_test.json').write_text(json.dumps(dict(messages=len(seen),first_stamp_ns=stamps[0],last_stamp_ns=stamps[-1],all_stamps_from_recorded_images=True,finite_pose_normalized_quaternion=True),indent=2));print('ROS topic path PASS',len(seen))
 finally:
  for process in [player,launch]:
   if process and process.poll() is None:
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
  rospy.signal_shutdown('test complete')
if __name__=='__main__':main()
