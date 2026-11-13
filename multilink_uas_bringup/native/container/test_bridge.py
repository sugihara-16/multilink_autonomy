"""Finite live transport check for source-built ROS1/ROS2 conversion and loops."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path('/root/setup-logs/bridge-smoke')
ROOT.mkdir(exist_ok=True)
children = []
prefix = 'export ROS_MASTER_URI=http://127.0.0.1:11991 ROS_DOMAIN_ID=191 ROS_IP=127.0.0.1\n'


def start(name, command, ros1=False):
    source = '/root/uas_native/env_ros1.sh' if ros1 else '/root/uas_native/env.sh'
    script = f'source {source}\n' + prefix + 'exec ' + command
    log = (ROOT / (name + '.log')).open('w')
    process = subprocess.Popen(['bash', '--noprofile', '--norc', '-ec', script],
                               env={k: v for k, v in os.environ.items() if k in ('HOME', 'PATH', 'LANG')},
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    children.append(process)
    return process


(ROOT / 'receiver.py').write_text('''import json,time,rospy
from sensor_msgs.msg import Imu
rospy.init_node('bridge_smoke_receiver')
rospy.set_param('topics',[dict(topic='/bridge_smoke/imu',type='sensor_msgs/msg/Imu',queue_size=200)])
received=[]
def callback(m):
 received.append(dict(stamp=m.header.stamp.to_nsec(),frame=m.header.frame_id,x=m.linear_acceleration.x,cov=list(m.orientation_covariance)))
s=rospy.Subscriber('/bridge_smoke/imu',Imu,callback,queue_size=200)
deadline=time.monotonic()+30
while time.monotonic()<deadline and len(received)<100:time.sleep(.01)
time.sleep(1)
open('/root/setup-logs/bridge-smoke/received.json','w').write(json.dumps(received))
assert len(received)==100,len(received)
for i,m in enumerate(received):
 assert m==dict(stamp=123000000000+i,frame='test_imu',x=i+.25,cov=[float(j)+.5 for j in range(9)]),m
''')
(ROOT / 'publisher.py').write_text('''import time,rclpy,json
from sensor_msgs.msg import Imu
rclpy.init();n=rclpy.create_node('bridge_smoke_publisher')
p=n.create_publisher(Imu,'/bridge_smoke/imu',200);seen=[]
s=n.create_subscription(Imu,'/bridge_smoke/imu',lambda m:seen.append(m.header.stamp.nanosec),200)
deadline=time.monotonic()+20
while p.get_subscription_count()<2:
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()>deadline:raise RuntimeError('Bridge subscription missing')
for i in range(100):
 m=Imu();m.header.stamp.sec=123;m.header.stamp.nanosec=i;m.header.frame_id='test_imu';m.linear_acceleration.x=i+.25;m.orientation_covariance=[float(j)+.5 for j in range(9)];p.publish(m)
 rclpy.spin_once(n,timeout_sec=.02);time.sleep(.01)
deadline=time.monotonic()+2
while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
open('/root/setup-logs/bridge-smoke/ros2_observed.json','w').write(json.dumps(seen))
assert seen==list(range(100)),('echo or loss',len(seen))
n.destroy_node();rclpy.shutdown()
''')
try:
    start('master', 'roscore -p 11991', True)
    time.sleep(2)
    receiver = start('receiver', 'python3 ' + str(ROOT / 'receiver.py'), True)
    time.sleep(2)
    bridge = start('bridge', '/root/multilink_autonomy_ws/build/reliable-bridge/reliable_bridge')
    publisher = start('publisher', 'python3 ' + str(ROOT / 'publisher.py'))
    assert publisher.wait(timeout=30) == 0, 'Publisher failed'
    assert receiver.wait(timeout=10) == 0, 'Receiver failed'
    assert bridge.poll() is None, 'Bridge stopped'
    (ROOT / 'result.json').write_text(json.dumps(dict(passed=True, messages=100,
        checks=['timestamps', 'frame', 'payload', 'fixed-size covariance', 'no ROS2 echo', 'no loss']), indent=2))
    print('100 IMU messages: exact payload, timestamps and covariance; no loss or echo')
finally:
    for process in reversed(children):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in children:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
