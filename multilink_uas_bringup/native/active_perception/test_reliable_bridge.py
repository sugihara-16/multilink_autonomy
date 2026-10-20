#!/usr/bin/env python3
"""Bidirectional bridge test: real QoS discovery, payload equality, no loss/loops."""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import threading

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]
OUT = WS/'build/reliable-bridge/test'
COUNT = 20000

def signature(m):
    return [m.header.frame_id,
            *[getattr(m.orientation, a) for a in 'xyzw'],
            *m.orientation_covariance,
            *[getattr(m.angular_velocity, a) for a in 'xyz'],
            *m.angular_velocity_covariance,
            *[getattr(m.linear_acceleration, a) for a in 'xyz'],
            *m.linear_acceleration_covariance]

if '--ros1' in sys.argv:
    import rospy
    from sensor_msgs.msg import Imu
    rospy.init_node('bridge_ros1_test')
    seen = []
    pub = rospy.Publisher('/test_return', Imu, queue_size=2000)
    def receive(m):
        i = m.header.stamp.to_nsec()//1000000
        assert signature(m) == ['sample', .1, .2, .3, .4, *map(float, range(9)),
                                float(i), -.3, .2, *([.012]*9),
                                .1, float(-i), 9.8, *([.034]*9)]
        seen.append(i)
        pub.publish(m)
        if len(seen) == COUNT:
            assert seen == list(range(COUNT))
            (OUT/'ros1.json').write_text(json.dumps({'count': len(seen), 'payload_exact': True}))
    sub = rospy.Subscriber('/test_input', Imu, receive, queue_size=2000)
    rospy.spin()
    sys.exit()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
OUT.mkdir(parents=True, exist_ok=True)
children = []
def start(name, command, ros1=False):
    env = {k: os.environ[k] for k in ('HOME', 'PATH', 'LANG') if k in os.environ}
    script = f'source {HERE.parent / ("env_ros1.sh" if ros1 else "env.sh")}\n'
    script += 'export ROS_MASTER_URI=http://127.0.0.1:11511 ROS_DOMAIN_ID=86\nexec '+shlex.join(command)
    with (OUT/(name+'.log')).open('w') as log:
        p = subprocess.Popen(['bash', '--noprofile', '--norc', '-ec', script],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(p)
    return p

rclpy.init()
node = Node('bridge_ros2_test')
spin_thread = None
try:
    start('master', ['roscore', '-p', '11511'], True)
    time.sleep(2)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    import xmlrpc.client
    master = xmlrpc.client.ServerProxy('http://127.0.0.1:11511')
    master.setParam('/test', '/topics', [dict(topic=t, type='sensor_msgs/msg/Imu', queue_size=2000)
                                        for t in ('/test_input', '/test_return')])
    start('bridge', [str(WS/'build/reliable-bridge/reliable_bridge')])
    start('ros1', ['python3', __file__, '--ros1'], True)
    received = []
    def receive2(m):
        i = (m.header.stamp.sec*1000000000+m.header.stamp.nanosec)//1000000
        assert signature(m) == ['sample', .1, .2, .3, .4, *map(float, range(9)),
                                float(i), -.3, .2, *([.012]*9),
                                .1, float(-i), 9.8, *([.034]*9)]
        received.append(i)
    sub = node.create_subscription(Imu, '/test_return', receive2, QoSProfile(depth=2000))
    source_seen=[]
    source_sub=node.create_subscription(Imu,'/test_input',lambda m:source_seen.append(
        (m.header.stamp.sec*1000000000+m.header.stamp.nanosec)//1000000),QoSProfile(depth=2000))
    pub = node.create_publisher(Imu, '/test_input', QoSProfile(depth=2000))
    deadline = time.monotonic()+20
    while True:
        info = [x for x in node.get_subscriptions_info_by_topic('/test_input')
                if x.node_name == 'uas_reliable_bridge']
        if info and pub.get_subscription_count() and node.count_publishers('/test_return'):
            assert all(x.qos_profile.reliability == ReliabilityPolicy.RELIABLE for x in info)
            break
        assert time.monotonic() < deadline, 'Bridge discovery timeout'
        rclpy.spin_once(node, timeout_sec=.01)
    time.sleep(2)
    for i in range(COUNT):
        m = Imu(); m.header.frame_id = 'sample'
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(i*1000000, 1000000000)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = .1, .2, .3, .4
        m.orientation_covariance = list(map(float, range(9)))
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = float(i), -.3, .2
        m.angular_velocity_covariance = [.012]*9
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = .1, float(-i), 9.8
        m.linear_acceleration_covariance = [.034]*9
        pub.publish(m)
        time.sleep(.0005)
    deadline = time.monotonic()+20
    while len(received) < COUNT and time.monotonic() < deadline:
        time.sleep(.01)
    deadline = time.monotonic()+1
    while time.monotonic() < deadline:
        time.sleep(.01)
    assert received == list(range(COUNT)), (len(received), len(set(received)))
    assert source_seen == list(range(COUNT)), ('Unwanted source echo',len(source_seen),len(set(source_seen)))
    assert json.loads((OUT/'ros1.json').read_text())['payload_exact']
    result = dict(samples_each_direction=COUNT, missing=0, duplicated=0, payload_exact=True,
                  subscriber_qos_verified_reliable=True, timestamps_exact=True,source_echoes_zero=True)
    (OUT/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(result)
finally:
    for proc in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
    rclpy.shutdown()
    if spin_thread: spin_thread.join(timeout=2)
    node.destroy_node()
