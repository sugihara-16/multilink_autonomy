#!/usr/bin/env python3
"""Observe a running DRAGON launch; no motion commands. Save sensor validation."""
import argparse
import json
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from sensor_msgs_py import point_cloud2

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--duration', type=float, default=8)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('verify_dragon_slam')
stamps = {}; latest = {}; subscriptions = []

def receive(topic, msg):
    latest[topic] = msg
    if hasattr(msg, 'header'):
        stamps.setdefault(topic, []).append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)

for topic, cls in [('/dragon/imu/raw', Imu), ('/dragon/lidar/points', PointCloud2),
                   ('/dragon/ground_truth', Odometry), ('/slam/odometry', Odometry),
                   ('/dragon/flight_state', UInt8)]:
    subscriptions.append(node.create_subscription(cls, topic,
        lambda msg, topic=topic: receive(topic,msg), qos_profile_sensor_data))
deadline = time.monotonic()+args.duration
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.05)
report = {'topics': {}}
for topic, timestamps in stamps.items():
    dt = np.diff(timestamps)
    report['topics'][topic] = dict(messages=len(timestamps),
        nonmonotonic=int(np.count_nonzero(dt <= 0)), rate_sim_hz=float(1/np.mean(dt)),
        max_gap_sim_s=float(dt.max()), frame=latest[topic].header.frame_id)
cloud = latest.get('/dragon/lidar/points')
if cloud:
    xyz = point_cloud2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)
    report['lidar'] = dict(width=cloud.width,height=cloud.height,finite_points=len(xyz))
robot = ET.parse(args.output/'dragon_slam.urdf')
report['lidar_sensor_count'] = len(robot.findall("./gazebo/sensor[@type='gpu_lidar']"))
report['imu_sensor_count'] = len(robot.findall("./gazebo/sensor[@type='imu']"))
report['flight_state'] = getattr(latest.get('/dragon/flight_state'), 'data', None)
report['passed'] = (len(stamps)==4 and all(v['messages']>10 and v['nonmonotonic']==0 for v in report['topics'].values())
    and report['lidar_sensor_count']==1 and report['imu_sensor_count']==1 and report.get('lidar',{}).get('finite_points',0)>1000)
(args.output/'sensor_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
