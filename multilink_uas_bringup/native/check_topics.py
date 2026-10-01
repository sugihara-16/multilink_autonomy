#!/usr/bin/env python3
"""Observe simulation topics without sending any control commands."""
import collections
import argparse
import json
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu, Image
from sensor_msgs_py import point_cloud2

parser = argparse.ArgumentParser()
parser.add_argument('--best-effort', action='store_true')
parser.add_argument('--duration', type=float, default=30)
args = parser.parse_args()
rclpy.init(args=[])
node = rclpy.create_node('uas_validation_observer')
counts = collections.Counter()
first = {}
last = {}
details = {}
max_gap = collections.defaultdict(float)
initial_position = None
max_displacement = 0.0
subscriptions = []

def callback(topic, msg):
    global initial_position, max_displacement
    counts[topic] += 1
    stamp = msg.clock if isinstance(msg, Clock) else msg.header.stamp
    t = stamp.sec + stamp.nanosec * 1e-9
    first.setdefault(topic, t)
    if topic in last:
        max_gap[topic] = max(max_gap[topic], t - last[topic])
    last[topic] = t
    if isinstance(msg, Odometry):
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)
        if initial_position is None:
            initial_position = pos
        max_displacement = max(max_displacement, math.dist(pos, initial_position))
    if topic in details:
        return
    if isinstance(msg, PointCloud2):
        valid = sum(all(math.isfinite(float(v)) for v in p)
                    for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False))
        details[topic] = {'points': msg.width * msg.height, 'finite_xyz': valid,
                          'frame': msg.header.frame_id,
                          'fields': [f.name for f in msg.fields]}
    elif isinstance(msg, Odometry):
        p = msg.pose.pose.position
        details[topic] = {'position': [p.x, p.y, p.z], 'frame': msg.header.frame_id}
    elif isinstance(msg, Image):
        details[topic] = {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}

types = {'/clock': Clock, '/rmf_unipilot/odom': Odometry,
         '/rmf_unipilot/lidar/points': PointCloud2,
         '/rmf_unipilot/lidar/points_downsampled': PointCloud2,
         '/cbf_pc_selector/output_pc': PointCloud2,
         '/rmf_unipilot/imu': Imu,
         '/rmf_unipilot/cam/rgb': Image, '/rmf_unipilot/cam/depth': Image}
for topic, msg_type in types.items():
    subscriptions.append(node.create_subscription(msg_type, topic,
                         lambda msg, topic=topic: callback(topic, msg),
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) if msg_type is PointCloud2 and not args.best_effort else qos_profile_sensor_data))
started = time.monotonic()
while time.monotonic() - started < args.duration:
    rclpy.spin_once(node, timeout_sec=0.1)
result = {}
for topic in types:
    span = last.get(topic, 0) - first.get(topic, 0)
    result[topic] = {'messages': counts[topic], 'sim_span_s': span,
                     'max_sim_gap_s': max_gap[topic],
                     'observed_hz_sim': (counts[topic] - 1) / span if span > 0 else None,
                     **details.get(topic, {})}
result['approx_rtf'] = (last.get('/clock', 0) - first.get('/clock', 0)) / (time.monotonic() - started)
result['max_displacement_m'] = max_displacement
print(json.dumps(result, indent=2))
node.destroy_node()
rclpy.shutdown()
required = ['/clock', '/rmf_unipilot/odom', '/rmf_unipilot/lidar/points', '/rmf_unipilot/lidar/points_downsampled']
raise SystemExit(0 if all(counts[t] > 1 for t in required) and details.get(required[2], {}).get('finite_xyz', 0) > 0 else 1)
