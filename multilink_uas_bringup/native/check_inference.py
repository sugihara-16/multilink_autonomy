#!/usr/bin/env python3
"""Exercise the policy and CBF while their actuator output is disconnected."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('uas_inference_observer')
topics = ['/sdf_nmpc/cmd/acc', '/validation/cbf_safe_cmd']
counts = dict.fromkeys(topics, 0)
maximum = dict.fromkeys(topics, 0.0)
finite = True
origin = None
displacement = 0.0
sent = False

def on_command(topic, msg):
    global finite
    if not sent:
        return
    values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
    counts[topic] += 1
    finite = finite and all(math.isfinite(x) for x in values)
    maximum[topic] = max(maximum[topic], *(abs(x) for x in values))

def on_odom(msg):
    global origin, displacement
    p = msg.pose.pose.position
    xyz = (p.x, p.y, p.z)
    if origin is None:
        origin = xyz
    displacement = max(displacement, math.dist(origin, xyz))

subscriptions = [node.create_subscription(Twist, t, lambda msg, t=t: on_command(t, msg), 10) for t in topics]
subscriptions.append(node.create_subscription(Odometry, '/rmf_unipilot/odom', on_odom, 10))
publisher = node.create_publisher(PoseStamped, '/target', 10)
deadline = time.monotonic() + 10
while time.monotonic() < deadline and (origin is None or publisher.get_subscription_count() == 0):
    rclpy.spin_once(node, timeout_sec=0.1)
if origin is None or publisher.get_subscription_count() == 0:
    raise RuntimeError('Simulation odometry and policy subscriber are required')
# DDS can discover an endpoint before its ROS node name/namespace arrive.
# Wait only for unresolved metadata; never send a target while it is unknown.
deadline = time.monotonic() + 5
while True:
    actuator_publishers = node.get_publishers_info_by_topic('/rmf_unipilot/cmd/acc')
    unresolved = not actuator_publishers or any(
        p.node_name == '_NODE_NAME_UNKNOWN_' or
        p.node_namespace == '_NODE_NAMESPACE_UNKNOWN_' for p in actuator_publishers)
    if not unresolved or time.monotonic() >= deadline:
        break
    rclpy.spin_once(node, timeout_sec=0.1)
endpoints = [(p.node_name, p.node_namespace) for p in actuator_publishers]
if not endpoints or any(p != ('ros_gz_bridge_cmd_twist', '/rmf_unipilot') for p in endpoints):
    raise RuntimeError('Actuator publishers are unresolved or include a controller; '
                       'refusing this stationary test: ' + repr(endpoints))
target = PoseStamped()
target.header.frame_id = 'map'
target.pose.position.x, target.pose.position.y = origin[:2]
target.pose.position.z = origin[2] + 1.0
target.pose.orientation.w = 1.0
publisher.publish(target)
sent = True
started = time.monotonic()
while time.monotonic() - started < 15:
    rclpy.spin_once(node, timeout_sec=0.1)
elapsed = time.monotonic() - started
print(json.dumps({'messages': counts, 'observed_wall_hz': {t: counts[t] / elapsed for t in topics},
                  'maximum_absolute_component': maximum, 'all_finite': finite,
                  'max_position_displacement_m': displacement}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if finite and all(n > 10 for n in counts.values()) and displacement < 0.001 else 1)
