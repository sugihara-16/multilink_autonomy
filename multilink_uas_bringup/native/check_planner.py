#!/usr/bin/env python3
"""Verify ROS2->ROS1 sensor delivery, TF, and a nonempty Voxblox mesh."""
import collections
import json
import time

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from voxblox_msgs.msg import Mesh

rospy.init_node('uas_planner_observer')
counts = collections.Counter()
max_vertices = 0
max_blocks = 0
max_cloud_points = 0

def cloud(msg):
    global max_cloud_points
    counts['cloud'] += 1
    max_cloud_points = max(max_cloud_points, msg.width * msg.height)

def odom(msg):
    counts['odom'] += 1

def mesh(msg):
    global max_vertices, max_blocks
    counts['mesh'] += 1
    max_blocks = max(max_blocks, len(msg.mesh_blocks))
    max_vertices = max(max_vertices, sum(len(b.x) for b in msg.mesh_blocks))

subscriptions = [rospy.Subscriber('/rmf_unipilot/lidar/points_downsampled', PointCloud2, cloud, queue_size=10),
                 rospy.Subscriber('/rmf_unipilot/odom', Odometry, odom, queue_size=10),
                 rospy.Subscriber('/gbplanner_node/mesh', Mesh, mesh, queue_size=10)]
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
start = time.monotonic()
while time.monotonic() - start < 30 and not rospy.is_shutdown():
    time.sleep(0.1)
tf_ok = buffer.can_transform('world', 'rmf_unipilot/base_link/lidar', rospy.Time(0))
print(json.dumps({'messages': dict(counts), 'max_input_cloud_points': max_cloud_points,
                  'max_mesh_blocks_per_message': max_blocks,
                  'max_mesh_vertices_per_message': max_vertices,
                  'world_to_lidar_tf': tf_ok}, indent=2))
raise SystemExit(0 if counts['cloud'] > 0 and counts['odom'] > 0 and max_vertices > 0 and tf_ok else 1)
