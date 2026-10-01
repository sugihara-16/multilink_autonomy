#!/usr/bin/env python3
"""ROS1 sensor adapter: instantaneous Gazebo cloud, IMU without orientation GT."""
import numpy as np
import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField

rospy.init_node('slam_inputs')
imu_pub = rospy.Publisher('/slam/input/imu', Imu, queue_size=200)
cloud_pub = rospy.Publisher('/slam/input/points', PointCloud2, queue_size=5)
first_imu_stamp = None


def imu_cb(msg):
    global first_imu_stamp
    stamp = msg.header.stamp.to_sec()
    if first_imu_stamp is None:
        first_imu_stamp = stamp
    # Exclude spawning/contact transients from stationary bias initialization.
    if stamp - first_imu_stamp < 2.0:
        return
    msg.header.frame_id = 'slam_imu'
    msg.orientation.x = msg.orientation.y = msg.orientation.z = 0.0
    msg.orientation.w = 1.0
    msg.orientation_covariance = [-1.0] + [0.0] * 8
    imu_pub.publish(msg)


def cloud_cb(msg):
    if msg.is_bigendian:
        rospy.logerr_throttle(5, 'Big-endian input is unsupported')
        return
    # Gazebo GPU scans are instantaneous: t=0 for every ray, no invented sweep time.
    source = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring'],
        'formats': ['<f4', '<f4', '<f4', '<f4', '<u2'],
        'offsets': [next(f.offset for f in msg.fields if f.name == n)
                    for n in ['x', 'y', 'z', 'intensity', 'ring']],
        'itemsize': msg.point_step})
    points = np.ndarray((msg.height, msg.width), dtype=source, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step)).reshape(-1)
    target = np.zeros(len(points), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
        ('t','<u4'),('intensity','<f4'),('tag','u1'),('line','u1')])
    for field in ['x','y','z','intensity']:
        target[field] = points[field]
    target['line'] = points['ring']
    out = PointCloud2(header=msg.header, height=1, width=len(target),
        fields=[PointField(name=n, offset=target.dtype.fields[n][1], datatype=t, count=1)
                for n,t in [('x',7),('y',7),('z',7),('t',6),('intensity',7),('tag',2),('line',2)]],
        is_bigendian=False, point_step=22, row_step=22*len(target),
        data=target.tobytes(), is_dense=False)
    out.header.frame_id = 'slam_lidar'
    cloud_pub.publish(out)


rospy.Subscriber(rospy.get_param('~imu_topic', '/rmf_unipilot/imu'), Imu, imu_cb, queue_size=200, tcp_nodelay=True)
rospy.Subscriber(rospy.get_param('~points_topic', '/rmf_unipilot/lidar/points'), PointCloud2, cloud_cb, queue_size=5,
                 buff_size=8*1024*1024, tcp_nodelay=True)
rospy.spin()
