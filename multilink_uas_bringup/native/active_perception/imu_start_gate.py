#!/usr/bin/env python3
"""Start VIO from a settled airborne view; preserve original IMU timestamps."""
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

rospy.init_node('vio_imu_start_gate')
enabled=False
pub=rospy.Publisher('/vio/initialized_imu',Imu,queue_size=200)
def enable(m):
    global enabled
    if m.data and not enabled:
        enabled=True
        rospy.loginfo('VIO initialization enabled after hover settling')
def imu(m):
    if enabled:pub.publish(m)
rospy.Subscriber('/active/initialize_vio',Bool,enable,queue_size=1)
rospy.Subscriber('/slam/input/imu',Imu,imu,queue_size=200,tcp_nodelay=True)
rospy.spin()
