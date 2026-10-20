#!/usr/bin/env python3
"""Manual takeoff / land services; ground truth position hold for inspection."""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class Hover(Node):
    def __init__(self):
        super().__init__('unipilot_manual_hover')
        self.declare_parameter('takeoff_height',1.5)
        self.odom=None;self.received=0.;self.target=None;self.origin=None;self.yaw=0.;self.landing=False
        self.cmd=self.create_publisher(Twist,'/rmf_unipilot/cmd/vel',10)
        self.enable=self.create_publisher(Bool,'/rmf_unipilot/enable',10)
        self.create_subscription(Odometry,'/rmf_unipilot/odom',self.on_odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/unipilot/takeoff',self.takeoff)
        self.create_service(Trigger,'/unipilot/land',self.land)
        self.create_timer(.02,self.control)

    def on_odom(self,msg):
        self.odom=msg;self.received=time.monotonic()

    def pose(self):
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        return np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])

    def takeoff(self,request,response):
        if self.odom is None or time.monotonic()-self.received>1:
            response.success=False;response.message='Waiting for current odometry';return response
        if self.target is not None:
            response.success=False;response.message='Already flying; land before another takeoff';return response
        height=float(self.get_parameter('takeoff_height').value)
        if not math.isfinite(height) or height<=0:
            response.success=False;response.message='takeoff_height must be positive';return response
        self.origin,R=self.pose();self.target=self.origin+np.array([0.,0.,height])
        self.yaw=R.as_euler('xyz')[2];self.landing=False
        response.success=True;response.message=f'Takeoff {height} m above starting height, then hold position';return response

    def land(self,request,response):
        if self.target is None:
            response.success=False;response.message='Already disarmed';return response
        self.target=self.origin-np.array([0.,0.,.1]);self.landing=True
        response.success=True;response.message='Returning to takeoff position and landing';return response

    def control(self):
        message=Twist()
        if self.target is not None and time.monotonic()-self.received<1:
            p,R=self.pose()
            if self.landing and p[2]<=self.origin[2]+.04 and np.linalg.norm(p[:2]-self.origin[:2])<.15:
                self.target=None
            else:
                velocity=R.inv().apply(np.clip(1.2*(self.target-p),[-.4,-.4,-.25],[.4,.4,.35]))
                message.linear.x,message.linear.y,message.linear.z=map(float,velocity)
                error=self.yaw-R.as_euler('xyz')[2]
                message.angular.z=float(np.clip(math.atan2(math.sin(error),math.cos(error)),-.3,.3))
        self.cmd.publish(message)
        self.enable.publish(Bool(data=self.target is not None))


def main():
    rclpy.init();node=Hover()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
