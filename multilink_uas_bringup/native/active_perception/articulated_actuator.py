"""ROS2 joint slew limiter: planner targets never reach the servo as steps."""
import sys,argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES

class Actuator(Node):
    def __init__(self,out):
        super().__init__('articulated_joint_slew');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.kin=Kinematics(out/'dragon_slam.urdf');self.goal=None;self.q=None;self.command=None;self.enabled=False;self.last=None;self.rate=0.;self.qstamp=None;self.goalstamp=None;self.log=(out/'joint_commands.jsonl').open('w',buffering=1)
        self.pub=self.create_publisher(JointState,'/dragon/joints_ctrl',10)
        self.create_subscription(JointState,'/articulated/target',self.target,10)
        self.create_subscription(JointState,'/dragon/articulated/states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Bool,'/articulated/enable',lambda m:setattr(self,'enabled',m.data),10)
        self.create_timer(.01,self.tick)
    def joints(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in NAMES):self.q=np.array([d[n] for n in NAMES]);self.qstamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def target(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in NAMES):
            q=np.array([d[n] for n in NAMES])
            if self.kin.valid(q):self.goal=q;self.goalstamp=self.get_clock().now().nanoseconds*1e-9
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        dt=.01 if self.last is None else min(.02,max(0.,now-self.last));self.last=now
        if not self.enabled or self.q is None or self.goal is None:self.command=None;self.rate=0.;return
        if now-self.qstamp>.15 or now-self.goalstamp>2.:self.command=self.q.copy();self.rate=0.;return
        if self.command is None:self.command=self.q.copy()
        if np.max(abs(self.command-self.q))>np.deg2rad(3.):self.rate=0.;return
        delta=self.goal-self.command;distance=np.max(abs(delta))
        desired=min(self.kin.speed,np.sqrt(2*self.kin.acceleration*distance))
        self.rate=min(desired,self.rate+self.kin.acceleration*dt)
        q=self.command+delta*min(1.,self.rate*dt/max(distance,1e-12))
        if not self.kin.path_valid(self.q,q):self.rate=0.;return
        self.command=q
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=NAMES;msg.position=q.tolist();self.pub.publish(msg)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();rclpy.init();n=Actuator(a.output)
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
