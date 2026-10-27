"""Drive only the camera gimbal; leave all six body joints at the square."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_gimbal import GIMBAL_NAMES,LIMITS


class Actuator(Node):
    def __init__(self,out):
        super().__init__('dragon_camera_gimbal');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.goal=np.zeros(2);self.command=None;self.last=None;self.received=None
        self.command_pubs=[self.create_publisher(Float64,'/active/gimbal/'+a,10) for a in ('yaw','pitch')]
        self.create_subscription(JointState,'/articulated/target',self.target,10)
        self.create_subscription(JointState,'/dragon/articulated/states',self.state,qos_profile_sensor_data)
        self.log=(out/'gimbal.csv').open('w',buffering=1);self.log.write('t,yaw,pitch,target_yaw,target_pitch\n');self.logged=-1
        self.create_timer(.01,self.tick)
    def target(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in GIMBAL_NAMES):
            g=np.array([d[n] for n in GIMBAL_NAMES])
            if np.isfinite(g).all():self.goal=np.clip(g,-LIMITS,LIMITS);self.received=self.get_clock().now().nanoseconds*1e-9
    def state(self,m):
        d=dict(zip(m.name,m.position))
        if not all(n in d for n in GIMBAL_NAMES):return
        self.q=np.array([d[n] for n in GIMBAL_NAMES]);self.state_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.command is None:self.command=self.q.copy()
        if self.state_time-self.logged>=.049:
            self.log.write(','.join(map(str,[self.state_time,*self.q,*self.goal]))+'\n');self.logged=self.state_time
    def tick(self):
        t=self.get_clock().now().nanoseconds*1e-9
        if self.command is None:return
        dt=0 if self.last is None else np.clip(t-self.last,0,.02);self.last=t
        if t-self.state_time>.15:return
        # A stale planner must not cause a blind long sweep. Hold at measured.
        goal=self.goal if self.received is not None and t-self.received<2 else self.q
        delta=goal-self.command;norm=np.linalg.norm(delta)
        self.command+=delta*min(1.,np.deg2rad(14.)*dt/max(norm,1e-12))
        for p,v in zip(self.command_pubs,self.command):p.publish(Float64(data=float(v)))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    rclpy.init();n=Actuator(a.output)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
