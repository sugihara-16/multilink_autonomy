"""ROS2 joint slew limiter: planner targets never reach the servo as steps."""
import sys,argparse,yaml,json
from pathlib import Path
import numpy as np
import rclpy
from synchronous_shutdown import initialize,spin
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,Float32MultiArray
from scipy.spatial.transform import Rotation
from articulated_motion import CoordinatedMotion,FlightInterlock
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES

class Actuator(Node):
    def __init__(self,out):
        super().__init__('articulated_joint_slew');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.kin=Kinematics(out/'dragon_slam.urdf');self.goal=None;self.q=None;self.command=None;self.enabled=False;self.last=None;self.rate=0.;self.qstamp=None;self.goalstamp=None;self.log=(out/'joint_commands.jsonl').open('w',buffering=1)
        cfg=yaml.safe_load((out/'planner.yaml').read_text())
        self.kin.speed=np.deg2rad(cfg.get('body_joint_speed_deg_s',10.))
        self.kin.acceleration=np.deg2rad(cfg.get('body_joint_acceleration_deg_s2',20.))
        safety=self.kin.control.config
        self.kin.speed=min(self.kin.speed,np.deg2rad(safety['max_body_speed_deg_s']))
        self.kin.acceleration=min(self.kin.acceleration,np.deg2rad(safety['max_body_acceleration_deg_s2']))
        self.interlock=FlightInterlock(safety);self.motion=None;self.last_log=-1.;self.last_reason=None
        (out/'control_envelope.json').write_text(json.dumps(self.kin.control.manifest(),indent=2))
        self.step_limit=.01 if 'body_joint_speed_deg_s' in cfg else .02
        self.pub=self.create_publisher(JointState,'/dragon/joints_ctrl',10)
        self.create_subscription(JointState,'/articulated/target',self.target,10)
        self.create_subscription(JointState,'/dragon/articulated/states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Bool,'/articulated/enable',lambda m:setattr(self,'enabled',m.data),10)
        self.create_subscription(Odometry,'/dragon/uav/cog/odom',self.flight_state,qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray,'/dragon/debug/target_vectoring_force',self.force,qos_profile_sensor_data)
        self.create_timer(.01,self.tick)
    def flight_state(self,m):
        p=m.pose.pose.orientation;w=m.twist.twist.angular
        quat=np.array([p.x,p.y,p.z,p.w])
        if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>.01:return
        rpy=Rotation.from_quat(quat).as_euler('xyz')
        self.interlock.odom=(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,float(np.rad2deg(abs(rpy[:2])).max()),float(np.rad2deg(np.linalg.norm([w.x,w.y,w.z]))))
    def force(self,m):
        if len(m.data)!=12:return
        self.interlock.force=(self.get_clock().now().nanoseconds*1e-9,float(np.linalg.norm(np.array(m.data).reshape(4,3),axis=1).max()/self.kin.control.thrust_limit))
    def joints(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in NAMES):
            q=np.array([d[n] for n in NAMES])
            if np.isfinite(q).all():self.q=q;self.qstamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def target(self,m):
        d=dict(zip(m.name,m.position))
        if all(n in d for n in NAMES):
            q=np.array([d[n] for n in NAMES])
            if self.kin.valid(q):self.goal=q;self.goalstamp=self.get_clock().now().nanoseconds*1e-9
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        dt=.01 if self.last is None else min(self.step_limit,max(0.,now-self.last));self.last=now
        if self.q is None or dt<=0:return
        if self.motion is None:
            if not self.enabled or self.goal is None:return
            self.motion=CoordinatedMotion(self.kin,self.q)
        reason=self.interlock.reason(now)
        if not self.enabled:reason='disabled'
        elif self.goal is None:reason='no_target'
        elif not -self.kin.control.config['feedback_future_tolerance_s']<=now-self.qstamp<=.15 or not 0<=now-self.goalstamp<=2.:reason='joint_or_target_stale'
        elif np.max(abs(self.motion.position-self.q))>np.deg2rad(1.):reason='tracking_error'
        elif not self.kin.valid(self.q):reason='measured_envelope'
        q,motion_reason=self.motion.step(self.goal if self.goal is not None else self.motion.position,dt,reason=='ok')
        self.command=q
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=NAMES;msg.position=q.tolist();self.pub.publish(msg)
        if now-self.last_log>=.5 or reason!=self.last_reason:
            self.log.write(json.dumps(dict(time=now,reason=reason,motion=motion_reason,command=q.tolist(),measured=self.q.tolist(),joint_age_s=now-self.qstamp,target_age_s=None if self.goalstamp is None else now-self.goalstamp,rate_deg_s=float(np.rad2deg(self.motion.rate)),flight=self.interlock.odom,force=self.interlock.force,envelope=self.kin.control.evaluate(self.q)))+'\n')
            self.last_log=now;self.last_reason=reason

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();initialize();n=Actuator(a.output)
    try:spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
