"""Finite takeoff / measured articulation / active view / landing test."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool,Empty,UInt8
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
import initialization_motion

class Test(Node):
    def __init__(self,out,distance=0.,speed=.5,gimbal=False,inspect_gimbal=False):
        super().__init__('articulated_hover_test');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.out=out;self.state=None;self.cog=None;self.joints={};self.stage='waiting';self.start_wall=time.monotonic();self.start=None;self.done=False;self.failed=None
        self.samples=[];self.events=[];self.names=[f'joint{i}_{a}' for i in (1,2,3) for a in ('pitch','yaw')]
        self.distance=distance;self.speed=speed;self.origin=None;self.fc=None
        self.gimbal=gimbal;self.inspect_gimbal=inspect_gimbal;self.inspection_index=0;self.inspection_since=None;self.last_image=None
        if gimbal:
            from sensor_msgs.msg import Image
            self.names+=['active_camera_yaw_joint','active_camera_pitch_joint']
            self.gimbal_target=self.create_publisher(JointState,'/articulated/target',10)
            if inspect_gimbal:self.create_subscription(Image,'/dragon/end_camera/image',lambda m:setattr(self,'last_image',m),qos_profile_sensor_data)
        self.nav=self.create_publisher(FlightNav,'/dragon/uav/nav',10)
        self.cmd={n:self.create_publisher(Empty,'/dragon/teleop_command/'+n,10) for n in ('start','takeoff','land')}
        self.gate=self.create_publisher(Bool,'/active/initialize_vio',10);self.active=self.create_publisher(Bool,'/articulated/enable',10)
        self.jp=self.create_publisher(JointState,'/dragon/joints_ctrl',10)
        self.create_subscription(UInt8,'/dragon/flight_state',lambda m:setattr(self,'state',m.data),10)
        self.create_subscription(Odometry,'/dragon/uav/cog/odom',lambda m:setattr(self,'cog',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/dragon/ground_truth',lambda m:setattr(self,'fc',m),qos_profile_sensor_data)
        self.create_subscription(JointState,'/dragon/articulated/states',lambda m:self.joints.update(zip(m.name,m.position)),qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    def transition(self,stage,command=None):
        self.stage=stage;self.start=self.get_clock().now().nanoseconds*1e-9
        self.events.append(dict(stage=stage,time=self.start))
        (self.out/'flight_events.json').write_text(json.dumps(self.events,indent=2))
        self.get_logger().info(stage)
        if command:self.cmd[command].publish(Empty())
        if stage in ('active','preparation'):
            self.origin=np.array([5.,0.,2.])

    def position_command(self,delta,velocity):
        m=FlightNav();m.header.stamp=self.get_clock().now().to_msg();m.control_frame=m.WORLD_FRAME;m.target=m.COG
        m.pos_xy_nav_mode=m.POS_VEL_MODE;m.pos_z_nav_mode=m.POS_VEL_MODE;m.yaw_nav_mode=m.POS_VEL_MODE
        offset=np.zeros(3)
        if self.fc is not None:
            a=self.cog.pose.pose.position;b=self.fc.pose.pose.position;offset=np.array([a.x-b.x,a.y-b.y,a.z-b.z])
        m.target_pos_x,m.target_pos_y,m.target_pos_z=map(float,self.origin+delta+offset)
        m.target_vel_x,m.target_vel_y,m.target_vel_z=map(float,velocity);m.target_yaw=0.
        self.nav.publish(m)

    def tick(self):
        if self.done:return
        t=self.get_clock().now().nanoseconds*1e-9
        elapsed=t-self.start if self.start is not None else 0
        if self.cog:
            p=self.cog.pose.pose.position;self.samples.append([t,p.x,p.y,p.z,*[self.joints.get(n,0) for n in self.names]])
        if self.stage=='waiting' and self.state==0 and self.cog and self.fc and all(n in self.joints for n in self.names):self.transition('arming','start')
        elif self.stage=='arming' and self.state==2:self.transition('takeoff','takeoff')
        elif self.stage=='takeoff' and self.state==5:self.transition('settle')
        elif self.stage=='settle' and elapsed>5:
            self.gate.publish(Bool(data=True));self.transition('initialize')
        elif self.stage=='initialize':
            self.gate.publish(Bool(data=True))
            if elapsed>12:self.transition('preparation')
        elif self.stage=='preparation':
            # Exact current UniPilot metric-scale preparation, two cycles.
            delta,velocity=initialization_motion.reference(elapsed%initialization_motion.DURATION)
            self.position_command(delta,velocity)
            if elapsed>=2*initialization_motion.DURATION:
                self.position_command(np.zeros(3),np.zeros(3));self.transition('articulation')
        elif self.stage=='articulation':
            self.position_command(np.zeros(3),np.zeros(3))
            if self.gimbal:
                if self.inspect_gimbal:
                    views=[(0,0),(-50,0),(-50,-90),(0,-90),(50,-90),(50,0),(50,90),(0,90),(-50,90),(0,0)]
                    goal=np.deg2rad(views[self.inspection_index])
                    q=np.array([self.joints[n] for n in self.names[-2:]])
                    if np.max(abs(q-goal))<np.deg2rad(.5):
                        if self.inspection_since is None:self.inspection_since=t
                        if t-self.inspection_since>1 and self.last_image is not None:
                            import cv2
                            m=self.last_image
                            stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                            if t-stamp<.15:
                                raw=np.ndarray((m.height,m.width,3),np.uint8,buffer=m.data,strides=(m.step,3,1))
                                image=cv2.cvtColor(raw,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else raw
                                folder=self.out/'inspection';folder.mkdir(exist_ok=True)
                                cv2.imwrite(str(folder/f'{self.inspection_index:02d}_yaw{views[self.inspection_index][0]}_pitch{views[self.inspection_index][1]}.png'),image)
                                (folder/f'{self.inspection_index:02d}.json').write_text(json.dumps(dict(time=t,image_stamp=stamp,command_deg=views[self.inspection_index],measured_deg=np.rad2deg(q).tolist(),body_joints_deg=[float(np.rad2deg(self.joints[n])) for n in self.names[:6]])))
                                self.inspection_index+=1;self.inspection_since=None
                                if self.inspection_index==len(views):self.transition('active')
                    else:self.inspection_since=None
                else:
                    # Same fixed body throughout: only the camera surveys yaw.
                    goal=np.array([np.deg2rad(50)*np.sin(2*np.pi*min(elapsed/40,1)),0.])
                    if elapsed>=40:self.transition('active')
                msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names[-2:];msg.position=goal.tolist();self.gimbal_target.publish(msg)
            else:
                # Same initial survey for both articulated comparison arms.
                q=np.array([0.,1.56]*3);blend=.5*(1-np.cos(2*np.pi*min(elapsed/40,1)))
                q[1::2]-=(1.56-np.pi/6)*blend
                msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=self.names;msg.position=q.tolist();self.jp.publish(msg)
                if elapsed>=40:self.transition('active')
        elif self.stage=='active':
            self.active.publish(Bool(data=True))
            if self.distance:
                # 0.3 m/s, cosine velocity ramps of two seconds at both ends.
                duration=self.distance/self.speed+2
                if elapsed<2:d=self.speed*.5*(elapsed-2/np.pi*np.sin(np.pi*elapsed/2));v=self.speed*.5*(1-np.cos(np.pi*elapsed/2))
                elif elapsed<duration-2:d=self.speed*(elapsed-1);v=self.speed
                elif elapsed<duration:
                    u=duration-elapsed;d=self.distance-self.speed*.5*(u-2/np.pi*np.sin(np.pi*u/2));v=self.speed*.5*(1-np.cos(np.pi*u/2))
                else:d=self.distance;v=0.
                self.position_command(np.array([d,0,0]),np.array([v,0,0]))
            if elapsed>=max(30,self.distance/self.speed+7):self.active.publish(Bool(data=False));self.transition('landing','land')
        elif self.stage=='landing' and self.state==0:self.done=True;self.save()
        if self.stage in ('settle','initialize','preparation','articulation','active') and self.state!=5:
            self.failed=f'Lost hover in {self.stage}: {self.state}';self.done=True;self.cmd['land'].publish(Empty());self.save()
        if time.monotonic()-self.start_wall>1200:
            self.failed='Wall timeout';self.done=True;self.cmd['land'].publish(Empty());self.save()
        status={'stage':self.stage,'sim_time':t,'flight_state':self.state,'failed':self.failed}
        tmp=self.out/'flight_status.tmp';tmp.write_text(json.dumps(status));tmp.replace(self.out/'flight_status.json')

    def save(self):
        np.savetxt(self.out/'flight.csv',self.samples,delimiter=',',header='t,x,y,z,p1,y1,p2,y2,p3,y3'+(',camera_yaw,camera_pitch' if self.gimbal else ''))
        (self.out/'flight_result.json').write_text(json.dumps(dict(passed=self.failed is None,failed=self.failed,stage=self.stage,flight_state=self.state,requested_distance_m=self.distance),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--distance',type=float,default=0.);p.add_argument('--speed',type=float,default=.5);p.add_argument('--gimbal',action='store_true');p.add_argument('--inspect-gimbal',action='store_true');a=p.parse_args();rclpy.init();n=Test(a.output,a.distance,a.speed,a.gimbal,a.inspect_gimbal)
    try:
        while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
    except KeyboardInterrupt:n.failed='Interrupted';n.save()
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if n.failed:raise SystemExit(1)
