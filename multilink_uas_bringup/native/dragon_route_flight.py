#!/usr/bin/env python3
"""GT-controlled repeating FC trajectory; independent estimators never control flight."""
import argparse,bisect,csv,json,math,shutil,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty,UInt8
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry,Path as PathMsg
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from aerial_robot_msgs.msg import FlightNav
from flight_slam import sample
from trajectory_math import blend,segment_value
from storage_capacity import StorageCapacity

class RouteFlight(Node):
    def __init__(self,out,route,require_recording=False,once=False,sensors_only=False):
        super().__init__('dragon_route_flight',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.out=out
        self.sensors_only=sensors_only
        self.storage_capacity=StorageCapacity(out)
        self.storage_capacity.read()  # Resolve WSL host drive before starting control timers.
        self.once=once
        self.world=json.loads((out/"world.json").read_text())["world_name"]
        self.require_recording=require_recording
        self.plan=json.loads((route/'route.json').read_text());self.segments=json.loads((route/'segments.json').read_text())['segments']
        self.starts=[s['t0'] for s in self.segments];self.duration=self.segments[-1]['t0']+self.segments[-1]['duration']
        self.expected=json.loads((out/'camera_manifest.json').read_text())['expected_joints']
        self.state=None;self.cog=None;self.gt=None;self.joint_error=None;self.joint_time=0
        self.sim=0.;self.last_sim=0.;self.stage='initialize';self.stage_time=0.;self.lap=0;self.route_time=0.;self.failure=None;self.tracking=0.;self.last_control=0.
        self.csvfile=(out/'route_tracking.csv').open('w',buffering=1);self.csv=csv.writer(self.csvfile)
        self.csv.writerow(['sim_t','lap','route_t','ref_x','ref_y','ref_z','gt_x','gt_y','gt_z','tracking_error_m','reference_yaw_rad'])
        self.nav=self.create_publisher(FlightNav,'/dragon/uav/nav',10)
        self.commands={k:self.create_publisher(Empty,'/dragon/teleop_command/'+k,10) for k in ('start','takeoff','land')}
        self.route_pub=self.create_publisher(PathMsg,'/evaluation/planned_route',2)
        self.create_subscription(Odometry,'/dragon/ground_truth',lambda m:setattr(self,'gt',sample(m)),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/dragon/uav/cog/odom',lambda m:setattr(self,'cog',sample(m)),qos_profile_sensor_data)
        self.create_subscription(UInt8,'/dragon/flight_state',lambda m:setattr(self,'state',m.data),10)
        self.create_subscription(JointState,'/dragon/joint_states',self.joints,qos_profile_sensor_data)
        self.create_timer(.02,self.control);self.create_timer(1.,self.status)
    def joints(self,m):
        v=dict(zip(m.name,m.position))
        if self.expected.keys()<=v.keys():self.joint_error=max(abs(v[k]-x) for k,x in self.expected.items());self.joint_time=self.sim
    def transition(self,s,command=None):
        self.stage=s;self.stage_time=self.sim;self.get_logger().info('Stage: '+s)
        if command:self.commands[command].publish(Empty())
    def pause(self,why):
        if self.failure:return
        self.failure=why;self.get_logger().error(why+'; pausing Gazebo, preserving recording')
        subprocess.run(['gz','service','-s',f'/world/{self.world}/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: true'],capture_output=True,timeout=8)
        self.transition('paused');self.status()
    def begin_approach(self):
        self.body_offset=Rotation.from_quat(self.gt[4:8]).inv().apply(np.array(self.cog[1:4])-self.gt[1:4])
        self.fc_to_cog=Rotation.from_quat(self.gt[4:8]).inv()*Rotation.from_quat(self.cog[4:8])
        self.approach_start=np.array(self.gt[1:4]);self.approach_yaw=Rotation.from_quat(self.gt[4:8]).as_euler('xyz')[2]
        (self.out/'control_geometry.json').write_text(json.dumps(dict(fc_to_cog_translation=self.body_offset.tolist(),fc_to_cog_rotation=self.fc_to_cog.as_quat().tolist()),indent=2)+'\n')
        self.transition('approach')
    def command(self,p,v,yaw,omega):
        r=Rotation.from_euler('z',yaw);offset=r.apply(self.body_offset);c=p+offset
        cv=v+np.cross([0,0,omega],offset);cyaw=(r*self.fc_to_cog).as_euler('xyz')[2]
        m=FlightNav();m.header.stamp=self.get_clock().now().to_msg();m.control_frame=m.WORLD_FRAME;m.target=m.COG
        m.pos_xy_nav_mode=m.POS_VEL_MODE;m.pos_z_nav_mode=m.POS_VEL_MODE;m.yaw_nav_mode=m.POS_VEL_MODE
        m.target_pos_x,m.target_pos_y,m.target_pos_z=map(float,c)
        m.target_vel_x,m.target_vel_y,m.target_vel_z=map(float,cv);m.target_yaw=float(cyaw);m.target_omega_z=float(omega)
        self.nav.publish(m);self.tracking=float(np.linalg.norm(p-self.gt[1:4]));self.csv.writerow([self.sim,self.lap,self.route_time,*p,*self.gt[1:4],self.tracking,yaw])
        if self.tracking>self.plan.get('tracking_stop_m',.8):self.pause(f'FC tracking error {self.tracking:.3f} m exceeds route limit')
    def control(self):
        self.sim=self.get_clock().now().nanoseconds*1e-9
        if self.failure or self.sim-self.last_control<.04:return
        dt=self.sim-self.last_control;self.last_control=self.sim
        if self.stage=='landing':
            if self.state==0:self.transition('complete');self.status()
            elif self.sim-self.stage_time>90:self.pause('Landing timeout')
            return
        if self.stage=='complete':return
        if self.stage=='initialize':
            try:counts=json.loads((self.out/('capture_status.json' if self.sensors_only else 'evaluation_status.json')).read_text())['counts']
            except (FileNotFoundError,json.JSONDecodeError):return
            recording=not self.require_recording or any((self.out/'rosbag').glob('*.db3'))
            if (self.state==0 and self.gt and self.cog and self.joint_error is not None and self.joint_error<.03
                    and counts and min(counts.values())>=30 and recording and self.nav.get_subscription_count()>0):self.transition('arming','start')
            return
        if self.stage=='arming':
            if self.state==2:self.transition('takeoff','takeoff')
            elif self.sim-self.stage_time>60:self.pause('Arming timeout')
            return
        if self.stage=='takeoff':
            if self.state==5:self.transition('settle')
            elif self.sim-self.stage_time>120:self.pause('Takeoff timeout')
            return
        if self.sim-self.gt[0]>.3 or self.sim-self.cog[0]>.3:self.pause('GT feedback stale');return
        if self.state!=5:self.pause(f'Unexpected flight state {self.state}');return
        if self.joint_error is None or self.joint_error>.03 or self.sim-self.joint_time>1:self.pause('Fixed square joint configuration invalid');return
        if self.stage=='settle':
            if self.sim-self.stage_time>8:self.begin_approach()
            return
        if self.stage in ('approach','repeat_turn'):
            T=12.;u=(self.sim-self.stage_time)/T;s,ds=blend(u);target=np.array(self.plan['waypoints'][0]);delta=target-self.approach_start
            heading=self.segments[0]['yaw0']
            dy=math.atan2(math.sin(heading-self.approach_yaw),math.cos(heading-self.approach_yaw))
            self.command(self.approach_start+s*delta,ds/T*delta,self.approach_yaw+s*dy,ds/T*dy)
            if u>=1 and self.tracking<.15:
                self.route_time=0.;self.lap+=1;self.transition('tracking')
            return
        if self.stage=='finish_hold':
            p,v,y,w=segment_value(self.segments[-1],self.duration)
            self.command(p,np.zeros(3),y,0.)
            if self.sim-self.stage_time>3:self.transition('landing','land')
            return
        if self.stage=='tracking':
            # Freeze schedule rather than cutting corners if position tracking lags.
            rate=1. if self.tracking<self.plan.get('tracking_slow_m',.4) else 0.
            self.route_time=min(self.duration,self.route_time+min(dt,.1)*rate)
            i=min(len(self.segments)-1,bisect.bisect_right(self.starts,self.route_time)-1)
            p,v,y,w=segment_value(self.segments[i],self.route_time);self.command(p,v*rate,y,w*rate)
            if self.route_time>=self.duration and self.tracking<.15:
                if self.once:self.transition('finish_hold')
                else:
                    self.approach_start=np.array(self.plan['waypoints'][0]);self.approach_yaw=y;self.transition('repeat_turn')
    def status(self):
        capacity=self.storage_capacity.read();free=capacity['available_bytes']
        report=dict(stage=self.stage,sim_time=self.sim,lap=self.lap,route_time_s=self.route_time,route_duration_s=self.duration,
                    tracking_error_m=self.tracking,flight_state=self.state,joint_error_rad=self.joint_error,failure=self.failure,
                    disk_free_gb=free/1e9 if free is not None else None,storage_capacity=capacity)
        temp=self.out/'route_status.tmp';temp.write_text(json.dumps(report,indent=2)+'\n');temp.replace(self.out/'route_status.json')
        if self.require_recording and free is None and not self.failure:self.pause(capacity['error'])
        reserve=self.plan.get('disk_reserve_gb',20)*1e9
        if free is not None and free<reserve and self.stage!='initialize' and not self.failure:self.pause('Actual free disk space below recording reserve')
        if self.sensors_only:return
        path=PathMsg();path.header.frame_id='evaluation_world';path.header.stamp=self.get_clock().now().to_msg()
        for p in self.plan['waypoints']:
            m=PoseStamped();m.header=path.header;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,p);m.pose.orientation.w=1.;path.poses.append(m)
        self.route_pub.publish(path)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--require-recording',action='store_true');p.add_argument('--once',action='store_true');p.add_argument('--sensors-only',action='store_true');args=p.parse_args()
    rclpy.init();n=RouteFlight(args.output,args.route,args.require_recording,args.once,args.sensors_only)
    try:
        while rclpy.ok() and n.stage!='complete':rclpy.spin_once(n,timeout_sec=.2)
    except KeyboardInterrupt:pass
    finally:n.status();n.csvfile.close();n.destroy_node()
