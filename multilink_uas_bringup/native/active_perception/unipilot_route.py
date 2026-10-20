#!/usr/bin/env python3
"""Same route and speed as DRAGON, controlled with UniPilot's velocity interface."""
import argparse,bisect,csv,json,math,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from trajectory_math import blend,segment_value
import corner_speed
import initialization_motion

class Flight(Node):
    def __init__(self,args):
        super().__init__('unipilot_route',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.args=args;self.out=args.output;self.gt=None;self.stage='initialize';self.stage_time=0.;self.route_time=0.;self.failure=None;self.last=None;self.tracking=0.;self.integral=np.zeros(3);self.dt=0.
        self.plan=json.loads((args.route/'route.json').read_text());self.segments=json.loads((args.route/'segments.json').read_text())['segments']
        # Build numeric arrays once; segment_value's np.asarray then returns a
        # view instead of rebuilding thousands of path points every control tick.
        for segment in self.segments:
            for key in ('start','end','points','arc'):
                if key in segment:segment[key]=np.asarray(segment[key])
            corner_speed.prepare(segment)
        self.previous_reference_speed=0.
        self.starts=[s['t0'] for s in self.segments];self.duration=self.segments[-1]['t0']+self.segments[-1]['duration']
        self.end=min(self.duration,args.end) if args.end>0 else self.duration
        self.cmd=self.create_publisher(Twist,'/rmf_unipilot/cmd/vel',10);self.enable=self.create_publisher(Bool,'/rmf_unipilot/enable',10)
        self.vio_start=self.create_publisher(Bool,'/active/initialize_vio',10)
        self.create_subscription(Odometry,'/rmf_unipilot/odom',self.odom,qos_profile_sensor_data)
        self.csvfile=(self.out/'route_tracking.csv').open('w',buffering=1);self.csv=csv.writer(self.csvfile)
        self.csv.writerow(['sim_t','lap','route_t','ref_x','ref_y','ref_z','gt_x','gt_y','gt_z','tracking_error_m','reference_yaw_rad'])
        self.create_timer(.02,self.control);self.create_timer(1.,self.status)
    def odom(self,m):self.gt=m
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def transition(self,stage):
        self.stage=stage;self.stage_time=self.now();self.get_logger().info('Stage: '+stage);self.status()
    def pose(self):
        p=self.gt.pose.pose.position;q=self.gt.pose.pose.orientation
        return np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])
    def command(self,p,v,y,w):
        position,R=self.pose();delta=p-position;self.tracking=float(np.linalg.norm(delta))
        # Remove the velocity-controller's steady hover offset before starting
        # the route. Anti-windup excludes the large initial takeoff error.
        self.integral=np.clip(self.integral+.6*delta*self.dt*(np.abs(delta)<.3),-.3,.3)
        world_velocity=v+np.clip(2.*delta+self.integral,[-.4,-.4,-.25],[.4,.4,.35])
        if self.args.velocity_feedback and self.stage=='tracking':
            measured=self.gt.twist.twist.linear
            correction=self.args.velocity_feedback*(v-R.apply([measured.x,measured.y,measured.z]))
            if self.args.velocity_feedback_xy_only:correction[2]=0.
            world_velocity+=correction
        body=R.inv().apply(world_velocity);yaw=R.as_euler('xyz')[2]
        m=Twist();m.linear.x,m.linear.y,m.linear.z=map(float,body);m.angular.z=float(np.clip(w+1.2*math.atan2(math.sin(y-yaw),math.cos(y-yaw)),-.4,.4))
        self.cmd.publish(m);self.enable.publish(Bool(data=True))
        self.csv.writerow([self.now(),1,self.route_time,*p,*position,self.tracking,y])
    def control(self):
        now=self.now();dt=0. if self.last is None else now-self.last;self.last=now;self.dt=min(dt,.1)
        if self.failure or self.stage=='complete':return
        if self.gt is None:return
        if self.stage=='initialize':
            if self.args.airborne_init:
                if now<3:return
            elif not self.estimators_ready():return
            self.origin,R=self.pose();self.initial_yaw=R.as_euler('xyz')[2];self.target=np.array(self.plan['waypoints'][0])
            self.transition('takeoff');return
        age=now-(self.gt.header.stamp.sec+self.gt.header.stamp.nanosec*1e-9)
        if age>.3:self.failure=f'GT feedback stale: {age}';self.status();return
        if self.stage=='takeoff':
            target=self.origin.copy();target[2]=self.target[2]
            self.command(target,np.zeros(3),self.initial_yaw,0.)
            if self.tracking<.12:self.transition('settle')
            elif now-self.stage_time>90:self.failure='Takeoff timeout';self.status()
        elif self.stage=='settle':
            target=self.origin.copy();target[2]=self.target[2];self.command(target,np.zeros(3),self.initial_yaw,0.)
            if now-self.stage_time>8:
                if self.args.airborne_init:self.transition('initialize_airborne')
                else:self.approach_start=self.pose()[0];self.transition('approach')
        elif self.stage=='initialize_airborne':
            target=self.origin.copy();target[2]=self.target[2];self.command(target,np.zeros(3),self.initial_yaw,0.)
            self.vio_start.publish(Bool(data=True))
            if self.estimators_ready():self.approach_start=self.pose()[0];self.transition('approach')
            elif now-self.stage_time>30:self.failure='Airborne estimator initialization timeout';self.status()
        elif self.stage=='approach':
            u=(now-self.stage_time)/12;s,ds=blend(u);delta=self.target-self.approach_start;y=self.segments[0]['yaw0']
            dy=math.atan2(math.sin(y-self.initial_yaw),math.cos(y-self.initial_yaw))
            self.command(self.approach_start+s*delta,ds/12*delta,self.initial_yaw+s*dy,ds/12*dy)
            if u>=1 and self.tracking<.15:
                self.transition('excitation' if self.args.initialization_excitation else 'tracking')
        elif self.stage=='excitation':
            elapsed=now-self.stage_time
            delta,velocity=initialization_motion.reference(elapsed)
            self.command(self.target+delta,velocity,self.segments[0]['yaw0'],0.)
            if self.tracking>.18:
                self.failure=f'Preparation tracking error {self.tracking:.3f} m';self.status()
            elif elapsed>=initialization_motion.DURATION and self.tracking<.05:
                self.transition('tracking')
        elif self.stage=='tracking':
            rate=1. if self.tracking<self.plan.get('tracking_slow_m',.4) else 0.
            if self.args.smooth_tracking_rate:
                slow=self.plan.get('tracking_slow_m',.4)
                u=float(np.clip((self.tracking-.5*slow)/(.5*slow),0.,1.))
                rate=.5*(1.+math.cos(math.pi*u))
            if self.args.corner_speed_limit:
                current=min(len(self.segments)-1,max(0,bisect.bisect_right(self.starts,self.route_time)-1))
                rate=min(rate,corner_speed.progress_rate(self.segments[current],self.route_time,self.previous_reference_speed,min(dt,.1)))
            self.route_time=min(self.end,self.route_time+min(dt,.1)*rate)
            i=min(len(self.segments)-1,max(0,bisect.bisect_right(self.starts,self.route_time)-1))
            p,v,y,w=segment_value(self.segments[i],self.route_time);self.command(p,v*rate,y,w*rate)
            self.previous_reference_speed=float(np.linalg.norm(v*rate))
            if self.tracking>self.plan.get('tracking_stop_m',.8):self.failure=f'Tracking error {self.tracking:.3f} m';self.status()
            if self.route_time>=self.end:self.transition('complete')
    def estimators_ready(self):
        if self.args.control_only:return True
        for directory in [self.out,*([self.out/'fixed'] if self.args.paired else [])]:
            try:health=json.loads((directory/'evaluation_status.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):return False
            if not health.get('startup',{}).get('checked'):return False
            if not health['startup']['valid']:
                self.failure='Invalid ROVIO initialization: '+directory.name;self.status();return False
            if min(health['counts'].values())<30:return False
        return True
    def status(self):
        report=dict(stage=self.stage,sim_time=self.now(),lap=1,route_time_s=self.route_time,route_duration_s=self.duration,tracking_error_m=self.tracking,flight_state=5,joint_error_rad=0.,failure=self.failure)
        tmp=self.out/'route_status.tmp';tmp.write_text(json.dumps(report,indent=2)+'\n');tmp.replace(self.out/'route_status.json')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--end',type=float,default=0);p.add_argument('--paired',action='store_true');p.add_argument('--airborne-init',action='store_true');p.add_argument('--velocity-feedback',type=float,default=0.);p.add_argument('--control-only',action='store_true',help='Controller diagnosis only; no estimator comparison');p.add_argument('--velocity-feedback-xy-only',action='store_true');p.add_argument('--smooth-tracking-rate',action='store_true');p.add_argument('--corner-speed-limit',action='store_true');p.add_argument('--initialization-excitation',action='store_true');a=p.parse_args()
    rclpy.init();n=Flight(a)
    try:
        while rclpy.ok() and n.stage!='complete' and not n.failure:rclpy.spin_once(n,timeout_sec=.2)
    except KeyboardInterrupt:pass
    finally:
        n.status();n.csvfile.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if n.failure:raise RuntimeError(n.failure)
if __name__=='__main__':main()
