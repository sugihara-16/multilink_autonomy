#!/usr/bin/env python3
"""ROS2: pair each exposure with interpolated measured gimbal angles, not commands."""
import argparse
from array import array as byte_array
from bisect import bisect_left
from collections import deque
import json
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation
import yaml
from core import camera_rotation

def stamp(m): return m.header.stamp.sec*1000000000+m.header.stamp.nanosec

class Encoder(Node):
    def __init__(self,calibration,fixed=False,reliable=False):
        super().__init__('fixed_image_encoder' if fixed else 'active_image_encoder')
        self.fixed=fixed
        prefix='/fixed' if fixed else ''
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time',value=True)])
        manifest=json.loads((calibration/'camera_manifest.json').read_text());cam=manifest['cameras'][0]
        self.expected=manifest['expected_joints'];self.square_stamp=None;self.square_ok=not self.expected
        T=np.array(cam['T_FC_camera']);self.home=T[:3,:3];self.position=T[:3,3]
        raw=yaml.safe_load((calibration/'raw_cam0.yaml').read_text())
        rect=yaml.safe_load((calibration/'cam0.yaml').read_text())
        self.width=rect['image_width'];self.height=rect['image_height']
        self.maps=None if raw==rect else cv2.initUndistortRectifyMap(np.array(raw['camera_matrix']['data']).reshape(3,3),
            np.zeros(5),np.eye(3),np.array(rect['camera_matrix']['data']).reshape(3,3),(self.width,self.height),cv2.CV_32FC1)
        self.enc=deque(maxlen=10000);self.images=deque();self.dropped=0;self.paired=0
        self.pub=self.create_publisher(Image,prefix+'/active/image',10)
        self.ex=self.create_publisher(PoseStamped,prefix+'/active/extrinsic',10)
        self.angles=self.create_publisher(Float64MultiArray,prefix+'/active/encoder',10)
        joint_qos=QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE) if reliable else qos_profile_sensor_data
        image_qos=QoSProfile(depth=30,reliability=ReliabilityPolicy.RELIABLE) if reliable else qos_profile_sensor_data
        if not fixed:self.create_subscription(JointState,'/active/gimbal/states',self.joints,joint_qos)
        self.create_subscription(Image,cam['topic'],self.image,image_qos)
        if self.expected:self.create_subscription(JointState,'/dragon/joint_states',self.body_joints,qos_profile_sensor_data)
        self.create_timer(10.,self.report)

    def report(self): self.get_logger().info(f'paired={self.paired} dropped={self.dropped} pending={len(self.images)}')

    def body_joints(self,m):
        positions=dict(zip(m.name,m.position))
        if not self.expected.keys()<=positions.keys():return
        self.square_ok=max(abs(positions[k]-v) for k,v in self.expected.items())<.03
        self.square_stamp=stamp(m)

    def joints(self,m):
        names=dict(zip(m.name,m.position))
        if not all('active_camera_'+a+'_joint' in names for a in ('yaw','pitch')): return
        t=stamp(m)
        if self.enc and t<=self.enc[-1][0]: return
        angles=np.array([names['active_camera_'+a+'_joint'] for a in ('yaw','pitch')])
        self.enc.append((t,angles))
        self.angles.publish(Float64MultiArray(data=[t/1e9,*angles]));self.drain()

    def image(self,m):
        if self.fixed:
            t=stamp(m);self.enc.append((t,np.zeros(2)))
            # A fixed virtual sensor has an exact, constant transform.
            if len(self.enc)==1:self.enc.append((t,np.zeros(2)))
            self.angles.publish(Float64MultiArray(data=[t/1e9,0.,0.]))
        self.images.append(m);self.drain()

    def drain(self):
        while self.images and len(self.enc)>1:
            m=self.images[0];t=stamp(m)
            if t>self.enc[-1][0]: break
            self.images.popleft()
            if self.expected and (not self.square_ok or self.square_stamp is None or abs(t-self.square_stamp)>200000000):
                self.dropped+=1;continue
            ts=[v[0] for v in self.enc];i=bisect_left(ts,t)
            if i==0 and t!=ts[0]:self.dropped+=1;continue
            if ts[i]==t:a=self.enc[i][1]
            else:
                t0,a0=self.enc[i-1];t1,a1=self.enc[i]
                if t1-t0>20000000:self.dropped+=1;continue
                a=a0+(a1-a0)*((t-t0)/(t1-t0))
            R=camera_rotation(self.home,*a);q=Rotation.from_matrix(R).as_quat()
            pose=PoseStamped();pose.header=m.header;pose.header.frame_id='fc'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,self.position)
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
            if m.encoding not in ('rgb8','bgr8'): raise ValueError('Expected raw RGB/BGR camera image')
            array=np.ndarray((m.height,m.width,3),dtype=np.uint8,buffer=m.data,strides=(m.step,3,1))
            gray=cv2.cvtColor(array,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            converted=gray if self.maps is None else cv2.remap(gray,*self.maps,interpolation=cv2.INTER_LINEAR)
            out=Image();out.header=m.header;out.height=self.height;out.width=self.width;out.encoding='mono8';out.step=self.width;out.data=byte_array('B',converted.tobytes())
            self.ex.publish(pose);self.pub.publish(out);self.paired+=1
            # Future exposures never need samples older than this exposure.
            # Retain its left bracket plus every newer encoder measurement.
            while len(self.enc)>2 and self.enc[1][0]<=t:self.enc.popleft()
        while len(self.images)>10:self.images.popleft();self.dropped+=1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--calibration',type=Path,required=True);p.add_argument('--fixed',action='store_true');p.add_argument('--reliable',action='store_true');a=p.parse_args()
    cv2.setNumThreads(1);rclpy.init();n=Encoder(a.calibration,a.fixed,a.reliable)
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
