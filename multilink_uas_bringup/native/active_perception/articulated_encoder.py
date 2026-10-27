"""ROS2: exposure-time measured six-joint FK, full camera SE(3), mono image."""
import argparse
from bisect import bisect_left
from collections import deque
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,JointState
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,NAMES

def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

class Encoder(Node):
    def __init__(self,out):
        super().__init__('articulated_image_encoder');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.out=out;self.kin=Kinematics(out/'dragon_slam.urdf');self.joints=deque(maxlen=10000);self.images=deque()
        self.paired=0;self.dropped=0;self.max_gap=0;self.first=None;self.last=None;self.positions=[];self.gaps=[]
        self.image_pub=self.create_publisher(Image,'/active/image',20)
        self.ex_pub=self.create_publisher(PoseStamped,'/active/extrinsic',20)
        # The joint publisher is 1000 Hz: depth five buffers only 5 ms while
        # an image callback computes full FK. Retain measured samples instead
        # of accepting longer interpolation gaps or dropping camera frames.
        self.create_subscription(JointState,'/dragon/articulated/states',self.on_joints,QoSProfile(depth=2000,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image,'/dragon/end_camera/image',self.on_image,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(2.,self.report)

    def on_joints(self,m):
        d=dict(zip(m.name,m.position));t=stamp(m)
        if not all(n in d for n in self.kin.names) or (self.joints and t<=self.joints[-1][0]):return
        self.joints.append((t,np.array([d[n] for n in self.kin.names])));self.drain()

    def on_image(self,m):self.images.append(m);self.drain()

    def drain(self):
        while self.images and len(self.joints)>1:
            m=self.images[0];t=stamp(m)
            if t>self.joints[-1][0]:break
            self.images.popleft();ts=[x[0] for x in self.joints];i=bisect_left(ts,t)
            if i==0 and ts[0]!=t:self.dropped+=1;continue
            if ts[i]==t:q=self.joints[i][1]
            else:
                a,qa=self.joints[i-1];b,qb=self.joints[i]
                if b-a>20000000:self.dropped+=1;continue
                q=qa+(qb-qa)*(t-a)/(b-a)
            T=self.kin.camera(q);r=Rotation.from_matrix(T[:3,:3]).as_quat()
            pose=PoseStamped();pose.header=m.header;pose.header.frame_id='fc'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,T[:3,3])
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,r)
            if m.encoding not in ('rgb8','bgr8'):raise ValueError('Unexpected camera encoding')
            raw=np.ndarray((m.height,m.width,3),np.uint8,buffer=m.data,strides=(m.step,3,1))
            gray=cv2.cvtColor(raw,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            image=Image();image.header=m.header;image.height=m.height;image.width=m.width
            image.encoding='mono8';image.step=m.width;image.data=gray.tobytes()
            self.ex_pub.publish(pose);self.image_pub.publish(image);self.paired+=1
            if self.last is not None:
                self.max_gap=max(self.max_gap,t-self.last)
                if t-self.last>50000000:self.gaps.append([self.last,t])
            if self.first is None:self.first=t
            self.last=t
            if len(self.positions)<20000:self.positions.append(T[:3,3].tolist())
            while len(self.joints)>2 and self.joints[1][0]<=t:self.joints.popleft()
        while len(self.images)>20:self.images.popleft();self.dropped+=1

    def report(self):
        result=dict(paired=self.paired,dropped=self.dropped,pending=len(self.images),first_ns=self.first,last_ns=self.last,max_gap_ns=self.max_gap,
                    translation_span_m=np.ptp(self.positions,axis=0).tolist() if self.positions else [],gap_intervals_ns=self.gaps)
        p=self.out/'encoder_audit.tmp';p.write_text(json.dumps(result));p.replace(self.out/'encoder_audit.json')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cv2.setNumThreads(1);rclpy.init();node=Encoder(a.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        # Humble may invalidate a subscription while SIGINT shuts its context
        # down. Never suppress a conversion error while ROS is still running.
        if rclpy.ok():raise
    finally:
        node.report();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
