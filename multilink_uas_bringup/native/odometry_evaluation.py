#!/usr/bin/env python3
"""Independent LIO/VIO comparison against interpolated FC truth, plus RViz data."""
import argparse,bisect,csv,json,time
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as PathMsg
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from trajectory_math import interpolate_pose,aligned_error
from flight_slam import sample

class Evaluation(Node):
    def __init__(self,out):
        super().__init__('independent_odometry_evaluation');self.out=out
        self.truth=deque(maxlen=4000);self.pending={k:deque(maxlen=1000) for k in ('lio','vio')}
        self.alignment={};self.alignment_info={};self.last={};self.counts={k:0 for k in ('lio','vio')};self.dropped={k:0 for k in ('lio','vio')}
        self.paths={k:deque(maxlen=20000) for k in ('lio','vio','truth')};self.path_stamp={k:-1 for k in self.paths}
        self.files=[];self.writers={};self.raw={}
        for k in self.paths:
            f=(out/f'{k}_raw.csv').open('w',buffering=1);self.files.append(f);w=csv.writer(f);w.writerow(['t','x','y','z','qx','qy','qz','qw']);self.raw[k]=w
        for k in ('lio','vio'):
            f=(out/f'{k}_errors.csv').open('w',buffering=1);self.files.append(f);w=csv.writer(f)
            w.writerow(['t','gt_x','gt_y','gt_z','aligned_x','aligned_y','aligned_z','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);self.writers[k]=w
        self.odom_pub={k:self.create_publisher(Odometry,f'/evaluation/{k}/odometry',10) for k in self.paths}
        self.path_pub={k:self.create_publisher(PathMsg,f'/evaluation/{k}/path',2) for k in self.paths}
        self.error_pub={k:self.create_publisher(Float64MultiArray,f'/evaluation/{k}/error',10) for k in ('lio','vio')}
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        self.create_subscription(Odometry,'/dragon/ground_truth',self.on_truth,qos_profile_sensor_data)
        for k,topic in [('lio','/slam/odometry'),('vio','/rovio/odometry')]:
            self.create_subscription(Odometry,topic,lambda m,k=k:self.on_est(k,m),100)
        self.create_timer(1.,self.publish_paths)
    def on_truth(self,msg):
        a=sample(msg)
        if self.truth and a[0]<=self.truth[-1][0]:return
        self.truth.append(a);self.raw['truth'].writerow(a)
        self.publish_pose('truth',msg.header.stamp,np.array(a[1:4]),Rotation.from_quat(a[4:8]))
        for k in self.pending:self.process(k)
    def on_est(self,k,msg):
        a=sample(msg);self.raw[k].writerow(a)
        if a[0]<=self.last.get(k,-1):return
        self.last[k]=a[0];self.pending[k].append((a,msg.header.stamp,msg.header.frame_id));self.process(k)
    def process(self,k):
        if len(self.truth)<2:return
        times=[a[0] for a in self.truth]
        while self.pending[k] and self.pending[k][0][0][0]<=times[-1]:
            a,stamp,frame=self.pending[k].popleft();t=a[0]
            j=bisect.bisect_right(times,t)
            if j==0:self.dropped[k]+=1;continue
            j=min(j,len(times)-1)
            if times[j]-times[j-1]>.1:self.dropped[k]+=1;continue
            pgt,rgt=interpolate_pose(self.truth[j-1],self.truth[j],t);r=Rotation.from_quat(a[4:8])
            if k not in self.alignment:
                ra=rgt*r.inv();pa=pgt-ra.apply(a[1:4]);self.alignment[k]=(ra,pa)
                self.alignment_info[k]=dict(time=t,source_frame=frame,rotation_xyzw=ra.as_quat().tolist(),translation=pa.tolist())
                if k=='lio':
                    tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='evaluation_world';tf.child_frame_id=frame
                    tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,pa)
                    tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,ra.as_quat())
                    self.static_tf.sendTransform(tf)
                (self.out/'alignment.json').write_text(json.dumps(self.alignment_info,indent=2)+'\n')
            p,q,error=aligned_error(a[1:4],r,pgt,rgt,self.alignment[k])
            self.writers[k].writerow([t,*pgt,*p,*error]);self.counts[k]+=1
            em=Float64MultiArray();em.data=list(map(float,[t,*error]));self.error_pub[k].publish(em)
            self.publish_pose(k,stamp,p,q)
    def publish_pose(self,k,stamp,p,q):
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='evaluation_world';m.child_frame_id='evaluation_'+k
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,p)
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,q.as_quat());self.odom_pub[k].publish(m)
        t=stamp.sec+stamp.nanosec*1e-9
        if t-self.path_stamp[k]>=.5:
            v=PoseStamped();v.header=m.header;v.pose=m.pose.pose;self.paths[k].append(v);self.path_stamp[k]=t
        if k=='truth':
            tf=TransformStamped();tf.header=m.header;tf.child_frame_id='evaluation_robot';tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,p);tf.transform.rotation=m.pose.pose.orientation;self.tf.sendTransform(tf)
    def publish_paths(self):
        for k,points in self.paths.items():
            if points:
                p=PathMsg();p.header=points[-1].header;p.poses=list(points);self.path_pub[k].publish(p)
        (self.out/'evaluation_status.json').write_text(json.dumps(dict(counts=self.counts,unmatched_truth=self.dropped,last_estimate_time=self.last),indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();rclpy.init();node=Evaluation(args.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        for f in node.files:f.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
