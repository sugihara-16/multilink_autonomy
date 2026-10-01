#!/usr/bin/env python3
"""Simulation flight and SLAM evaluation. GT controls flight, never SLAM inputs."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from sensor_msgs_py import point_cloud2


def sample(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return [msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
            p.x,p.y,p.z,q.x,q.y,q.z,q.w]


class Flight(Node):
    def __init__(self, output, *, node_name='unipilot_flight_slam',
                 gt_topic='/rmf_unipilot/odom', command_topic='/rmf_unipilot/cmd/vel'):
        super().__init__(node_name)
        self.output = Path(output)
        self.gt, self.est = [], []
        self.map_count, self.map_points = 0, 0
        self.cloud = None
        self.last_gt_wall = self.last_slam_wall = 0
        self.sim_time = 0.0
        self.last_clock_wall = 0.0
        self.start_wall = time.monotonic()
        self.start = None
        self.done = False
        self.failed = None
        self.takeoff_confirmed = False
        self.create_subscription(Odometry, gt_topic, self.on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/slam/odometry', self.on_slam, 50)
        self.create_subscription(PointCloud2, '/slam/map', self.on_map, 5)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, command_topic, 10) if command_topic else None
        self.create_timer(.02, self.control)

    def on_gt(self, msg):
        self.gt.append(sample(msg))
        self.last_gt_wall = time.monotonic()

    def on_slam(self, msg):
        self.est.append(sample(msg))
        self.last_slam_wall = time.monotonic()

    def on_map(self, msg):
        self.map_count += 1
        self.map_points = msg.width*msg.height
        self.cloud = msg

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec*1e-9
        self.last_clock_wall = time.monotonic()

    def control(self):
        now = time.monotonic()
        if self.done:
            return
        if self.start is not None and now-self.last_clock_wall > 1:
            # A paused simulator is not sensor failure. Hold until clock resumes.
            self.cmd.publish(Twist())
            return
        if self.start is None:
            if (len(self.est) < 30 or not self.gt or self.map_count < 1
                    or now-self.last_slam_wall > 1 or now-self.last_gt_wall > 1):
                if now - self.start_wall > 180:
                    self.failed = 'SLAM initialization timed out; no takeoff command sent'
                    self.save()
                    self.done = True
                    self.get_logger().error(self.failed)
                return
            self.start = self.gt[-1][0]
            self.map_at_takeoff = self.map_count
            self.origin = np.array(self.gt[-1][1:4])
            self.get_logger().info('SLAM ready: takeoff, 2 m square, return and land; GT position control')
        t = self.gt[-1][0] - self.start
        # Smooth references: rise 1.5 m, trace square, then descend to the ground.
        times = [0,10,18,30,42,54,66,74,86,92]
        offsets = np.array([[0,0,0],[0,0,1.5],[0,0,1.5],[2,0,1.5],
            [2,2,1.5],[0,2,1.5],[0,0,1.5],[0,0,1.5],[0,0,-.1],[0,0,-.1]])
        target = self.origin + np.array([np.interp(t,times,offsets[:,i]) for i in range(3)])
        if self.sim_time - self.gt[-1][0] > 1:
            self.cmd.publish(Twist())
            self.failed = 'Ground truth feedback timeout'
            self.save(); self.done = True
            return
        if self.sim_time - self.est[-1][0] > 3 and self.failed is None:
            self.failed = 'SLAM output lost during flight; landing with GT control'
            self.get_logger().error(self.failed)
        pos = np.array(self.gt[-1][1:4])
        self.takeoff_confirmed = self.takeoff_confirmed or bool(pos[2]-self.origin[2] > 1.25)
        if t >= 18 and not self.takeoff_confirmed and self.failed is None:
            self.failed = 'Takeoff did not reach required altitude; returning to ground'
            self.get_logger().error(self.failed)
        if self.failed:
            target = np.array([self.origin[0], self.origin[1], self.origin[2]-.1])
        vel = np.clip(1.2*(target-pos), [-.4,-.4,-.3], [.4,.4,.35])
        body_vel = Rotation.from_quat(self.gt[-1][4:8]).inv().apply(vel)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float,body_vel)
        yaw = Rotation.from_quat(self.gt[-1][4:8]).as_euler('xyz')[2]
        msg.angular.z = float(np.clip(-yaw,-.3,.3))
        self.cmd.publish(msg)
        if t >= 92:
            self.cmd.publish(Twist())
            self.save(); self.done = True
            if self.failed:
                self.get_logger().error('Flight aborted; evaluation saved: '+self.failed)
            else:
                self.get_logger().info('Flight complete; evaluation saved. Ctrl+C to stop simulation.')

    def save(self):
        gt, est = np.array(self.gt), np.array(self.est)
        np.savez_compressed(self.output/'trajectories.npz', ground_truth=gt, slam=est)
        report = dict(failure=self.failed, control_source='Gazebo ground truth',
            takeoff_confirmed=bool(self.takeoff_confirmed),
            estimator_inputs=['LiDAR','IMU'], map_messages=self.map_count,
            map_updates_during_flight=self.map_count-getattr(self,'map_at_takeoff',self.map_count),
            map_points=self.map_points, slam_messages=len(est))
        if len(gt)>1 and len(est)>1:
            est = est[(est[:,0]>=gt[0,0]) & (est[:,0]<=gt[-1,0])]
            if len(est)>1:
                truth = np.column_stack([np.interp(est[:,0],gt[:,0],gt[:,i]) for i in range(1,4)])
                truth_rot = Slerp(gt[:,0],Rotation.from_quat(gt[:,4:8]))(est[:,0])
                est_rot = Rotation.from_quat(est[:,4:8])
                align = truth_rot[0]*est_rot[0].inv()
                aligned = align.apply(est[:,1:4]-est[0,1:4])+truth[0]
                error = np.linalg.norm(aligned-truth,axis=1)
                active = est[:,0] >= (self.start or est[0,0])
                error = error[active]
                report.update(position_rmse_m=float(np.sqrt(np.mean(error**2))),
                    position_max_error_m=float(error.max()),
                    max_slam_gap_s=float(np.diff(est[:,0]).max()),
                    altitude_span_m=float(np.ptp(gt[:,3])),
                    horizontal_span_m=np.ptp(gt[:,1:3],axis=0).tolist())
                np.savetxt(self.output/'aligned_trajectory.csv',
                    np.column_stack([est[:,0],truth,aligned]),delimiter=',',
                    header='time,gt_x,gt_y,gt_z,slam_x,slam_y,slam_z',comments='')
        if self.cloud is not None:
            xyz = point_cloud2.read_points(self.cloud,field_names=('x','y','z'),skip_nans=True)
            xyz = np.column_stack([xyz[n] for n in ('x','y','z')]).astype(np.float32)
            np.save(self.output/'slam_map_xyz.npy',xyz)
            with (self.output/'slam_map.ply').open('wb') as stream:
                stream.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(xyz))+
                    '\nproperty float x\nproperty float y\nproperty float z\nend_header\n').encode())
                stream.write(xyz.astype('<f4').tobytes())
        (self.output/'flight_slam_result.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Flight(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.done:
            node.failed = 'Interrupted before completion'
            node.save()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
