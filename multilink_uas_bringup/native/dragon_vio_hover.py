#!/usr/bin/env python3
"""Validate three-camera VIO and fixed-square kinematics through DRAGON takeoff."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, PointCloud2
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation, Slerp
from dragon_hover import Hover
from vio_flight import VioFlight


class VioHover(Hover):
    def __init__(self, output, land_after_check):
        self.vio = []
        self.camera_counts = [0, 0, 0]
        self.feature_seen = set()
        self.last_vio_wall = 0
        self.joint_error = None
        self.max_joint_error = 0.
        self.last_joints_wall = 0.
        self.joint_samples = []
        self.expected_joints = json.loads((Path(output)/'camera_manifest.json').read_text())['expected_joints']
        super().__init__(output, land_after_check)
        self.create_subscription(Odometry, '/rovio/odometry', self.on_vio, 100)
        self.create_subscription(PointCloud2, '/rovio/pcl', self.on_features, 10)
        self.create_subscription(JointState, '/dragon/joint_states', self.on_joints, qos_profile_sensor_data)
        for i, link in enumerate((1, 3, 4)):
            self.create_subscription(Image, f'/dragon/cam_link{link}/image',
                lambda msg, i=i: self.on_camera(i, msg), qos_profile_sensor_data)

    on_vio = VioFlight.on_vio
    on_features = VioFlight.on_features
    on_camera = VioFlight.on_camera

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not self.expected_joints.keys() <= values.keys():
            return
        self.joint_error = max(abs(values[k]-v) for k,v in self.expected_joints.items())
        self.last_joints_wall = time.monotonic()
        if self.start is not None:
            self.max_joint_error = max(self.max_joint_error, self.joint_error)
        self.joint_samples.append([self.sim_time]+[values[k] for k in self.expected_joints])

    def control(self):
        if self.done:
            return
        now = time.monotonic()
        if self.stage == 'initialize':
            if (len(self.vio)<30 or self.feature_seen!={0,1,2} or min(self.camera_counts)<30
                    or self.joint_error is None or self.joint_error>.03
                    or now-self.last_joints_wall>2 or now-self.last_vio_wall>2):
                if now-self.start_wall>300:
                    self.failed = 'VIO, all three cameras or square joint configuration unavailable'
                    self.save(); self.done = True
                    self.get_logger().error(self.failed)
                return
        elif self.stage in ('arming', 'takeoff', 'hover') and now-self.last_clock_wall<2:
            if self.sim_time-self.vio[-1][0]>3:
                self.fail('ROVIO output stale')
                return
            if self.joint_error>.03 or now-self.last_joints_wall>3:
                self.fail('Fixed-square calibration invalid: body joints moved or unavailable')
                return
        super().control()

    def save(self):
        super().save()
        path = self.output/'flight_slam_result.json'
        report = json.loads(path.read_text())
        report.update(estimator_inputs=['LiDAR','IMU','three-camera ROVIO'],
            camera_links=['link1','link3','link4'], camera_messages=self.camera_counts,
            cameras_with_features=sorted(self.feature_seen), vio_messages=len(self.vio),
            fixed_square_max_joint_error_rad=self.max_joint_error,
            fixed_square_joint_tolerance_rad=.03, expected_joints=self.expected_joints)
        v, gt = np.array(self.vio), np.array(self.gt)
        np.save(self.output/'vio_trajectory.npy', v)
        np.save(self.output/'body_joints.npy', np.array(self.joint_samples))
        if len(v)>1 and len(gt)>1:
            v=v[(v[:,0]>=gt[0,0]) & (v[:,0]<=gt[-1,0])]
            if len(v)>1:
                truth=np.column_stack([np.interp(v[:,0],gt[:,0],gt[:,i]) for i in range(1,4)])
                r=Slerp(gt[:,0],Rotation.from_quat(gt[:,4:8]))(v[:1,0])[0]*Rotation.from_quat(v[0,4:8]).inv()
                aligned=r.apply(v[:,1:4]-v[0,1:4])+truth[0]
                error=np.linalg.norm(aligned-truth,axis=1)
                report.update(vio_position_rmse_m=float(np.sqrt(np.mean(error**2))),
                    vio_position_max_error_m=float(error.max()),vio_max_gap_s=float(np.diff(v[:,0]).max()))
                np.savetxt(self.output/'vio_aligned.csv',np.column_stack([v[:,0],truth,aligned]),delimiter=',',
                    header='t,gt_x,gt_y,gt_z,vio_x,vio_y,vio_z')
        path.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--land-after-check',action='store_true')
    args=parser.parse_args()
    rclpy.init()
    node=VioHover(args.output,args.land_after_check)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.done:
            node.failed='Interrupted before completion';node.save()
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
