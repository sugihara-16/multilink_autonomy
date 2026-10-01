#!/usr/bin/env python3
"""Validate actual three-camera ROVIO tracks and VIO/LVI trajectories during flight."""
import argparse
import json
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation, Slerp
from flight_slam import Flight, sample


class VioFlight(Flight):
    def __init__(self, output):
        self.vio=[]; self.camera_counts=[0,0,0]; self.feature_seen=set(); self.last_vio_wall=0
        super().__init__(output,node_name='unipilot_vio_validation')
        self.create_subscription(Odometry,'/rovio/odometry',self.on_vio,100)
        self.create_subscription(PointCloud2,'/rovio/pcl',self.on_features,10)
        for i,name in enumerate(['front','left','right']):
            self.create_subscription(Image,f'/rmf_unipilot/cam_{name}/image',lambda msg,i=i:self.on_camera(i,msg),10)

    def on_camera(self,i,msg):
        self.camera_counts[i]+=1

    def on_vio(self,msg):
        self.vio.append(sample(msg)); self.last_vio_wall=time.monotonic()

    def on_features(self,msg):
        points=point_cloud2.read_points(msg,field_names=('camId','x','y','z'),skip_nans=True)
        self.feature_seen.update(int(p['camId']) for p in points if int(p['camId']) in (0,1,2))

    def control(self):
        if self.start is None and (len(self.vio)<30 or self.feature_seen!={0,1,2}):
            if time.monotonic()-self.start_wall>300 and not self.done:
                self.failed='ROVIO initialization or tracks from all three cameras missing'
                self.save();self.done=True;self.get_logger().error(self.failed)
            return
        if self.start is not None and not self.done and self.sim_time-self.vio[-1][0]>3:
            self.failed=self.failed or 'VIO output stale; landing using ground truth'
        super().control()

    def save(self):
        super().save()
        report=json.loads((self.output/'flight_slam_result.json').read_text())
        report.update(estimator_inputs=['LiDAR','IMU','three-camera ROVIO'],camera_messages=self.camera_counts,
                      cameras_with_features=sorted(self.feature_seen),vio_messages=len(self.vio))
        v=np.array(self.vio);gt=np.array(self.gt)
        np.save(self.output/'vio_trajectory.npy',v)
        if len(v)>1 and len(gt)>1:
            v=v[(v[:,0]>=gt[0,0])&(v[:,0]<=gt[-1,0])]
            if len(v)>1:
                truth=np.column_stack([np.interp(v[:,0],gt[:,0],gt[:,i]) for i in range(1,4)])
                r=Slerp(gt[:,0],Rotation.from_quat(gt[:,4:8]))(v[:1,0])[0]*Rotation.from_quat(v[0,4:8]).inv()
                aligned=r.apply(v[:,1:4]-v[0,1:4])+truth[0]
                e=np.linalg.norm(aligned-truth,axis=1)
                report.update(vio_position_rmse_m=float(np.sqrt(np.mean(e**2))),vio_position_max_error_m=float(e.max()),vio_max_gap_s=float(np.diff(v[:,0]).max()))
                np.savetxt(self.output/'vio_aligned.csv',np.column_stack([v[:,0],truth,aligned]),delimiter=',',header='t,gt_x,gt_y,gt_z,vio_x,vio_y,vio_z')
        (self.output/'flight_slam_result.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();node=VioFlight(a.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.done:node.failed='Interrupted before completion';node.save()
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
