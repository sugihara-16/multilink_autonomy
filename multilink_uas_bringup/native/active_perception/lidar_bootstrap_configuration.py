"""Explicit identical camera/LiDAR calibration for the two experimental arms."""
from pathlib import Path
import numpy as np
import yaml


def parameters(imu_parameters,camera_yaml,T_B_L,output):
    camera=yaml.safe_load(Path(camera_yaml).read_text())
    K=np.asarray(camera['camera_matrix']['data'],dtype=float).reshape(3,3)
    D=np.asarray(camera['distortion_coefficients']['data'],dtype=float)
    T=np.asarray(T_B_L,dtype=float)
    if not np.isfinite(K).all() or min(K[0,0],K[1,1])<=0 or not np.array_equal(K[2],[0.,0.,1.]):
        raise ValueError('Finite pinhole intrinsics required')
    if not np.isfinite(D).all() or np.any(D!=0):raise ValueError('Depth projection requires the undistorted simulation camera')
    if T.shape!=(7,) or not np.isfinite(T).all() or abs(np.linalg.norm(T[3:])-1)>1e-6:raise ValueError('Known LiDAR extrinsic required')
    result={}
    for name,directory in [('rovio_gimbal',Path(output)),('fixed_rovio',Path(output)/'fixed')]:
        result[name]=dict(imu_parameters,lidar_depth_bootstrap=True,depth_intrinsics=K.ravel().tolist(),
                         depth_T_B_L=T.tolist(),lidar_bootstrap_csv=str(directory/'lidar_initialization.csv'))
    return result
