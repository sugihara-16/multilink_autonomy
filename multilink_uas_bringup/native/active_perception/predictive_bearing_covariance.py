"""Constant-velocity local forecast for correlated 3D visual landmarks.

The target velocity stays in the current body frame. Future landmark bearings
use the predicted future body frame. This propagates existing velocity/depth
correlations; it is not an exact future EKF or an unseen-texture predictor.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def forecast(points, velocity, covariance, camera_position, angular_velocity, horizon_s):
    points=np.asarray(points,dtype=float);velocity=np.asarray(velocity,dtype=float)
    C=np.asarray(covariance,dtype=float);camera_position=np.asarray(camera_position,dtype=float)
    omega=np.asarray(angular_velocity,dtype=float);n=len(points)
    if points.shape!=(n,3) or C.shape!=(3+3*n,3+3*n) or velocity.shape!=(3,) or camera_position.shape!=(3,) or omega.shape!=(3,):
        raise ValueError('Invalid forecast dimensions')
    if not all(np.isfinite(x).all() for x in [points,velocity,C,camera_position,omega]) or not np.isfinite(horizon_s) or horizon_s<0:
        raise ValueError('Invalid forecast state/time')
    if not np.allclose(C,C.T,atol=1e-10,rtol=1e-10):raise ValueError('Asymmetric joint geometry covariance')
    R=Rotation.from_rotvec(-omega*horizon_s).as_matrix()
    rays=(points-velocity*horizon_s)@R.T-camera_position
    distance=np.linalg.norm(rays,axis=1);valid=distance>.1
    unit=rays/np.maximum(distance[:,None],1e-12)
    J=np.zeros((3+2*n,3+3*n));J[:3,:3]=np.eye(3)
    for i in range(n):
        if not valid[i]:continue  # Exclude a point at/behind the optical centre from scoring.
        axis=np.eye(3)[np.argmin(abs(unit[i]))]
        e=np.cross(unit[i],axis);e/=np.linalg.norm(e)
        basis=np.column_stack([e,np.cross(unit[i],e)])
        B=basis.T@R/distance[i]
        J[3+2*i:5+2*i,:3]=-horizon_s*B
        J[3+2*i:5+2*i,3+3*i:6+3*i]=B
    result=J@C@J.T
    return (result+result.T)*.5,rays,valid,J
