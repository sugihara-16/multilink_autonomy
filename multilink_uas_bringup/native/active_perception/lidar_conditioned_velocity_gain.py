"""Local planning benefit of a visual velocity update after LiDAR constraints.

Predict position increment over the existing fusion interval from body velocity.
This deliberately remains a planning proxy: cross-sensor correlations and
future new landmarks are not modeled, and no resulting matrix is fused.
"""
import numpy as np
from joint_velocity_gain import conditional_velocity_covariance


def fused_position_gain(pose_information, joint, visibility, bearing_sigma, horizon_s):
    H=np.asarray(pose_information,dtype=float)
    if H.shape!=(6,6) or not np.isfinite(H).all() or not np.isfinite(horizon_s) or horizon_s<=0:
        raise ValueError('Finite pose information and positive fusion horizon required')
    if not np.allclose(H,H.T,atol=1e-10,rtol=1e-10):raise ValueError('Symmetric pose information required')
    H=(H+H.T)*.5;np.linalg.cholesky(H)
    # Marginalize uncertain orientation; do not assume rotation is known.
    S=H[3:,3:]-H[3:,:3]@np.linalg.solve(H[:3,:3],H[:3,3:])
    S=(S+S.T)*.5
    P=conditional_velocity_covariance(joint,visibility,bearing_sigma)
    baseline=S+np.linalg.solve(np.asarray(joint)[:3,:3]*horizon_s**2,np.eye(3))
    candidate=S[None]+np.linalg.solve(P*horizon_s**2,np.broadcast_to(np.eye(3),P.shape))
    L=np.linalg.cholesky(baseline);C=np.linalg.cholesky((candidate+candidate.transpose(0,2,1))*.5)
    return 2*(np.log(np.diagonal(C,axis1=1,axis2=2)).sum(axis=1)-np.log(np.diag(L)).sum())
