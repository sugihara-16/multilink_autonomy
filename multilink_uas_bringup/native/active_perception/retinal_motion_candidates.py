"""Bounded gimbal steps that stabilize observed bearings in camera coordinates.

Unlike converting a nearly vertical ray to azimuth, the bearing Jacobian also
accounts for image rotation about the optical axis. This is a local kinematic
candidate generator, not an estimator update or a claim of accuracy improvement.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from core import camera_rotation


def bounded_least_squares(J,b,lower,upper):
    J=np.asarray(J,dtype=float);b=np.asarray(b,dtype=float)
    lo=np.asarray(lower,dtype=float);hi=np.asarray(upper,dtype=float)
    if J.ndim!=2 or J.shape[1]!=2 or b.shape!=(len(J),) or lo.shape!=(2,) or hi.shape!=(2,):
        raise ValueError('Invalid two-axis least-squares dimensions')
    if not all(np.isfinite(a).all() for a in (J,b,lo,hi)) or np.any(lo>hi):raise ValueError('Invalid bounded least squares')
    candidates=[np.clip(np.zeros(2),lo,hi)]
    unconstrained=np.linalg.lstsq(J,b,rcond=1e-12)[0]
    if np.all(unconstrained>=lo)&np.all(unconstrained<=hi):candidates.append(unconstrained)
    for axis in range(2):
        other=1-axis;den=float(J[:,other]@J[:,other])
        for edge in (lo[axis],hi[axis]):
            x=np.zeros(2);x[axis]=edge
            x[other]=np.clip(float(J[:,other]@(b-J[:,axis]*edge))/den if den>1e-24 else 0.,lo[other],hi[other])
            candidates.append(x)
    candidates=np.array(candidates);cost=np.sum((candidates@J.T-b)**2,axis=1)
    best=np.flatnonzero(cost<=cost.min()+1e-14*max(1.,float(b@b)))
    return candidates[best[np.argmin(np.sum(candidates[best]**2,axis=1))]]


def bearing_jacobians(unit,R_home,current):
    unit=np.asarray(unit,dtype=float);Rbc=camera_rotation(R_home,*current)
    axes=np.column_stack([R_home[:,2],R_home@Rotation.from_euler('Z',current[0]).apply([0.,1.,0.])])
    # d(R_BC.T u_B)/dtheta = R_BC.T (u_B x joint_axis_B).
    J=np.stack([np.cross(unit,axes[:,k])@Rbc for k in range(2)],axis=-1)
    return unit@Rbc,J,Rbc


def candidate_steps(unit,rates,R_home,current,omega,dt,scan_speed,lower,upper):
    unit=np.asarray(unit,dtype=float);rates=np.asarray(rates,dtype=float)
    if unit.ndim!=2 or unit.shape[1]!=3 or rates.shape!=unit.shape or len(unit)==0:
        raise ValueError('Observed unit bearings and rates required')
    if not all(np.isfinite(x).all() for x in (unit,rates,R_home,current,omega)) or not np.allclose(np.linalg.norm(unit,axis=1),1.,atol=1e-8):
        raise ValueError('Invalid bearing state')
    if not np.isfinite(dt) or dt<=0 or not np.isfinite(scan_speed) or scan_speed<0:raise ValueError('Invalid command interval/rate')
    camera,J,Rbc=bearing_jacobians(unit,R_home,current)
    motion=-np.cross(np.asarray(omega),unit)+rates
    motion-=np.sum(motion*unit,axis=1)[:,None]*unit
    direction=np.array([0.,0.,1.])-camera[:,2,None]*camera
    size=np.linalg.norm(direction,axis=1)
    step=np.minimum(np.arctan2(size,camera[:,2]),scan_speed*dt)
    centering=direction*(step/np.maximum(size,1e-12))[:,None]
    desired=-(motion@Rbc)*dt+centering
    commands=[bounded_least_squares(j,b,lower,upper) for j,b in zip(J,desired)]
    # Stabilize the whole observed pattern as an additional candidate. A single
    # central ray cannot detect image roll; several off-axis rays can.
    commands.append(bounded_least_squares(J.reshape(-1,2),desired.ravel(),lower,upper))
    return np.asarray(commands)
