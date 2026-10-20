"""Shared trajectory and timestamp-matched pose math; no ROS side effects."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

def blend(u):
    u=float(np.clip(u,0,1))
    return 10*u**3-15*u**4+6*u**5,30*u**2-60*u**3+30*u**4

def segment_value(segment,t):
    T=segment['duration']
    if segment.get('kind') in ('trapezoid','polyline'):
        a=np.asarray(segment['start']);delta=np.asarray(segment['end'])-a
        length=segment['arc'][-1] if segment['kind']=='polyline' else np.linalg.norm(delta);direction=delta/np.linalg.norm(delta)
        ramp=segment['ramp_s'];speed=segment['speed_mps'];u=float(np.clip(t-segment['t0'],0,T))
        if u<ramp: distance=.5*speed/ramp*u*u;velocity=speed*u/ramp
        elif u>T-ramp: remaining=T-u;distance=length-.5*speed/ramp*remaining*remaining;velocity=speed*remaining/ramp
        else: distance=speed*(u-.5*ramp);velocity=speed
        if segment['kind']=='polyline':
            points=np.asarray(segment['points']);arc=np.asarray(segment['arc'])
            i=min(len(arc)-2,max(0,int(np.searchsorted(arc,distance,side='right')-1)))
            span=arc[i+1]-arc[i];fraction=(distance-arc[i])/span
            return points[i]+fraction*(points[i+1]-points[i]),velocity*(points[i+1]-points[i])/span,segment['yaw0'],0.
        return a+distance*direction,velocity*direction,segment['yaw0'],0.
    s,ds=blend((t-segment['t0'])/T)
    a=np.asarray(segment['start']);delta=np.asarray(segment['end'])-a
    yaw=segment['yaw0'];dy=segment['yaw1']-yaw
    return a+s*delta,ds/T*delta,yaw+s*dy,ds/T*dy

def interpolate_pose(a,b,t):
    u=(t-a[0])/(b[0]-a[0]);p=(1-u)*np.asarray(a[1:4])+u*np.asarray(b[1:4])
    q=Slerp([a[0],b[0]],Rotation.from_quat([a[4:8],b[4:8]]))([t])[0]
    return p,q

def aligned_error(position,rotation,truth_position,truth_rotation,alignment):
    r0,p0=alignment
    p=r0.apply(position)+p0;q=r0*rotation
    # Euler angles of the relative rotation, expressed in the truth body frame.
    error=np.r_[p-truth_position,(truth_rotation.inv()*q).as_euler('xyz',degrees=True)]
    return p,q,error
