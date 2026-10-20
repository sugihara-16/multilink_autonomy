"""Conservative speed scheduling at sharp polyline vertices; geometry unchanged."""
import math
import numpy as np

def prepare(segment):
    segment['_sharp_arcs']=np.empty(0)
    if segment.get('kind')!='polyline':return
    points=np.asarray(segment['points']);delta=np.diff(points,axis=0);length=np.linalg.norm(delta,axis=1)
    direction=delta/np.maximum(length[:,None],1e-12)
    turns=np.sum(direction[:-1]*direction[1:],axis=1)<math.cos(math.radians(20))
    segment['_sharp_arcs']=np.asarray(segment['arc'])[1:-1][turns]

def progress_rate(segment,t,previous_speed,dt,acceleration=.15,corner_speed=.02):
    if segment.get('kind') not in ('polyline','trapezoid'):return 1.
    length=segment['arc'][-1] if segment['kind']=='polyline' else np.linalg.norm(np.asarray(segment['end'])-segment['start'])
    duration=segment['duration'];ramp=segment['ramp_s'];speed=segment['speed_mps'];u=float(np.clip(t-segment['t0'],0,duration))
    if u<ramp:distance=.5*speed/ramp*u*u;nominal=speed*u/ramp
    elif u>duration-ramp:
        remaining=duration-u;distance=length-.5*speed/ramp*remaining*remaining;nominal=speed*remaining/ramp
    else:distance=speed*(u-.5*ramp);nominal=speed
    if nominal<1e-9:return 1.
    cap=previous_speed+acceleration*max(dt,0.)
    corners=segment.get('_sharp_arcs',np.empty(0));index=np.searchsorted(corners,distance-.005)
    if index<len(corners):cap=min(cap,math.sqrt(corner_speed**2+2*acceleration*max(0.,corners[index]-distance)))
    return float(np.clip(cap/nominal,0.,1.))
