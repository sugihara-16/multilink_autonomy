"""Preserve the estimator's actual tracked points during a camera transition.

An accumulated planning map can contain many corners while the estimator is
losing its own tracks. Map visibility is not a substitute for live support.
"""
import numpy as np
from articulated_search import view_weights
from reachable_view import future_points


def segment_position(current,goal,speed,acceleration,t,norm=np.inf):
    delta=np.asarray(goal)-current;length=float(np.linalg.norm(delta,ord=norm))
    if length<1e-12:return np.array(current)
    if acceleration is None:travel=min(length,speed*t)
    else:
        peak=min(speed,np.sqrt(length*acceleration));ramp=peak/acceleration
        cruise=max(0.,length/peak-ramp)
        if t<ramp:travel=.5*acceleration*t*t
        elif t<ramp+cruise:travel=.5*peak*ramp+peak*(t-ramp)
        else:travel=length-.5*acceleration*max(0.,2*ramp+cruise-t)**2
    return np.asarray(current)+delta*(travel/length)


def retained_support(kin,current,goal,points,velocity,omega,cfg,horizon):
    initial=view_weights(points,kin.camera(current),cfg)*kin.visible(current,points)
    acceleration=getattr(kin,'acceleration',None)
    selected=[];hold=[]
    for t in sorted(set([min(.2,horizon),horizon/2,horizon])):
        future=future_points(points,velocity,omega,t)
        q=segment_position(current,goal,kin.speed,acceleration,t,norm=2 if acceleration is None else np.inf)
        hold.append(float(np.minimum(initial,view_weights(future,kin.camera(current),cfg)*kin.visible(current,future)).sum()))
        selected.append(float(np.minimum(initial,view_weights(future,kin.camera(q),cfg)*kin.visible(q,future)).sum()))
    valid=all(s+1e-9>=cfg.get('retain_fraction',.9)*h for s,h in zip(selected,hold))
    return valid,selected,hold
