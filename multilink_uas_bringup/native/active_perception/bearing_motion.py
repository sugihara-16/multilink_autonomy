"""Causal, metric-scale-independent bearing motion for gimbal prediction.

The IMU removes body rotation between observations. The remaining observed
bearing change includes translation and the camera lever arm, without mixing
ROVIO landmark depth with another estimator's metric velocity.
"""
from collections import deque
import numpy as np


def unit_rows(v):
    v=np.asarray(v,dtype=float).reshape(-1,3)
    return v/np.maximum(np.linalg.norm(v,axis=1)[:,None],1e-12)


def advance_bearings(unit,rates,old_to_now,dt):
    """old_to_now maps current body vectors into the earlier body frame."""
    transported=unit_rows(unit)@old_to_now
    rates=np.asarray(rates)@old_to_now
    predicted=unit_rows(transported+rates*dt)
    rates-=np.sum(rates*predicted,axis=1)[:,None]*predicted
    return predicted,rates


class BearingMotion:
    def __init__(self,window=.3,max_gap=.25):
        self.window=window;self.max_gap=max_gap;self.frames=deque()

    def update(self,stamp,ids,rays,rotation):
        rays=unit_rows(rays);rates=np.zeros_like(rays)
        if self.frames and stamp<=self.frames[-1][0]:return rates
        if self.frames and stamp-self.frames[-1][0]>self.max_gap:self.frames.clear()
        while len(self.frames)>1 and self.frames[1][0]<=stamp-self.window:self.frames.popleft()
        if self.frames:
            old_stamp,old=self.frames[0];dt=stamp-old_stamp
            if dt>=.1-1e-9:
                R=rotation(old_stamp,stamp)
                if R is not None:
                    for k,ident in enumerate(ids):
                        previous=old.get(int(ident))
                        if previous is not None:
                            rate=(rays[k]-previous@R)/dt
                            rates[k]=rate-rays[k]*np.dot(rate,rays[k])
        self.frames.append((float(stamp),{int(i):r.copy() for i,r in zip(ids,rays)}))
        return rates
