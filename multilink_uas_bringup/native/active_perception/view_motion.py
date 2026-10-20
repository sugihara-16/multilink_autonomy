"""Short causal gyro history for aligning the planner's input acquisition times."""
from collections import deque
import threading
import numpy as np
from scipy.spatial.transform import Rotation


class RotationHistory:
    def __init__(self,duration=2.):self.samples=deque();self.duration=duration;self.lock=threading.Lock()

    def append(self,t,w):
        with self.lock:
            if self.samples and t<=self.samples[-1][0]:return
            self.samples.append((float(t),np.asarray(w,dtype=float)))
            while len(self.samples)>2 and self.samples[1][0]<t-self.duration:self.samples.popleft()

    def latest(self):
        with self.lock:return self.samples[-1][0] if self.samples else None

    def rotation(self,start,end):
        if end<start:raise ValueError('Cannot integrate backwards')
        if end-start<1e-9:return np.eye(3)
        with self.lock:samples=list(self.samples)
        if not samples or start<samples[0][0]-1e-8 or end>samples[-1][0]+1e-8:return None
        times=np.array([t for t,w in samples]);rates=np.array([w for t,w in samples])
        interior=(times>start)&(times<end)
        t=np.r_[start,times[interior],end]
        w=np.vstack([np.array([np.interp(start,times,rates[:,i]) for i in range(3)]),rates[interior],np.array([np.interp(end,times,rates[:,i]) for i in range(3)])])
        if np.diff(t).max()>.003:raise ValueError('Missing gyro interval')
        increments=Rotation.from_rotvec((w[:-1]+w[1:])*.5*np.diff(t)[:,None]).as_matrix()
        R=np.eye(3)
        for d in increments:R=R@d
        return R

    def average_rate(self,end,window=.3):
        R=self.rotation(end-window,end)
        return None if R is None else Rotation.from_matrix(R).as_rotvec()/window
