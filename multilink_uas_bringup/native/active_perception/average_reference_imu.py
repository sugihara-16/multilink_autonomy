#!/usr/bin/env python3
"""Diagnostic interval averages of clean high-rate measurements. No truth inputs."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
p=argparse.ArgumentParser();p.add_argument('slow',type=Path);p.add_argument('fast',type=Path);p.add_argument('output',type=Path);a=p.parse_args();slow=np.loadtxt(a.slow,delimiter=',',skiprows=1);fast=np.loadtxt(a.fast,delimiter=',',skiprows=1);dt=5000000
slow=slow[(slow[:,0]>=fast[0,0]+dt)&(slow[:,0]<=fast[-1,0])];t=fast[:,0];integral=cumulative_trapezoid(fast[:,1:],t*1e-9,axis=0,initial=0);averages=np.column_stack([(np.interp(slow[:,0],t,integral[:,i])-np.interp(slow[:,0]-dt,t,integral[:,i]))/(dt*1e-9) for i in range(6)])
np.savetxt(a.output,np.c_[slow[:,0],averages],delimiter=',',header='stamp_ns,gx,gy,gz,ax,ay,az',comments='',fmt=['%.0f']+['%.17g']*6)
a.output.with_suffix('.json').write_text(json.dumps(dict(scope='Diagnostic only: trapezoidal mean over preceding 5ms, labelled at interval end. One missing high-rate sample is linearly bridged by trapezoidal integration. No truth, no Gaussian noise, no known-bias injection.',slow=str(a.slow),fast=str(a.fast),samples=len(slow),maximum_fast_gap_ns=float(np.diff(t).max())),indent=2)+'\n')
