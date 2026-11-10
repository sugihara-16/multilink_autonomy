#!/usr/bin/env python3
"""Evaluate conditional fusion diagnostics, retaining every completed variant."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def report(p):
 ev={e['stage']:e['time'] for e in json.loads((p/'flight_events.json').read_text())};start=ev['active'];protocol=json.loads((p/'protocol.json').read_text());end=min(ev['landing'],start+protocol['distance_m']/protocol['speed_mps']+2.)
 gt=np.genfromtxt(p/'truth.csv',delimiter=',',names=True);tt=gt['stamp_ns']*1e-9
 truth=np.column_stack([gt[k] for k in ['x','y','z']]);qr=Rotation.from_quat(np.column_stack([gt[k] for k in ['qx','qy','qz','qw']]))
 primary=np.genfromtxt(p/'mimosa/sensor_states.csv',delimiter=',',names=True,dtype=None,encoding=None)
 duplicate=np.genfromtxt(p/'duplicate/sensor_states.csv',delimiter=',',names=True,dtype=None,encoding=None)
 result={'scope':'Conditional identical-input controls; only primary fusion steers camera. First tracking pose SE3 align, no scale fit.','duplicate_exact':bool(np.array_equal(primary,duplicate)),'interval':[start,end],'variants':{}}
 for f in p.glob('*/graph_manager_odometry.tum'):
  a=np.loadtxt(f);a=a[(a[:,0]>=start-1e-8)&(a[:,0]<=end+1e-8)];t=a[:,0];q=Slerp(tt,qr)(t)
  align=q[0]*Rotation.from_quat(a[0,4:8]).inv();pos=np.column_stack([np.interp(t,tt,truth[:,i]) for i in range(3)])
  error=align.apply(a[:,1:4]-a[0,1:4])-pos+pos[0]
  result['variants'][f.parent.name]={'rmse_m':float(np.sqrt(np.mean((error**2).sum(1)))),'end_error_m':float(np.linalg.norm(error[-1])),'estimated_displacement_m':float(np.linalg.norm(a[-1,1:4]-a[0,1:4])),'count':len(t),'max_gap_s':float(np.diff(t).max())}
 result['completion']=json.loads((p/'completion.json').read_text());(p/'diagnostic_report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('flight',type=Path);report(parser.parse_args().flight)
