#!/usr/bin/env python3
"""Diagnostic only: noiseless IMU integration against truth, no pose differentiation."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid as integrate
from scipy.spatial.transform import Rotation
from report_routes import poses,interp,load
from audit_recorded_imu import imu_records
p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('imu',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
c=np.loadtxt(a.imu,delimiter=',',skiprows=1);gt=poses(a.flight/'truth.csv');lo=max(gt[0,0],next(imu_records(a.flight/'vio_input.bin'))[0])+20000000;hi=min(gt[-1,0]-20000000,load(a.flight/'route_tracking.csv')['sim_t'][-1]*1e9);c=c[(c[:,0]>=lo)&(c[:,0]<=hi)];stamps=c[:,0];t=(stamps-stamps[0])*1e-9;c=c[:,1:7];truth,q=interp(gt,stamps)
qs=np.zeros((len(t),4));qs[0]=q[0].as_quat();current=q[0]
for i in range(1,len(t)):
 current=current*Rotation.from_rotvec((c[i-1,:3]+c[i,:3])*.5*(t[i]-t[i-1]));qs[i]=current.as_quat()
qi=Rotation.from_quat(qs);result=dict(scope='Diagnostic, NOT estimator accuracy. Exact initial orientation; best initial velocity fit over full interval. Truth orientation is interpolated from recorded 100Hz poses. No bias/noise correction needed for clean reference sensor.',samples=len(t),dt_percentiles_s=np.percentile(np.diff(t),[0,50,100]).tolist(),rotation_rmse_deg=float(np.sqrt(np.mean(np.rad2deg((q.inv()*qi).magnitude())**2))))
errors={}
for name,rotation in [('truth_orientation',q),('integrated_orientation',qi)]:
 acc=rotation.apply(c[:,3:])+[0,0,-9.8];velocity=integrate(acc,t,axis=0,initial=0);position=integrate(velocity,t,axis=0,initial=0);delta=position-(truth-truth[0]);v0=-np.sum(t[:,None]*delta,axis=0)/np.sum(t*t);error=delta+t[:,None]*v0
 result[name]=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),axis_rmse_m=np.sqrt(np.mean(error**2,axis=0)).tolist(),final_error_m=error[-1].tolist(),fitted_initial_velocity=v0.tolist());errors[name]=error
np.savez_compressed(a.output/'integration.npz',t=t,**errors);(a.output/'integration.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
