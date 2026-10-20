#!/usr/bin/env python3
"""Truth-assisted IMU audit/interventions. Never usable as an ordinary estimator input."""
import argparse,json,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from report_routes import poses,interp

def imu_records(path):
 with path.open('rb') as f:
  while h:=f.read(8):
   kind,size=struct.unpack('<II',h)
   if kind!=0:f.seek(size,1);continue
   payload=f.read(size);seq,sec,nsec,length=struct.unpack_from('<IIII',payload)
   off=16+length;assert len(payload)==off+296
   yield [sec*10**9+nsec,*struct.unpack_from('<3d',payload,off+104),*struct.unpack_from('<3d',payload,off+200)]

def main():
 p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--reference-imu',type=Path);a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
 imu=np.array(list(imu_records(a.flight/'vio_input.bin')));gt=poses(a.flight/'truth.csv');total_count=len(imu);imu=imu[(imu[:,0]>=gt[0,0]+20000000)&(imu[:,0]<=gt[-1,0]-20000000)];t=imu[:,0]*1e-9;tg=gt[:,0]*1e-9;q=Rotation.from_quat(gt[:,4:8]);dt=np.diff(tg)
 # Body angular velocity from successive truth orientations; midpoint sampling.
 w=(q[:-1].inv()*q[1:]).as_rotvec()/dt[:,None];mid=(tg[:-1]+tg[1:])/2;omega=np.column_stack([np.interp(t,mid,w[:,i]) for i in range(3)])
 # Cubic interpolation of dense truth position; centered derivative evaluation.
 spline=CubicSpline(tg,gt[:,1:4]);_,qi=interp(gt,imu[:,0]);acc=qi.inv().apply(spline(t,2)+[0,0,9.8]);clean=np.c_[omega,acc];residual=imu[:,1:]-clean
 if a.reference_imu:
  reference=np.loadtxt(a.reference_imu,delimiter=',',skiprows=1);stamps=reference[:,0].astype(np.int64);index=np.searchsorted(stamps,imu[:,0].astype(np.int64));assert np.all(index<len(stamps)) and np.array_equal(stamps[index],imu[:,0].astype(np.int64))
  clean=reference[index,1:];omega=clean[:,:3];acc=clean[:,3:];residual=imu[:,1:]-clean
 # Fit the specified deterministic bias decay; exclude rapidly rotating/accelerating periods.
 valid=(np.linalg.norm(omega,axis=1)<.05)&(np.linalg.norm(acc[:,:2],axis=1)<1)&(t>t[0]+2)&(t<t[-1]-2)
 parent=a.flight.parent if a.flight.name=='fixed' else a.flight
 model=ET.parse(parent/'unipilot_gimbal.sdf');noises=[model.find(f".//sensor[@name='imu']/imu/{kind}/{axis}/noise") for kind in ('angular_velocity','linear_acceleration') for axis in 'xyz'];tau=np.array([float(n.findtext('dynamic_bias_correlation_time')) for n in noises]);specified_std=[float(n.findtext('stddev')) for n in noises];basis=np.exp(-(t-t[0])[:,None]/tau)
 initial=(basis[valid]*residual[valid]).sum(axis=0)/(basis[valid]**2).sum(axis=0);bias=basis*initial
 highpass=residual-bias
 report=dict(scope=('Exact colocated noiseless IMU, timestamp matched; diagnostic oracle, not deployable.' if a.reference_imu else 'UNVALIDATED pose differentiation: not an exact clean IMU; do not use as causal oracle.'),imu_count=len(t),unmodified_outside_truth_coverage=total_count-len(t),true_pose_step_s=np.percentile(dt,[0,50,100]).tolist(),imu_step_s=np.percentile(np.diff(t),[0,50,100]).tolist(),fit_sample_count=int(valid.sum()),fitted_bias_at_first_imu=initial.tolist(),residual_std_after_decay_fit=highpass[valid].std(axis=0).tolist(),specified_sample_std=specified_std)
 # Each intervention changes one physical IMU component while preserving all images and stamps.
 variants={'oracle_gyro':np.c_[omega,imu[:,4:7]],'oracle_acc':np.c_[imu[:,1:4],acc],'oracle_both':clean,'oracle_gyro_bias':np.c_[imu[:,1:4]-bias[:,:3],imu[:,4:7]],'oracle_acc_bias':np.c_[imu[:,1:4],imu[:,4:7]-bias[:,3:]]}
 for name,values in variants.items():np.savetxt(a.output/(name+'.csv'),np.c_[imu[:,0],values],delimiter=',',header='stamp_ns,gx,gy,gz,ax,ay,az',comments='',fmt=['%.0f']+['%.17g']*6)
 np.savez_compressed(a.output/'imu_audit.npz',stamp_ns=imu[:,0],measured=imu[:,1:],true=clean,bias_fit=bias,residual=residual)
 (a.output/'imu_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
