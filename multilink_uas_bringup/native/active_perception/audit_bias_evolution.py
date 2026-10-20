#!/usr/bin/env python3
"""Compare estimated IMU bias to colocated noiseless reference; no estimator feedback."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from report_routes import load
from audit_recorded_imu import imu_records

def main():
 p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 parent=a.flight.parent if a.flight.name=='fixed' else a.flight
 u=np.array(list(imu_records(a.flight/'vio_input.bin')));ref=np.loadtxt(parent/'diagnostic_sensors/imu.csv',delimiter=',',skiprows=1);idx=np.searchsorted(ref[:,0],u[:,0]);assert np.array_equal(ref[idx,0],u[:,0]);noise=u[:,1:]-ref[idx,1:]
 model=ET.parse(parent/'unipilot_gimbal.sdf');ns=[model.find(f".//sensor[@name='imu']/imu/{kind}/{axis}/noise") for kind in ['angular_velocity','linear_acceleration'] for axis in 'xyz'];sigma=np.array([float(n.findtext('stddev')) for n in ns]);tau=np.array([float(n.findtext('dynamic_bias_correlation_time')) for n in ns]);dt=(u[:,0]-u[0,0])*1e-9;basis=np.exp(-dt[:,None]/tau);initial=(basis*noise).sum(axis=0)/(basis**2).sum(axis=0);fit=basis*initial
 # Independent scalar Kalman smoother, with SDF noise dynamics, estimates the
 # actual time-varying bias from noisy-minus-clean measurements only.
 measurement_variance=np.array([float(n.findtext('stddev'))**2+float(n.findtext('precision','0'))**2/12 for n in ns]);driving=np.array([float(n.findtext('dynamic_bias_stddev'))**2 for n in ns]);prior=np.array([float(n.findtext('bias_mean'))**2+float(n.findtext('bias_stddev'))**2 for n in ns]);xf=np.empty_like(noise);pf=np.empty_like(noise);state=np.zeros(6);cov=prior.copy()
 for i in range(len(u)):
  delta=0. if i==0 else (u[i,0]-u[i-1,0])*1e-9;decay=np.exp(-delta/tau);process=driving*tau/2*(-np.expm1(-2*delta/tau));state=decay*state;cov=decay**2*cov+process;gain=cov/(cov+measurement_variance);state=state+gain*(noise[i]-state);cov=(1-gain)*cov;xf[i]=state;pf[i]=cov
 smooth=xf.copy();variance=pf.copy()
 for i in range(len(u)-2,-1,-1):
  delta=(u[i+1,0]-u[i,0])*1e-9;decay=np.exp(-delta/tau);process=driving*tau/2*(-np.expm1(-2*delta/tau));prediction=decay**2*pf[i]+process;gain=pf[i]*decay/prediction;smooth[i]=xf[i]+gain*(smooth[i+1]-decay*xf[i]);variance[i]=pf[i]+gain**2*(variance[i+1]-prediction)
 d=load(a.flight/'vio_diagnostic.csv');stamps=d['stamp_ns'];est=np.column_stack([d[k] for k in ['gyb_x','gyb_y','gyb_z','acb_x','acb_y','acb_z']]);fit_at=initial*np.exp(-(stamps-u[0,0])[:,None]*1e-9/tau)
 route=load(a.flight/'route_tracking.csv');m=route['route_t']>0;lo=route['sim_t'][m][0]*1e9;hi=route['sim_t'][m][-1]*1e9;mask=(stamps>=lo)&(stamps<=hi);smooth_at=np.column_stack([np.interp(stamps,u[:,0],smooth[:,i]) for i in range(6)]);variance_at=np.column_stack([np.interp(stamps,u[:,0],variance[:,i]) for i in range(6)]);e=est[mask]-smooth_at[mask]
 # 1-second residual averages provide a second independent view of bias trajectory.
 bins=np.floor(dt).astype(int);means=np.array([noise[bins==k].mean(axis=0) for k in np.unique(bins)]);centers=np.array([u[bins==k,0].mean() for k in np.unique(bins)]);counts=np.bincount(bins)
 report=dict(scope=__doc__,flight=str(a.flight),reference_missing_samples=0,model='Independent Kalman smoothing of noisy-minus-clean IMU with SDF dynamics; conditional uncertainty saved. Exponential fit and 1s averages also retained.',expected_sample_sigma=sigma.tolist(),measured_sigma=(noise-fit).std(axis=0).tolist(),bias_at_first_imu_fit=initial.tolist(),gyro_bias_tracking_rmse_rad_s=np.sqrt(np.mean(e[:,:3]**2,axis=0)).tolist(),acc_bias_tracking_rmse_m_s2=np.sqrt(np.mean(e[:,3:]**2,axis=0)).tolist(),gyro_bias_at_end_estimate=est[mask][-1,:3].tolist(),gyro_bias_at_end_reference=smooth_at[mask][-1,:3].tolist(),reference_gyro_sigma_median_rad_s=np.median(np.sqrt(variance_at[mask,:3]),axis=0).tolist(),one_second_gyro_standard_error_rad_s=(sigma[:3]/np.sqrt(1000)).tolist())
 np.savez_compressed(a.output/'bias_evolution.npz',stamp_ns=stamps,estimate=est,reference_fit=fit_at,reference_smooth=smooth_at,reference_variance=variance_at,binned_time_ns=centers,binned_bias=means)
 (a.output/'bias_evolution.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
