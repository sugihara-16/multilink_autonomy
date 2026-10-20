#!/usr/bin/env python3
"""Calibrate reported body-velocity uncertainty using the old fixed flights only."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np,yaml
from report_routes import load,poses,interp


def main():
    p=argparse.ArgumentParser();p.add_argument('original',type=Path);p.add_argument('replay',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Do not overwrite calibration')
    original=json.loads(a.original.read_text());replays=json.loads((a.replay/'results.json').read_text())
    moments=[];results=[]
    for source in original['sources']:
        fixed=Path(source['path']);name=fixed.parent.name.removesuffix('_active');csv=a.replay/(name+'_25')/'vio.csv'
        if not replays['flights'][name]['25']['null_byte_equal']:raise ValueError('Replay changed estimator output')
        cfg=yaml.safe_load((fixed/'mimosa.yaml').read_text())
        if cfg['odometry']['T_B_S']!=[0,0,0,0,0,0,1]:raise ValueError('Calibration currently requires coincident body/IMU')
        d=load(csv);t=d['stamp_ns'];gt=poses(fixed/'truth.csv');route=load(fixed/'route_tracking.csv');tr=route['sim_t'][route['route_t']>0]
        good=(t>=tr[0]*1e9)&(t<=tr[-1]*1e9)&(t>gt[0,0]+2e7)&(t<gt[-1,0]-2e7)
        t=t[good];vel=np.column_stack([d['v'+axis][good] for axis in 'xyz'])
        C=np.array([d['velocity_cov_'+str(i)+str(j)][good] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
        eigen,U=np.linalg.eigh(C)
        if eigen.min()<=0 or not np.isfinite(C).all():raise ValueError('Invalid published covariance')
        _,q=interp(gt,t);errors=[]
        for h in [1e7,2e7]:
            before,_=interp(gt,t-h);after,_=interp(gt,t+h)
            errors.append(vel-q.inv().apply((after-before)/(2*h*1e-9)))
        e=errors[0];invroot=(U/np.sqrt(eigen[:,None,:]))@U.transpose(0,2,1)
        white=np.einsum('nij,nj->ni',invroot,e);M=white.T@white/len(white);moments.append(M)
        results.append(dict(source=str(fixed),replay_sha256=hashlib.sha256(csv.read_bytes()).hexdigest(),samples=len(t),
            rmse_m_s=np.sqrt(np.mean(e*e,axis=0)).tolist(),bias_m_s=e.mean(axis=0).tolist(),
            normalized_second_moment=M.tolist(),normalized_error_per_dof=float(np.trace(M)/3),
            derivative_resolution_rmse_change=float(np.linalg.norm(np.sqrt(np.mean(errors[1]**2,axis=0))-np.sqrt(np.mean(e*e,axis=0))))))
    aggregate=np.mean(moments,axis=0);scale=max(1.,float(np.linalg.eigvalsh(aggregate).max()))
    result=dict(original);result.update(velocity_covariance_scale=scale,velocity_period_s=1.,velocity_calibration=results,
        velocity_protocol='Direct body-frame velocity, with published covariance multiplied by one conservative scalar. Scalar is max(1, maximum eigenvalue of equal-flight mean whitened-error second moment), old fixed calibration flights only; no bias subtraction or performance search. One factor per second, matching existing smoother horizon.',
        velocity_limitations=['Published velocity and gravity direction share IMU and feature information; their cross covariance is not supplied by nav_msgs/Odometry.',
                             'One-second spacing reduces repeated information; it does not eliminate all temporal or shared-IMU correlation.'])
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(scale=scale,flights=results),indent=2))


if __name__=='__main__':main()
