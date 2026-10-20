#!/usr/bin/env python3
"""Fit velocity-error persistence on the two original independent fixed flights.

No development-flight errors or fused outcomes enter this calibration. The
one-second period and reported-covariance scale are inherited. A scalar AR(1)
coefficient uses previous whitened error transported by measured VIO rotation.
Innovation noise is at least the stationary unit-variance AR model requires.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from report_routes import load,poses,interp


def inverse_root(C):
    eigen,U=np.linalg.eigh(C)
    if np.any(eigen<=0):raise ValueError('Nonpositive covariance')
    return (U/np.sqrt(eigen[:,None,:]))@U.transpose(0,2,1)


def main():
    p=argparse.ArgumentParser();p.add_argument('calibration',type=Path);p.add_argument('replay',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve existing calibration')
    cfg=json.loads(a.calibration.read_text());training=[];sources=[]
    period=float(cfg['velocity_period_s']);scale=float(cfg['velocity_covariance_scale'])
    for source in cfg['velocity_calibration']:
        fixed=Path(source['source']);name=fixed.parent.name.removesuffix('_active');csv=a.replay/(name+'_25')/'vio.csv'
        if hashlib.sha256(csv.read_bytes()).hexdigest()!=source['replay_sha256']:raise ValueError('Calibration replay changed')
        d=load(csv);all_t=d['stamp_ns'];route=load(fixed/'route_tracking.csv');tr=route['sim_t'][route['route_t']>0];gt=poses(fixed/'truth.csv')
        good=(all_t>=tr[0]*1e9)&(all_t<=tr[-1]*1e9)&(all_t>gt[0,0]+1e7)&(all_t<gt[-1,0]-1e7)
        indices=np.flatnonzero(good)
        # Use one deterministic phase, not twenty overlapping phase trials.
        indices=indices[::20];t=all_t[indices]
        if not np.all(np.abs(np.diff(t)-period*1e9)<=2):raise ValueError('Noncontinuous calibration')
        v=np.column_stack([d['v'+c][indices] for c in 'xyz'])
        C=np.array([d[f'velocity_cov_{i}{j}'][indices] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)*scale
        before,_=interp(gt,t-1e7);after,_=interp(gt,t+1e7);_,truth_rotation=interp(gt,t)
        e=v-truth_rotation.inv().apply((after-before)/.02)
        w=np.einsum('nij,nj->ni',inverse_root(C),e)
        measured_rotation=Rotation.from_quat(np.column_stack([d['q'+c][indices] for c in 'xyzw']))
        transport=measured_rotation[1:].inv()*measured_rotation[:-1]
        x=transport.apply(w[:-1]);y=w[1:];training.append((x,y))
        sources.append(dict(source=str(csv),sha256=source['replay_sha256'],samples=len(t)))
    numerator=np.mean([np.mean(np.sum(x*y,axis=1)) for x,y in training])
    denominator=np.mean([np.mean(np.sum(x*x,axis=1)) for x,y in training])
    rho=float(numerator/denominator)
    if not 0<=rho<1:raise ValueError('Fitted persistence is outside stationary positive AR model')
    innovations=[y-rho*x for x,y in training]
    moment=np.mean([z.T@z/len(z) for z in innovations],axis=0)
    variance=max(1-rho*rho,float(np.linalg.eigvalsh(moment).max()))
    result=dict(scope=__doc__,sources=sources,source_calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest(),
        period_s=period,velocity_covariance_scale=scale,rho=rho,innovation_variance=variance,
        empirical_innovation_second_moment=moment.tolist(),
        limitations=['AR(1) is an approximation; it does not remove shared IMU correlation.',
                     'Moving-camera errors may have different persistence; require independent validation.',
                     'Measured relative VIO rotation transports residual coordinates; its uncertainty is not propagated.'])
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
