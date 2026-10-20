#!/usr/bin/env python3
"""Posthoc error/declared uncertainty at the exact sparse fusion timestamps.

Truth is used only here for diagnosis. No weights, estimator inputs or acceptance
criteria are changed. Samples within each flight are temporally correlated.
"""
import argparse,json
from pathlib import Path
import numpy as np
from report_routes import load,poses,interp


def describe(base,tracking):
    gt=poses(base/'truth.csv');d=load(base/'mimosa/velocity_factors.csv');t=d['stamp_ns']
    good=(t>=tracking[0]*1e9)&(t<=tracking[-1]*1e9)&(t>gt[0,0]+2e7)&(t<gt[-1,0]-2e7)
    t=t[good];v=np.column_stack([d['v'+c][good] for c in 'xyz'])
    C=np.array([d[f'c{i}{j}'][good] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
    _,q=interp(gt,t);e=[]
    for h in [1e7,2e7]:
        before,_=interp(gt,t-h);after,_=interp(gt,t+h)
        e.append(v-q.inv().apply((after-before)/(2*h*1e-9)))
    error=e[0];L=np.linalg.cholesky(C);white=np.linalg.solve(L,error[:,:,None])[:,:,0]
    nees=(white*white).sum(axis=1)
    tilt=load(base/'mimosa/tilt_factors.csv')
    if not np.array_equal(tilt['stamp_ns'][good],t):raise ValueError('Velocity/tilt factor stamps differ')
    up=np.column_stack([tilt['up_'+c][good] for c in 'xyz']);true_up=q.inv().apply(np.tile([0.,0.,1.],(len(t),1)))
    angle=np.arctan2(np.linalg.norm(np.cross(up,true_up),axis=1),(up*true_up).sum(axis=1))
    sigma=tilt['sigma_rad'][good]
    return dict(samples=len(t),velocity_rmse_mps=float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
        velocity_bias_mps=error.mean(axis=0).tolist(),
        velocity_normalized_error_per_dof=float(nees.mean()/3),
        velocity_declared_sigma_trace_median=float(np.median(np.sqrt(np.trace(C,axis1=1,axis2=2)))),
        velocity_95percent_ellipsoid_coverage=float(np.mean(nees<=7.814727903251179)),
        velocity_true_derivative_check_rmse_mps=float(np.sqrt(np.mean(np.sum((e[1]-e[0])**2,axis=1)))),
        tilt_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(angle**2)))),
        tilt_normalized_error_per_dof=float(np.mean((angle/sigma)**2)/2))


def main():
    p=argparse.ArgumentParser();p.add_argument('campaigns',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve previous diagnosis')
    rows=[]
    for campaign in a.campaigns:
        for base in sorted(campaign.glob('*_active')):
            runtime=json.loads((base/'runtime.json').read_text())
            if runtime['status']!='route_complete' or not runtime['stopped']:raise ValueError('Only complete stopped flights')
            route=load(base/'route_tracking.csv');tracking=route['sim_t'][route['route_t']>0]
            saved=json.loads((campaign/'results.json').read_text())[base.name]
            rows.append(dict(campaign=campaign.name,flight=base.name,original_valid=saved['valid'],
                failed_checks=[k for k,v in saved['checks'].items() if not v],
                fixed=describe(base/'fixed',tracking),active=describe(base,tracking)))
    result=dict(scope=__doc__,rows=rows);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(rows,indent=2))


if __name__=='__main__':main()
