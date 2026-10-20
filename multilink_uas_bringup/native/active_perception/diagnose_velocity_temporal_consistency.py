#!/usr/bin/env python3
"""Posthoc temporal consistency of the velocity observations actually fused.

Truth rotates errors/covariances into a common frame for this diagnostic only.
Nonoverlapping blocks compare mean error with the covariance implied by treating
the recorded observations as independent. This is not a causal fusion ablation.
"""
import argparse,json
from pathlib import Path
import numpy as np
from report_routes import load,poses,interp


def describe(base,tracking,temporal=None):
    gt=poses(base/'truth.csv');d=load(base/'mimosa/velocity_factors.csv')
    t=d['stamp_ns'];good=(t>=tracking[0]*1e9)&(t<=tracking[-1]*1e9)&(t>gt[0,0]+1e7)&(t<gt[-1,0]-1e7)
    t=t[good]
    if len(t)<21 or not np.all(np.abs(np.diff(t)-1e9)<=2):raise ValueError('Need continuous 1 Hz velocity factors')
    v=np.column_stack([d['v'+c][good] for c in 'xyz'])
    C=np.array([d[f'c{i}{j}'][good] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
    before,_=interp(gt,t-1e7);after,_=interp(gt,t+1e7);_,q=interp(gt,t)
    R=q.as_matrix();e=q.apply(v)-(after-before)/.02
    C=R@C@R.transpose(0,2,1)
    blocks={}
    for n in [1,2,5,10,20]:
        count=len(t)//n
        error=e[:count*n].reshape(count,n,3).mean(axis=1)
        covariance=C[:count*n].reshape(count,n,3,3).sum(axis=1)/(n*n)
        white=np.linalg.solve(np.linalg.cholesky(covariance),error[:,:,None])[:,:,0]
        nees=(white*white).sum(axis=1)
        blocks[str(n)]=dict(blocks=count,mean_error_rmse_mps=float(np.sqrt(np.mean(np.sum(error*error,axis=1)))),
            normalized_error_per_dof=float(nees.mean()/3),coverage_95=float(np.mean(nees<=7.814727903251179)))
    centered=e-e.mean(axis=0);correlations={}
    for lag in [1,2,5,10,20]:
        x=centered[:-lag];y=centered[lag:]
        denominator=np.sqrt(np.sum(x*x,axis=0)*np.sum(y*y,axis=0))
        correlations[str(lag)]=(np.sum(x*y,axis=0)/np.maximum(denominator,1e-30)).tolist()
    result=dict(samples=len(t),world_mean_error_mps=e.mean(axis=0).tolist(),
        world_component_centered_lag_correlation=correlations,nonoverlapping_block_tests=blocks)
    if temporal is not None:
        from calibrate_velocity_temporal import inverse_root
        body_error=q.inv().apply(e);body_cov=R.transpose(0,2,1)@C@R
        white=np.einsum('nij,nj->ni',inverse_root(body_cov),body_error)
        _,measured_rotation=interp(poses(base/'rovio.csv'),t)
        transport=measured_rotation[1:].inv()*measured_rotation[:-1]
        innovation=white[1:]-temporal['rho']*transport.apply(white[:-1])
        nees=(innovation*innovation).sum(axis=1)/temporal['innovation_variance']
        result['independently_calibrated_ar_model']=dict(normalized_innovation_error_per_dof=float(nees.mean()/3),coverage_95=float(np.mean(nees<=7.814727903251179)))
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('campaigns',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--temporal-calibration',type=Path);a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve previous diagnosis')
    rows=[];temporal=json.loads(a.temporal_calibration.read_text()) if a.temporal_calibration else None
    for campaign in a.campaigns:
        saved=json.loads((campaign/'results.json').read_text())
        for name,item in saved.items():
            base=campaign/name;runtime=json.loads((base/'runtime.json').read_text())
            if runtime['status']!='route_complete' or not runtime['stopped']:raise ValueError('Need complete stopped flight')
            route=load(base/'route_tracking.csv');tracking=route['sim_t'][route['route_t']>0]
            rows.append(dict(campaign=campaign.name,flight=name,original_valid=item['valid'],
                fixed=describe(base/'fixed',tracking,temporal),active=describe(base,tracking,temporal)))
    result=dict(scope=__doc__,rows=rows)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    for row in rows:
        print(row['campaign'],row['flight'],{side:{k:round(row[side]['nonoverlapping_block_tests'][str(k)]['normalized_error_per_dof'],3) for k in [1,5,20]} for side in ['fixed','active']})


if __name__=='__main__':main()
