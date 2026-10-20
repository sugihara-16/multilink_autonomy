#!/usr/bin/env python3
"""Inspect old fixed-view calibration by observed quality; never change weights.

Bins are descriptive and temporally correlated. They are not independent tests
and cannot justify a new adaptive covariance law without separate validation.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from report_routes import load, poses, interp


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args()
    calibration=json.loads((a.root/'native_fusion_velocity_gravity_calibration.json').read_text())
    results={}
    for source in calibration['sources']:
        fixed=Path(source['path']);name=fixed.parent.name.removesuffix('_active')
        data=load(a.root/'rovio-uncertainty-calibration'/(name+'_25')/'vio.csv')
        route=load(fixed/'route_tracking.csv');tracking=route['sim_t'][route['route_t']>0]
        gt=poses(fixed/'truth.csv');t=data['stamp_ns']
        keep=(t>=tracking[0]*1e9)&(t<=tracking[-1]*1e9)&(t>gt[0,0]+2e7)&(t<gt[-1,0]-2e7)
        t=t[keep];C=np.array([data['velocity_cov_'+str(i)+str(j)][keep]
                              for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
        v=np.column_stack([data['v'+c][keep] for c in 'xyz']);_,q=interp(gt,t)
        before,_=interp(gt,t-2e7);after,_=interp(gt,t+2e7)
        error=v-q.inv().apply((after-before)/.04)
        eigen,U=np.linalg.eigh(C)
        if eigen.min()<=0:raise ValueError('Nonpositive published covariance')
        invroot=(U/np.sqrt(eigen[:,None,:]))@U.transpose(0,2,1)
        white=np.einsum('nij,nj->ni',invroot,error)
        observed={'depth_m':(data['median_depth'][keep],[0.,5.,10.,20.,40.,np.inf]),
                  'tracked_features':(data['features'][keep],[0.,4.,8.,16.,np.inf])}
        strata={}
        for kind,(values,bins) in observed.items():
            rows=[]
            for lo,hi in zip(bins[:-1],bins[1:]):
                select=(values>=lo)&(values<hi);n=int(select.sum())
                row=dict(lower=lo,upper=hi if np.isfinite(hi) else None,samples=n)
                if n:
                    e=error[select];w=white[select];M=w.T@w/n
                    row.update(velocity_rmse_mps=float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),
                        velocity_bias_mps=e.mean(axis=0).tolist(),
                        published_covariance_median_trace=float(np.median(np.trace(C[select],axis1=1,axis2=2))),
                        normalized_error_per_dof=float(np.trace(M)/3.),
                        maximum_normalized_second_moment_eigenvalue=float(np.linalg.eigvalsh(M).max()))
                rows.append(row)
            strata[kind]=rows
        results[name]=strata
    result=dict(scope=__doc__,calibration_unchanged=True,evaluation_flights_used=False,results=results)
    (a.root/'velocity_calibration_quality_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
    for name,groups in results.items():
        print(name)
        for kind,rows in groups.items():
            print(kind,[(r['lower'],r['upper'],r['samples'],round(r.get('maximum_normalized_second_moment_eigenvalue',0),3)) for r in rows])


if __name__=='__main__':main()
