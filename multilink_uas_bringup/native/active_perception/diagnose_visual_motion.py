#!/usr/bin/env python3
"""Compare actual sampled velocity and one-second displacement errors offline."""
import argparse,json
from pathlib import Path
import numpy as np
from report_routes import poses,interp,load

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--candidate',default='v35');a=p.parse_args();out={}
    for campaign in sorted(a.root.glob(a.candidate+'-short-seed*')):
        for b in sorted(campaign.glob('*_active')):
            runtime=b/'runtime.json'
            if not runtime.exists() or json.loads(runtime.read_text())['status']!='route_complete':continue
            route=load(b/'route_tracking.csv');inside=route['route_t']>0;rt=route['sim_t'][inside];progress=route['route_t'][inside]
            rt=rt[np.r_[True,np.diff(progress)>1e-8]];truth=poses(b/'truth.csv');result={}
            for mode,d in [('fixed',b/'fixed'),('active',b)]:
                data=load(d/'mimosa/velocity_factors.csv');stamp=data['stamp_ns'];keep=(stamp>=(rt[0]+1)*1e9)&(stamp<=(rt[-1]-.02)*1e9);stamp=stamp[keep]
                reported=np.column_stack([data[k][keep] for k in ['vx','vy','vz']]);C=np.array([data['c'+str(i)+str(j)][keep] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
                gp,gq=interp(truth,stamp);before,_=interp(truth,stamp-20000000);after,_=interp(truth,stamp+20000000)
                velocity_error=reported-gq.inv().apply((after-before)/.04)
                vp,vq=interp(poses(d/'rovio.csv'),stamp);p0,q0=interp(poses(d/'rovio.csv'),stamp-1000000000);g0,gq0=interp(truth,stamp-1000000000)
                displacement_error=q0.inv().apply(vp-p0)-gq0.inv().apply(gp-g0)
                def stats(e):return dict(samples=len(e),rmse_axis=np.sqrt(np.mean(e*e,axis=0)).tolist(),norm_rmse=float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),bias=e.mean(axis=0).tolist())
                result[mode]=dict(velocity_mps=stats(velocity_error),displacement_1s_m=stats(displacement_error),velocity_normalized_error_per_dimension=float(np.einsum('ni,nij,nj->',velocity_error,np.linalg.inv(C),velocity_error)/(3*len(stamp))))
            out[campaign.name+'/'+b.name]=result
    result=dict(scope='Offline truth evaluation of paired actual one-Hz velocity samples versus same-endpoint one-second body displacement. Calibration is unchanged. Adjacent observations and shared IMU are correlated.',results=out)
    (a.root/(a.candidate+'_visual_motion_errors.json')).write_text(json.dumps(result,indent=2)+'\n')
    for name,r in out.items():print(name,{mode:{k:round(v['norm_rmse'],5) for k,v in values.items() if isinstance(v,dict)} for mode,values in r.items()})
if __name__=='__main__':main()
