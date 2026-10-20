#!/usr/bin/env python3
"""Offline audit of the actual VIO gravity directions fed into fusion.

World vertical is shared with the gravity-aligned simulator. No yaw alignment,
position alignment, scale correction or estimator feedback is used. These
observations are correlated; normalized errors are descriptive, not tests
with independent samples. Partial flights are explicitly labeled.
"""
import argparse, json
from pathlib import Path
import numpy as np
from report_routes import poses, interp, load

def audit(flight):
    route=load(flight/'route_tracking.csv')
    rt=route['sim_t'][route['route_t']>0]
    truth=poses(flight/'truth.csv')
    runtime=json.loads((flight/'runtime.json').read_text())
    result=dict(complete=runtime['status']=='route_complete', modes={})
    for mode, directory in [('fixed',flight/'fixed'),('active',flight)]:
        d=load(directory/'mimosa/tilt_factors.csv')
        t=d['stamp_ns'];keep=(t>=rt[0]*1e9)&(t<=rt[-1]*1e9)
        t=t[keep];_,q=interp(truth,t)
        actual=q.inv().apply(np.tile([0.,0.,1.],(len(t),1)))
        reported=np.column_stack([d['up_'+axis][keep] for axis in 'xyz'])
        angle=np.arctan2(np.linalg.norm(np.cross(actual,reported),axis=1), np.einsum('ni,ni->n',actual,reported))
        _,estimated_q=interp(poses(directory/'mimosa.csv'),t)
        estimated=estimated_q.inv().apply(np.tile([0.,0.,1.],(len(t),1)))
        fusion_angle=np.arctan2(np.linalg.norm(np.cross(actual,estimated),axis=1), np.einsum('ni,ni->n',actual,estimated))
        result['modes'][mode]=dict(samples=len(t), vio_tilt_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(angle**2)))),
            vio_tilt_p95_deg=float(np.rad2deg(np.percentile(angle,95))),
            fused_tilt_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(fusion_angle**2)))),
            normalized_tilt_error_per_dimension=float(np.mean((angle/d['sigma_rad'][keep])**2)/2),
            source='actual sparse up-direction observation used by tilt factor; truth only offline')
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('flights',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    data={str(f):audit(f) for f in a.flights}
    a.output.write_text(json.dumps(data,indent=2)+'\n');print(json.dumps(data,indent=2))
if __name__=='__main__':main()
