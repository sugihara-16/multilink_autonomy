#!/usr/bin/env python3
"""Posthoc gravity tilt error; never feeds truth back to an estimator."""
import argparse,json
from pathlib import Path
import numpy as np
from report_routes import poses,interp,load


def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);a=p.parse_args();b=a.flight
    runtime=json.loads((b/'runtime.json').read_text())
    if runtime['status']!='route_complete':raise ValueError('Completed tracking required')
    protocol=json.loads((b/'protocol.json').read_text())
    route=load(b/'route_tracking.csv');m=route['route_t']>0
    rt,t=route['route_t'][m],route['sim_t'][m];keep=np.r_[True,np.diff(rt)>1e-8];rt,t=rt[keep],t[keep]
    grid=np.arange(.1,protocol['route_end']-.001,.1)
    if rt[-1]<grid[-1]:raise ValueError('Incomplete route coverage')
    stamps=np.rint(np.interp(grid,rt,t)*1e9)
    _,gq=interp(poses(b/'truth.csv'),stamps);world_up=np.array([0.,0.,1.])
    truth_up=gq.inv().apply(world_up);results={}
    for mode,directory in [('fixed',b/'fixed'),('active',b)]:
        results[mode]={}
        for estimator in ['rovio','mimosa']:
            _,q=interp(poses(directory/(estimator+'.csv')),stamps)
            up=q.inv().apply(world_up)
            # Both output frames are gravity aligned. Global yaw cancels.
            angle=np.rad2deg(np.arctan2(np.linalg.norm(np.cross(up,truth_up),axis=1),np.sum(up*truth_up,axis=1)))
            def stats(x):return dict(rmse_deg=float(np.sqrt(np.mean(x*x))),p95_deg=float(np.percentile(x,95)),max_deg=float(x.max()))
            results[mode][estimator]=dict(stats(angle),segments=[dict(start_s=float(grid[ii[0]]),end_s=float(grid[ii[-1]]),**stats(angle[ii])) for ii in np.array_split(np.arange(len(angle)),3)])
    result=dict(scope=('Absolute gravity tilt in fixed-gravity frames' if protocol.get('fixed_gravity_imu',False) else 'World-Z tilt only: MIMOSA may estimate a non-Z gravity direction; this is not its gravity-relative error')+'; no initial alignment; invariant to global yaw; tracking only; truth used offline for evaluation.',results=results)
    (b/'absolute_tilt.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
