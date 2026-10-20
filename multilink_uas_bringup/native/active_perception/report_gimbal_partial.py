#!/usr/bin/env python3
"""Read complete rows from live CSVs; partial performance is never acceptance evidence."""
import argparse,io,json
from pathlib import Path
import numpy as np
from report_routes import interp


def rows(p):
    text=p.read_text();text=text[:text.rfind('\n')+1]
    return np.atleast_1d(np.genfromtxt(io.StringIO(text),delimiter=',',names=True))


def poses(p):
    d=rows(p);out=np.column_stack([d[k] for k in ['stamp_ns','x','y','z','qx','qy','qz','qw']])
    return out[np.r_[True,np.diff(out[:,0])>0]]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('flight',type=Path);a=parser.parse_args();b=a.flight
    protocol=json.loads((b/'protocol.json').read_text())
    r=rows(b/'route_tracking.csv');m=r['route_t']>0;t=r['sim_t'][m];rt=r['route_t'][m]
    if not len(rt):raise RuntimeError('Tracking has not started')
    keep=np.r_[True,np.diff(rt)>1e-8];t=t[keep];rt=rt[keep]
    gt=poses(b/'truth.csv');inputs={(s,n):poses(p/(n+'.csv')) for s,p in [('active',b),('fixed',b/'fixed')] for n in ['rovio','mimosa']}
    end=min(rt[-1],*[np.interp(e[-1,0]*1e-9,t,rt) for e in [gt,*inputs.values()]])
    grid=np.arange(.1,end-.1,.1)
    if not len(grid):raise RuntimeError('Not enough common tracking coverage yet')
    ts=np.rint(np.interp(grid,rt,t)*1e9);g,gq=interp(gt,ts);output={}
    for side in ['fixed','active']:
        output[side]={}
        for name in ['rovio','mimosa']:
            est=inputs[(side,name)]
            if ts[0]<est[0,0] or ts[-1]>est[-1,0]:raise RuntimeError('Missing common coverage')
            pos,q=interp(est,ts);R=gq[0]*q[0].inv();e=R.apply(pos-pos[0])+g[0]-g
            angle=np.rad2deg((gq.inv()*R*q).magnitude())
            output[side][name]=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(e*e,axis=1)))),rotation_rmse_deg=float(np.sqrt(np.mean(angle*angle))))
            if protocol.get('fixed_gravity_imu',False):
                # Absolute gravity tilt; unlike the pose metric above, there
                # is no alignment rotation. Only valid in fixed-gravity frames.
                truth_up=gq.inv().apply([0.,0.,1.]);up=q.inv().apply([0.,0.,1.])
                tilt=np.rad2deg(np.arctan2(np.linalg.norm(np.cross(up,truth_up),axis=1),np.sum(up*truth_up,axis=1)))
                output[side][name].update(absolute_tilt_rmse_deg=float(np.sqrt(np.mean(tilt*tilt))),last_absolute_tilt_deg=float(tilt[-1]))
    result=dict(scope='Partial route only. Not final accuracy or acceptance.',route_time_s=float(end),
                planned_end_s=protocol['route_end'],metrics=output)
    (b.parent/'partial_progress.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
