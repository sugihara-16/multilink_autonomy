#!/usr/bin/env python3
"""Offline world-vertical heading increments; no feedback or scale fit."""
import argparse, json
from pathlib import Path
import numpy as np
from report_routes import poses, interp, load

def heading(q):
    x=q.apply(np.tile([1.,0.,0.],(len(q),1)))
    if np.min(np.linalg.norm(x[:,:2],axis=1))<.5:raise ValueError('Body forward nearly vertical')
    return np.arctan2(x[:,1],x[:,0])
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def audit(flight):
    route=load(flight/'route_tracking.csv');rt=route['sim_t'][route['route_t']>0]
    gt=poses(flight/'truth.csv');result={}
    for mode,directory in [('fixed',flight/'fixed'),('active',flight)]:
        result[mode]={}
        for estimator in ['rovio','mimosa']:
            a=poses(directory/(estimator+'.csv'));results={}
            for lag in [.5,1.]:
                t=np.arange(max(rt[0],a[0,0]/1e9,gt[0,0]/1e9)+lag,min(rt[-1],a[-1,0]/1e9,gt[-1,0]/1e9),.1)*1e9
                _,q=interp(a,t);_,q0=interp(a,t-lag*1e9);_,g=interp(gt,t);_,g0=interp(gt,t-lag*1e9)
                e=wrap(heading(q)-heading(q0)-heading(g)+heading(g0))
                results[str(lag)]=dict(samples=len(t),rmse_deg=float(np.rad2deg(np.sqrt(np.mean(e*e)))),bias_deg=float(np.rad2deg(np.mean(e))))
            result[mode][estimator]=results
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('flights',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    data={str(f):audit(f) for f in a.flights};a.output.write_text(json.dumps(data,indent=2)+'\n')
    for f,r in data.items():print(f,{mode:{e:v['1.0']['rmse_deg'] for e,v in s.items()} for mode,s in r.items()})
if __name__=='__main__':main()
