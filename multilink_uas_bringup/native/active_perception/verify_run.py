#!/usr/bin/env python3
"""Summarize an online smoke test; this is not a trajectory accuracy campaign."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from report_routes import csv_path

def evaluate(folder):
    folder=Path(folder);protocol=json.loads((folder/'protocol.json').read_text())
    decisions=[json.loads(l) for l in (folder/'decisions.jsonl').read_text().splitlines()]
    enc=np.genfromtxt(csv_path(folder/'encoders.csv'),delimiter=',',names=True)
    t=enc['t'];angles=np.column_stack([enc['yaw'],enc['pitch']]);dt=np.diff(t)
    speed=np.linalg.norm(np.diff(angles,axis=0),axis=1)[dt>0]/dt[dt>0]
    truth=np.genfromtxt(folder/'truth.csv',delimiter=',',names=True)
    truth=np.array([truth[k] for k in truth.dtype.names]).T
    truth=truth[np.r_[True,np.diff(truth[:,0])>0]]
    result=dict(mode=protocol['mode'],decisions=len(decisions),evaluated=sum(d['reason']=='evaluated' for d in decisions),
        maximum_map_points=max(d['map_points'] for d in decisions),
        planning_ms=dict(zip(['median','p95','max'],map(float,np.percentile([d['compute_ms'] for d in decisions],[50,95,100])))),
        encoder_span_deg=np.rad2deg(np.ptp(angles,axis=0)).tolist(),maximum_encoder_speed_deg_s=float(np.rad2deg(speed.max())),
        scope='One stationary takeoff/hover smoke test; initial pose alignment only, no performance advantage claim',estimators={})
    for name in ('rovio','mimosa'):
        raw=np.genfromtxt(folder/(name+'.csv'),delimiter=',',names=True)
        a=np.array([raw[k] for k in raw.dtype.names]).T
        a=a[(a[:,0]>=max(truth[0,0],5e9))&(a[:,0]<=truth[-1,0])]
        if len(a)<100 or not np.isfinite(a).all():raise AssertionError(f'{name} insufficient/nonfinite outputs')
        gt=np.column_stack([np.interp(a[:,0],truth[:,0],truth[:,i]) for i in range(1,4)])
        qr=Slerp(truth[:,0]*1e-9,Rotation.from_quat(truth[:,4:8]))(a[:,0]*1e-9)
        qe=Rotation.from_quat(a[:,4:8]);align=qr[0]*qe[0].inv()
        pos=align.apply(a[:,1:4]-a[0,1:4])+gt[0]
        error=np.linalg.norm(pos-gt,axis=1);rot=np.rad2deg((qr.inv()*(align*qe)).magnitude())
        result['estimators'][name]=dict(count=len(a),duration_s=float(np.ptp(a[:,0])*1e-9),
            max_gap_s=float(np.diff(a[:,0]).max()*1e-9),position_rmse_m=float(np.sqrt(np.mean(error**2))),
            position_max_m=float(error.max()),rotation_rmse_deg=float(np.sqrt(np.mean(rot**2))),rotation_max_deg=float(rot.max()))
    flight=json.loads((folder/'flight_slam_result.json').read_text())
    result['takeoff_and_30s_hover']=flight['takeoff_confirmed'] and flight['failure'] is None and flight['hover_duration_s']>=29.9
    assert result['takeoff_and_30s_hover']
    assert result['maximum_encoder_speed_deg_s']<=20.01
    if protocol['mode']=='active':assert result['evaluated']>10 and max(result['encoder_span_deg'])>3
    else:assert max(result['encoder_span_deg'])<.1
    for m in result['estimators'].values():assert m['max_gap_s']<.3 and m['position_max_m']<1
    (folder/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folders',nargs='+',type=Path);a=p.parse_args()
    for folder in a.folders:evaluate(folder)
