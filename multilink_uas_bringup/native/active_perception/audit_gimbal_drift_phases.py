#!/usr/bin/env python3
"""Post-flight relative errors by body turn and gaze; truth never controls gaze."""
import argparse,json
from pathlib import Path
import numpy as np
from report_routes import load,poses,interp

def audit(base):
    route=load(base/'route_tracking.csv');ok=route['route_t']>0
    start=route['sim_t'][ok][0];end=route['sim_t'][ok][-1]
    times=np.arange(start+.1,end-1.01,.1);stamps=np.rint(times*1e9)
    gp,gq=interp(poses(base/'truth.csv'),stamps)
    gn,gr=interp(poses(base/'truth.csv'),stamps+1000000000)
    translation=gq.inv().apply(gn-gp);rotation=gq.inv()*gr
    rate=np.rad2deg(rotation.magnitude())
    enc=load(base/'encoders.csv')
    pitch=np.rad2deg(np.interp(times,enc['t'],enc['pitch']))
    masks={'all':np.ones(len(times),bool),'body_rotation_lt3deg_s':rate<3,
           'body_rotation_3to10deg_s':(rate>=3)&(rate<10),'body_rotation_ge10deg_s':rate>=10,
           'gaze_pitch_abs_lt30deg':np.abs(pitch)<30,'gaze_pitch_abs_ge30deg':np.abs(pitch)>=30}
    result={'scope':'1-second relative pose errors sampled every 0.1 second within tracking. Overlapping windows are not independent samples; descriptive statistics only. Truth is used only in this report.',
            'duration_s':float(end-start),'groups':{}}
    for mode,path in [('fixed',base/'fixed'),('active',base)]:
        result[mode]={}
        for estimator in ('rovio','mimosa'):
            raw=poses(path/(estimator+'.csv'));p,q=interp(raw,stamps);pn,qn=interp(raw,stamps+1000000000)
            pos=np.linalg.norm(q.inv().apply(pn-p)-translation,axis=1)
            angle=np.rad2deg((rotation.inv()*q.inv()*qn).magnitude())
            result[mode][estimator]={}
            for name,mask in masks.items():
                result['groups'][name]=int(mask.sum())
                if not mask.any():continue
                result[mode][estimator][name]={'translation_1s_rmse_m':float(np.sqrt(np.mean(pos[mask]**2))),
                    'rotation_1s_rmse_deg':float(np.sqrt(np.mean(angle[mask]**2))),
                    'translation_1s_p95_m':float(np.percentile(pos[mask],95))}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);a=p.parse_args()
    result=audit(a.flight);(a.flight/'drift_phases.json').write_text(json.dumps(result,indent=2)+'\n')
    for mode in ('fixed','active'):
        for name,groups in result[mode].items():print(mode,name,{k:(round(v['translation_1s_rmse_m'],3),round(v['rotation_1s_rmse_deg'],3),result['groups'][k]) for k,v in groups.items()})
