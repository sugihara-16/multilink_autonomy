#!/usr/bin/env python3
"""Show gaze decisions alongside local sensing quality; no truth-driven control."""
import argparse,json,gzip
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report_routes import load

def summarize(base):
    route=load(base/'route_tracking.csv');positive=route['route_t']>0
    st=route['sim_t'][positive];rt=route['route_t'][positive]
    path=base/'decisions.jsonl';op=open
    if not path.exists():path=path.with_suffix('.jsonl.gz');op=gzip.open
    with op(path,'rt') as f:rows=[json.loads(line) for line in f]
    rows=[r for r in rows if st[0]<=r['stamp_ns']*1e-9<=st[-1]]
    t=np.interp([r['stamp_ns']*1e-9 for r in rows],st,rt)
    angle=np.rad2deg([r['encoder'] for r in rows]);weak=np.array([max(r['lidar_weakness']) for r in rows])
    gate=np.array([r.get('degeneracy_gate_probability',np.nan) for r in rows])
    enabled=np.array([r.get('view_control_enabled',r.get('degeneracy_gate_active',False)) for r in rows])
    active_times=t[enabled]
    out=dict(decisions=len(rows),reasons=dict(Counter(r['reason'] for r in rows)),
        gated_decision_fraction=float(enabled.mean()),
        first_gate_open_s=float(active_times[0]) if len(active_times) else None,
        accumulated_joint_motion_deg=float(np.linalg.norm(np.diff(angle,axis=0),axis=1).sum()),
        max_abs_encoder_deg=np.abs(angle).max(axis=0).tolist(),
        description='Decision fractions count samples, not independent statistical trials. Geometry quality is estimated online; no ground truth enters the policy.')
    fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for i,name in enumerate(['yaw','pitch']):axs[0].plot(t,angle[:,i],label=name,lw=.9)
    axs[0].set_ylabel('Gimbal angle [deg]');axs[0].legend()
    axs[1].plot(t,weak,alpha=.35,label='Instantaneous LiDAR degeneracy',lw=.7)
    if np.isfinite(gate).any():
        axs[1].plot(t,gate,label='Filtered probability',lw=.9)
        axs[1].fill_between(t,0,1,where=enabled,color='orange',alpha=.13,label='Active view enabled')
    axs[1].set_ylim(-.02,1.02);axs[1].set_ylabel('Probability');axs[1].legend()
    axs[2].plot(t,[r.get('image_tracks',np.nan) for r in rows],label='Tracked features',lw=.9)
    axs[2].plot(t,[r.get('mature_tracks',np.nan) for r in rows],label='Mature depth features (when evaluated)',lw=.9)
    if any('tracks_used' in r for r in rows):
        axs[2].plot(t,[r.get('tracks_used',np.nan) for r in rows],label='Features used for image stabilization',lw=.9)
    axs[2].set_ylabel('Feature count');axs[2].set_xlabel('Trajectory time [s]');axs[2].legend()
    for ax in axs:ax.grid(alpha=.3)
    fig.suptitle(base.parent.name+' / '+base.name+' / measured gaze and online diagnostics')
    fig.tight_layout();output=base.parent/'plots';output.mkdir(exist_ok=True)
    fig.savefig(output/(base.name+'_gaze.png'),dpi=120);fig.savefig(output/(base.name+'_gaze.pdf'));plt.close(fig)
    (base/'gaze_summary.json').write_text(json.dumps(out,indent=2)+'\n')
    return out

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args()
    for base in sorted(a.campaign.glob('*_active')):
        runtime=base/'runtime.json'
        if runtime.exists() and json.loads(runtime.read_text()).get('status')=='route_complete':print(base.name,json.dumps(summarize(base)))
