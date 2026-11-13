#!/usr/bin/env python3
"""Full-flight and predeclared interior-zone metrics, with actual camera angles."""
import argparse
import gzip
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parents[1]/'active_perception'
sys.path.insert(0,str(HERE))
sys.path.insert(0,str(HERE.parent))
from practical_comparison_metrics import evaluate
from build_switching_tunnel import SEGMENTS
from dragon_articulated import Kinematics


def csv(path):
    if not path.exists():path=path.with_name(path.name+'.gz')
    return np.loadtxt(path,delimiter=',',skiprows=1,ndmin=2)


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args()
    root=a.root;out=root/'report';summary=json.loads((out/'summary.json').read_text())
    arms=['unipilot','gimbal','joint'];seeds=summary['protocol']['run_seeds']
    extras={}
    for condition,record in summary['conditions'].items():
        extra=extras[condition]={}
        fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
        for i,seed in enumerate(seeds):
            for arm in arms:
                key=f'{arm}_seed{seed}';trial=record['trials'][key];directory=root/condition/key
                if not trial['full_interval_available']:continue
                start=trial['tracking_start_s'];truth=csv(directory/'truth.csv')[:,:8];truth[:,0]*=1e-9
                result=extra[key]={}
                for estimator in ['mimosa','rovio']:
                    estimate=csv(directory/f'{estimator}.csv')[:,:8];estimate[:,0]*=1e-9
                    t,error,_,metrics=evaluate(truth,estimate,start,262.)
                    # Ten-second displacement differences use the same initial
                    # orientation alignment; never refit an individual zone.
                    delta=error[100:,0]-error[:-100,0]
                    zones=[]
                    for lo,hi in SEGMENTS:
                        mask=(t>=lo)&(t<=hi)
                        pair=(t[:-100]>=lo)&(t[100:]<=hi)
                        zones.append(dict(time_s=[lo,hi],position_rmse_m=float(np.sqrt(np.mean(np.sum(error[mask]**2,axis=1)))),axial_10s_displacement_rms_m=float(np.sqrt(np.mean(delta[pair]**2)))))
                    result[estimator]=dict(full=metrics,zones=zones,axial_10s_displacement_rms_m=float(np.sqrt(np.mean(delta**2))))
                if arm == 'joint':
                    measured=csv(directory/'joint_measurements.csv')
                    measured=measured[(measured[:,0]>=start)&(measured[:,0]<=start+262)][::10]
                    kin=Kinematics(directory/'dragon_slam.urdf')
                    direction=np.array([kin.camera(q[1:])[:3,2] for q in measured])
                    enc=np.column_stack([measured[:,0],np.arctan2(direction[:,1],direction[:,0]),
                                         np.arctan2(-direction[:,2],np.hypot(direction[:,0],direction[:,1]))])
                    result['view_angle_source']='Measured joints at about 10Hz + recorded URDF forward kinematics; camera optical axis relative to FC IMU'
                else:
                    enc=csv(directory/'encoders.csv');enc=enc[(enc[:,0]>=start)&(enc[:,0]<=start+262)]
                    result['view_angle_source']='Measured gimbal encoders relative to FC IMU'
                # A crossing of +/-180 degrees is not a physical full turn.
                enc[:,1]=np.unwrap(enc[:,1])
                result['yaw_display']='Continuous unwrapped azimuth'
                step=max(1,len(enc)//3000)
                for j in (0,1):axes[i,j].plot(enc[::step,0]-start,np.rad2deg(enc[::step,j+1]),label=arm)
                result['actual_view_span_deg']=np.rad2deg(np.ptp(enc[:,1:3],axis=0)).tolist()
                decision=directory/('joint_decisions.jsonl' if arm=='joint' else 'decisions.jsonl')
                stream=decision.open() if decision.exists() else gzip.open(str(decision)+'.gz','rt')
                with stream:decisions=[json.loads(line) for line in stream]
                active=[d for d in decisions if start <= (d['time'] if arm=='joint' else d['stamp_ns']*1e-9) <= start+262]
                reasons={}
                for d in active:reasons[d['reason']]=reasons.get(d['reason'],0)+1
                result['view_decision_reasons']=reasons
            for j in (0,1):
                axes[i,j].set_ylabel(f'{seed}: '+('yaw' if j==0 else 'pitch')+' [deg]')
                axes[i,j].grid(alpha=.25)
                for boundary in (9,73,137,201):axes[i,j].axvline(boundary,color='k',alpha=.15)
        axes[0,0].legend()
        for ax in axes[-1]:ax.set_xlabel('Tracking time [s]')
        fig.suptitle(condition+' | actual camera direction, three runs')
        fig.tight_layout();fig.savefig(out/f'{condition}_view_angles.png',dpi=130);plt.close(fig)
    (out/'zone_and_view_analysis.json').write_text(json.dumps(dict(zones_s=SEGMENTS,zone_selection='Historical fixed interior windows, chosen before outcomes; exclude transitions.',method='Initial flight alignment only; 10-second axial displacement error is the change of aligned X error over 10 seconds.',conditions=extras),indent=2))
    fig,ax=plt.subplots(figsize=(11,5))
    colors=['#377eb8','#ff7f00','#4daf4a']
    for i,(condition,record) in enumerate(summary['conditions'].items()):
        for j,arm in enumerate(arms):
            vals=record['fusion_position_rmse_m'][arm]['values'];x=i+(j-1)*.22
            valid=[v for v in vals if v is not None]
            ax.scatter(x+np.linspace(-.04,.04,len(valid)),valid,color=colors[j],label=arm if i==0 else None)
            if len(valid)==3:ax.plot([x-.075,x+.075],[np.mean(valid)]*2,color=colors[j],linewidth=3)
    ax.set_xticks(range(len(extras)),list(extras));ax.set_ylabel('Fusion position RMSE [m]')
    ax.set_title('130 m | dots: individual runs; bars: means of all three')
    ax.grid(axis='y',alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/'individual_runs.png',dpi=150);plt.close(fig)
    page=(out/'index.html').read_text()
    marker='<!-- switching details -->'
    page=page.split(marker)[0]+marker+'<h2>全試行と視線の動き</h2><img src="individual_runs.png"><p><a href="zone_and_view_analysis.json">区間別誤差・10秒移動誤差・視線の動作記録</a>。区間は以前の試験で定めた4区間を使用し、今回の結果を見て選んでいません。</p>'
    for c in extras:page+=f'<h3>{c}</h3><img src="{c}_view_angles.png">'
    (out/'index.html').write_text(page)


if __name__=='__main__':main()
