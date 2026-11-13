#!/usr/bin/env python3
"""Preserve configuration/input checks and observed failure traces without retuning."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'active_perception'))
from compact_record_io import record_path
from report_dragon_fusion_fix import registration
from report_practical_comparison import read_pose
from practical_comparison_metrics import evaluate


def main():
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    root=args.root.resolve();out=root/'diagnostics';out.mkdir(exist_ok=True)
    reference=root.parent/'practical-comparison-20261111-v1/gimbal_seed20262801'
    result=dict(gimbal={},joint_failures={},scope='Checks constrain possible causes; they do not prove why accuracy differs or isolate the trigger of attitude loss.')
    for seed in (20262601,20262602,20262603):
        p=root/f'seed{seed}'/f'gimbal_seed{seed}';checks={}
        for name in ('mimosa.yaml','planner.yaml','cam0.yaml','rovio_runtime.yaml'):
            a,b=[yaml.safe_load((v/name).read_text()) for v in (p,reference)]
            if name=='mimosa.yaml':a.pop('logs_directory',None);b.pop('logs_directory',None)
            checks[name]=a==b
        checks['rovio.info']=(p/'rovio.info').read_text()==(reference/'rovio.info').read_text()
        start=next(e['time'] for e in json.loads((p/'flight_events.json').read_text()) if e['stage']=='active')
        row=result['gimbal'][str(seed)]=dict(adopted_configuration_checks=checks,
            input_audit=json.loads((p/'input_audit.json').read_text()),
            encoder_audit=json.loads((p/'encoder_audit.json').read_text()),
            validation=json.loads((p/'validation.json').read_text()),
            registration=registration(p/'mimosa/drpm_registration.csv',start,start+182),estimators={})
        truth=read_pose(p/'truth.csv')
        for estimator in ('mimosa','rovio'):
            _,error,_,metrics=evaluate(truth,read_pose(p/f'{estimator}.csv'),start,182)
            row['estimators'][estimator]=dict(position_rmse_m=metrics['position_rmse_m'],axis_rmse_m=np.sqrt(np.mean(error**2,axis=0)).tolist(),end_error_xyz_m=error[-1].tolist())
    candidates=list(root.glob('seed*/joint_seed*'))+list(root.glob('startup_retry*/joint_seed*'))
    for p in candidates:
        if not p.is_dir() or not (p/'completion.json').exists():continue
        completion=json.loads((p/'completion.json').read_text())
        if completion['flight_completed']:continue
        row=result['joint_failures'][str(p.relative_to(root))]=dict(completion=completion)
        for name in ('flight_result','flight_status','input_audit'):
            path=p/(name+'.json')
            if path.exists():row[name]=json.loads(path.read_text())
        if not (p/'flight_events.json').exists():
            row['classification']='Startup failure before sensor acquisition; not flight accuracy'
            continue
        start=next(e['time'] for e in json.loads((p/'flight_events.json').read_text()) if e['stage']=='active')
        a=read_pose(p/'truth.csv');a=a[a[:,0]>=start]
        rpy=Rotation.from_quat(a[:,4:8]).as_euler('xyz');abort=row['flight_status']['sim_time'];pre=a[:,0]<=abort
        crossing=np.flatnonzero((np.abs(rpy[:,:2])>1).any(axis=1))
        row.update(classification='In-flight loss of attitude / force landing; precise dynamics trigger not isolated',
                   start_s=start,abort_s=abort,observed_travel_x_m=float(a[-1,1]-a[0,1]),
                   max_abs_roll_pitch_before_abort_rad=np.abs(rpy[pre,:2]).max(axis=0).tolist(),
                   first_tilt_over_1rad_s=float(a[crossing[0],0]) if len(crossing) else None)
        fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
        mask=a[:,0]>=abort-8
        axes[0].plot(a[mask,0]-start,np.rad2deg(rpy[mask,0]),label='roll')
        axes[0].plot(a[mask,0]-start,np.rad2deg(rpy[mask,1]),label='pitch')
        axes[0].set_ylabel('Body tilt [deg]');axes[0].legend()
        axes[1].plot(a[mask,0]-start,a[mask,3]);axes[1].set_ylabel('Height [m]');axes[1].set_xlabel('Tracking time [s]')
        for ax in axes:ax.axvline(abort-start,color='r',linestyle='--');ax.grid(alpha=.3)
        fig.suptitle(p.name+' | observed attitude loss, red: failure reported')
        fig.tight_layout();fig.savefig(out/(p.parent.name+'_'+p.name+'_abort.png'),dpi=130);plt.close(fig)
    (out/'findings.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(gimbal_settings_match=all(all(r['adopted_configuration_checks'].values()) for r in result['gimbal'].values()),joint_failures=list(result['joint_failures'])),indent=2))


if __name__=='__main__':main()
