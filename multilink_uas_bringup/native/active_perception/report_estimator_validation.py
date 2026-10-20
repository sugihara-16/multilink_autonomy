#!/usr/bin/env python3
"""Report paired full flights and independent estimator correctness checks."""
import argparse,json,html
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import poses,interp,load
from audit_recorded_imu import imu_records

def evaluate(flight,name):
 p=json.loads((flight/'protocol.json').read_text());r=load(flight/'route_tracking.csv');m=r['route_t']>0;t=r['sim_t'][m];rt=r['route_t'][m];keep=np.r_[True,np.diff(rt)>1e-8];t=t[keep];rt=rt[keep];grid=np.arange(.1,p['route_end']-.001,.1);assert rt[-1]>=grid[-1]
 ts=np.rint(np.interp(grid,rt,t)*1e9);gt=poses(flight/'truth.csv');est=poses(flight/(name+'.csv'));assert ts[0]>=max(gt[0,0],est[0,0]) and ts[-1]<=min(gt[-1,0],est[-1,0])
 pos,q=interp(est,ts);g,gq=interp(gt,ts);R=gq[0]*q[0].inv();pe=R.apply(pos-pos[0])+g[0]-g;angle=np.rad2deg((gq.inv()*R*q).magnitude());euler=np.rad2deg(((R*q).as_euler('xyz')-gq.as_euler('xyz')+np.pi)%(2*np.pi)-np.pi);err=np.c_[pe,euler];norm=np.linalg.norm(pe,axis=1)
 result=dict(position_rmse_m=float(np.sqrt(np.mean(norm**2))),rotation_rmse_deg=float(np.sqrt(np.mean(angle**2))),position_max_m=float(norm.max()),position_end_m=float(norm[-1]),samples=len(ts),coverage=1.,route_duration_s=p['route_end'],max_output_gap_s=float(np.diff(est[(est[:,0]>=ts[0])&(est[:,0]<=ts[-1]),0]).max()*1e-9))
 return result,grid,err,ts

def main():
 p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);plots=a.output/'plots';plots.mkdir(exist_ok=True);manifest=json.loads((a.campaign/'campaign.json').read_text());results={};validation={};figs=[]
 with PdfPages(plots/'comparison.pdf') as pdf:
  for label in ['run1_tail_a1','run1_tail_b1','run1_prefix']:
   base=a.campaign/(label+'_active')
   assert (base/'runtime.json').exists(),'Incomplete flight: '+str(base)
   runtime=json.loads((base/'runtime.json').read_text());assert runtime['status']=='route_complete'
   paths={'fixed':base/'fixed','active':base};result={};errors={}
   for mode,d in paths.items():
    result[mode]={};errors[mode]={}
    for name in ['rovio','mimosa']:
     result[mode][name],grid,errors[mode][name],ts=evaluate(d,name)
   configs=[json.loads((d/'protocol.json').read_text()) for d in paths.values()]
   corner_limit=bool(configs[0].get('corner_speed_limit',False));assert all(bool(p.get('corner_speed_limit',False))==corner_limit for p in configs)
   assert configs[0]['binary_sha256']==configs[1]['binary_sha256'] and all(not p['drpm_fusion'] for p in configs)
   assert all(p['stationary_mode']=='wait' and p['imu_sampling']=='resolved' for p in configs)
   assert (base/'rovio.info').read_bytes()==(base/'fixed/rovio.info').read_bytes()
   assert (base/'cam0.yaml').read_bytes()==(base/'fixed/cam0.yaml').read_bytes()
   imu=[np.array(list(imu_records(d/'vio_input.bin'))) for d in paths.values()];imu=[u[(u[:,0]>=ts[0])&(u[:,0]<=ts[-1])] for u in imu];assert np.array_equal(imu[0],imu[1]);imu_count=len(imu[0]);del imu
   configs=[yaml.safe_load((d/'mimosa.yaml').read_text()) for d in paths.values()]
   for c in configs:c.pop('logs_directory')
   assert configs[0]==configs[1]
   audits={mode:json.loads((d/'delivered_input_audit.json').read_text()) for mode,d in paths.items()}
   for mode,v in audits.items():
    assert v['passed'] and not v['full_stream_imu_gaps'],(label,mode,v)
    assert v['imu']['missing_samples']==0 and v['image']['max_gap_ns']==50000000,(label,mode,v)
    assert v['extrinsic_rotation_max_error_rad']<1e-5 and v['extrinsic_position_max_error_m']<1e-10,(label,mode,v)
   assert runtime['stopped'],runtime
   nonzero={k:v for k,v in runtime['processes'].items() if v!=0}
   shutdown_audit=None
   if nonzero:
    # Retain the real exit code. A diagnostic-only subscriber can fail after
    # tracking finishes without invalidating estimator inputs or outputs.
    assert nonzero=={'diagnostic_sensors':1},runtime
    shutdown_audit=json.loads((base/'diagnostic_shutdown_audit.json').read_text())
    assert shutdown_audit['all_reference_streams_cover_tracking_end'] and shutdown_audit['reference_streams_used_for_estimation'] is False
   validation[label]=dict(same_physical_flight=True,same_imu_samples=True,tracking_imu_samples=imu_count,same_binary_and_parameters=True,corner_speed_limit=corner_limit,full_coverage=True,full_stream_imu_loss_zero=True,processes_exit_zero=not nonzero,post_tracking_diagnostic_shutdown=shutdown_audit,delivered_inputs=audits)
   results[label]=result
   for estimator in ['rovio','mimosa']:
    fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
    for mode,color in [('fixed','#2474b5'),('active','#df6619')]:
     for i,ax in enumerate([*axs[:,0],*axs[:,1]]):ax.plot(grid,errors[mode][estimator][:,i],color=color,lw=.85,label=mode);ax.set_ylabel(['x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'][i]);ax.grid(alpha=.3)
    axs[0,0].legend();axs[-1,0].set_xlabel('Trajectory time [s]');axs[-1,1].set_xlabel('Trajectory time [s]');fig.suptitle(label+' / '+estimator+' / same physical flight\n1000 Hz IMU, standard initialization, DRPM OFF; no scale fit');fig.tight_layout();name=label+'_'+estimator+'.png';fig.savefig(plots/name,dpi=135);pdf.savefig(fig);plt.close(fig);figs.append(name)
 report=dict(scope='Same physical flight, same IMU, independent fixed/active monocular estimators. One trial per route. Tracking-only first-pose rigid alignment; no scale fitting. Oracle experiments are excluded from this table.',results=results,validation=validation)
 (a.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
 rows=[]
 for label,modes in results.items():
  for mode,r in modes.items():rows.append(f'<tr><td>{label}</td><td>{mode}</td><td>{r["rovio"]["position_rmse_m"]:.3f}</td><td>{r["rovio"]["rotation_rmse_deg"]:.3f}</td><td>{r["mimosa"]["position_rmse_m"]:.3f}</td><td>{r["mimosa"]["rotation_rmse_deg"]:.3f}</td></tr>')
 text='<!doctype html><meta charset="utf-8"><title>Estimator verification and paired gimbal comparison</title><style>body{font-family:sans-serif;max-width:1150px;margin:30px auto;line-height:1.6}img{width:100%}td,th{border:1px solid #ddd;padding:8px}table{border-collapse:collapse}</style><h1>推定器検証と固定・可動の同時比較</h1>'
 if (a.output/'findings.html').exists():text+=(a.output/'findings.html').read_text()
 text+='<h2>3run比較</h2><p>同じ物理飛行・同じIMU入力。DRPM OFF。軌道追従中のみ、開始時の位置・姿勢で整合し、尺度補正なし。各経路1試行。誤差は位置[m]、回転[deg]のRMSE。</p><p>prefixのみ、既知の急な曲がり角で先読み減速を有効化。経路形状・停止上限0.18m・巡航0.5m/sは不変。固定/可動は減速も含め同じ物理飛行を共有します。前の不採用prefixは入力欠落/追従停止として記録を保持し、この表から除外しています。</p><table><tr><th>run</th><th>カメラ</th><th>ROVIO位置</th><th>ROVIO回転</th><th>MIMOSA+ROVIO位置</th><th>MIMOSA+ROVIO回転</th></tr>'+''.join(rows)+'</table><p><a href="plots/comparison.pdf">六軸比較PDF</a></p>'+''.join(f'<img src="plots/{f}">' for f in figs)
 if any(v['post_tracking_diagnostic_shutdown'] for v in validation.values()):
  text+='<p>prefixの診断専用記録器は評価区間終了後の停止処理中に終了コード1となりました。参照IMUの記録が区間末尾まであることを別途確認しています。推定器へ与えるセンサ入力・全推定器は正常で、欠落監査も合格しています。実際の終了コードと照合結果はcomparison.jsonに残し、今後の記録器の停止処理を修正しました。</p>'
 (a.output/'index.html').write_text(text);print(json.dumps(results,indent=2))
if __name__=='__main__':main()
