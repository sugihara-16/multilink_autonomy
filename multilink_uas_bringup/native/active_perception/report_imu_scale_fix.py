#!/usr/bin/env python3
"""Report correctness fixes and controlled scale checks; never fit scale in metrics."""
import argparse,json,html,struct,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import poses,interp,load
p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;plots=root/'plots';plots.mkdir(exist_ok=True);summary={};sections=[]
with PdfPages(plots/'comparison.pdf') as pdf:
 for label in ['a1','b1']:
  init=json.loads((root/f'{label}_initialization/report.json').read_text());initial=json.loads((root/label/'report.json').read_text());flight=Path(init['flight']);baseline=root/f'{label}_initialization/online_check';fixed=root/f'{label}_initialization/wait_only';assert (baseline/'rovio.info').read_bytes()==(fixed/'rovio.info').read_bytes();assert init['runs']['online_check']['online_byte_identical'];x=load(baseline/'vio.csv');y=load(fixed/'vio.csv');assert x['stamp_ns'][0]==y['stamp_ns'][0]
  modes=[('1000Hz + old stationary initialization',baseline),('1000Hz + upstream initialization after same delay',fixed),('1000Hz + no median depth seeding (not adopted)',root/label/'depth_no_median')]
  fig,axs=plt.subplots(3,2,figsize=(14,10),sharex=True)
  for name,path in modes:
   d=np.load(path/'errors.npz')
   for i,ax in enumerate(axs.flat):ax.plot(d['route_t'],d['error'][:,i],label=name,lw=1)
  for i,ax in enumerate(axs.flat):ax.set_ylabel(['x (m)','y (m)','z (m)','roll (deg)','pitch (deg)','yaw (deg)'][i]);ax.grid(alpha=.3)
  axs[0,0].legend(fontsize=7);axs[2,0].set_xlabel('Tracking time (s)');axs[2,1].set_xlabel('Tracking time (s)');fig.suptitle(label+': identical sensor input; no scale fitting');fig.tight_layout();pdf.savefig(fig);fig.savefig(plots/f'{label}_errors.png',dpi=140);plt.close(fig)
  geometry=json.loads((root/f'{label}_initialization/geometry/feature_geometry.json').read_text());good=[v for v in geometry['tracks'] if v['reprojection_rms_px']<2 and v['positive_fraction']>.9];route=load(flight/'route_tracking.csv');mask=route['route_t']>0;times=np.interp([v['start_sim_s'] for v in good],route['sim_t'][mask],route['route_t'][mask]);ratios=[v['median_depth_ratio'] for v in good]
  d=np.load(fixed/'errors.npz');gt,_=interp(poses(flight/'truth.csv'),d['stamp_ns']);estimate=gt+d['error'][:,:3];fig,axs=plt.subplots(1,2,figsize=(13,5));axs[0].plot(gt[:,0]-gt[0,0],gt[:,1]-gt[0,1],label='Truth');axs[0].plot(estimate[:,0]-estimate[0,0],estimate[:,1]-estimate[0,1],label='ROVIO, adopted correction');axs[0].axis('equal');axs[0].legend();axs[0].set(xlabel='x (m)',ylabel='y (m)');axs[1].scatter(times,ratios,s=8);axs[1].axhline(1,c='k',ls='--');axs[1].set(xlabel='Feature start: tracking time (s)',ylabel='Estimated / GT-triangulated range');[ax.grid(alpha=.3) for ax in axs];fig.tight_layout();pdf.savefig(fig);fig.savefig(plots/f'{label}_scale.png',dpi=140);plt.close(fig)
  rows=[]
  for name,v in [('1000Hz・旧初期化',init['runs']['online_check']),('1000Hz・修正後初期化（採用）',init['runs']['wait_only']),('初期化待ちを完全に省略（不採用）',initial['runs']['no_static_init']),('旧初期化・距離中央値の引継ぎ停止（不採用）',initial['runs']['depth_no_median'])]:rows.append(f'<tr><td>{name}</td><td>{v["position_rmse_m"]:.3f}</td><td>{v["rotation_rmse_deg"]:.3f}</td></tr>')
  summary[label]=dict(adopted=init['runs']['wait_only'],imu_only=init['runs']['online_check'],initialization_input_identical=True,initialization_config_identical=True,initialization_first_output_stamp_equal=True,feature_depth_ratio_percentiles=geometry['depth_ratio_percentiles'],feature_consistent_tracks=geometry['consistent_tracks'],feature_eligible_tracks=geometry['triangulatable_tracks'],input_sha256=init['input_sha256'],binary_sha256=init['binary_sha256'])
  sections.append(f'<h2>{label}</h2><table><tr><th>条件</th><th>位置RMSE [m]</th><th>回転RMSE [deg]</th></tr>'+''.join(rows)+f'</table><img src="plots/{label}_errors.png"><img src="plots/{label}_scale.png">')
(root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
intro=(root/'findings.html').read_text() if (root/'findings.html').exists() else ''
(root/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>UniPilot IMU and scale checks</title><style>body{font:16px sans-serif;max-width:1200px;margin:30px auto;line-height:1.7}img{width:100%}td,th{border:1px solid #aaa;padding:6px 12px}table{border-collapse:collapse}</style><h1>UniPilot IMU修正と距離・移動量の尺度確認</h1>'+intro+'<p>追従区間だけを0.1秒間隔で評価。初回姿勢による剛体変換のみ、縮尺補正なし。条件間で入力・初期化時刻を共有。各物理飛行は1回。ノイズなし参照や真値によるbias補正は使用していない。</p><p><a href="plots/comparison.pdf">PDF</a> / <a href="summary.json">数値JSON</a></p>'+''.join(sections))
print(root/'index.html')
