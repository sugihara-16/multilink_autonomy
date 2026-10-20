#!/usr/bin/env python3
"""Reproducible plots and audit table for controlled UniPilot ROVIO diagnosis."""
import argparse,json,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import poses,interp,load
p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;out=root/'plots';out.mkdir(exist_ok=True)
labels={'online_check':'Native baseline','image_clean':'No image noise','image_wide':'Wider clean image','image_far':'Far clip 60m, clean','oracle_both_no_static':'Clean IMU, no static mean (oracle)','velocity_noise_dragon':'Higher velocity process uncertainty','depth_no_median':'Disable depth-median seeding','rate_clean_200':'Clean IMU 200Hz','rate_clean_1000':'Clean IMU 1000Hz','rate_clean_200_average':'Clean IMU 200Hz, interval average'}
allrows=[];scale={};sections=[]
with PdfPages(out/'diagnosis.pdf') as pdf:
 for label in ('a1','b1'):
  d=json.loads((root/f'{label}_controlled/report.json').read_text());flight=Path(d['flight']);table=[]
  for name,r in d['runs'].items():
   row=dict(flight=label,mode=name,position_rmse_m=r['position_rmse_m'],rotation_rmse_deg=r['rotation_rmse_deg']);allrows.append(row);table.append('<tr>'+''.join(f'<td>{html.escape(str(v))}</td>' for v in [labels.get(name,name),f"{row['position_rmse_m']:.3f}",f"{row['rotation_rmse_deg']:.3f}"] )+'</tr>')
  modes=['online_check','image_clean','oracle_both_no_static','velocity_noise_dragon','depth_no_median'];fig,axs=plt.subplots(3,2,figsize=(14,10),sharex=True)
  for name in modes:
   n=np.load(root/f'{label}_controlled'/name/'errors.npz')
   for j,ax in enumerate(axs.flat):ax.plot(n['route_t'],n['error'][:,j],label=labels[name],lw=1)
  for j,ax in enumerate(axs.flat):ax.set_ylabel(['x (m)','y (m)','z (m)','roll (deg)','pitch (deg)','yaw (deg)'][j]);ax.grid(alpha=.3)
  # Arrange in row-major order, labels explicitly identify each component.
  axs[0,0].legend(fontsize=8);axs[2,0].set_xlabel('Tracking time (s)');axs[2,1].set_xlabel('Tracking time (s)');fig.suptitle(f'{label}: controlled replay, initial-pose alignment, no scale fitting');fig.tight_layout();fig.savefig(out/f'{label}_six_axes.png',dpi=140);pdf.savefig(fig);plt.close(fig)
  n=np.load(root/f'{label}_controlled/online_check/errors.npz');g,_=interp(poses(flight/'truth.csv'),n['stamp_ns']);est=n['error'][:,:3]+g;v=est-est[0];w=g-g[0];s=np.sum(v*w)/np.sum(v*v)
  scale[label]=dict(posthoc_scale_to_truth=float(s),posthoc_scaled_rmse_m=float(np.sqrt(np.mean(np.sum((s*v-w)**2,axis=1)))),path_length_ratio=float(np.linalg.norm(np.diff(est,axis=0),axis=1).sum()/np.linalg.norm(np.diff(g,axis=0),axis=1).sum()),scope='Posthoc diagnostic only; no scale fitting in primary accuracy metrics.')
  feat=json.loads((root/f'{label}_features/geometry/feature_geometry.json').read_text());rows=[r for r in feat['tracks'] if r['reprojection_rms_px']<2 and r['positive_fraction']>.9];route=load(flight/'route_tracking.csv');mask=route['route_t']>0;ts=route['sim_t'][mask];rt=route['route_t'][mask];ft=np.interp([r['start_sim_s'] for r in rows],ts,rt)
  fig,axs=plt.subplots(2,2,figsize=(13,9));axs[0,0].plot(w[:,0],w[:,1],label='Truth');axs[0,0].plot(v[:,0],v[:,1],label='ROVIO');axs[0,0].plot(s*v[:,0],s*v[:,1],label='ROVIO with posthoc scale (diagnostic)');axs[0,0].axis('equal');axs[0,0].legend(fontsize=8);axs[0,0].set(xlabel='x (m)',ylabel='y (m)')
  axs[0,1].scatter(ft,[r['median_depth_ratio'] for r in rows],s=8,alpha=.6);axs[0,1].axhline(1,color='k',ls='--');axs[0,1].set(xlabel='Feature start: tracking time (s)',ylabel='Estimated / triangulated range')
  diag=load(root/f'{label}_controlled/online_check/vio.csv');tt=np.interp(diag['stamp_ns']*1e-9,ts,rt);m=(diag['stamp_ns']>=ts[0]*1e9)&(diag['stamp_ns']<=ts[-1]*1e9)
  axs[1,0].plot(tt[m],diag['features'][m]);axs[1,0].set(xlabel='Tracking time (s)',ylabel='Features in filter')
  axs[1,1].plot(n['route_t'],n['position_error']);axs[1,1].set(xlabel='Tracking time (s)',ylabel='Position error (m)')
  for ax in axs.flat:ax.grid(alpha=.3)
  fig.suptitle(f'{label}: distance / motion scale diagnosis; GT used only for evaluation');fig.tight_layout();fig.savefig(out/f'{label}_mechanism.png',dpi=140);pdf.savefig(fig);plt.close(fig)
  sections.append(f'<h2>{label}</h2><table><tr><th>Condition</th><th>Position RMSE (m)</th><th>Rotation RMSE (deg)</th></tr>'+''.join(table)+f'</table><img src="plots/{label}_mechanism.png"><img src="plots/{label}_six_axes.png">')
 rate=root/'a1_rate/report.json'
 if rate.exists():
  d=json.loads(rate.read_text());fig,axs=plt.subplots(2,1,figsize=(12,8),sharex=True)
  rate_rows=[]
  for name,r in d['runs'].items():
   rate_rows.append('<tr><td>'+html.escape(labels.get(name,name))+f"</td><td>{r['position_rmse_m']:.3f}</td><td>{r['rotation_rmse_deg']:.3f}</td></tr>")
   if name not in ('rate_clean_200','rate_clean_1000','rate_clean_200_average'):
    allrows.append(dict(flight='a1_rate',mode=name,position_rmse_m=r['position_rmse_m'],rotation_rmse_deg=r['rotation_rmse_deg']));continue
   n=np.load(rate.parent/name/'errors.npz');axs[0].plot(n['route_t'],n['position_error'],label=labels.get(name,name));axs[1].plot(n['route_t'],n['rotation_error'],label=labels.get(name,name));allrows.append(dict(flight='a1_rate',mode=name,position_rmse_m=r['position_rmse_m'],rotation_rmse_deg=r['rotation_rmse_deg']))
  axs[0].set_ylabel('Position error (m)');axs[1].set_ylabel('Orientation error (deg)');axs[1].set_xlabel('Tracking time (s)');axs[0].legend();[ax.grid(alpha=.3) for ax in axs];fig.tight_layout();fig.savefig(out/'rate.png',dpi=140);pdf.savefig(fig);plt.close(fig);sections.append('<h2>IMU sampling rate: same physical flight and images</h2><table><tr><th>Condition</th><th>Position RMSE (m)</th><th>Rotation RMSE (deg)</th></tr>'+''.join(rate_rows)+'</table><img src="plots/rate.png">')
(root/'summary.json').write_text(json.dumps(dict(metrics=allrows,scale_diagnostics=scale),indent=2)+'\n')
intro=(root/'findings.html').read_text() if (root/'findings.html').exists() else '<p>Analysis in progress.</p>'
(root/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>UniPilot ROVIO drift diagnosis</title><style>body{font:16px sans-serif;max-width:1250px;margin:32px auto;line-height:1.6}table{border-collapse:collapse}td,th{padding:5px 12px;border:1px solid #bbb}img{width:100%}code{background:#eee}</style><h1>UniPilot ROVIO drift diagnosis</h1>'+intro+'<p>Tracking only; same 0.1 s timestamp grid within each flight; first-pose rigid alignment; no scale fit in accuracy tables. Each flight is one realization. Oracle conditions use noiseless references/truth-assisted bias correction solely to diagnose causes, not deployable performance. All axes show full range.</p><p><a href="plots/diagnosis.pdf">PDF</a> | <a href="summary.json">Numerical summary</a></p>'+''.join(sections))
print(root/'index.html')
