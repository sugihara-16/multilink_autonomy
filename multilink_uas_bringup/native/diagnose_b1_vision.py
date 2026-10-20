#!/usr/bin/env python3
"""Read-only phase and visual-update diagnosis from saved b1 outputs."""
import json,re,csv
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_fast_camera_comparison import read,errors
from plot_livo_comparison import interpolate
from calibrate_rovio_relative import relative_error
ROOT=Path(__file__).resolve().parent;B=ROOT/'logs/zab-offline';OUT=B/'b1_vision_diagnosis';OUT.mkdir(exist_ok=True);L='run1_tail_b1';SRC=ROOT/'logs/20261002-233545-95177';F=B/'fast_official_raw'/L
start=61800000000;end=334200000000
truth=read(B/'drpm_three_camera/inputs'/L/'truth.csv')
route=np.genfromtxt(SRC/'route_tracking.csv',delimiter=',',names=True);route=route[route['lap']==1];segs=json.loads((SRC/'route/segments.json').read_text())['segments'];starts=np.array([s['t0'] for s in segs]);names=['side_out','turn180_start','side_return','turn_to_main','main_straight','turn_to_cross','cross_out','turn180_end','cross_return']
def phase(ts):return np.clip(np.searchsorted(starts,np.interp(np.asarray(ts)*1e-9,route['sim_t'],route['route_t']),side='right')-1,0,8)
phaseinfo=[]
for i,s in enumerate(segs):
 mask=phase(np.rint(route['sim_t']*1e9).astype(np.int64))==i
 phaseinfo.append(dict(name=names[i],start_s=max(0.,float(route['sim_t'][mask][0]-start*1e-9)),end_s=min((end-start)*1e-9,float(route['sim_t'][mask][-1]-start*1e-9)),kind=s['kind']))
paths={'mimosa_off3':B/'three_camera_no_drpm/fusion'/L/'odometry.csv','drpm3':B/'drpm_three_camera/fusion'/L/'odometry.csv','rovio3':B/'drpm_three_camera/rovio'/L/'result_vio.csv','drpm1':B/'drpm_fast_failures/fusion'/L/'odometry.csv','rovio1':B/'drpm_fast_failures/rovio'/L/'result_vio.csv','fast1':F/'odometry_post_visual.csv'}
result={};curves={};relative={};C=np.array(json.loads((B/'drpm_three_camera/calibration.json').read_text())['covariance'])
for name,path in paths.items():
 v=read(path);grid=np.arange(start,min(end,int(v[0][-1]))+1,100000000,dtype=np.int64);e,m=errors(v,truth,start,grid);angle=np.degrees(Rotation.from_euler('xyz',e[:,3:],degrees=True).magnitude());pn=np.linalg.norm(e[:,:3],axis=1);curves[name]=(grid,e,pn,angle);ph=phase(grid);details=[]
 for i in range(9):
  k=ph==i
  if not k.any():continue
  details.append(dict(phase=names[i],start_s=float((grid[k][0]-start)*1e-9),end_s=float((grid[k][-1]-start)*1e-9),position_rmse_m=float(np.sqrt(np.mean(pn[k]**2))),rotation_rmse_deg=float(np.sqrt(np.mean(angle[k]**2))),position_start_m=float(pn[k][0]),position_end_m=float(pn[k][-1]),rotation_start_deg=float(angle[k][0]),rotation_end_deg=float(angle[k][-1])))
 thresholds={}
 for key,vals,threshold in [('position_1m',pn,1),('position_10m',pn,10),('rotation_1deg',angle,1),('rotation_5deg',angle,5)]:
  k=np.flatnonzero(vals>threshold);thresholds[key]=float((grid[k[0]]-start)*1e-9) if len(k) else None
 result[name]=dict(full=m,phases=details,first_crossings=thresholds)
 # Relative error on exact 50ms intervals; used diagnostically, never to tune these runs.
 if name in ['mimosa_off3','drpm3','rovio3','rovio1']:
  t,p,r=v;k=(t>=start)&(t<=end);t,p,r=t[k],p[k],r[k];gp,gr=interpolate(truth,t);ts,ee=relative_error(t,p,r,gp,gr,1);ph=phase(ts);z={}
  for i in range(9):
   a=ee[ph==i]
   if len(a):z[names[i]]=dict(rot_rmse_deg=np.degrees(np.sqrt(np.mean(a[:,:3]**2,axis=0))).tolist(),trans_rmse_mm=(1000*np.sqrt(np.mean(a[:,3:]**2,axis=0))).tolist(),rotation_bias_deg=np.degrees(a[:,:3].mean(0)).tolist(),normalized_squared_residual_mean=float(np.mean(np.einsum('ni,ij,nj->n',a,np.linalg.inv(C),a))))
  relative[name]=z
# Pair actual lidar-post / visual-post at identical timestamps, then map each log block by ordinal.
a=np.loadtxt(F/'mat_out.txt');assert len(a)%2==0;pre=a[::2];post=a[1::2];assert np.array_equal(pre[:,0],post[:,0]) and (pre[:,-1]>0).all() and (post[:,-1]==0).all()
ts=np.rint(pre[:,0]*1e9).astype(np.int64)+2100000000;official=read(F/'odometry_post_visual.csv');assert np.array_equal(ts,official[0])
blocks=(F/'estimator.log').read_text().split('[ VIO ] Raw feature num:')[1:];assert len(blocks)==len(ts)
retr=[];app=[];liores=[];lioeff=[]
for x in blocks:
 q=re.search(r'Retrieve (\d+) points',x);retr.append(int(q[1]) if q else 0);q=re.search(r'Append (\d+) new',x);app.append(int(q[1]) if q else 0)
# LIO statistics preceding each VIO block, last iteration per lidar scan.
chunks=(F/'estimator.log').read_text().split('[ VIO ] Raw feature num:')[:-1]
for x in chunks:
 q=re.findall(r'effective feature num: (\d+) average residual: (\S+)',x);lioeff.append(int(q[-1][0]) if q else 0);liores.append(float(q[-1][1]) if q else 0)
retr=np.array(retr);app=np.array(app);Rpre=Rotation.from_euler('xyz',pre[:,1:4]/57.3);Rpost=Rotation.from_euler('xyz',post[:,1:4]/57.3);dp=np.linalg.norm(post[:,4:7]-pre[:,4:7],axis=1);dq=np.degrees((Rpre.inv()*Rpost).magnitude())
# Same fixed initial transform for both sides of each visual update.
_,fm=errors(official,truth,start,ts[(ts>=start)&(ts<=end)]);align=Rotation.from_quat(fm['alignment_rotation_xyzw']);offset=np.array(fm['alignment_translation_m']);gp,gr=interpolate(truth,ts);pepre=np.linalg.norm(align.apply(pre[:,4:7])+offset-gp,axis=1);pepost=np.linalg.norm(align.apply(post[:,4:7])+offset-gp,axis=1);repre=np.degrees((gr.inv()*align*Rpre).magnitude());repost=np.degrees((gr.inv()*align*Rpost).magnitude())
with (OUT/'fast_updates.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['stamp_ns','elapsed_s','phase','retrieved','new_points','position_correction_m','rotation_correction_deg','pre_position_error_m','post_position_error_m','pre_rotation_error_deg','post_rotation_error_deg','inverse_exposure','gyro_bias_norm','accel_bias_norm','lio_effective','lio_residual_m']);w.writerows(zip(ts,(ts-start)*1e-9,[names[i] for i in phase(ts)],retr,app,dp,dq,pepre,pepost,repre,repost,post[:,16],np.linalg.norm(post[:,10:13],axis=1),np.linalg.norm(post[:,13:16],axis=1),lioeff,liores))
fastbins=[]
for low in range(0,100,10):
 k=((ts-start)*1e-9>=low)&((ts-start)*1e-9<low+10)
 if not k.any():continue
 fastbins.append(dict(start_s=low,count=int(k.sum()),retrieved_median=float(np.median(retr[k])),retrieved_min=int(retr[k].min()),zero_retrieved=int(sum(retr[k]==0)),position_start=float(pepost[k][0]),position_end=float(pepost[k][-1]),rotation_start=float(repost[k][0]),rotation_end=float(repost[k][-1]),correction_pos_max=float(dp[k].max()),correction_rot_max=float(dq[k].max()),visual_rotation_worsened_fraction=float(np.mean(repost[k]>repre[k]+1e-5)),gyro_bias_end=float(np.linalg.norm(post[k][-1,10:13])),accel_bias_end=float(np.linalg.norm(post[k][-1,13:16])),inverse_exposure_end=float(post[k][-1,16]),lio_residual_median=float(np.median(np.array(liores)[k]))))
# ROVIO support and bias histories.
features={}
for name,folder in [('mono','drpm_fast_failures'),('three','drpm_three_camera')]:
 a=np.genfromtxt(B/folder/'rovio'/L/'result_state.csv',names=True,delimiter=',');parts={}
 for i in range(9):
  k=(phase(a['stamp_ns'])==i)&(a['stamp_ns']>=start)&(a['stamp_ns']<=end)
  parts[names[i]]=dict(feature_mean=[float(a[f'n{j}'][k].mean()) for j in range(3)],feature_min=int((a['n0'][k]+a['n1'][k]+a['n2'][k]).min()),acc_bias_norm_max=float(np.linalg.norm(np.column_stack([a[f'acb_{j}'][k] for j in 'xyz']),axis=1).max()))
 features[name]=parts
# Locate first visual-support loss and attribute instantaneous correction to LIO or vision.
prelog=np.loadtxt(F/'mat_pre.txt');events=[]
for elapsed in [70,75,77,78,79,80,81,82]:
 i=int(np.argmin(abs((ts-start)*1e-9-elapsed)));rows=prelog[np.isclose(prelog[:,0],pre[i,0],atol=1e-8,rtol=0)];assert len(rows)==2
 assert np.allclose(rows[1,4:7],pre[i,4:7],rtol=0,atol=1e-6)
 rp=Rotation.from_euler('xyz',rows[0,1:4]/57.3)
 events.append(dict(elapsed_s=float((ts[i]-start)*1e-9),retrieved=int(retr[i]),lio_rotation_change_deg=float(np.degrees((rp.inv()*Rpre[i]).magnitude())),vision_rotation_change_deg=float(dq[i]),rotation_error_before_vision_deg=float(repre[i]),rotation_error_after_vision_deg=float(repost[i]),position_error_m=float(pepost[i])))
zero=np.flatnonzero((ts>=start)&(retr==0));first_zero=float((ts[zero[0]]-start)*1e-9) if len(zero) else None
initialization={}
for name,folder in [('mono','drpm_fast_failures'),('three','drpm_three_camera')]:
 v=read(B/folder/'rovio'/L/'result_vio.csv');features_rows=list(csv.DictReader((B/folder/'rovio'/L/'result_features.csv').open()));samples=[]
 for sec in [5,10,20,30,60]:
  t=int(sec*1e9);p,_=interpolate(v,[t-50000000,t]);gp,_=interpolate(truth,[t-50000000,t]);d=np.array([float(r['depth']) for r in features_rows if int(r['stamp_ns'])==t]);samples.append(dict(source_s=sec,speed_estimated_mps=float(np.linalg.norm(p[1]-p[0])/.05),speed_true_mps=float(np.linalg.norm(gp[1]-gp[0])/.05),features=len(d),depth_median_m=float(np.median(d)),depth_max_m=float(d.max())))
 initialization[name]=samples
report=dict(fast_correction_events=events,fast_first_zero_support_elapsed_s=first_zero,initialization=initialization,phases=phaseinfo,estimators=result,relative_errors=relative,fast_10s_bins=fastbins,rovio_support=features,reference_cov_sigma_rotation_deg=np.degrees(np.sqrt(C.diagonal()[:3])).tolist(),reference_cov_sigma_translation_mm=(1000*np.sqrt(C.diagonal()[3:])).tolist())
def clean(v):
 if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
 if isinstance(v,list):return [clean(x) for x in v]
 if isinstance(v,float) and not np.isfinite(v):return None
 return v
report=clean(report)
(OUT/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
fig,axs=plt.subplots(5,1,figsize=(14,14),sharex=True)
for n in ['mimosa_off3','drpm3','rovio3']:
 t,e,p,r=curves[n];axs[0].plot((t-start)*1e-9,p,label=n);axs[1].plot((t-start)*1e-9,r,label=n)
axs[2].plot((ts-start)*1e-9,repost,label='FAST rotation error');axs[3].plot((ts-start)*1e-9,pepost,label='FAST position error');axs[4].plot((ts-start)*1e-9,retr,label='FAST usable visual points')
for ax in axs:
 for s in phaseinfo:
  ax.axvline(s['start_s'],color='#999',ls=':',lw=.7)
 ax.grid(alpha=.2);ax.legend()
axs[0].set_ylabel('Position error [m]');axs[1].set_ylabel('Rotation error [deg]');axs[2].set_ylabel('FAST rotation [deg]');axs[3].set_ylabel('FAST position [m]');axs[4].set_ylabel('Visual points');axs[-1].set_xlabel('Tracking elapsed time [s]');fig.tight_layout();fig.savefig(OUT/'diagnosis.png',dpi=125);plt.close(fig)
print(json.dumps(dict(phases=phaseinfo,fast_10s_bins=fastbins,first_crossings={k:v['first_crossings'] for k,v in result.items()}),indent=2))
