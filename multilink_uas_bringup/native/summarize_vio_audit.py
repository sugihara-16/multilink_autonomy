#!/usr/bin/env python3
"""Evaluate diagnostic variants with the existing plots' exact alignment time."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Slerp
from plot_offline_errors import read
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit'
def numeric(path):
 with path.open() as f:names=f.readline().strip().split(',')
 return np.loadtxt(path,delimiter=',',skiprows=1,dtype=[(name,'f8') for name in names])
def main():
 evaluation=json.loads((ROOT/'logs/zab-offline/error_plots/evaluation.json').read_text());results={}
 for folder in sorted((OUT/'experiments').iterdir()):
  label=folder.name;tt,tp,tr=read(ROOT/'logs/zab-offline/results'/label/'truth.csv');ot,op,orr=read(ROOT/'logs/zab-offline/results'/label/'vio.csv');anchor=evaluation[label]['anchor_ns'];runs={}
  for dest in sorted(folder.iterdir()):
   if not (dest/'run.json').exists():continue
   t,p,r=read(dest/'result_vio.csv');all_t=t.copy();common,i,j=np.intersect1d(ot,t,return_indices=True);rawdiff=float(np.max(np.linalg.norm(op[i]-p[j],axis=1)));angular=float(np.max((orr[i].inv()*r[j]).magnitude()))
   valid=(t>=anchor)&(t<=tt[-1]);t=t[valid];p=p[valid];r=r[valid];assert t[0]==anchor
   truth=np.column_stack([np.interp((t-anchor)*1e-9,(tt-anchor)*1e-9,tp[:,k]) for k in range(3)]);q=Slerp((tt-anchor)*1e-9,tr)((t-anchor)*1e-9);rotation=q[0]*r[0].inv();pe=rotation.apply(p-p[0])+truth[0]-truth;ae=(q.inv()*(rotation*r)).as_euler('xyz',degrees=True);e=np.column_stack([pe,ae]);norm=np.linalg.norm(pe,axis=1)
   np.savez_compressed(dest/'errors.npz',stamp_ns=t,errors=e)
   state=numeric(dest/'result_state.csv');state=state[(state['stamp_ns']>=anchor)&(state['stamp_ns']<=tt[-1])];features=numeric(dest/'result_features.csv');features=features[(features['stamp_ns']>=anchor)&(features['stamp_ns']<=tt[-1])]
   statuses=np.column_stack([features[f'status{i}'] for i in range(3)]);own=statuses[np.arange(len(features)),features['cam'].astype(int)];other=statuses.copy();other[np.arange(len(features)),features['cam'].astype(int)]=0;tracked=own==4;times,idx,counts=np.unique(features['stamp_ns'],return_inverse=True,return_counts=True);tracks=np.bincount(idx,weights=tracked);cross=np.bincount(idx,weights=np.any(other==4,axis=1))
   record=dict(max_position_m=float(norm.max()),rmse_position_m=float(np.sqrt(np.mean(norm**2))),end_position_m=float(norm[-1]),max_abs_six_axes=np.max(np.abs(e),axis=0).tolist(),count=len(t),anchor_ns=anchor,raw_position_difference_from_original_max_m=rawdiff,raw_attitude_difference_from_original_max_rad=angular,missing_original_timestamps=int(len(ot)-len(common)),acb_start=[float(state[f'acb_{k}'][0]) for k in 'xyz'],acb_end=[float(state[f'acb_{k}'][-1]) for k in 'xyz'],acb_sigma_end=[float(np.sqrt(state[f'acb_var_{k}'][-1])) for k in 'xyz'],features_mean_by_camera=[float(state[f'n{k}'].mean()) for k in range(3)],tracked_per_frame_quantiles=np.quantile(tracks,[0,.1,.5,.9,1]).tolist(),cross_tracked_per_frame_mean=float(cross.mean()),depth_quantiles=np.quantile(features['depth'],[0,.1,.5,.9,1]).tolist(),wall_s=json.loads((dest/'run.json').read_text())['wall_s'])
   record['gyb_start']=[float(state[f'gyb_{k}'][0]) for k in 'xyz'];record['gyb_end']=[float(state[f'gyb_{k}'][-1]) for k in 'xyz']
   (dest/'evaluation.json').write_text(json.dumps(record,indent=2));runs[dest.name]=record
  results[label]=runs
 (OUT/'summary.json').write_text(json.dumps(results,indent=2))
 for label,runs in results.items():
  print(label)
  for name,r in runs.items():print(name,'max %.3f end %.3f tracked %.1f cross %.2f olddiff %.4g'%(r['max_position_m'],r['end_position_m'],r['tracked_per_frame_quantiles'][2],r['cross_tracked_per_frame_mean'],r['raw_position_difference_from_original_max_m']))
if __name__=='__main__':main()
