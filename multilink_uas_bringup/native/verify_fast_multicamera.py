#!/usr/bin/env python3
"""Validate real camera participation, posterior chaining, timestamps and accuracy."""
import csv,json,re
from pathlib import Path
import numpy as np
from plot_fast_camera_comparison import read,errors,BASE
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_multicamera'
def main():
 results={};hashes=set();all_counts={}
 for summarypath in sorted(OUT.glob('*/*/trial*/summary.json')):
  d=summarypath.parent;summary=json.loads(summarypath.read_text());assert summary['returncode']==0;hashes.add(summary['binary_sha256']);label=d.parent.name;method=d.parent.parent.name;w=summary['window'];s=w['start_ns'];end=w['end_ns'];log=(d/'run.log').read_text()
  a=read(d/'odometry.csv');native=a[0][(a[0]>=s)&(a[0]<=end)];expected=np.arange((s+49999999)//50000000*50000000,end+1,50000000,dtype=np.int64);assert np.array_equal(native,expected),str(d)
  assert np.isfinite(a[1]).all() and np.isfinite(a[2].as_quat()).all()
  grid=np.arange((s+99999999)//100000000*100000000,end+1,100000000,dtype=np.int64);e,m=errors(a,read(BASE/'inputs'/label/'truth.csv'),s,grid)
  with (d/'errors.csv').open('w') as f:
   wr=csv.writer(f);wr.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);wr.writerows(zip(grid,*e.T))
  inp=list(map(int,re.search(r'INPUT_COUNTS ([0-9 ]+)',log)[1].split()));updates=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',log);assert int(updates[3])==0
  all_counts.setdefault(label,set()).add(tuple(inp[:2]+[int(updates[1]),int(updates[2])]))
  audit=dict(input_counts=inp,lio_updates=int(updates[1]),visual_sets=int(updates[2]),native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9))
  if not summary['legacy']:
   sync={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'CAMERA_SYNC (.+)',log)[1])};assert all(v==0 for v in sync.values()),sync
   counted={name:[int(a),int(b)] for name,a,b in re.findall(r'CAMERA_COUNTS (\w+) (\d+) (\d+)',log)}
   rows=re.findall(r'CAMERA_UPDATE ([0-9.e+-]+) (\w+) (\d+) ([0-9.e+-]+) ([0-9.e+-]+) ([0-9.e+-]+)',log)
   names=summary['cameras'];n=len(names);assert len(rows)%n==0
   contribution={name:dict(frames=0,frames_with_constraints=0,frames_with_pose_change=0) for name in names}
   for k in range(0,len(rows),n):
    group=rows[k:k+n];assert [r[1] for r in group]==names;assert len({r[0] for r in group})==1
    for i,r in enumerate(group):
     if i:assert float(r[4])==float(group[i-1][5]),'Camera prior covariance did not match preceding posterior'
     z=contribution[r[1]];z['frames']+=1;z['frames_with_constraints']+=int(int(r[2])>0);z['frames_with_pose_change']+=int(float(r[3])>1e-12)
   for name,z in contribution.items():assert z['frames']==counted[name][0] and z['frames_with_constraints']==counted[name][1] and z['frames_with_pose_change']>100
   audit.update(sync=sync,cameras=contribution,sequential_covariance_chain_exact=True)
  results[f'{method}/{label}/{d.name}']=dict(metrics=m,verification=audit,wall_s=summary['wall_s'])
  print(method,label,d.name,m['position_rmse_m'],m['rotation_rmse_deg'],audit.get('cameras'),flush=True)
 assert len(hashes)==1,hashes
 assert all(len(v)==1 for v in all_counts.values()),all_counts
 (OUT/'evaluation.json').write_text(json.dumps(results,indent=2))
 print('All trials: complete tracking timestamps, identical IMU/LiDAR and update counts, same binary; all configured cameras contributed.')
if __name__=='__main__':main()
