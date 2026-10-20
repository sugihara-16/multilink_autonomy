#!/usr/bin/env python3
import csv,json,yaml
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parent; C=ROOT/'logs/zab-offline';OUT=C/'environment_analysis'
LABELS=['run1_prefix','run1_middle','run1_tail_a1','run1_tail_a2_prefix','run1_tail_b1','run1_tail_b2','run2a_prefix','run2a_end','run2b']
def pose(label):
 with (C/'results'/label/'truth.csv').open() as f:r=list(csv.DictReader(f))
 return np.array([int(x['stamp_ns']) for x in r])*1e-9,np.array([[float(x[k]) for k in ['x','y','z']] for x in r]),Rotation.from_quat([[float(x[k]) for k in ['qx','qy','qz','qw']] for x in r])
def main():
 manifest=json.loads((C/'results/manifest.json').read_text());profiles={};jobs={}
 for label in LABELS:
  item=next(x for x in manifest['runs'] if x['label']==label);tt,tp,tr=pose(label);source=ROOT/'logs'/item['source_run'];stats={}
  for k in ['lio','vio']:
   a=np.genfromtxt(C/'error_plots'/f'{label}_{k}_errors.csv',delimiter=',',names=True);pos=np.column_stack([a[n] for n in ['ex_m','ey_m','ez_m']]);angles=np.column_stack([a[n] for n in ['eroll_deg','epitch_deg','eyaw_deg']]);norm=np.linalg.norm(pos,axis=1);rot=Rotation.from_euler('xyz',angles,degrees=True).magnitude()*180/np.pi
   def first(cond):return float(a['stamp_ns'][np.flatnonzero(cond)[0]]*1e-9) if np.any(cond) else None
   stats[k]={'max_position_m':float(norm.max()),'end_position_m':float(norm[-1]),'max_rotation_deg':float(rot.max()),'position_2m_at':first(norm>2),'position_10m_at':first(norm>10),'rotation_10deg_at':first(rot>10),'rotation_60deg_at':first(rot>60),'end_error_xyz':pos[-1].tolist()}
  choices=[v for k,v in stats['lio'].items() if k in ['position_2m_at','rotation_10deg_at'] and v is not None];onset=min(choices) if choices else (tt[0]+tt[-1])/2
  sample_times={'early':float(tt[0]+5),'lio_onset':float(onset)}
  for stage,candidate in [('vio_onset',stats['vio']['position_2m_at']),('rotation_onset',stats['lio']['rotation_10deg_at'])]:
   if candidate is not None and min(abs(candidate-v) for v in sample_times.values())>20:sample_times[stage]=float(candidate)
  # Motion geometry, measured from actual truth, with 1 s spacing to reject tracking jitter.
  grid=np.arange(tt[0],tt[-1],1.);pp=np.column_stack([np.interp(grid,tt,tp[:,j]) for j in range(3)]);rr=Slerp(tt,tr)(grid);delta=np.diff(pp,axis=0);dist=np.linalg.norm(delta,axis=1);v=delta/np.maximum(dist[:,None],1e-9);bends=np.arccos(np.clip(np.sum(v[:-1]*v[1:],axis=1),-1,1))*180/np.pi
  world=json.loads((source/'world.json').read_text());stats.update(source_run=item['source_run'],start_s=float(tt[0]),end_s=float(tt[-1]),rate_cap=item['window']['rate_cap'],length_m=float(dist.sum()),z_range= [float(tp[:,2].min()),float(tp[:,2].max())],turns_over_10deg_per_s=int(np.sum(bends>10)),sample_times=sample_times)
  profiles[label]=stats
  adopted=set(yaml.safe_load((source/'cloud_metadata.yaml').read_text())['rosbag2_bagfile_information']['relative_file_paths'])
  receipts=[json.loads(p.read_text()) for p in (source/'cloud_chunks').glob('rosbag_*.json') if p.stem+'.mcap' in adopted]
  for receipt in receipts:receipt['remote']=item['window']['source_remote']+'/'+receipt['cloud_file']
  for stage,t in sample_times.items():
   matches=[r for r in receipts if r['file_metadata']['starting_time']['nanoseconds_since_epoch']*1e-9<=t<=(r['file_metadata']['starting_time']['nanoseconds_since_epoch']+r['file_metadata']['duration']['nanoseconds'])*1e-9]
   assert matches,(label,t)
   r=matches[0];key=item['source_run']+'/'+r['cloud_file'];job=jobs.setdefault(key,dict(source_run=item['source_run'],receipt=r,requests=[]));job['requests'].append(dict(label=label,stage=stage,time=t,xyz=[float(np.interp(t,tt,tp[:,j])) for j in range(3)]))
 (OUT/'profiles.json').write_text(json.dumps(profiles,indent=2)+'\n');(OUT/'probe_jobs.json').write_text(json.dumps(list(jobs.values()),indent=2)+'\n')
 print('jobs',len(jobs),'compressed GB',sum(x['receipt']['compressed_bytes'] for x in jobs.values())/1e9)
 for l,p in profiles.items():print(l,'length',round(p['length_m']), 'LIO',p['lio'],'VIO',p['vio'],flush=True)
if __name__=='__main__':main()
