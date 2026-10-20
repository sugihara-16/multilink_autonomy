#!/usr/bin/env python3
"""Check complete input delivery, exact output stamps and report integrity."""
import json,re
from pathlib import Path
import numpy as np
from summarize_vio_audit import numeric
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit'
def main():
 summary=json.loads((OUT/'summary.json').read_text());count=0;result={}
 for label,runs in summary.items():
  inp=OUT/'inputs'/label;meta=json.loads((inp/'manifest.json').read_text());stamps=[];nimu=0
  initializations=re.findall(r'-- Filter: Initialized at t = ([0-9.]+)',(ROOT/'logs/zab-offline'/label/'rovio.log').read_text());assert len(initializations)==1
  initialization_ns=round(float(initializations[0])*1e9)
  for name in meta['chunks']:
   with np.load(inp/name) as a:
    stamps.extend(a['cam0_t'].tolist());nimu+=int(np.count_nonzero(a['imu_t']>=initialization_ns))
  expected=set(stamps);reference=None;checks={}
  for variant,evaluation in runs.items():
   dest=OUT/'experiments'/label/variant;log=(dest/'replay.log').read_text();run=json.loads((dest/'run.json').read_text());v=numeric(dest/'result_vio.csv');state=numeric(dest/'result_state.csv');t=v['stamp_ns'].astype(np.int64)
   assert np.all(np.diff(t)>0) and set(t).issubset(expected)
   assert run['counts']==[nimu,len(stamps),len(stamps),len(stamps)]
   delivery=re.search(r'AUDIT_INPUT (\d+) (\d+) (\d+) (\d+) sets (\d+) incomplete (\d+)',log);assert delivery
   assert list(map(int,delivery.groups()[:4]))==run['counts'] and int(delivery[5])==len(stamps) and int(delivery[6])==0
   assert re.findall(r'-- Filter: Initialized at t = ([0-9.]+)',log)==initializations
   assert 'addPredictionMeas] Warning' not in log
   late=[line for line in log.splitlines() if 'addUpdateMeas] Warning' in line];assert all(f'time 2 before safeTime {initializations[0]}' in line for line in late)
   assert evaluation['missing_original_timestamps']==0
   assert all(np.all(np.isfinite(v[name])) for name in v.dtype.names)
   assert np.max(abs(sum(v['q'+k]**2 for k in 'xyzw')-1))<1e-10
   assert all(np.all(state['acb_var_'+k]>=0) for k in 'xyz')
   if reference is None:reference=t
   else:assert np.array_equal(reference,t)
   errors=np.load(dest/'errors.npz');assert errors['stamp_ns'][0]==evaluation['anchor_ns'];assert np.max(abs(errors['errors'][0]))<1e-9
   assert abs(float(np.linalg.norm(errors['errors'][:,:3],axis=1).max())-evaluation['max_position_m'])<1e-9
   checks[variant]=dict(output_count=len(t),tracking_count=evaluation['count'],input_counts=run['counts'],all_input_delivered=True,tracking_stamps_complete=True,finite_pose=True,late_imu_count=0,initialization_count=1,expected_preinit_image_warning_count=len(late));count+=1
  assert runs['baseline']['raw_position_difference_from_original_max_m']==0
  assert runs['baseline']['raw_attitude_difference_from_original_max_rad']<1e-14
  result[label]=checks
 assert count==45
 report=OUT/'report';content=(report/'index.html').read_text();links=re.findall(r'(?:src|href)="([^"]+)"',content)
 for link in links:assert (report/link).exists(),link
 out=dict(status='passed',experiments=count,flights=3,details=result,report_links=len(links),note='Input timestamps and values in original reports preserved; IMU initialization stamp matches the original replay. Original/normal launch config not modified.')
 (OUT/'validation.json').write_text(json.dumps(out,indent=2));(report/'validation.json').write_text(json.dumps(out,indent=2));print('PASS: 45 experiments, 3 exact baseline reproductions, no input delivery losses or tracking stamp gaps')
if __name__=='__main__':main()
