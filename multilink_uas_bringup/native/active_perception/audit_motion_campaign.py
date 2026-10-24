#!/usr/bin/env python3
"""Check complete owned flights and report fixed-mean and preselected comparisons separately."""
import argparse,json,hashlib,os,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('--frozen',type=Path,required=True);a=p.parse_args();v=a.campaign.resolve();root=v;m=json.loads((v/'campaign.json').read_text());assert m['status']=='complete'
f=json.loads(a.frozen.read_text());frozen={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in f['files'].items()};assert all(frozen.values())
source_checks={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in m['source_sha256'].items()};assert all(source_checks.values())
checks={};pids={};wall=0
for key in m['order']:
 trial=v/key;comparison=json.loads((trial/'comparison.json').read_text());array=json.loads((trial/'array_comparison.json').read_text());runtime=json.loads((trial/'campaign'/(key+'_active')/'runtime.json').read_text());completion=json.loads((trial/'completion.json').read_text())
 checks[key]=dict(paired=comparison['checks'],array=array['checks'],processes=runtime['processes'],completion=completion,valid=comparison['valid'] and array['valid'] and runtime['stopped'] and all(c==0 for c in runtime['processes'].values()) and all(c==0 for c in completion['processes'].values()),comparison_identical_after_compression=comparison==json.loads((trial/'comparison_before_compression.json').read_text()))
 assert checks[key]['valid'] and checks[key]['comparison_identical_after_compression'];wall+=completion['wall_s']
 for name,pid in json.loads((trial/'owners.json').read_text()).items():
  path=Path(f'/proc/{pid}/cmdline');pids[f'{key}/{name}']=path.read_bytes().replace(b'\0',b' ').decode(errors='replace') if path.exists() else None
assert not any(pids.values())
summary=json.loads((v/'report/summary.json').read_text());sections={};allpairs=[];primary=[];fused=[]
for key,s in summary['sections'].items():
 records=[]
 for t in s['windows']['10']['trials']:
  methods=t['methods'];fixed={n:methods[n]['visual']['rmse'] for n in ['yaw_m50','front','yaw_p50','pitch_p90','pitch_m90']};active=methods['active']['visual']['rmse'];mean=float(np.mean(list(fixed.values())));best=min(fixed,key=fixed.get)
  ref='yaw_m50' if key.startswith('zab_b1') else ('pitch_m90' if key.startswith('futatsugoya') else None)
  row=dict(trial=t['trial'],active=active,fixed_mean=mean,best_fixed=best,best_fixed_error=fixed[best],preselected=ref,preselected_error=fixed.get(ref),ratio_to_fixed_mean=active/mean,active_wins=int(sum(active<x for x in fixed.values())),fixed=fixed,vector_rms={n:q['vector_rmse_mps'] for n,q in methods.items()},fused_displacement={n:q['fused_displacement']['rmse'] for n,q in methods.items()})
  records.append(row);allpairs.append(row)
  if key in ['zab_b1_0','zab_b1_1','futatsugoya_0']:primary.append(row)
 sections[key]=dict(bounds_s=s['bounds_s'],records=records,active_mean=float(np.mean([x['active'] for x in records])),fixed_mean=float(np.mean([x['fixed_mean'] for x in records])),best_direction_of_mean=min(records[0]['fixed'],key=lambda n:np.mean([x['fixed'][n] for x in records])),wins_vs_fixed_mean=sum(x['active']<x['fixed_mean'] for x in records),wins_vs_preselected=None if records[0]['preselected'] is None else sum(x['active']<x['preselected_error'] for x in records))
 for name in ['active_mean','fixed_mean']:assert np.isfinite(sections[key][name])
counts=lambda rows:dict(section_flight_pairs=0,wins_fixed_mean=0,wins_all_five=0,active_mean=None,fixed_mean=None,mean_paired_ratio=None) if not rows else dict(section_flight_pairs=len(rows),wins_fixed_mean=sum(x['active']<x['fixed_mean'] for x in rows),wins_all_five=sum(x['active_wins']==5 for x in rows),active_mean=float(np.mean([x['active'] for x in rows])),fixed_mean=float(np.mean([x['fixed_mean'] for x in rows])),mean_paired_ratio=float(np.mean([x['ratio_to_fixed_mean'] for x in rows])))
result=dict(candidate=f['candidate'],stage=m['stage'],physical_flights=len(m['order']),sections=sections,all_sections=counts(allpairs),primary_sections=counts(primary),primary_wins_vs_preselected=sum(x['active']<x['preselected_error'] for x in primary),note='Section-flight pairs are correlated within a physical flight. No significance claim. Fixed mean means uniform choice over these five fixed views. Best direction is retrospective; preselected references come from the earlier survey.')
(v/'report/final_summary.json').write_text(json.dumps(result,indent=2)+'\n')
space=os.statvfs('/mnt/c');size=sum(p.stat().st_size for p in root.rglob('*') if p.is_file() and not p.is_symlink());audit=dict(checks=checks,frozen=frozen,execution_sources=source_checks,owners=pids,all_valid=True,wall_s=wall,storage_bytes=size,windows_free_bytes=space.f_bavail*space.f_frsize)
(v/'report/final_audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(json.dumps({k:result[k] for k in ['all_sections','primary_sections','primary_wins_vs_preselected']},indent=2))
