#!/usr/bin/env python3
"""Audit recorded road coverage and verified sensor data across capture segments."""
import json
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
C=ROOT/'logs/zab-full-campaign'
labels=['run3','run1_prefix','run4','run1_middle','run1_tail_branch','run1_tail_link','run1_tail_a1','run1_tail_a2_prefix','run1_tail_a2_end','run1_tail_b1','run1_tail_b2','run2a_prefix','run2a_end','run2b']
results=json.loads((C/'results.json').read_text())
missing=set(labels)-results.keys()
if missing:raise RuntimeError('Captures still missing: '+str(sorted(missing)))
expected=set()
for name in ['run1','run2','run3','run4']:
 r=json.loads((ROOT/'routes/zab_full'/name/'route.json').read_text())
 expected.update(tuple(sorted(e)) for e in r['edge_visits'])
covered=set();intervals={};entries=[]
for name in labels:
 result=results[name];run=Path(result['path']);bag=result['bag']
 assert bag['lossless_verified']
 assert all(not s['missing_intervals'] and not s['nonmonotonic'] for s in bag['sensors'].values())
 plan=json.loads((run/'route/route.json').read_text());ss=json.loads((run/'route/segments.json').read_text())['segments'];polys=[s for s in ss if s['kind']=='polyline']
 assert len(polys)==len(plan['edge_visits']),(name,len(polys),len(plan['edge_visits']))
 if name=='run1_prefix':end=1070.0  # conservative endpoint before the failed resume
 else:
  flight=result['flight'];assert not flight['failure']
  end=flight['route_time_s']
  if not result.get('partial'):assert flight['stage']=='complete'
 ranges=plan.get('edge_fraction_ranges',[[0.,1.]]*len(polys))
 edges=[]
 for segment,e,(lo,hi) in zip(polys,plan['edge_visits'],ranges):
  u=max(0.,min(segment['duration'],end-segment['t0']))
  T=segment['duration'];ramp=segment['ramp_s'];speed=segment['speed_mps'];length=segment['arc'][-1]
  if u<ramp:distance=.5*speed/ramp*u*u
  elif u>T-ramp:distance=length-.5*speed/ramp*(T-u)**2
  else:distance=speed*(u-.5*ramp)
  fraction=max(0.,min(1.,distance/length))
  if fraction<=0:continue
  right=lo+(hi-lo)*fraction
  edge=tuple(sorted(e));span=(lo,right) if tuple(e)==edge else (1-right,1-lo)
  intervals.setdefault(edge,[]).append(span)
  if fraction>=1.-1e-6:edges.append(edge)
 entries.append(dict(label=name,run=run.name,remote=bag['remote'],partial=result.get('partial',False),recorded_route_end_s=end,complete_road_traversals=len(edges),bag=bag))
for edge,spans in intervals.items():
 end=0.
 for lo,hi in sorted(spans):
  if lo>end+1e-6:break
  end=max(end,hi)
 if end>=1.-1e-6:covered.add(edge)
assert covered==expected,dict(missing_edges=sorted(expected-covered),unexpected_edges=sorted(covered-expected))
summary=dict(status='complete',speed_mps=.5,estimators=False,gui=False,valid_bags=entries,covered_unique_edges=len(covered),expected_unique_edges=len(expected),missing_edges=[],total_messages=sum(x['bag']['total_messages'] for x in entries),compressed_bytes=sum(x['bag']['bytes'] for x in entries),source_bytes=sum(x['bag']['source_bytes'] for x in entries),sensor_gaps=0,notes=['Four connected components, recorded as fourteen independently started bags.','run1_prefix excludes failed-resume data. Later captures use simulation-clock and paused-force fixes.','run2a_prefix includes chunks 0..21 only; chunks 22..25 are excluded for one LiDAR gap. run2a_end re-records the endpoint with overlap.','Restart segments have independent takeoff and yaw alignment; bag clocks reset between runs.','excluded_attempts and run 20261002-220031-79824 are not part of this dataset.'])
(C/'dataset_manifest.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('valid_bags','notes')},indent=2))
