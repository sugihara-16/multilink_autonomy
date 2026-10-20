#!/usr/bin/env python3
"""Audit completion and timestamp coverage before counting a runtime speedup."""
import argparse,csv,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args()
m=json.loads((a.campaign/'campaign.json').read_text());result=dict(campaign_status=m['status'],wall_s=m['wall_s'],runs={})
for key,entry in m['runs'].items():
 root=Path(entry['path']);r=dict(status=entry['status']);result['runs'][key]=r
 r['node_crash_logs']=[f.name for f in root.glob('*.log') if 'has died!' in f.read_text(errors='replace')]
 for name in ['runtime','route_status','planner_health','pacing','drain','input_audit']:
  f=root/(name+'.json')
  if f.exists():r[name]=json.loads(f.read_text())
 trajectory=root/'route_tracking.csv'
 if not trajectory.exists():continue
 rows=list(csv.DictReader(trajectory.open()));tracking=[x for x in rows if float(x['route_t'])>0]
 if not tracking:continue
 last_t=max(float(x['route_t']) for x in tracking)
 end=max(float(x['sim_t']) for x in tracking);start=float(tracking[0]['sim_t']);tracking=[x for x in tracking if float(x['sim_t'])<=end];r['tracking_window_s']=[start,end]
 r['tracking_error_max_m']=max(float(x['tracking_error_m']) for x in tracking)
 r['streams']={}
 for mode,folder in [('active',root),('fixed',root/'fixed')]:
  if not folder.exists():continue
  for kind in ['rovio','mimosa','truth']:
   f=folder/(kind+'.csv')
   if not f.exists():continue
   rows=list(csv.DictReader(f.open()));rows=[x for x in rows if start<=int(x['stamp_ns'])*1e-9<=end]
   if not rows:continue
   stamps=np.array([int(x['stamp_ns']) for x in rows],dtype=np.int64);gaps=np.diff(stamps)
   xyz=np.array([[float(x[k]) for k in ['x','y','z']] for x in rows]);period=50000000 if kind=='rovio' else None
   r['streams'][mode+'/'+kind]=dict(first_gap_s=float(stamps[0]*1e-9-start),last_gap_s=float(end-stamps[-1]*1e-9),count=len(rows),unique=len(np.unique(stamps)),max_gap_ns=int(gaps.max()) if len(gaps) else None,
      finite=bool(np.isfinite(xyz).all()),max_displacement_m=float(np.linalg.norm(xyz-xyz[0],axis=1).max()),
      missing_periods=int(np.maximum(0,gaps//period-1).sum()) if period else None)
 debug=root/'processing_times.jsonl'
 if debug.exists():
  data=[json.loads(l) for l in debug.read_text().splitlines()];r['lidar_processing']={}
  for name in ['lidar','fixed_lidar']:
   rows=[x for x in data if x['name']==name and start<=x['stamp']<=end]
   if len(rows)<2:continue
   gaps=np.round(np.diff([x['stamp'] for x in rows])*1e9).astype(np.int64)
   r['lidar_processing'][name]=dict(count=len(rows),max_gap_ns=int(gaps.max()),missing_scans=int(np.maximum(0,gaps//100000000-1).sum()),full_ms_percentiles=np.percentile([x['full_ms'] for x in rows],[50,95,99,100]).tolist())
expected={'run1_prefix_active':1070.,'run1_tail_a1_active':228.49281198075158,'run1_tail_b1_active':263.38475383}
result['all_routes_complete']=(m['status']=='complete' and set(m['runs'])==set(expected)
 and all(result['runs'][k].get('status')=='route_complete'
 and result['runs'][k].get('route_status',{}).get('route_time_s',0)>=duration-.001 for k,duration in expected.items()))
result['timestamp_coverage_complete']=bool(result['runs']) and all(
 not r.get('node_crash_logs') and r.get('planner_health',{}).get('stale',1)==0
 and {'active/rovio','fixed/rovio','active/mimosa','fixed/mimosa','active/truth','fixed/truth'} <= set(r.get('streams',{}))
 and (not r.get('input_audit') or all(x['missing_periods']==0 and x['duplicates']==0 for x in r['input_audit'].values()))
 and all(x['finite'] and x['count']==x['unique'] and x['first_gap_s']<=(.100001 if k.endswith('/mimosa') else .050001) and x['last_gap_s']<=(.100001 if k.endswith('/mimosa') else .050001) for k,x in r.get('streams',{}).items())
 and len(r.get('lidar_processing',{}))==2
 and all(x['missing_scans']==0 for x in r.get('lidar_processing',{}).values())
 and all(x.get('missing_periods')==0 and x['count']==x['unique'] and x['first_gap_s']<=.050001 and x['last_gap_s']<=.050001 for k,x in r.get('streams',{}).items() if k.endswith('/rovio'))
 for r in result['runs'].values())
result['conservative_previous_s']=4698.860443
result['previous_approximate_s']=5400
result['speedup_from_previous_log_spans']=4698.860443/m['wall_s'] if result['all_routes_complete'] else None
result['fivefold_runtime_reached']=bool(result['all_routes_complete'] and result['timestamp_coverage_complete'] and m['wall_s']<=4698.860443/5)
# A timing result alone is not a quality/accuracy validation.
result['full_accuracy_equivalence_verified']=False
out=a.campaign/'performance_audit.json';out.write_text(json.dumps(result,indent=2)+'\n');print(out)
