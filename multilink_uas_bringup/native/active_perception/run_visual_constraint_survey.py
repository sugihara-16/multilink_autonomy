#!/usr/bin/env python3
"""Repeated visual-constraint accuracy survey, retaining all fixed and active outcomes."""
import argparse,hashlib,json,signal,subprocess,sys
from pathlib import Path
from run_lio_failure_comparisons import HERE,NATIVE,INPUTS,read,run_one,verify_selected
from run_zab_additional_conditions import save

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output.resolve();root.mkdir(exist_ok=False,parents=True)
 c=read(INPUTS/'zab_catalog.json');f=read(NATIVE.parents[3]/'assets/futatsugoya/routes.json')
 entries={'zab_a1':dict(c['run1_tail_a1']),'zab_b1':dict(c['run1_tail_b1']),'futatsugoya':dict(f['interior'])}
 entries['zab_a1']['route_end_s']=228.49281198075157
 entries['futatsugoya']['route_end_s']=362.
 segments={'zab_a1':[[40,100],[160,220]],'zab_b1':[[60,120],[180,240]],'futatsugoya':[[60,120],[180,240],[300,360]]}
 source_paths=[HERE/n for n in ['run_visual_constraint_survey.py','run_lio_failure_comparisons.py','observe_pure_lio.py','record_common_lidar.py','run_view_array.py','fixed_view_array.py','audit_view_array_inputs.py']]
 sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
 design=dict(primary_metric='Actual fused VIO velocity RMS projected onto the common weakest LiDAR translation direction, before fusion. Report bias, scatter, uncertainty calibration and all-direction velocity error separately. No selection by fused trajectory quality.',primary_window_s=10,auxiliary_window_s=[5,20],segments_s=segments,
   seeds=[20261301,20261302,20261303],selection='Replicate ZaB a1 return fixed-front advantage, test b1 later geometry and extend Futatsugoya to 300-360 s. Seven sections fixed before new flights; retain every direction and all failures. Exploratory counterexample collection, not an estimate of prevalence across environments.',
   time_basis='Elapsed simulation time from tracking onset. History retained; only evaluation is cropped.',
   comparison='Each of five fixed directions against active, by paired independent flight. Report all comparisons and leave-one-repeat-out selection using VIO error only. Fixed-camera winners are exploratory unless confirmed independently.',
   direction='Lowest-eigenvalue translational direction of the pure-LIO diagnostic information, rotation held fixed; transformed to world using truth only for evaluation, shared by every camera method. Direction fixed at each window start.',
   weak_threshold='lambda_min / lambda_max < 0.05; report all windows and weak subset separately; diagnostic effective information, not actual fusion posterior.',
   displacement='Per-window initial-pose alignment: R_gt(t0) R_est(t0)^T (p_est(t1)-p_est(t0)) - (p_gt(t1)-p_gt(t0)), projected on common direction; no scale fit.',
   velocity='Actually fused body velocity rotated using truth; projected error, bias, centered scatter and RMS.',
   windows='Non-overlapping windows per length within each predeclared 60 s segment; identical endpoints for all methods; no exclusion during gimbal motion.',
   caveats=['Three new independent flights per route; windows are correlated, not independent trial counts. Earlier flights used for selection, not pooled as validation.','Weak direction from drifting pure-LIO map is a shared diagnostic proxy, not ground-truth information.','A smaller fused error alone does not prove absence of cancellation.'])
 m=dict(status='running',design=design,entries=entries,order=[f'{route}_seed{seed}' for seed in design['seeds'] for route in entries],runs={},source_sha256=sources,selected_checks=verify_selected())
 save(root/'campaign.json',m)
 def interrupt(*_):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
 template=read(INPUTS/'templates/paired_flight.json');template[1]=str(HERE/'run_view_array.py')
 try:
  for key in m['order']:
   route,seed=key.rsplit('_seed',1)
   assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sources.items())
   m['runs'][key]=dict(status='running');save(root/'campaign.json',m);print('START',key,flush=True)
   r=run_one(root,key,entries[route],template,int(seed),record_geometry=True)
   subprocess.run([sys.executable,str(HERE/'report_view_array.py'),'--trial',str(root/key)],check=True)
   array=read(root/key/'array_comparison.json');geo=read(root/key/'observers/common_lidar.jsonl.status.json')
   if not r['valid'] or not array['valid'] or geo.get('samples',0)<100:raise RuntimeError('Failed trial audit/geometry: '+key)
   r.update(array_valid=array['valid'],geometry=geo);m['runs'][key]=r;save(root/'campaign.json',m);print('FINISHED',key,flush=True)
  m['status']='complete';m['final_selected_checks']=verify_selected()
 except BaseException as exc:m['status']='failed';m['error']=repr(exc);raise
 finally:save(root/'campaign.json',m)
if __name__=='__main__':main()
