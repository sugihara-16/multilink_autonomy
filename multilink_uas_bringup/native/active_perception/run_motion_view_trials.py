#!/usr/bin/env python3
"""Same-flight fixed-five comparison for a frozen candidate policy."""
import argparse,hashlib,json,shutil,signal,subprocess,sys
from pathlib import Path
from run_lio_failure_comparisons import HERE,NATIVE,INPUTS,read,run_one,verify_selected
from run_zab_additional_conditions import save

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--seed',type=int,nargs='+',required=True);p.add_argument('--routes',nargs='+',default=['zab_b1','futatsugoya']);p.add_argument('--stage',choices=['development','validation'],default='development');p.add_argument('--full',action='store_true');a=p.parse_args()
 root=a.output.resolve();root.mkdir(exist_ok=False,parents=True);c=read(INPUTS/'zab_catalog.json');f=read(NATIVE.parents[3]/'assets/futatsugoya/routes.json')
 entries={'zab_a1':dict(c['run1_tail_a1']),'zab_b1':dict(c['run1_tail_b1']),'futatsugoya':dict(f['interior'])}
 entries['zab_a1']['route_end_s']=228.49281198075157;entries['futatsugoya']['route_end_s']=362. if a.full else 122.
 segments={'zab_a1':[[40,100],[160,220]],'zab_b1':[[60,120],[180,240]],'futatsugoya':[[60,120],[180,240],[300,360]] if a.full else [[60,120]]}
 entries={k:entries[k] for k in a.routes};segments={k:segments[k] for k in a.routes}
 files=[HERE/n for n in ['motion_view_planner.py','motion_view_planner_node.py','run_motion_view_array.py','run_motion_view_trials.py','run_view_array.py','run_lio_failure_comparisons.py','report_visual_constraint_survey.py','report_direction_repeats.py','window_metrics.py']]+[a.config.resolve()]
 impl=root/'implementation';impl.mkdir();sha={}
 for path in files:sha[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();shutil.copy2(path,impl/path.name)
 m=dict(status='running',stage=a.stage,design=dict(primary_metric='Actual fused monocular VIO velocity RMS on a common weak LiDAR translation direction, before fusion.',primary_window_s=10,auxiliary_window_s=[5,20],segments_s=segments,seeds=a.seed,comparison='Same physical flight, five independent fixed monocular estimators and one active. Estimators, sensor settings and fusion weights unchanged. All results retained. Truth only for flight control and posthoc evaluation.',fixed_comparators='All five directions, their uniform mean, and the previously observed winner: yaw_m50 on ZaB b1, pitch_m90 on Futatsugoya. Best-in-test is descriptive, not preselected.',secondary='3D velocity error, tilt error, fused local displacement and full trajectory; no claim that lower trajectory error implies better visual observation.',stage=a.stage),entries=entries,order=[f'{route}_seed{seed}' for seed in a.seed for route in entries],runs={},source_sha256=sha,selected_checks=verify_selected())
 save(root/'campaign.json',m)
 def interrupt(*_):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
 template=read(INPUTS/'templates/paired_flight.json');template[1]=str(HERE/'run_motion_view_array.py');template[template.index('--planner-config')+1]=str(a.config.resolve())
 try:
  for key in m['order']:
   assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sha.items())
   route,seed=key.rsplit('_seed',1);m['runs'][key]=dict(status='running');save(root/'campaign.json',m);print('START',key,flush=True)
   r=run_one(root,key,entries[route],template,int(seed),record_geometry=True)
   subprocess.run([sys.executable,str(HERE/'report_view_array.py'),'--trial',str(root/key)],check=True)
   ar=read(root/key/'array_comparison.json');geo=read(root/key/'observers/common_lidar.jsonl.status.json')
   if not r['valid'] or not ar['valid'] or geo.get('samples',0)<100:raise RuntimeError('Trial audit failed: '+key)
   r.update(array_valid=ar['valid'],geometry=geo);m['runs'][key]=r;save(root/'campaign.json',m);print('FINISHED',key,flush=True)
   subprocess.run([sys.executable,str(HERE/'report_visual_constraint_survey.py'),str(root)],check=True)
  m['status']='complete';m['final_selected_checks']=verify_selected()
 except BaseException as exc:m['status']='failed';m['error']=repr(exc);raise
 finally:save(root/'campaign.json',m)
 subprocess.run([sys.executable,str(HERE/'report_visual_constraint_survey.py'),str(root)],check=True)
if __name__=='__main__':main()
