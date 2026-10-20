#!/usr/bin/env python3
"""Finish the already running main capture, then the remaining ZaB components."""
import json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
campaign=ROOT/'logs/zab-full-campaign';campaign.mkdir(exist_ok=True)
ledger=ROOT/'routes/zab_full/capture_runs.json';runs=json.loads(ledger.read_text())
state=json.loads((ROOT/'state.json').read_text());runs['run1']=state['logs'];ledger.write_text(json.dumps(runs,indent=2))
results={}
def watch(label,path):
 last_print=0;last_sim=None;last_wall=time.monotonic()
 while (ROOT/'state.json').exists():
  now=time.monotonic()
  try:s=json.loads((path/'route_status.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):s={}
  if s.get('failure'):
   subprocess.run(['python3',str(ROOT/'manager.py'),'stop'],check=True)
   raise RuntimeError(f'{label}: {s}')
  if now-last_print>=45:
   sim=s.get('sim_time',0);factor=(sim-last_sim)/(now-last_wall) if last_sim is not None else None
   entry=dict(component=label,stage=s.get('stage','startup'),route_s=s.get('route_time_s',0),duration_s=s.get('route_duration_s',0),sim=sim,actual_factor=factor,tracking_error=s.get('tracking_error_m'),run=str(path))
   (campaign/'progress.json').write_text(json.dumps(entry,indent=2));print(json.dumps(entry),flush=True)
   last_print=now;last_wall=now;last_sim=sim
  time.sleep(2)
 s=json.loads((path/'route_status.json').read_text())
 if s['stage']!='complete' or s['failure']:raise RuntimeError(f'Incomplete {label}: {s}')
 with (campaign/(label+'-validation.log')).open('w') as f:subprocess.run(['python3',str(ROOT/'verify_raw_bag.py'),str(path/'rosbag')],stdout=f,stderr=subprocess.STDOUT,check=True)
 results[label]=dict(path=str(path),flight=s,bag=json.loads((path/'raw_bag_validation.json').read_text()))
 (campaign/'results.json').write_text(json.dumps(results,indent=2));print(label+' completed and bag validated',flush=True)
watch('run1',Path(runs['run1']))
for label in ['run2','run4']:
 with (campaign/(label+'-launch.log')).open('w') as f:
  p=subprocess.Popen(['ros2','launch','multilink_uas_bringup','zab.launch.py','headless:=true','rviz:=false','estimators:=false','record:=true',f'route:={ROOT}/routes/zab_full/{label}'],stdout=f,stderr=subprocess.STDOUT)
  deadline=time.monotonic()+60
  while not (ROOT/'state.json').exists():
   if p.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Capture startup failed')
   time.sleep(.2)
  state=json.loads((ROOT/'state.json').read_text());runs[label]=state['logs'];ledger.write_text(json.dumps(runs,indent=2));watch(label,Path(runs[label]));p.wait(timeout=30)
results['run3']=dict(path=runs['run3'],flight=json.loads((Path(runs['run3'])/'route_status.json').read_text()),bag=json.loads((Path(runs['run3'])/'raw_bag_validation.json').read_text()))
(campaign/'results.json').write_text(json.dumps(results,indent=2));print('ALL FOUR CAPTURES COMPLETE',flush=True)
