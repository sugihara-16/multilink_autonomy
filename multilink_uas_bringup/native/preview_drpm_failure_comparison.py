"""Evaluate an already prepared common prefix while full data acquisition continues."""
import json,fcntl,shutil
from pathlib import Path
import run_drpm_failure_comparison as r
main_out=r.OUT;r.OUT=main_out/'common_preview';r.OUT.mkdir(exist_ok=True)
label='run1_prefix';inp=r.OUT/'inputs'/label;inp.mkdir(parents=True,exist_ok=True)
e=next(e for e in json.loads((r.ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if e['label']==label)
w=json.loads((r.BASE/'results'/label/'window.json').read_text());w['end_ns']=json.loads((r.BASE/'fast_official_raw'/label/'metrics.json').read_text())['evaluation_end_ns']
names=[f'rosbag_{i}.npz' for i in range(15)]
for n in names:
 p=inp/n
 if not p.exists():p.symlink_to(main_out/'inputs'/label/n)
r.save(inp/'manifest.json',dict(source=e,window=w,chunks=names,purpose='Provisional common-window evaluation; full flight is still being acquired'))
r.fixed_calibration()
for phase in ['rovio','bag','fusion']:
 print('PREVIEW',phase,flush=True)
 if phase=='fusion':
  with (main_out/'fusion_parameters.lock').open('w') as lock:
   fcntl.flock(lock,fcntl.LOCK_EX);r.stage_impl(phase,label)
 else:r.stage_impl(phase,label)
import evaluate_drpm_failure_comparison as ev
ev.OUT=r.OUT;ev.evaluate(label)
(r.OUT/'inputs'/f'{label}.bag').unlink()
for p in inp.glob('*.npz'):p.unlink()
print('PREVIEW COMPLETE',flush=True)
