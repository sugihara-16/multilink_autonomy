#!/usr/bin/env python3
"""Cancel unused downloads only after a flight's preregistered early stop.
Estimator/input reader processes are never signalled here.
"""
import json,time
from pathlib import Path
import psutil
from check_fast_monocular_failures import OUT,LABELS
while True:
 live=list(psutil.process_iter(['name','cmdline']))
 for label in LABELS:
  d=OUT/label;prefix=f'/fast_monocular_failures/cache/{label}_';active=[]
  for proc in live:
   if proc.info['name']=='rclone' and any(prefix in x for x in (proc.info['cmdline'] or [])):
    active.append(proc)
  if (d/'stop_decision.json').exists():
   for proc in active:
    try:proc.terminate();print('cancel unused',label,proc.pid,flush=True)
    except psutil.NoSuchProcess:pass
  if (d/'metrics.json').exists() and not active:
   for f in (OUT/'cache').glob(label+'_*'):f.unlink(missing_ok=True)
 if all((OUT/l/'metrics.json').exists() for l in LABELS):break
 time.sleep(5)
