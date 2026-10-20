#!/usr/bin/env python3
"""Preserve calibration, route and implementation alongside verified bag data."""
import sys,tarfile
from pathlib import Path
from cloud_archive import ROOT,REMOTE,upload
for name in sys.argv[1:]:
 run=Path(name).resolve()
 dest=run/'capture_context.tar.gz'
 if not dest.exists():
  with tarfile.open(dest,'w:gz') as out:
   for p in sorted(run.iterdir()):
    if p.is_file() and p.suffix in ('.json','.yaml','.urdf','.info'):
     out.add(p,arcname=p.name)
   for label in ('route','implementation'):
    if (run/label).exists():out.add(run/label,arcname=label)
   for patch in ('gz-ros2-control-pause.patch','spinal-sim-clock.patch'):
    out.add(ROOT/patch,arcname='current_environment_patches/'+patch)
 with (run/'context_upload.log').open('a') as log:
  upload(dest,f'{REMOTE}/{run.name}/capture_context.tar.gz',log)
 print('context verified',run.name,flush=True)
