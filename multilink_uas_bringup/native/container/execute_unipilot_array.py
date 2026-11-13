#!/usr/bin/env python3
"""Host owner: run the full campaign, audit cleanup, then stop its container."""
import datetime,json,subprocess,sys
from pathlib import Path

C='multilink-comparison-20261111'
NAME='unipilot-array-heldout-20261113-v2'
NATIVE=Path(__file__).resolve().parents[1]
ROOT=NATIVE/'logs'/NAME
remote='/root/uas_native/logs/'+NAME
worlds=[f'/root/multilink_autonomy_ws/assets/fair_dragon_20261113/{seed}/world.sdf' for seed in range(24611,24621)]
command=['docker','exec',C,'bash','-lc','source /root/uas_native/env.sh\nexec env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 /root/uas_native/active_perception/run_unipilot_array_campaign.py --output '+remote+' --jobs 3 --worlds '+' '.join(worlds)]
rc=1
try:
    rc=subprocess.run(command).returncode
finally:
    code='''import os,json
from pathlib import Path
r=Path("REMOTE_ROOT")
live=[]
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
 try:
  env=(proc/'environ').read_bytes().split(b'\\0')
  if any(f'ROS_DOMAIN_ID={161+i}'.encode() in env or f'ROS_MASTER_URI=http://127.0.0.1:{12001+i}'.encode() in env for i in range(3)):
   live.append(dict(pid=int(proc.name),command=(proc/'cmdline').read_bytes().replace(b'\\0',b' ').decode(errors='replace')))
 except OSError:pass
if r.exists():
 (r/'owned_process_audit.json').write_text(json.dumps(dict(passed=not live,remaining=live),indent=2))
 for p in [r,*r.rglob('*')]:
  if not p.is_symlink():os.chown(p,1000,1000)
'''.replace('REMOTE_ROOT',remote)
    audit=subprocess.run(['docker','exec','-i',C,'python3','-'],input=code,text=True).returncode
    stopped=subprocess.run(['docker','stop',C]).returncode
    state=json.loads(subprocess.check_output(['docker','inspect',C,'--format','{{json .State}}'],text=True))
    if ROOT.exists():
        (ROOT/'shutdown_audit.json').write_text(json.dumps(dict(passed=stopped==0 and not state['Running'] and state['Pid']==0 and not state['OOMKilled'],state=state,pipeline_returncode=rc,process_audit_returncode=audit,completed_at=datetime.datetime.now(datetime.timezone.utc).isoformat()),indent=2))
print('HOST_PIPELINE_FINISHED',rc,flush=True)
sys.exit(rc)
