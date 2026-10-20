#!/usr/bin/env python3
"""Inspect/resume the explicitly paused live all-flight campaign.
This is a live-process pause, not an estimator checkpoint that survives reboot.
"""
import argparse,json,os,signal,time
from pathlib import Path
OUT=Path(__file__).resolve().parent/'logs/zab-offline/fast_multicamera_all'
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['status','resume']);a=p.parse_args()
 f=OUT/'pause_state.json';s=json.loads(f.read_text());errors=[];current=[]
 if s['status']=='complete':
  print('All 14 flights are complete; owned processes and temporary resources were cleaned up.')
  if a.action=='resume':raise SystemExit('Already complete; no process resumed.')
  return
 if s['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():errors.append('WSL/kernel restarted; live estimator state was lost.')
 for r in s['processes']:
  try:
   fields=Path(f"/proc/{r['pid']}/stat").read_text().rsplit(')',1)[1].split()
   if int(fields[19])!=r['start_ticks'] or os.getpgid(r['pid'])!=r['pgid']:raise ValueError('Process identity changed')
   current.append(dict(pid=r['pid'],state=fields[0]))
   if fields[0]=='Z':errors.append(f"Process {r['pid']} has exited")
  except (FileNotFoundError,ProcessLookupError,ValueError):errors.append(f"Process {r['pid']} missing or replaced")
 print(json.dumps(dict(status=s['status'],progress=s['progress'],processes=current,errors=errors),indent=2))
 if a.action=='status':return
 if errors:raise SystemExit('Cannot resume safely. Preserve completed 13 runs; archive partial run1_middle and rerun only that flight. See RESUME.md.')
 if s['status']!='paused':raise SystemExit('Campaign is not recorded as paused; no signals sent.')
 if not all(r['state'] in ['T','t'] for r in current):raise SystemExit('Not all owned processes are paused; inspect first.')
 for pg in s['resume_order']:os.killpg(pg,signal.SIGCONT)
 s['status']='running';s['resumed_at']=time.time();f.write_text(json.dumps(s,indent=2))
 r=json.loads((OUT/'runtime.json').read_text());r['status']='running';r['last_resume_at']=s['resumed_at'];(OUT/'runtime.json').write_text(json.dumps(r,indent=2));print('Resumed existing estimator, source reader/downloads, ROS and finalizer. Monitor campaign_resumed.log and finalize_status.json.')
if __name__=='__main__':main()
