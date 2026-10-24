#!/usr/bin/env python3
"""Resume a frozen campaign; retain audit-failed trials before a single retry."""
import argparse,hashlib,json,shutil,signal,subprocess,sys,time
import yaml
from pathlib import Path
from run_lio_failure_comparisons import HERE,INPUTS,read,run_one,verify_selected
from run_zab_additional_conditions import save

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--retry-invalid',action='store_true');p.add_argument('--frozen',type=Path);a=p.parse_args();root=a.root.resolve();m=read(root/'campaign.json')
 sha=dict(m['source_sha256']);sha.update(read(a.frozen)['files'] if a.frozen else {});assert all(hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in sha.items()),'Frozen sources changed'
 verify_selected();configs=list((root/'implementation').glob('motion_view_v*.yaml'));assert len(configs)==1
 config=HERE/configs[0].name;assert config.read_bytes()==configs[0].read_bytes()
 source=root/'resume_implementation';source.mkdir(exist_ok=True);shutil.copy2(__file__,source/Path(__file__).name)
 template=read(INPUTS/'templates/paired_flight.json');cfg=yaml.safe_load(config.read_text());cycles=cfg.get('common_preparation_cycles',1);assert cycles in (1,2),'Unsupported preparation protocol';template[1]=str(HERE/('run_preview_view_array.py' if cfg.get('preview_preparation') else 'run_guarded_view_array.py' if cfg.get('committed_track_recovery') else 'run_prepared_view_array.py' if cycles==2 else 'run_motion_view_array.py'));template[template.index('--planner-config')+1]=str(config)
 def stop(*_):raise KeyboardInterrupt
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
 m['status']='running';m.setdefault('resumptions',[]).append(dict(unix_time=time.time(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),retry_invalid=a.retry_invalid));save(root/'campaign.json',m)
 try:
  for key in m['order']:
   if m['runs'].get(key,{}).get('status')=='complete':continue
   assert all(hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in sha.items())
   old=root/key
   if old.exists():
    assert a.retry_invalid,'Existing unfinished trial needs explicit retry'
    completion=read(old/'completion.json');assert completion['stopped'] and all(v==0 for v in completion['processes'].values())
    for pid in read(old/'owners.json').values():assert not Path('/proc',str(pid)).exists(),'Recorded owner still exists'
    c=read(old/'comparison.json');ar=read(old/'array_comparison.json');assert not (c['valid'] and ar['valid']),'Do not repeat valid results'
    excluded=root/'invalid_trials';excluded.mkdir(exist_ok=True);destination=excluded/key;assert not destination.exists()
    old.rename(destination)
    m.setdefault('invalid_trials',[]).append(dict(trial=key,path=str(destination),reason='Failed pre-existing trial audit; exact failed checks preserved below',failed_checks={n:[k for k,v in d['checks'].items() if not v] for n,d in [('reference',c),('array',ar)]}))
    save(destination/'relocation.json',dict(original_path=str(old),archived_path=str(destination),reason='Preserve invalid attempt before one retry; internal absolute paths refer to original acquisition path'))
   route,seed=key.rsplit('_seed',1);m['runs'][key]=dict(status='running');save(root/'campaign.json',m);print('START',key,flush=True)
   r=run_one(root,key,m['entries'][route],template,int(seed),record_geometry=True)
   subprocess.run([sys.executable,str(HERE/'report_view_array.py'),'--trial',str(root/key)],check=True)
   ar=read(root/key/'array_comparison.json');geo=read(root/key/'observers/common_lidar.jsonl.status.json');r.update(array_valid=ar['valid'],geometry=geo)
   m['runs'][key]=r;save(root/'campaign.json',m)
   if not r['valid'] or not ar['valid'] or geo.get('samples',0)<100:raise RuntimeError('Trial audit failed: '+key)
   print('FINISHED',key,flush=True);subprocess.run([sys.executable,str(HERE/'report_visual_constraint_survey.py'),str(root)],check=True)
  m['status']='complete';m['final_selected_checks']=verify_selected()
 except BaseException as exc:m['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';m['error']=repr(exc);raise
 finally:save(root/'campaign.json',m)
 subprocess.run([sys.executable,str(HERE/'report_visual_constraint_survey.py'),str(root)],check=True)
 subprocess.run([sys.executable,str(HERE/'report_motion_candidates.py'),'--output',str(root/'report/candidate'),str(root)],check=True)
if __name__=='__main__':main()
