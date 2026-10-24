#!/usr/bin/env python3
"""Frozen-policy comparison on predeclared texture layouts; resume completed flights."""
import argparse,hashlib,json,shutil,signal,subprocess,sys,time
from pathlib import Path
from run_lio_failure_comparisons import HERE,NATIVE,INPUTS,read,run_one,verify_selected
from run_zab_additional_conditions import save
from build_switching_tunnel import SEGMENTS

def main():
    p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--frozen',type=Path,required=True);a=p.parse_args()
    root=a.output.resolve();root.mkdir(parents=True,exist_ok=True);frozen=read(a.frozen)
    def check():
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==s for p,s in frozen['files'].items()),'Frozen inputs changed'
        assert shutil.disk_usage('/mnt/c').free>8*2**30,'Host storage guard'
        verify_selected()
    check();path=root/'campaign.json';cfg=HERE/'motion_view_v10.yaml'
    if path.exists():m=read(path)
    else:
        entries=read(a.assets/'catalog.json');seeds=[20262401,20262402,20262403]
        m=dict(status='prepared',stage='validation',entries=entries,order=[f'{r}_seed{s}' for s in seeds for r in entries],runs={},source_sha256=frozen['files'],design=dict(
            seeds=seeds,segments_s={k:SEGMENTS for k in entries},primary_metric='Actual VIO velocity error along known tunnel axis X; pooled fixed predeclared regions, then paired flight-level comparison.',secondary='Common estimated weak direction, 3D velocity, fused local displacement and full trajectory; fixed-best-in-test is retrospective.',comparison='Same physical flight, five independent fixed monocular estimators plus active v10. DRPM OFF; no policy, estimator, weight or quality tuning during validation. Truth for body control and posthoc scoring only.',fixed_comparators='All five fixed directions; uniform mean; one best fixed direction over the entire flight; per-zone oracle shown separately.',stage='validation',synthetic_environment=True,layout_input_to_controller=False))
        impl=root/'implementation';impl.mkdir();shutil.copy2(cfg,impl/cfg.name);shutil.copy2(a.frozen,root/'frozen.json');save(path,m)
    template=read(INPUTS/'templates/paired_flight.json');template[1]=str(HERE/'run_guarded_view_array.py');template[template.index('--planner-config')+1]=str(cfg)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    m['status']='running';save(path,m)
    try:
        for key in m['order']:
            if key in m['runs']:
                if m['runs'][key].get('status')=='complete':continue
                raise RuntimeError('Inspect retained unfinished trial before resume: '+key)
            check();route,seed=key.rsplit('_seed',1);m['runs'][key]=dict(status='running');save(path,m);print('START',key,flush=True)
            result=run_one(root,key,m['entries'][route],template,int(seed),record_geometry=True)
            subprocess.run([sys.executable,str(HERE/'report_view_array.py'),'--trial',str(root/key)],check=True)
            array=read(root/key/'array_comparison.json');geo=read(root/key/'observers/common_lidar.jsonl.status.json')
            if not result['valid'] or not array['valid'] or geo.get('samples',0)<100:raise RuntimeError('Trial audit failed: '+key)
            result.update(array_valid=True,geometry=geo);m['runs'][key]=result;save(path,m);print('FINISHED',key,flush=True)
        m['status']='complete';check()
    except BaseException as exc:m['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';m['error']=repr(exc);raise
    finally:save(path,m)
    subprocess.run([sys.executable,str(HERE/'report_visual_constraint_survey.py'),str(root)],check=True)
    subprocess.run([sys.executable,str(HERE/'report_switching_tunnel.py'),str(root)],check=True)

if __name__=='__main__':main()
