#!/usr/bin/env python3
"""Six predeclared 90 m flights: matched scoring-only view ablation."""
import argparse,hashlib,json,shutil,signal,subprocess,sys,time
from pathlib import Path
import yaml
from run_lio_failure_comparisons import HERE,NATIVE,INPUTS,read,run_one,verify_selected
from run_zab_additional_conditions import save
from weathered_trial_acceptance import trial_acceptance


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assets',type=Path,default=Path('/root/multilink_autonomy_ws/assets/weathered_tunnel_20261024'))
    a=p.parse_args();root=a.output.resolve();root.mkdir(parents=True,exist_ok=True);path=root/'campaign.json'
    if path.exists():m=read(path)
    else:
        configs=root/'configs';configs.mkdir()
        cfg=yaml.safe_load((HERE/'motion_view_v10.yaml').read_text())
        for mode in ['lidar_conditioned','visual_only']:
            (configs/(mode+'.yaml')).write_text(yaml.safe_dump(dict(cfg,view_information_mode=mode)))
        entry=read(a.assets/'catalog.json')['weathered24201'];seeds=[20262701,20262702,20262703]
        order=[]
        for i,s in enumerate(seeds):
            modes=['lidar_conditioned','visual_only'] if i%2==0 else ['visual_only','lidar_conditioned']
            order.extend(f'{mode}_seed{s}' for mode in modes)
        files=set(read(HERE/'selected_gimbal.json')['files'])
        files.update(str(f) for f in HERE.rglob('*') if f.is_file() and f.suffix in ['.py','.yaml','.json','.hpp','.cpp','.info'] and '__pycache__' not in f.parts)
        files.update(str(f) for f in a.assets.rglob('*') if f.is_file())
        files.update(str(f) for f in configs.glob('*.yaml'))
        frozen={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in sorted(files)}
        m=dict(status='prepared',created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),entries={mode:entry for mode in ['lidar_conditioned','visual_only']},order=order,runs={},source_sha256=frozen,
            design=dict(seeds=seeds,segments_s=[[10.,170.]],distance_m=90,speed_mps=.5,drpm=False,
              primary_metric='Actual accepted visual velocity axial RMS and availability; compare complete RMS only with 100 percent coverage.',
              secondary_metrics=['VIO 10 s axial displacement RMS','fused 10 s axial displacement RMS','full fused position RMSE','camera angles and controller decisions'],
              difference='Only score input H/U/weak: visual_only uses zero LiDAR information, identity directions, equal translation weights. Same finite prior, visual joint covariance and forecast, candidates, retention, recovery, rate/angle limits, common preparation, estimator and fusion.',
              interpretation='Visual-only means no LiDAR information matrix in view scoring. Both arms still use MIMOSA state/time synchronization and identical LiDAR-visual fusion. Not an IGLOV reproduction.',
              comparison_unit='Three separately simulated trials per arm with matching requested seeds and alternating order; not identical sensor streams across arms. Five fixed views and pure LIO remain same-flight controls.',
              acceptance='All acquisition checks required; predeclared planner_continuous hold outcome allowed and reported separately, following completed weathered protocol.',
              exclusion='No accuracy-based exclusion. Infrastructure failures retained; resume requires inspection. Truth only for body control and posthoc evaluation. No parameter tuning during trials.'))
        save(path,m)
        snapshot=root/'implementation';snapshot.mkdir()
        for name in ['view_information_ablation.py','ablation_planner_node.py','run_information_ablation_array.py','run_information_ablation_trials.py','report_information_ablation.py','test_view_information_ablation.py']:
            shutil.copy2(HERE/name,snapshot/name)
    def check():
        changed=[p for p,s in m['source_sha256'].items() if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=s]
        if changed:raise RuntimeError('Frozen inputs changed: '+str(changed))
        if shutil.disk_usage('/mnt/c').free<8*2**30:raise RuntimeError('Windows free space below 8 GiB')
        verify_selected()
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    check();m['status']='running';save(path,m)
    try:
        for key in m['order']:
            if key in m['runs']:
                if m['runs'][key]['status']=='complete':continue
                raise RuntimeError('Inspect unfinished attempt before resume: '+key)
            check();mode,seed=key.rsplit('_seed',1)
            template=read(INPUTS/'templates/paired_flight.json');template[1]=str(HERE/'run_information_ablation_array.py')
            template[template.index('--planner-config')+1]=str(root/'configs'/(mode+'.yaml'))
            m['runs'][key]=dict(status='running',mode=mode);save(path,m);print('START',key,flush=True)
            result=run_one(root,key,m['entries'][mode],template,int(seed),record_geometry=True)
            subprocess.run([sys.executable,str(HERE/'report_view_array.py'),'--trial',str(root/key)],check=True)
            acceptance=trial_acceptance(root/key)
            geo=read(root/key/'observers/common_lidar.jsonl.status.json')
            if geo.get('samples',0)<100:raise RuntimeError('Missing geometry')
            result.update(status='complete',mode=mode,acceptance=acceptance,geometry=geo)
            m['runs'][key]=result;check();save(path,m);print('FINISHED',key,flush=True)
            subprocess.run([sys.executable,str(HERE/'report_information_ablation.py'),str(root)],check=True)
        m['status']='complete';check()
    except BaseException as exc:
        m['status']='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';m['error']=repr(exc);raise
    finally:save(path,m)
    subprocess.run([sys.executable,str(HERE/'report_information_ablation.py'),str(root)],check=True)

if __name__=='__main__':main()
