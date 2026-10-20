#!/usr/bin/env python3
"""One same-input LIO/fixed-VIO/gimbal-VIO comparison per legacy high-error route.

Selection is fixed before new measurements: legacy DRAGON LIO position RMSE
above 1 metre (nine of fourteen). Old results select routes only; all new
comparisons use frozen UniPilot v57. Do not filter new results by their outcome.
"""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
NATIVE=HERE.parent
REFERENCE=NATIVE/'logs/prefix-pure-lio-20261019'
ORDER=['run2a_end','run1_tail_a2_prefix','run1_tail_a1','run1_tail_b1',
       'run2a_prefix','run1_tail_b2','run2b','run1_middle']


def read(path):return json.loads(path.read_text())


def verify_selected():
    selected=read(HERE/'selected_gimbal.json')
    checked={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in selected['files'].items()}
    if not all(checked.values()):raise RuntimeError('Frozen estimator/planner changed')
    return checked


def compress_completed(out):
    subprocess.run([sys.executable,str(HERE/'compress_finished_numeric.py'),str(out)],check=True)
    paths=[p for p in (out/'observers').rglob('*') if p.is_file() and p.suffix in ['.csv','.tum'] and p.stat().st_size>100000]
    paths.extend(out.glob('campaign/*_active/processing_times.jsonl'))
    records=[]
    for p in paths:
        dest=p.with_suffix(p.suffix+'.gz')
        if dest.exists():raise RuntimeError('Existing compression archive: '+str(dest))
        data=p.read_bytes();sha=hashlib.sha256(data).hexdigest()
        with gzip.open(dest,'wb',compresslevel=3) as stream:stream.write(data)
        with gzip.open(dest,'rb') as stream:
            if hashlib.sha256(stream.read()).hexdigest()!=sha:raise RuntimeError('Compression mismatch')
        records.append(dict(source=str(p.relative_to(out)),archive=str(dest.relative_to(out)),sha256=sha,
                            original_bytes=len(data),compressed_bytes=dest.stat().st_size))
        p.unlink()
    (out/'observer_compression.json').write_text(json.dumps(records,indent=2)+'\n')


def run_one(root,label,entry,template,seed):
    out=root/label
    out.mkdir(exist_ok=False)
    campaign=out/'campaign';campaign.mkdir()
    flight=campaign/(label+'_active')
    command=template.copy()
    def replace(flag,values):
        pos=command.index(flag)+1;command[pos:pos+len(values)]=[str(x) for x in values]
    replace('--route',[entry['route']]);replace('--route-end',[entry['route_end_s']])
    replace('--world',[entry['world']]);replace('--spawn',entry['spawn'])
    replace('--seed',[seed]);replace('--output',[flight])
    duration=max(1200.,3*entry['route_end_s']+300.)
    replace('--duration',[duration])
    sidecar=[sys.executable,str(HERE/'observe_pure_lio.py'),'--flight',str(flight),
             '--output',str(out/'observers'),'--port','11485']
    gpu=[sys.executable,str(HERE/'gpu_keepalive.py'),'--duration',str(duration+30),
         '--blocks','32','--interval','.001','--cycles','1000000']
    (out/'commands.json').write_text(json.dumps(dict(flight=command,observers=sidecar,gpu=gpu),indent=2)+'\n')
    (out/'selected_source_checks.json').write_text(json.dumps(verify_selected(),indent=2)+'\n')
    processes={};started=time.monotonic()
    try:
        for name,cmd in [('observers',sidecar),('gpu',gpu),('flight',command)]:
            with (out/(name+'.log')).open('w') as log:
                processes[name]=subprocess.Popen(cmd,cwd=NATIVE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (out/'owners.json').write_text(json.dumps({k:p.pid for k,p in processes.items()},indent=2)+'\n')
        while processes['flight'].poll() is None:
            if processes['observers'].poll() is not None and not (flight/'runtime.json').exists():
                raise RuntimeError('Observer exited before the flight')
            if processes['gpu'].poll() is not None:raise RuntimeError('GPU keepalive exited early')
            time.sleep(.5)
        if processes['flight'].returncode:raise RuntimeError('Flight failed')
        processes['observers'].wait(timeout=30)
        if processes['observers'].returncode:raise RuntimeError('Observer failed')
    finally:
        for p in processes.values():
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes.values():
            try:p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait()
        (out/'completion.json').write_text(json.dumps(dict(stopped=True,wall_s=time.monotonic()-started,
            processes={k:p.returncode for k,p in processes.items()}),indent=2)+'\n')
    with (out/'report.log').open('w') as log:
        subprocess.run([sys.executable,str(HERE/'report_pure_lio.py'),str(out)],stdout=log,stderr=subprocess.STDOUT,check=True)
    result=read(out/'comparison.json')
    shutil.copy2(out/'comparison.json',out/'comparison_before_compression.json')
    compress_completed(out)
    with (out/'report_after_compression.log').open('w') as log:
        subprocess.run([sys.executable,str(HERE/'report_pure_lio.py'),str(out)],stdout=log,stderr=subprocess.STDOUT,check=True)
    if read(out/'comparison.json')!=result:raise RuntimeError('Recomputed compressed result differs')
    return dict(status='complete' if result['valid'] else 'invalid',path=str(out),
                valid=result['valid'],metrics=result['metrics'],wall_s=time.monotonic()-started)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    path=root/'campaign.json'
    if path.exists():manifest=read(path)
    else:
        legacy=read(NATIVE/'logs/zab-offline/error_plots/evaluation.json')
        original=read(NATIVE/'logs/zab-full-campaign/results.json')
        selection={}
        for label,item in legacy.items():
            error=math.sqrt(sum(x*x for x in item['estimators']['lio']['rmse'][:3]))
            p=Path(original[label]['path']);plan=read(p/'route/route.json');segments=read(p/'route/segments.json')['segments'];world=read(p/'world.json')
            route_end=1070. if label=='run1_prefix' else original[label]['flight']['route_time_s']
            route_end=min(route_end,segments[-1]['t0']+segments[-1]['duration'])
            selection[label]=dict(selected=error>1.,legacy_lio_rmse_m=error,route=str(p/'route'),route_end_s=route_end,
                world=world['world_file'],spawn=[*plan['waypoints'][0][:2],world['spawn_xyz'][2]],
                selection_platform='DRAGON',new_comparison_platform='UniPilot')
        if {k for k,v in selection.items() if v['selected']}!=set(ORDER+['run1_prefix']):
            raise RuntimeError('Legacy selection changed; review before execution')
        prefix=read(REFERENCE/'comparison.json')
        if not prefix['valid']:raise RuntimeError('Reusable prefix result is invalid')
        manifest=dict(scope=__doc__,seed=20261026,threshold_m=1.,selection=selection,order=ORDER,
            status='prepared',runs={'run1_prefix':dict(status='reused',valid=True,path=str(REFERENCE),metrics=prefix['metrics'])})
        impl=root/'implementation';impl.mkdir(exist_ok=True)
        for name in ['run_lio_failure_comparisons.py','observe_pure_lio.py','report_pure_lio.py']:
            shutil.copy2(HERE/name,impl/name)
        (root/'selected_source_checks.json').write_text(json.dumps(verify_selected(),indent=2)+'\n')
    def save():
        temp=path.with_suffix('.tmp');temp.write_text(json.dumps(manifest,indent=2)+'\n');temp.replace(path)
    save()
    if args.prepare_only:
        print(json.dumps(dict(selected=[k for k,v in manifest['selection'].items() if v['selected']],
            new_simulation_seconds=sum(manifest['selection'][k]['route_end_s'] for k in ORDER)),indent=2));return
    def interrupt(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    manifest['status']='running';save()
    template=read(REFERENCE/'campaign/campaign.json')['runs']['run1_prefix_active']['command']
    try:
        for label in manifest['order']:
            if label in manifest['runs']:
                if manifest['runs'][label]['status'] in ['complete','reused']:continue
                raise RuntimeError('Previous incomplete attempt retained: '+label)
            manifest['runs'][label]=dict(status='running',path=str(root/label));save()
            print('START',label,manifest['selection'][label]['route_end_s'],flush=True)
            try:manifest['runs'][label]=run_one(root,label,manifest['selection'][label],template,manifest['seed'])
            except Exception as exc:
                manifest['runs'][label].update(status='failed',error=str(exc));save();print('FAILED',label,str(exc),flush=True)
                # An infrastructure failure requires inspection before creating more data.
                raise
            save();print('FINISHED',label,manifest['runs'][label]['status'],flush=True)
        manifest['status']='complete'
    except KeyboardInterrupt:
        manifest['status']='interrupted';raise
    except Exception:
        manifest['status']='failed';raise
    finally:save()


if __name__=='__main__':main()
