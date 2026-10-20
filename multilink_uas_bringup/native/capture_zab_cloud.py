#!/usr/bin/env python3
"""Finish the paused ZaB capture with verified cloud offload and disk backpressure."""
import concurrent.futures
import argparse
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from cloud_archive import ROOT, REMOTE, archive_split, finalize, save, prepare_remote

CAMPAIGN = ROOT/'logs/zab-full-campaign'
LEDGER = ROOT/'routes/zab_full/capture_runs.json'


def world_pause(paused):
    # Setting the same pause state again is safe when a reply was lost.
    for attempt in range(3):
        try:
            result = subprocess.run(['gz','service','-s','/world/zab/control',
                '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                '--timeout','5000','--req',f'pause: {str(paused).lower()}'],
                capture_output=True,text=True,timeout=8,check=True)
            if 'data: true' in result.stdout:
                return
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            pass
        if attempt < 2:
            time.sleep(1)
    raise RuntimeError('Gazebo pause/resume was not acknowledged after 3 attempts')


def running(run):
    try:
        return Path(json.loads((ROOT/'state.json').read_text())['logs']).resolve() == run.resolve()
    except FileNotFoundError:
        return False


def process_run(label, run, initially_paused=False):
    prepare_remote(run)
    paused = initially_paused
    submitted = set()
    futures = {}
    last_print = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        while True:
            active = running(run)
            files = sorted((run/'rosbag').glob('*.db3'), key=lambda p:int(p.stem.rsplit('_',1)[1]))
            eligible = files[:-1] if active else files
            for source in eligible:
                if source not in submitted and len(futures)<2:
                    futures[pool.submit(archive_split,run,source)] = source
                    submitted.add(source)
            for future in list(futures):
                if future.done():
                    source = futures.pop(future)
                    try:
                        receipt = future.result()
                    except Exception:
                        if active:
                            world_pause(True)
                        raise
                    print(json.dumps(dict(event='archived',component=label,file=source.name,
                                          bytes=receipt['compressed_bytes'])),flush=True)
            backlog = 0
            for p in (run/'rosbag').glob('*.db3'):
                try:
                    backlog += p.stat().st_size
                except FileNotFoundError:
                    pass  # Verified upload just removed this closed split.
            host_free = shutil.disk_usage('/mnt/c').free
            status = json.loads((run/'route_status.json').read_text())
            if status.get('failure'):
                if active:
                    world_pause(True)
                raise RuntimeError('Flight failed: '+str(status))
            if active:
                if not paused and (backlog>2.5*1024**3 or host_free<8e9):
                    world_pause(True)
                    paused=True
                elif paused and backlog<1.2*1024**3 and host_free>=9e9:
                    world_pause(False)
                    paused=False
            if time.monotonic()-last_print>=45:
                progress=dict(component=label,stage=status['stage'],cloud_paused=paused,
                    route_s=status['route_time_s'],duration_s=status['route_duration_s'],
                    sim=status['sim_time'],tracking_error=status['tracking_error_m'],
                    local_raw_gb=backlog/1e9,windows_free_gb=host_free/1e9,
                    archived_chunks=len(list((run/'cloud_chunks').glob('*.json'))),run=str(run))
                save(CAMPAIGN/'progress.json',progress)
                print(json.dumps(progress),flush=True)
                last_print=time.monotonic()
            if not active and not files and not futures:
                break
            time.sleep(2)
    if status['stage']!='complete':
        raise RuntimeError('Flight did not complete: '+str(status))
    result=finalize(run)
    for name,s in result['sensors'].items():
        if s['missing_intervals'] or s['nonmonotonic']:
            raise RuntimeError(f'Sensor gaps/order errors: {name}: {s}')
    print(label+' complete; cloud archive verified',flush=True)
    return dict(path=str(run),flight=status,bag=result)


def main(resume=False, labels=None):
    CAMPAIGN.mkdir(exist_ok=True)
    runs=json.loads(LEDGER.read_text())
    if not resume and (runs or (ROOT/'state.json').exists()):
        raise RuntimeError('Fresh capture requires an empty run ledger and no active simulation')
    save(CAMPAIGN/'campaign_storage.json',dict(remote=REMOTE,raw_storage='Linux ext4',conversion_spool='/mnt/c/Users/j-sug/AppData/Local/uas-cloud-spool'))
    result_path=CAMPAIGN/'results.json'
    results=json.loads(result_path.read_text()) if resume and result_path.exists() else {}
    for label in (labels or ['run3','run1','run2','run4']):
        if label in results:
            continue
        proc=None
        existing=label in runs
        if not existing:
            # Keep 6 GB on the real Windows volume, with at most 2.5 GiB
            # raw backlog and two temporary compressed splits.
            route=ROOT/'routes/zab_full'/label
            plan=json.loads((route/'route.json').read_text())
            plan['disk_reserve_gb']=6
            save(route/'route.json',plan)
            launch_log=(CAMPAIGN/(label+'-cloud-launch.log')).open('w')
            proc=subprocess.Popen(['ros2','launch','multilink_uas_bringup','zab.launch.py',
                'headless:=true','rviz:=false','estimators:=false','record:=true',f'route:={route}'],
                stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True,
                env={k:v for k,v in os.environ.items() if k!='UAS_RECORD_SPOOL'})
            deadline=time.monotonic()+60
            while not (ROOT/'state.json').exists():
                if proc.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Capture failed to start')
                time.sleep(.2)
            runs[label]=json.loads((ROOT/'state.json').read_text())['logs']
            save(LEDGER,runs)
            # Wait for status while startup initializes the controller.
            status=Path(runs[label])/'route_status.json'
            while not status.exists():
                if proc.poll() is not None or time.monotonic()>deadline+60:
                    raise RuntimeError('Flight status did not appear')
                time.sleep(.2)
        run=Path(runs[label]).resolve()
        if existing and running(run):
            world_pause(True)
        results[label]=process_run(label,run,initially_paused=existing and running(run))
        if proc is not None:
            proc.wait(timeout=30)
            launch_log.close()
        save(CAMPAIGN/'results.json',results)
    print('REQUESTED CAPTURES COMPLETE',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--labels', nargs='+')
    args=parser.parse_args()
    try:
        main(args.resume, args.labels)
    except BaseException:
        if (ROOT/'state.json').exists():
            try:
                world_pause(True)
            except Exception as exc:
                print('Pause during shutdown was unavailable: '+str(exc),flush=True)
        raise
