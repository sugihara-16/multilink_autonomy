#!/usr/bin/env python3
"""Continue the frozen validation, at most two simulations and staggered long runs."""
import argparse,json,os,signal,subprocess,sys,time,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent

def read(path):
    try:return json.loads(path.read_text())
    except (FileNotFoundError,json.JSONDecodeError):return {}

def next_prefix_domain(first_finished,first_time,second_short_finished,available_bytes,separate):
    if available_bytes<=3e9:return None
    # Reuse the released long-run slot while the second short campaign is
    # still running. Separate campaign directories avoid concurrent writers.
    if first_finished and (separate or second_short_finished):return 85
    if second_short_finished and first_time>=750:return 86
    return None

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--candidate',default='v6')
    p.add_argument('--development-prefix',default='v6-ordered-excited-prefix-seed17')
    p.add_argument('--seeds',nargs=2,type=int,default=[20261018,20261019])
    p.add_argument('--separate-prefix',action='store_true',help='Separate output owners let the long run overlap the same-seed short runs')
    p.add_argument('--fusion-imu-noise',choices=['legacy','sensor','sensor_gyro'],default='legacy')
    p.add_argument('--camera-far',type=float,default=None)
    p.add_argument('--max-concurrent',type=int,choices=[1,2],default=1)
    a=p.parse_args();a.root=a.root.resolve()
    frozen=read(a.root/f'frozen_candidate_{a.candidate}.json')
    if frozen.get('validation_seeds')!=a.seeds:raise ValueError('Seeds do not match frozen protocol')
    for key,value in [('camera_far',a.camera_far),('fusion_imu_noise',a.fusion_imu_noise)]:
        if key in frozen.get('protocol_fields',{}) and frozen['protocol_fields'][key]!=value:
            raise ValueError('Runtime setting does not match frozen protocol: '+key)
    first,second=a.seeds
    def folder(seed,prefix=False):return a.root/(f'validation-{a.candidate}-seed{seed-20261000}'+('-prefix' if prefix and a.separate_prefix else ''))
    def verify_frozen():
        for filename,expected in frozen['files'].items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest()!=expected:
                raise RuntimeError('Frozen candidate changed: '+filename)
    verify_frozen()
    started={};finished=set();stopping=False
    def interrupt(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    def campaign(key,seed,labels,domain):
        verify_frozen()
        cmd=[sys.executable,str(HERE/'unipilot_campaign.py'),'--output',str(folder(seed,labels==['run1_prefix'])),'--labels',*labels,
            '--paired','--shared-initialization','--initialization-excitation','--fast','--airborne-init','--gpu-keepalive',
            '--jobs','1','--factor','1.2','--paced','--fast-icp','--compact-map','--map-variant','prefetch',
            '--native-adapters','--native-monitor','--clock-hz','1000','--ordered-fusion','--tbb-threads','1',
            '--wall-limit','2700','--domain-base',str(domain),'--planner-config',str(HERE/f'tracking_aware_{a.candidate}.yaml'),
            '--min-host-free-gb','8',
            '--diagnostic-vio','--seed',str(seed),'--fusion-calibration',frozen.get('fusion_calibration',str(a.root/'native_fusion_calibration.json')),'--fusion-imu-noise',frozen.get('protocol_fields',{}).get('fusion_imu_noise',a.fusion_imu_noise),'--corner-speed-limit']
        if a.camera_far is not None:cmd+=['--camera-far',str(a.camera_far)]
        if frozen.get('drpm_fusion',False):cmd.append('--drpm-fusion')
        if frozen.get('protocol_fields',{}).get('safe_shutdown',False):cmd.append('--safe-shutdown')
        if frozen.get('protocol_fields',{}).get('tilt_fusion',False):cmd.append('--tilt-fusion')
        if frozen.get('protocol_fields',{}).get('tilt_translation',False):cmd.append('--tilt-translation')
        if frozen.get('protocol_fields',{}).get('fixed_gravity_imu',False):cmd.append('--fixed-gravity-imu')
        if frozen.get('protocol_fields',{}).get('sensor_bias_model',False):cmd.append('--sensor-bias-model')
        if frozen.get('protocol_fields',{}).get('batched_fusion',False):cmd.append('--batched-fusion')
        if frozen.get('protocol_fields',{}).get('joint_rovio_information',False):cmd.append('--joint-rovio-information')
        if frozen.get('protocol_fields',{}).get('geometry_rovio_information',False):cmd.append('--geometry-rovio-information')
        if frozen.get('protocol_fields',{}).get('lidar_four_dof',False):cmd.append('--lidar-four-dof')
        if frozen.get('protocol_fields',{}).get('fusion_initial_velocity_sigma') is not None:
            cmd.extend(['--fusion-initial-velocity-sigma',str(frozen['protocol_fields']['fusion_initial_velocity_sigma'])])
        if frozen.get('protocol_fields',{}).get('velocity_fusion',False):cmd.append('--velocity-fusion')
        with (a.root/(key+'.log')).open('w') as log:proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,cwd=NATIVE)
        started[key]=proc;print('START',key,proc.pid,flush=True)
    try:
        while len(finished)<3:
            for key,proc in list(started.items()):
                if key in finished or proc.poll() is None:continue
                if proc.returncode:raise RuntimeError(f'{key} failed; preserve its incomplete results')
                finished.add(key);print('DONE',key,flush=True)
            short18=read(folder(first)/'campaign.json')
            # The existing first-seed a1/b1 campaign is owned by the calling agent.
            short18_done=all(short18.get('runs',{}).get(label+'_active',{}).get('status')=='route_complete' for label in ['run1_tail_a1','run1_tail_b1'])
            def slot_available():return len(set(started)-finished)+(not short18_done)<a.max_concurrent
            if short18.get('status')=='failed' and not short18_done:raise RuntimeError('Existing seed18 short campaign failed')
            if short18_done and 'validation19_short' not in started and slot_available():
                campaign('validation19_short',second,['run1_tail_a1','run1_tail_b1'],86)
            dev=read(a.root/a.development_prefix/'run1_prefix_active/runtime.json')
            if dev.get('stopped') and (short18_done or a.separate_prefix) and 'validation18_prefix' not in started and slot_available():
                if dev.get('status')!='route_complete':raise RuntimeError('Development prefix failed')
                campaign('validation18_prefix',first,['run1_prefix'],85)
            prefix=read(folder(first,True)/'run1_prefix_active/route_status.json')
            mem=dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
            available=int(mem['MemAvailable'].strip().split()[0])*1024
            if 'validation18_prefix' in started and 'validation19_prefix' not in started and slot_available():
                # At 750/1070 s the first long run has ~5 min left. Starting
                # another only then avoids two late-flight render caches at once.
                domain=next_prefix_domain('validation18_prefix' in finished,prefix.get('route_time_s',0),
                    'validation19_short' in finished,available,a.separate_prefix)
                if domain is not None:
                    campaign('validation19_prefix',second,['run1_prefix'],domain)
            tmp=a.root/'validation_progress.tmp';tmp.write_text(json.dumps(dict(started={k:p.pid for k,p in started.items()},finished=sorted(finished),available_memory_bytes=available),indent=2)+'\n');tmp.replace(a.root/'validation_progress.json')
            time.sleep(3)
        verify_frozen()
        for seed in a.seeds:
            paths=[folder(seed)]+([folder(seed,True)] if a.separate_prefix else [])
            for path in paths:subprocess.run([sys.executable,str(HERE/'report_gimbal_campaign.py'),str(path),'--plots',*(['--drpm-fusion'] if frozen.get('drpm_fusion',False) else [])],check=True)
        subprocess.run([sys.executable,str(HERE/'report_gimbal_improvement.py'),str(a.root),'--candidate',a.candidate],check=True)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for proc in started.values():
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
        for proc in started.values():
            if proc.poll() is None:proc.wait(timeout=45)
if __name__=='__main__':main()
