#!/usr/bin/env python3
"""Frozen, symmetric DRAGON mechanism comparison; preserve every attempt."""
import argparse,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
NATIVE=HERE.parent
WS=NATIVE.parents[3]

def write(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worlds',type=Path,nargs='+',required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=[2026111301,2026111302,2026111303])
    p.add_argument('--jobs',type=int,default=5)
    p.add_argument('--arms',nargs='+',choices=['gimbal','joint','gimbal_v10'],default=['gimbal_v10','joint','gimbal'])
    p.add_argument('--development',action='store_true')
    a=p.parse_args();root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    files=set(HERE.glob('*.py'))|set(HERE.glob('*.cpp'))|set(HERE.glob('*.hpp'))|set(HERE.glob('*.yaml'))|set(NATIVE.glob('*.py'))|set(NATIVE.glob('*.sh'))
    for w in a.worlds:files.update(x for x in w.parent.rglob('*') if x.is_file())
    files.update([WS/'build/rovio-articulated/rovio_articulated',WS/'build/mimosa-batched-vision/mimosa_batched_node',WS/'build/mimosa-batched-drpm-support-view/libmimosa_lib.so'])
    robot_ws=Path('/root/ros2/aerial_robot_base_ws')
    for package in ('gimbalrotor','dragon','aerial_robot_control','aerial_robot_model'):
        files.update((robot_ws/'install'/package).rglob('*.so'))
    files.update((robot_ws/'src/aerial_robot_base/robots/dragon/config').glob('*.yaml'))
    hashes={str(x):digest(x) for x in sorted(files) if x.is_file()}
    tasks=[]
    for k,w in enumerate(a.worlds):
        for j,base_seed in enumerate(a.seeds):
            seed=base_seed+100*k
            order=a.arms[(k+j)%len(a.arms):]+a.arms[:(k+j)%len(a.arms)]
            if ((k*len(a.seeds)+j)//len(a.arms))%2:order=order[::-1]
            for arm in order:
                tasks.append(dict(layout=k,world=str(w),seed=seed,arm=arm,name=f'layout{k}_{arm}_seed{seed}'))
    protocol=dict(stage='development' if a.development else 'held_out',tasks=tasks,seeds=a.seeds,arms=a.arms,
        seed_policy='Unique noise seeds per layout: requested base seed + 100 * zero-based layout index; same nominal seed for paired arms, not identical raw noise streams',
        jobs=a.jobs,start_spacing_wall_s=3.,worlds=list(map(str,a.worlds)),distance_m=90,speed_mps=.5,origin=[5,0,2],
        configuration='--mechanism-comparison --fusion-profile drpm_isotropic20_view; same full SE3 estimator, mass, mount, camera, IMU, LiDAR, route, preparation. gimbal/joint share planner; gimbal_v10 is established gimbal control with the same estimator.',
        primary='Joint method versus established gimbal_v10: full 182 s position RMSE, first-pose alignment only; equal weight per layout and run. Compare shared-planner gimbal separately to identify view-policy effects; never pick a weak baseline after results.',
        preparation='All arms first receive an additional fixed-view 26 s lateral-first excitation immediately after initialization, before approach. No resets or quality-based exclusions. All arms may select a view during approach and first excitation; all hold their selected view in second excitation. No truth or texture labels enter view selection.',
        secondary=['ROVIO RMSE','physical completion','integration and shutdown','paired layout differences and uncertainty','camera and joint motion'],
        failure_policy='Every planned flight retained. No accuracy-based retry or exclusion. Incomplete intervals are failures, not shortened RMSE. Infrastructure failure before any flight may be separately repeated only with an amendment.',
        interpretation='Safe feasible mechanism comparison, not equal joint speed or equal view envelope; simulation truth drives flight control only, never perception or view selection.',
        retention='No raw images, point clouds or bags. Numerical records, configuration, source hashes and compressed diagnostics only.',sha256=hashes)
    write(root/'protocol.json',protocol)
    active={};done=[];pending=tasks.copy();start=time.monotonic();next_launch=0.
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while pending or active:
            for slot in range(a.jobs):
                if slot in active or not pending:continue
                if time.monotonic()<next_launch:break
                task=pending.pop(0);out=root/task['name']
                cmd=[sys.executable,str(HERE/'run_articulated_dragon.py'),'--output',str(out),'--world',task['world'],'--seed',str(task['seed']),
                     '--domain',str(151+slot),'--port',str(11901+slot),'--mechanism-comparison','--fusion-profile','drpm_isotropic20_view']
                if task['arm'].startswith('gimbal'):cmd.append('--gimbal')
                if task['arm']=='gimbal_v10':cmd.append('--gimbal-v10')
                write(root/(task['name']+'.command.json'),cmd)
                log=(root/(task['name']+'.log')).open('w')
                child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                next_launch=time.monotonic()+3.
                active[slot]=(task,child,log);print('START',task['name'],child.pid,flush=True)
            for slot,(task,child,log) in list(active.items()):
                out=root/task['name'];loaded=out/'loaded_fusion_libraries.json'
                if not loaded.exists() and (out/'owners.json').exists():
                    owners=json.loads((out/'owners.json').read_text())
                    # roslaunch puts each node in a new session. Match its
                    # direct child by parent PID, not by session identity.
                    sid=owners.get('mimosa');records=[]
                    if sid:
                        for proc in Path('/proc').iterdir():
                            if not proc.name.isdigit():continue
                            try:
                                status=dict(line.split(':',1) for line in (proc/'status').read_text().splitlines() if ':' in line)
                                if int(status['PPid'])!=sid:continue
                                matches=[line.split()[-1] for line in (proc/'maps').read_text().splitlines() if 'libmimosa_lib.so' in line]
                                if matches:records.append(dict(pid=int(proc.name),paths=sorted(set(matches))))
                            except (OSError,ProcessLookupError,PermissionError):pass
                    if records:write(loaded,records)
                if child.poll() is not None:
                    log.close();done.append(dict(**task,returncode=child.returncode));del active[slot]
                    print('FINISH',task['name'],child.returncode,flush=True)
            write(root/'status.json',dict(status='running',completed=done,active=[t[0]['name'] for t in active.values()],pending=len(pending),wall_s=time.monotonic()-start))
            if pending or active:time.sleep(3)
        changed=[x for x,h in hashes.items() if not Path(x).exists() or digest(Path(x))!=h]
        write(root/'freeze_audit.json',dict(passed=not changed,checked=len(hashes),changed=changed))
        write(root/'status.json',dict(status='complete',completed=done,wall_s=time.monotonic()-start))
    finally:
        for task,child,log in active.values():child.send_signal(signal.SIGINT)
        for task,child,log in active.values():
            child.wait(timeout=90);log.close()

if __name__=='__main__':main()
