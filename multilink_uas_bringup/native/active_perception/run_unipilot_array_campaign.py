#!/usr/bin/env python3
"""Acquire all simultaneous camera comparisons and finalize compact records."""
import argparse, hashlib, json, os, signal, subprocess, sys, time
from pathlib import Path
import yaml
from run_practical_comparison import unipilot_command, HERE, NATIVE, WS


def write(path, data):
    tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def capture_libraries(out, port):
    target=out/'loaded_array_libraries.json'
    if target.exists() or not out.exists():return
    records=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            cmd=(proc/'cmdline').read_bytes().split(b'\0')
            if not cmd or not cmd[0].endswith(b'/mimosa_batched_node'):continue
            if f'ROS_MASTER_URI=http://127.0.0.1:{port}'.encode() not in (proc/'environ').read_bytes().split(b'\0'):continue
            paths=sorted(set(line.split()[-1] for line in (proc/'maps').read_text().splitlines() if 'libmimosa_lib.so' in line))
            if paths:records.append(dict(pid=int(proc.name),command=[x.decode(errors='replace') for x in cmd if x],paths=paths))
        except (OSError,ProcessLookupError):pass
    if len(records)==6:write(target,records)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--worlds',type=Path,nargs='+',required=True);p.add_argument('--jobs',type=int,default=3)
    p.add_argument('--seeds',type=int,nargs='+',default=[2026111401,2026111402,2026111403])
    a=p.parse_args();root=a.output.resolve();root.mkdir(parents=True,exist_ok=False)
    if not 1<=a.jobs<=6:raise ValueError('jobs must be 1..6')
    cfg=yaml.safe_load((HERE/'motion_view_v10.yaml').read_text());cfg['view_information_mode']='lidar_conditioned'
    write(root/'unipilot_planner.yaml',cfg)  # JSON is valid YAML.
    files=set(HERE.glob('*.py'))|set(HERE.glob('*.yaml'))|set(NATIVE.glob('*.py'))|set(NATIVE.glob('*.sh'))
    files.add(root/'unipilot_planner.yaml')
    files.update(p for p in (NATIVE/'simulation_inputs').rglob('*') if p.is_file())
    for world in a.worlds:
        route=json.loads((world.parent/'route/route.json').read_text())
        assert route['length_m']==90 and route['waypoints'][0]==[5.,0.,2.]
        files.update(p for p in world.parent.rglob('*') if p.is_file())
    files.update(p for p in (WS/'build').rglob('*') if p.is_file() and ('.so' in p.name or p.name in ('rovio_gimbal','mimosa_batched_node','native_monitor','native_image_encoder','native_slam_inputs','native_clock_bridge','reliable_bridge')))
    robot_ws=Path('/root/ros2/aerial_robot_base_ws')
    files.update(p for p in (robot_ws/'install').rglob('*') if p.is_file() and ('.so' in p.name or p.suffix in ('.yaml','.xacro','.urdf')))
    hashes={str(p):digest(p) for p in sorted(files)}
    tasks=[dict(layout=i,layout_seed=int(w.parent.name),world=str(w),seed=seed+100*i,name=f'layout{i}_seed{seed+100*i}') for i,w in enumerate(a.worlds) for seed in a.seeds]
    protocol=dict(tasks=tasks,jobs=a.jobs,distance_m=90,speed_mps=.5,duration_s=182,
        cameras=['yaw_m50','front','yaw_p50','pitch_p90','pitch_m90','active'],
        design='Same physical flight, simultaneous fixed five and gimbal. Six independent monocular ROVIO+MIMOSA pipelines, no cross-camera fusion. Common IMU/LiDAR. Same camera centre, intrinsics and filter settings except orientation. Virtual sensors add no mass.',
        configuration='Established v10 lidar-conditioned gimbal; DRPM OFF; fresh Linux build, not historical binary identity. All six streams use identical odometry publisher queue 2000 instead of upstream queue 1; estimator arithmetic unchanged.',
        preparation='Two common physical preparation cycles, active selection in first and hold in second. Fixed cameras face their own direction throughout. Front images shared with gimbal only while their measured poses coincide.',
        seed_policy='Three distinct noise seeds per layout; base + 100*layout. Pairing is within each physical flight, not a claim of identical pixel noise across cameras.',
        primary='Full 182s MIMOSA position RMSE at common 10Hz, initial position+orientation alignment only, no scale fitting. Compare gimbal against EACH of five fixed directions.',
        inference='Average three paired run differences within each layout, then treat 10 layouts as analysis units. Exact paired sign-flip test of mean difference (requires symmetric independent layout differences under null), one-sided fixed-minus-gimbal>0 predeclared; also two-sided. Holm for individual five comparisons. All-five superiority is intersection-union, max of five one-sided p values. 95% paired layout bootstrap percentile intervals are supplementary, not test inversions.',
        secondary='ROVIO RMSE; fixed-direction average; retrospective best fixed per layout (not deployable preselection); physical and data completion.',
        failure_policy='All 30 flights retained; no accuracy-based retries/exclusions. No shortened-interval RMSE. Missing full interval prevents complete mean for that view. No sample-size changes based on p values.',
        scope='Ten generated texture layouts in one constant-section synthetic tunnel family; does not establish superiority in arbitrary real tunnels.',
        retention='Numerical trajectories, event counts, decisions, configuration/hashes and compressed diagnostics. No continuous images, point clouds or bags; five diagnostic snapshots per flight.',sha256=hashes)
    write(root/'protocol.json',protocol)
    active={};done=[];pending=tasks.copy();started=time.monotonic();next_launch=0.
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        while pending or active:
            for slot in range(a.jobs):
                if slot in active or not pending or time.monotonic()<next_launch:continue
                task=pending.pop(0);out=root/task['name'];port=12001+slot
                cmd=unipilot_command(out,task['seed'],161+slot,port,Path(task['world']))
                cmd[1]=str(HERE/'run_rebuilt_unipilot_array.py');cmd[cmd.index('--duration')+1]='3600'
                write(root/(task['name']+'.command.json'),cmd)
                log=(root/(task['name']+'.owner.log')).open('w')
                child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                active[slot]=(task,child,log);next_launch=time.monotonic()+4
                print('START',task['name'],child.pid,flush=True)
            for slot,(task,child,log) in list(active.items()):
                capture_libraries(root/task['name'],12001+slot)
                if child.poll() is not None:
                    log.close();done.append(dict(**task,returncode=child.returncode));del active[slot]
                    print('FINISH',task['name'],child.returncode,flush=True)
            write(root/'status.json',dict(status='running',completed=done,active=[x[0]['name'] for x in active.values()],pending=len(pending),wall_s=time.monotonic()-started))
            if pending or active:time.sleep(3)
        changed=[p for p,h in hashes.items() if not Path(p).exists() or digest(Path(p))!=h]
        write(root/'freeze_audit.json',dict(passed=not changed,checked=len(hashes),changed=changed))
        write(root/'status.json',dict(status='complete',completed=done,wall_s=time.monotonic()-started))
    finally:
        for task,child,log in active.values():child.send_signal(signal.SIGINT)
        for task,child,log in active.values():child.wait(timeout=60);log.close()
    # Finalization belongs to this owned job, not a later interactive turn.
    report=[sys.executable,str(HERE/'report_unipilot_array_campaign.py'),str(root)]
    subprocess.run(report,check=True)
    before=digest(root/'report/summary.json')
    subprocess.run([sys.executable,str(NATIVE/'container/compact_comparison.py'),str(root)],check=True)
    subprocess.run(report,check=True)
    after=digest(root/'report/summary.json')
    write(root/'recomputation_audit.json',dict(passed=before==after,before_sha256=before,after_sha256=after))
    if before!=after:raise RuntimeError('Compressed-data recomputation changed results')
    write(root/'pipeline_complete.json',dict(analysis=True,lossless_compaction=True,recomputation=True))
    print('PIPELINE_COMPLETE',root,flush=True)


if __name__=='__main__':main()
