#!/usr/bin/env python3
"""Foreground owner for a single-camera DRPM+MIMOSA+ROVIO gimbal experiment."""
import argparse
import fcntl
import hashlib
import json
import os
import re
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client
import yaml
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3];BUILD=WS/'build/active_perception'
sys.path.insert(0,str(NATIVE))
from rovio_initialization import match_accelerometer_bias_prior
from manager import CLEAN

def run(args):
    lock=(NATIVE/'.supervisor.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    for f in ('libfif_active.so','rovio_gimbal','libmimosa_lib.so'):
        if not (BUILD/f).exists():raise RuntimeError(f'Run python3 {HERE}/build.py first: missing {f}')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',11379))==0:raise RuntimeError('ROS master port 11379 already in use')
    out=args.output or NATIVE/'logs'/('active-gimbal-'+time.strftime('%Y%m%d-%H%M%S'))
    out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'implementation';snapshot.mkdir()
    for path in HERE.iterdir():
        if path.is_file():shutil.copy2(path,snapshot/path.name)
    binaries={name:hashlib.sha256((BUILD/name).read_bytes()).hexdigest()
              for name in ('libfif_active.so','rovio_gimbal','libmimosa_lib.so')}
    if args.route:
        shutil.copytree(args.route,out/'route')
    if args.seed is not None:
        bindir=out/'bin';bindir.mkdir()
        real_gz=shutil.which('gz')
        wrapper=bindir/'gz'
        wrapper.write_text('#!/bin/bash\nif [[ "$1" == sim ]]; then\nshift\nexec '+shlex.quote(real_gz)+' sim --seed '+str(args.seed)+' "$@"\nfi\nexec '+shlex.quote(real_gz)+' "$@"\n')
        wrapper.chmod(0o755)
    processes=[];oneshots=set()
    def stop(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def start(name,cmd,ros1=False,once=False,extra=''):
        env=NATIVE/('env_ros1.sh' if ros1 else 'env_dragon.sh')
        text=f'source {shlex.quote(str(env))}\nexport ROS_MASTER_URI=http://127.0.0.1:11379\n'
        if ros1:text+=f'source {WS}/slam_ros1_ws/devel/setup.bash\nsource {WS}/comparison_ws/devel/setup.bash\n'
        if args.seed is not None:text+=f'export PATH={shlex.quote(str(out / "bin"))}:$PATH\n'
        text+='export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1\n'+extra+'\nexec '+shlex.join(map(str,cmd))
        with (out/(name+'.log')).open('w') as log:
            p=subprocess.Popen(['bash','--noprofile','--norc','-c',text],env=CLEAN,cwd=NATIVE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,p))
        if once:oneshots.add(name)
        return p
    started=time.monotonic();status='interrupted'
    print(f'Active gimbal ({args.mode}): {out}',flush=True)
    try:
        start('master',['roscore','-p','11379'],True)
        master=xmlrpc.client.ServerProxy('http://127.0.0.1:11379')
        for _ in range(100):
            try:master.getPid('/active_startup');break
            except OSError:time.sleep(.1)
        else:raise RuntimeError('ROS master timeout')
        types={'/clock':'rosgraph_msgs/msg/Clock','/dragon/imu/raw':'sensor_msgs/msg/Imu',
               '/dragon/lidar/points':'sensor_msgs/msg/PointCloud2','/active/image':'sensor_msgs/msg/Image',
               '/active/extrinsic':'geometry_msgs/msg/PoseStamped','/active/encoder':'std_msgs/msg/Float64MultiArray',
               '/active/gimbal/yaw':'std_msgs/msg/Float64','/active/gimbal/pitch':'std_msgs/msg/Float64',
               '/slam/odometry':'nav_msgs/msg/Odometry','/slam/map':'sensor_msgs/msg/PointCloud2',
               '/rovio/odometry':'nav_msgs/msg/Odometry'}
        if args.validate or args.route:types['/dragon/ground_truth']='nav_msgs/msg/Odometry'
        bridge=dict(active_lidar_diagnostics=args.no_drpm,topics=[dict(topic=t,type=v,queue_size=200 if t.endswith('imu/raw') else 10) for t,v in types.items()],services_1_to_2=[],services_2_to_1=[],use_sim_time=True)
        (out/'bridge.yaml').write_text(yaml.safe_dump(bridge))
        p=start('parameters',['rosparam','load',out/'bridge.yaml'],True,True)
        if p.wait(timeout=20)!=0:raise RuntimeError('ROS parameter setup failed')
        start('bridge',['ros2','run','ros1_bridge','parameter_bridge'])
        sim=['ros2','launch',NATIVE/'dragon_sim.launch.py','headless:=true','vio:=true','gimbal:=true',f'output:={out}',
             *[f'spawn_{a}:={v}' for a,v in zip('xyz',args.spawn)]]
        if args.world:sim.append(f'world_file:={args.world}')
        if args.route:sim.append('takeoff_height:='+str(json.loads((args.route/'route.json').read_text())['waypoints'][0][2]))
        start('simulation',sim)
        for _ in range(240):
            if (out/'dragon_slam.urdf').exists() and (out/'camera_manifest.json').exists():break
            if processes[-1][1].poll() is not None:raise RuntimeError('Simulation exited during setup')
            time.sleep(.25)
        else:raise RuntimeError('Camera generation timeout')
        camera=re.search(r'Camera0\s*\{[^}]*\}',(out/'rovio.info').read_text())[0]
        cfg,variances=match_accelerometer_bias_prior((HERE/'rovio_template.info').read_text()+camera+'\n',out/'dragon_slam.urdf')
        (out/'rovio.info').write_text(cfg)
        config=yaml.safe_load((HERE/'mimosa.yaml').read_text());(out/'mimosa').mkdir()
        if args.no_drpm:config['lidar']['geometric']['scan_to_map']['drpm_enabled']=False
        config['logs_directory']=str(out/'mimosa')+'/'
        (out/'mimosa.yaml').write_text(yaml.safe_dump(config))
        (out/'protocol.json').write_text(json.dumps(dict(mode=args.mode,imu_bias_prior=variances,
            takeoff=args.takeoff,record_bag=False,seed=args.seed,route=str(args.route) if args.route else None,route_end=args.route_end,
            drpm_fusion=not args.no_drpm,view_only_drpm_diagnostics=args.no_drpm,binary_sha256=binaries,upstream=json.loads((BUILD/'source_manifest.json').read_text()),
            gimbal='ideal intersecting axes at optical centre, measured encoders',truth_input_to_estimators=False),indent=2)+'\n')
        start('image_encoder',['python3',HERE/'image_encoder.py','--calibration',out])
        start('slam_inputs',['python3',NATIVE/'slam_inputs.py','_imu_topic:=/dragon/imu/raw','_points_topic:=/dragon/lidar/points'],True)
        start('rovio',[BUILD/'rovio_gimbal',f'_config:={out}/rovio.info',f'_calibration:={out}/cam0.yaml'],True)
        start('mimosa',['roslaunch',NATIVE/'mimosa_unipilot.launch',f'config:={out}/mimosa.yaml',f'logs:={out}/mimosa','external_odometry:=/rovio/odometry'],True,
              extra=f'export LD_LIBRARY_PATH={BUILD}:{WS}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}')
        start('planner',['python3',HERE/'planner_node.py','--output',out,'--library',BUILD/'libfif_active.so','--mode',args.mode],True)
        if args.validate or args.route:start('monitor',['python3',HERE/'monitor.py','--output',out],True)
        if args.world:start('controller_ready',['python3',NATIVE/'zab_controller_ready.py','--output',out],once=True)
        if args.route:start('route',['python3',HERE/'route_flight.py','--output',out,'--route',out/'route','--end',str(args.route_end)],once=True)
        elif args.takeoff:start('hover',['python3',HERE/'hover.py','--output',out],once=True)
        while True:
            for name,p in processes:
                code=p.poll()
                if code is not None and not (name in oneshots and code==0):raise RuntimeError(f'{name} exited {code}; see {out}/{name}.log')
            if args.route and (out/'route_status.json').exists():
                report=json.loads((out/'route_status.json').read_text())
                if report['failure']:raise RuntimeError('Flight failed: '+report['failure'])
                if report['stage']=='complete':
                    for name,proc in processes:
                        if name=='route':proc.wait(timeout=10)
                    status='route_complete';break
            if args.duration and time.monotonic()-started>=args.duration:status='duration_complete';break
            time.sleep(.5)
    except KeyboardInterrupt:pass
    except Exception:
        status='failed'
        raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for name,p in reversed(processes):
            try:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+15
        for name,p in reversed(processes):
            try:p.wait(timeout=max(.05,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                p.wait()
        (out/'runtime.json').write_text(json.dumps(dict(status=status,stopped=True,
            processes={name:p.returncode for name,p in processes}),indent=2)+'\n')
        print(f'Stopped owned processes. Results: {out}',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['active','fixed'],default='active')
    p.add_argument('--world',type=Path);p.add_argument('--spawn',nargs=3,type=float,default=[40,5,.5])
    p.add_argument('--output',type=Path);p.add_argument('--takeoff',action='store_true')
    p.add_argument('--validate',action='store_true',help='Record estimator/GT CSV for isolated verification only')
    p.add_argument('--no-drpm',action='store_true',help='Disable DRPM factors; retain view-only diagnosis')
    p.add_argument('--route',type=Path);p.add_argument('--route-end',type=float,default=0)
    p.add_argument('--seed',type=int,default=None)
    p.add_argument('--duration',type=float,default=0.,help='Optional wall-time test limit; 0 runs until Ctrl+C')
    run(p.parse_args())
