#!/usr/bin/env python3
"""Foreground owner for a single-camera DRPM+MIMOSA+ROVIO gimbal experiment."""
import argparse
import fcntl
import hashlib
import json
import math
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
import xml.etree.ElementTree as ET
import yaml
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3];BUILD=WS/'build/active_perception'
sys.path.insert(0,str(NATIVE))
from rovio_initialization import match_accelerometer_bias_prior
from manager import CLEAN
from unipilot_calibration import prepare, imu_runtime_parameters, fusion_white_noise
from simulation_shutdown import gazebo_exit_status, launch_exit_status, stop_world_clients
from sensor_bias_parameters import parameters as sensor_bias_parameters

def run(args):
    if args.pacing_extra_lidar and not args.paced:
        raise ValueError('Diagnostic completion pacing requires --paced')
    if args.lidar_four_dof and (not args.fixed_gravity_imu or not args.tilt_fusion or not args.no_drpm):
        raise ValueError('Four-DoF LiDAR comparison requires fixed gravity, VIO tilt, and DRPM disabled')
    if not math.isfinite(args.stationary_duration) or args.stationary_duration<0:raise ValueError("Invalid stationary duration")
    if args.shared_initialization and not args.paired:raise ValueError('Shared image initialization requires a paired experiment')
    if args.geometry_rovio_information and not args.joint_rovio_information:raise ValueError('Geometry export requires joint information')
    PORT=args.port
    if args.temporal_fusion and (not args.ordered_fusion or not args.fusion_calibration):
        raise ValueError('Temporal fusion requires ordered fusion and independent calibration')
    if args.translation_fusion and (args.temporal_fusion or not args.ordered_fusion or not args.fusion_calibration):
        raise ValueError('Translation fusion requires ordered fusion and calibration; excludes temporal fusion')
    ordered_binary=WS/('build/mimosa-temporal/mimosa_temporal_node' if args.temporal_fusion else 'build/mimosa-compact/mimosa_ordered_node')
    if args.translation_fusion:ordered_binary=WS/'build/mimosa-translation/mimosa_translation_node'
    if args.tilt_fusion and not args.fixed_gravity_imu:
        raise ValueError('World-Z tilt factors require the corrected fixed-gravity IMU implementation')
    if args.tilt_fusion:
        if args.temporal_fusion or args.translation_fusion or not args.ordered_fusion or not args.fusion_calibration:
            raise ValueError('Tilt fusion requires ordered fusion and calibration; excludes temporal/translation variants')
        ordered_binary=WS/'build/mimosa-tilt/mimosa_tilt_node'
    if args.tilt_translation:
        if not args.tilt_fusion:raise ValueError('--tilt-translation requires --tilt-fusion')
        ordered_binary=WS/'build/mimosa-gravity-translation/mimosa_gravity_translation_node'
    if args.velocity_fusion:
        if not args.tilt_fusion or args.tilt_translation:raise ValueError('Velocity fusion requires tilt and excludes displacement fusion')
        ordered_binary=WS/'build/mimosa-velocity/mimosa_velocity_node'
    if args.fixed_angles_deg is not None and args.mode!='fixed':raise ValueError("Fixed angles require --mode fixed")
    compact_dir=WS/('build/mimosa-compact'+('' if args.map_variant=='compact' else '-'+args.map_variant))
    if args.map_variant!='compact' and not args.compact_map:raise ValueError('--map-variant requires --compact-map')
    if args.fixed_gravity_imu:
        if not args.compact_map or args.map_variant!='prefetch':raise ValueError('Fixed-gravity library requires compact prefetch map')
        compact_dir=WS/'build/mimosa-fixed-gravity'
    if args.sensor_bias_model:
        if not args.fixed_gravity_imu:raise ValueError('Sensor bias model requires the fixed-gravity library')
        compact_dir=WS/'build/mimosa-sensor-bias'
    if args.batched_fusion:
        if not args.velocity_fusion or not args.sensor_bias_model:
            raise ValueError('Batched fusion requires body velocity and sensor bias model')
        compact_dir=WS/'build/mimosa-batched-vision'
        ordered_binary=compact_dir/'mimosa_batched_node'
    lock=(NATIVE/f'.unipilot-{args.domain}.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    for f in ('libfif_active.so','rovio_gimbal','libmimosa_lib.so'):
        if not (BUILD/f).exists():raise RuntimeError(f'Run python3 {HERE}/build.py first: missing {f}')
    rovio_binary=(WS/'build/rovio-joint-information/rovio_gimbal') if args.joint_rovio_information else (BUILD/'rovio_gimbal')
    if args.geometry_rovio_information:rovio_binary=WS/'build/rovio-joint-geometry/rovio_gimbal'
    if not rovio_binary.exists():raise RuntimeError('Build the selected ROVIO information publisher first')
    if not (WS/'build/reliable-bridge/reliable_bridge').exists():
        raise RuntimeError(f'Run python3 {HERE}/build_reliable_bridge.py first')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('ROS master port PORT already in use')
    out=args.output or NATIVE/'logs'/('active-gimbal-'+time.strftime('%Y%m%d-%H%M%S'))
    if Path('/mnt/c').exists() and shutil.disk_usage('/mnt/c').free<args.min_host_free_gb*1e9:
        raise RuntimeError('Windows free space below configured safety reserve')
    out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'implementation';snapshot.mkdir()
    for path in HERE.iterdir():
        if path.is_file():shutil.copy2(path,snapshot/path.name)
    shutil.copytree(HERE/'performance',snapshot/'performance')
    for name in ('unipilot_gimbal.py','unipilot_gimbal_sim.launch.py','rovio_initialization.py'):
        shutil.copy2(NATIVE/name,snapshot/name)
    shutil.copytree(NATIVE/'sensor_profiles',snapshot/'sensor_profiles')
    binaries={name:hashlib.sha256((BUILD/name).read_bytes()).hexdigest()
              for name in ('libfif_active.so','rovio_gimbal','libmimosa_lib.so')}
    binaries['rovio_gimbal']=hashlib.sha256(rovio_binary.read_bytes()).hexdigest()
    if args.joint_rovio_information:
        shutil.copy2(rovio_binary.parent/'manifest.json',snapshot/'joint_rovio_build.json')
        shutil.copy2(rovio_binary.parent/'rovio_gimbal.cpp',snapshot/'joint_rovio_generated.cpp')
    if args.fast_icp or args.compact_map:
        selected=compact_dir if args.compact_map else WS/'build/mimosa-speed'
        binaries['libmimosa_lib.so']=hashlib.sha256((selected/'libmimosa_lib.so').read_bytes()).hexdigest()
    if args.tbb_threads:
        binaries['libtbb_limit.so']=hashlib.sha256((WS/'build/mimosa-speed/libtbb_limit.so').read_bytes()).hexdigest()
    if args.serialized_fusion:
        binaries['mimosa_node']=hashlib.sha256((WS/'build/mimosa-speed/mimosa_serial_node').read_bytes()).hexdigest()
    if args.ordered_fusion:
        binaries['mimosa_node']=hashlib.sha256(ordered_binary.read_bytes()).hexdigest()
    if args.clock_hz:binaries['native_clock_bridge']=hashlib.sha256((WS/'build/native-adapters/native_clock_bridge').read_bytes()).hexdigest()
    binaries['reliable_bridge']=hashlib.sha256((WS/'build/reliable-bridge/reliable_bridge').read_bytes()).hexdigest()
    transport_retry=WS/'build/zmq-retry/libzmq_eintr_retry.so'
    if not transport_retry.exists():raise RuntimeError('Run active_perception/build_zmq_retry.py first')
    binaries['libzmq_eintr_retry.so']=hashlib.sha256(transport_retry.read_bytes()).hexdigest()
    stop_core=WS/'build/gz-stop/lib/libgz-sim8.so.8.15.0'
    core_guard=WS/'build/no-crash-dump/libno_crash_dump.so'
    if args.safe_shutdown:
        for path in (stop_core,core_guard):
            if not path.exists():raise RuntimeError('Build the isolated shutdown fix and core guard first')
            binaries[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
        shutil.copy2(WS/'build/gz-stop/manifest.json',snapshot/'gazebo_shutdown_build.json')
        shutil.copy2(WS/'build/no-crash-dump/manifest.json',snapshot/'core_guard_build.json')
    if args.native_monitor:binaries['native_monitor']=hashlib.sha256((WS/'build/native-adapters/native_monitor').read_bytes()).hexdigest()
    if args.native_adapters:
        for name in ['native_image_encoder','native_slam_inputs']:
            binaries[name]=hashlib.sha256((WS/'build/native-adapters'/name).read_bytes()).hexdigest()
    if args.fast_map:
        for path in (WS/'build/gtsam-points-speed/libgtsam_points.so',WS/'build/allocator-speed/usr/lib/x86_64-linux-gnu/libjemalloc.so.2'):
            binaries[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    manifests={}
    for label,directory in [('map',compact_dir),('adapters',WS/'build/native-adapters'),('bridge',WS/'build/reliable-bridge'),('transport_retry',WS/'build/zmq-retry')]:
        if (directory/'manifest.json').exists():manifests[label]=json.loads((directory/'manifest.json').read_text())
    (snapshot/'performance_builds.json').write_text(json.dumps(manifests,indent=2)+'\n')
    if args.temporal_fusion:shutil.copy2(WS/'build/mimosa-temporal/manifest.json',snapshot/'temporal_build.json')
    if args.translation_fusion:shutil.copy2(WS/'build/mimosa-translation/manifest.json',snapshot/'translation_build.json')
    if args.tilt_fusion:shutil.copy2(WS/('build/mimosa-velocity/manifest.json' if args.velocity_fusion else ('build/mimosa-gravity-translation/manifest.json' if args.tilt_translation else 'build/mimosa-tilt/manifest.json')),snapshot/'tilt_build.json')
    if args.batched_fusion:
        shutil.copy2(compact_dir/'manifest.json',snapshot/'batched_fusion_build.json')
        (snapshot/'batched_fusion').mkdir()
        for name in ['manifest.json','graph_manager.cpp','ordered_mimosa_node.cpp']:
            shutil.copy2(compact_dir/name,snapshot/'batched_fusion'/name)
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
        text=f'source {shlex.quote(str(env))}\nexport ROS_MASTER_URI=http://127.0.0.1:{PORT}\n'
        if ros1:text+=f'source {WS}/slam_ros1_ws/devel/setup.bash\nsource {WS}/comparison_ws/devel/setup.bash\n'
        if args.seed is not None:text+=f'export PATH={shlex.quote(str(out / "bin"))}:$PATH\n'
        text+=f'export ROS_DOMAIN_ID={args.domain} GZ_PARTITION=uas_speed_{args.domain}\n'
        # Retry only interrupted ZeroMQ sends in Gazebo and its ROS2 transport
        # bridge. No estimator values, timestamps, or sensor rates are changed.
        if name=='simulation':
            # WSL D3D12 registers a thread-local destructor, but unloads its
            # library before the render thread exits. Keep the code mapped
            # through process exit; this changes no rendering calculations.
            d3d12=Path('/usr/lib/wsl/lib/libd3d12core.so')
            retained=str(d3d12)+':' if d3d12.exists() else ''
            text+=f'export LD_PRELOAD={transport_retry}:{retained}${{LD_PRELOAD:-}}\n'
            if args.safe_shutdown:
                text+=f'export LD_LIBRARY_PATH={stop_core.parent}:${{LD_LIBRARY_PATH:-}}\n'
                text+='export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}\n'
                text+=f'export LD_PRELOAD={core_guard}:${{LD_PRELOAD:-}}\n'
        if args.inline_render:
            rendering=WS/'build/gz-rendering-speed-install/lib'
            if not (rendering/'gz-rendering-8/engine-plugins/libgz-rendering-ogre2.so').exists():
                raise RuntimeError('Build the isolated rendering scheduling backport first')
            text+=f'export GZ_RENDERING_PLUGIN_PATH={rendering}/gz-rendering-8/engine-plugins\nexport LD_LIBRARY_PATH={rendering}:${{LD_LIBRARY_PATH:-}}\n'
        text+='export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE\n'+extra+'\nexec '+shlex.join(map(str,cmd))
        with (out/(name+'.log')).open('w') as log:
            p=subprocess.Popen(['bash','--noprofile','--norc','-c',text],env=CLEAN,cwd=NATIVE,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,p))
        if once:oneshots.add(name)
        return p
    fast_extra=''
    if args.fast_map:
        library=WS/'build/gtsam-points-speed/libgtsam_points.so'
        allocator=WS/'build/allocator-speed/usr/lib/x86_64-linux-gnu/libjemalloc.so.2'
        if not library.exists() or not allocator.exists():raise RuntimeError('Build isolated map/allocator libraries first')
        fast_extra=f'\nexport LD_PRELOAD={library}:{allocator}\nexport MALLOC_CONF=background_thread:true'
    if args.fast_icp or args.compact_map:
        fast_extra+='\nexport LD_LIBRARY_PATH='+str(compact_dir if args.compact_map else WS/'build/mimosa-speed')+':${LD_LIBRARY_PATH:-}'
    if args.tbb_threads:
        fast_extra+=f'\nexport UAS_TBB_THREADS={args.tbb_threads}\nexport LD_PRELOAD={WS}/build/mimosa-speed/libtbb_limit.so:${{LD_PRELOAD:-}}'
    started=time.monotonic();status='interrupted'
    print(f'Active gimbal ({args.mode}): {out}',flush=True)
    try:
        start('master',['roscore','-p',str(PORT)],True)
        master=xmlrpc.client.ServerProxy(f'http://127.0.0.1:{PORT}')
        for _ in range(100):
            try:master.getPid('/active_startup');break
            except OSError:time.sleep(.1)
        else:raise RuntimeError('ROS master timeout')
        types={'/clock':'rosgraph_msgs/msg/Clock','/rmf_unipilot/imu':'sensor_msgs/msg/Imu',
               '/rmf_unipilot/lidar/points':'sensor_msgs/msg/PointCloud2','/active/image':'sensor_msgs/msg/Image',
               '/active/extrinsic':'geometry_msgs/msg/PoseStamped','/active/encoder':'std_msgs/msg/Float64MultiArray',
               '/active/gimbal/yaw':'std_msgs/msg/Float64','/active/gimbal/pitch':'std_msgs/msg/Float64',
               '/slam/odometry':'nav_msgs/msg/Odometry',
               '/rovio/odometry':'nav_msgs/msg/Odometry'}
        if args.paired:
            for topic in ('/active/image','/active/extrinsic','/active/encoder'):
                types['/fixed'+topic]=types[topic]
        if args.airborne_init:types['/active/initialize_vio']='std_msgs/msg/Bool'
        if args.validate or args.route:types['/rmf_unipilot/odom']='nav_msgs/msg/Odometry'
        bridge=dict(active_lidar_diagnostics=args.no_drpm,fixed={'active_lidar_diagnostics':False},topics=[dict(topic=t,type=v,queue_size=2000 if t.endswith(('/imu','/encoder')) else 10) for t,v in types.items()],services_1_to_2=[],services_2_to_1=[],use_sim_time=True)
        # Installed parameter_bridge YAML QoS affects publishers only. The
        # isolated wrapper uses upstream conversion factories with explicit
        # reliable QoS for BOTH directions; no sensor values are modified.
        (out/'bridge.yaml').write_text(yaml.safe_dump(bridge))
        p=start('parameters',['rosparam','load',out/'bridge.yaml'],True,True)
        if p.wait(timeout=20)!=0:raise RuntimeError('ROS parameter setup failed')
        start('bridge',[WS/'build/reliable-bridge/reliable_bridge'])
        sim=['ros2','launch',NATIVE/'unipilot_gimbal_sim.launch.py','headless:=true','manual_hover:=false','paused:=true',f'batch_render:={str(args.batch_render).lower()}',f'paired:={str(args.paired).lower()}',f'serial_postupdates:={str(args.serial).lower()}',f'output:={out}',
             *[f'spawn_{a}:={v}' for a,v in zip('xyz',args.spawn)]]
        if args.world:sim.append(f'world_file:={args.world}')
        if args.route:sim.append('takeoff_height:='+str(json.loads((args.route/'route.json').read_text())['waypoints'][0][2]))
        sim.append('sensor_profile:='+args.sensor_profile)
        sim.append('imu_sampling:='+args.imu_sampling)
        sim.append('diagnostic_sensors:='+str(args.diagnostic_sensors).lower())
        sim.append('diagnostic_imu_only:='+str(args.diagnostic_imu_only).lower())
        if args.camera_far is not None:sim.append(f'camera_far:={args.camera_far}')
        if args.clock_hz:
            sim.append('bridge_clock:=false')
            start('clock_bridge',[WS/'build/native-adapters/native_clock_bridge','--hz',str(args.clock_hz)])
        start('simulation',sim)
        for _ in range(240):
            if (out/'unipilot_gimbal.sdf').exists():break
            if processes[-1][1].poll() is not None:raise RuntimeError('Simulation exited during setup')
            time.sleep(.25)
        else:raise RuntimeError('Camera generation timeout')
        config,variances=prepare(out,args.rovio_noise_profile);(out/'mimosa').mkdir()
        if args.sensor_bias_model:
            bias_parameters=sensor_bias_parameters(out/'unipilot_gimbal.sdf')
            config['imu']['preintegration'].update(bias_parameters)
            (out/'imu_bias_model.json').write_text(json.dumps(bias_parameters,indent=2)+'\n')
        if args.fixed_gravity_imu:
            if config['imu']['preintegration']['use_estimated_gravity'] or not config['imu']['preintegration']['gravity_aligned_initialization']:
                raise ValueError('This fixed-gravity experiment requires gravity-aligned initialization and use_estimated_gravity=false')
        fusion_imu=fusion_white_noise(config,out/'unipilot_gimbal.sdf',args.fusion_imu_noise)
        (out/'fusion_imu_noise.json').write_text(json.dumps(fusion_imu,indent=2)+'\n')
        if args.fusion_initial_velocity_sigma is not None:
            sigma=args.fusion_initial_velocity_sigma
            if not math.isfinite(sigma) or sigma<=0:
                raise ValueError('Initial velocity uncertainty must be finite and positive')
            config['graph']['manager']['smoother']['initial_velocity_sigma']=sigma
        if args.fusion_calibration:
            import numpy as np
            calibration=json.loads(args.fusion_calibration.read_text());C=np.array(calibration['covariance'])
            if C.shape!=(6,6) or not np.isfinite(C).all() or not np.allclose(C,C.T) or np.linalg.eigvalsh(C).min()<=0:
                raise ValueError('Fusion calibration must be a finite positive-definite 6x6 covariance')
            config['odometry']['manager']['relative_pose_covariance']=C.ravel().tolist()
            if args.temporal_fusion:
                model=calibration['temporal_model'];rho=np.array(model['rho']);Q=np.array(model['innovation_covariance'])
                if rho.shape!=(6,) or not np.isfinite(rho).all() or np.any(abs(rho)>=1):
                    raise ValueError('Temporal calibration needs six stable coefficients')
                if Q.shape!=(6,6) or not np.isfinite(Q).all() or not np.allclose(Q,Q.T) or np.linalg.eigvalsh(Q).min()<=0:
                    raise ValueError('Temporal innovation covariance must be positive definite')
                if abs(model['period_s']-.05)>1e-9:raise ValueError('Current camera requires 50ms temporal calibration')
                config['odometry']['manager'].update(temporal_rho=rho.tolist(),
                    temporal_innovation_covariance=Q.ravel().tolist(),temporal_period_s=model['period_s'])
            if args.translation_fusion or args.tilt_translation:
                T=np.array(calibration['translation_covariance'])
                if T.shape!=(3,3) or not np.isfinite(T).all() or not np.allclose(T,T.T) or np.linalg.eigvalsh(T).min()<=0:
                    raise ValueError('Translation calibration must be positive definite 3x3')
                config['odometry']['manager']['translation_covariance']=T.ravel().tolist()
            if args.velocity_fusion:
                scale=float(calibration['velocity_covariance_scale']);period=float(calibration['velocity_period_s'])
                if not math.isfinite(scale) or scale<1 or period!=1.:raise ValueError('Velocity calibration needs scale >=1 and 1s period')
                config['odometry']['manager'].update(velocity_covariance_scale=scale,velocity_period_s=period)
            shutil.copy2(args.fusion_calibration,out/'fusion_calibration.json')
        if args.tilt_fusion:
            gravity=float(config['imu']['preintegration']['gravity_magnitude'])
            sigma=math.sqrt(max(variances['accelerometer']))/gravity
            tilt_source='Sensor acceleration bias second moment divided by squared gravity'
            if args.tilt_translation or args.velocity_fusion:
                sigma=float(calibration['gravity_sigma_rad']);tilt_source=calibration['gravity_protocol']
            if not math.isfinite(sigma) or sigma<=0:raise ValueError('Invalid sensor-derived gravity tilt noise')
            config['odometry']['manager'].update(vio_gravity_aligned=True,tilt_sigma_rad=sigma,tilt_period_s=1.0)
            (out/'tilt_fusion.json').write_text(json.dumps(dict(sigma_rad=sigma,period_s=1.0,
                source=tilt_source,
                measurement='ROVIO body gravity direction; no yaw/position/truth',
                limitation='Shared IMU and temporal correlation are not exactly modelled; sparse conservative approximation'),indent=2)+'\n')
        if args.planner_config:
            planner=yaml.safe_load((out/'planner.yaml').read_text())
            overrides=yaml.safe_load(args.planner_config.read_text())
            protected={'limits_min','limits_max','bearing_sigma','tan_half_hfov','tan_half_vfov','range_max','axis_max_speed','joint_stop_margin','joint_stop_steps','fusion_velocity_covariance_scale','fusion_velocity_period_s'}
            if protected&overrides.keys():raise ValueError('Planner override cannot change physical calibration')
            planner.update(overrides);(out/'planner.yaml').write_text(yaml.safe_dump(planner))
        planner_settings=yaml.safe_load((out/'planner.yaml').read_text());pace_planner=bool(planner_settings.get('pace_planner',False))
        if planner_settings.get('match_actuator_reachability',False):
            from actuator_reachability import parameters as actuator_parameters
            planner_settings.update(actuator_parameters(out/'unipilot_gimbal.sdf',args.world or WS/'assets/zab/zab_harmonic.sdf'))
            (out/'planner.yaml').write_text(yaml.safe_dump(planner_settings))
        if planner_settings.get('fixation_predictive_geometry',False) and not args.geometry_rovio_information:
            raise ValueError('Predictive view selection requires joint geometry information')
        if planner_settings.get('fixation_joint_velocity',False) and not args.joint_rovio_information:
            raise ValueError('Joint velocity view selection requires joint ROVIO information')
        if planner_settings.get('fixation_lidar_conditioned_velocity',False):
            if not args.velocity_fusion:raise ValueError('LiDAR conditioned tracking requires calibrated velocity fusion')
            planner_settings.update(fusion_velocity_covariance_scale=config['odometry']['manager']['velocity_covariance_scale'],
                fusion_velocity_period_s=config['odometry']['manager']['velocity_period_s'])
            (out/'planner.yaml').write_text(yaml.safe_dump(planner_settings))
        planner_period=1/float(planner_settings['planning_rate'])
        if pace_planner and not args.paced:raise ValueError('The selected planner preset requires --paced')
        if pace_planner and 1.5*planner_period>=planner_settings['max_dt']:raise ValueError('Planner pacing stop must be within the maximum command interval')
        imu_parameters=imu_runtime_parameters(out/'unipilot_gimbal.sdf',args.world or WS/'assets/zab/zab_harmonic.sdf',args.imu_model)
        # ROS command-line private arguments parse arrays as strings. Load typed YAML.
        (out/'rovio_runtime.yaml').write_text(yaml.safe_dump({name:imu_parameters for name in ('rovio_gimbal','fixed_rovio')}))
        p=start('imu_parameters',['rosparam','load',out/'rovio_runtime.yaml'],True,True)
        if p.wait(timeout=20)!=0:raise RuntimeError('IMU parameter setup failed')
        if args.no_drpm:config['lidar']['geometric']['scan_to_map']['drpm_enabled']=False
        if args.lidar_four_dof:
            scan=config['lidar']['geometric']['scan_to_map']
            if scan['project_on_degneneracy']:raise ValueError('Degeneracy projection would replace the four-DoF projection')
            scan['reg_4_dof']=True
        config['logs_directory']=str(out/'mimosa')+'/'
        (out/'mimosa.yaml').write_text(yaml.safe_dump(config))
        (out/'protocol.json').write_text(json.dumps(dict(mode=args.mode,fixed_angles_deg=args.fixed_angles_deg,platform='unipilot',paired=args.paired,shared_initialization=args.shared_initialization,
            factor=args.factor,serial_postupdates=args.serial,inline_render=args.inline_render,batch_render=args.batch_render,safe_shutdown=args.safe_shutdown,fusion_imu_noise=args.fusion_imu_noise,fusion_initial_velocity_sigma=args.fusion_initial_velocity_sigma,temporal_fusion=args.temporal_fusion,translation_fusion=args.translation_fusion,tilt_fusion=args.tilt_fusion,tilt_translation=args.tilt_translation,fixed_gravity_imu=args.fixed_gravity_imu,velocity_fusion=args.velocity_fusion,diagnostic_extra_lidar_topics=args.pacing_extra_lidar,
            rovio_noise_profile=args.rovio_noise_profile,sensor_profile=args.sensor_profile,imu_sampling=args.imu_sampling,imu_model=args.imu_model,imu_runtime_parameters=imu_parameters,slam_hover_initialization=args.airborne_init and not args.legacy_slam_start,camera_far=args.camera_far,stationary_initialization_duration=args.stationary_duration,stationary_mode=args.stationary_mode,diagnostic_sensors=args.diagnostic_sensors,diagnostic_imu_only=args.diagnostic_imu_only,record_vio_input=args.record_vio_input,imu_bias_prior=variances,
            airborne_initialization=args.airborne_init,paced=args.paced,pace_planner=pace_planner,fast_map=args.fast_map,fast_icp=args.fast_icp,compact_map=args.compact_map,map_variant=args.map_variant,native_adapters=args.native_adapters,native_monitor=args.native_monitor,velocity_feedback=args.velocity_feedback,velocity_feedback_xy_only=args.velocity_feedback_xy_only,smooth_tracking_rate=args.smooth_tracking_rate,corner_speed_limit=args.corner_speed_limit,initialization_excitation=args.initialization_excitation,clock_hz=args.clock_hz,profile=args.profile,serialized_fusion=args.serialized_fusion,ordered_fusion=args.ordered_fusion,tbb_threads=args.tbb_threads,
            takeoff=args.takeoff,record_bag=False,seed=args.seed,route=str(args.route) if args.route else None,route_end=args.route_end,
            geometry_rovio_information=args.geometry_rovio_information,joint_rovio_information=args.joint_rovio_information,batched_fusion=args.batched_fusion,lidar_four_dof=args.lidar_four_dof,sensor_bias_model=args.sensor_bias_model,drpm_fusion=not args.no_drpm,view_only_drpm_diagnostics=args.no_drpm,gazebo_transport_eintr_retry=True,binary_sha256=binaries,upstream=json.loads((BUILD/'source_manifest.json').read_text()),
            gimbal='ideal intersecting axes at optical centre, measured encoders',truth_input_to_estimators=False),indent=2)+'\n')
        monitor=[WS/'build/native-adapters/native_monitor'] if args.native_monitor else ['python3',HERE/'monitor.py']
        encoder=[WS/'build/native-adapters/native_image_encoder'] if args.native_adapters else ['python3',HERE/'image_encoder.py']
        imu_rate=float(ET.parse(out/'unipilot_gimbal.sdf').findtext(".//sensor[@name='imu']/update_rate"))
        inputs=[WS/'build/native-adapters/native_slam_inputs'] if args.native_adapters else ['python3',NATIVE/'slam_inputs.py']
        if args.diagnostic_sensors:start('diagnostic_sensors',['python3',HERE/'record_diagnostic_sensors.py','--output',out])
        start('image_encoder',[*encoder,'--calibration',out,'--reliable'])
        shared_remaps=[]
        if args.shared_initialization:
            start('paired_initialization',['python3',HERE/'paired_initialization.py','--output',out],True)
            shared_remaps=['/active/image:=/paired/active/image','/active/extrinsic:=/paired/active/extrinsic']
        start('slam_inputs',[*inputs,'_imu_topic:=/rmf_unipilot/imu','_points_topic:=/rmf_unipilot/lidar/points',f'_gate_slam:={str(args.airborne_init and not args.legacy_slam_start).lower()}',*([f'_audit_path:={out}/input_audit.json',f'_imu_rate:={imu_rate}'] if args.native_adapters else [])],True)
        imu_remap=['/slam/input/imu:=/vio/initialized_imu'] if args.airborne_init else []
        if args.airborne_init and not args.native_adapters:start('imu_start_gate',['python3',HERE/'imu_start_gate.py'],True)
        start('rovio',[rovio_binary,f'_config:={out}/rovio.info',f'_calibration:={out}/cam0.yaml',f'_stationary_initialization_duration:={args.stationary_duration}',f'_stationary_mode:={args.stationary_mode}',*([f'_input_record:={out}/vio_input.bin'] if args.record_vio_input else []),*([f'_diagnostic_csv:={out}/vio_diagnostic.csv'] if args.record_vio_input or args.diagnostic_vio else []),*imu_remap,*shared_remaps],True)
        launch_path=NATIVE/'mimosa_unipilot.launch'
        if args.serialized_fusion or args.ordered_fusion or args.profile:
            wrapper=out/'serial_fusion.sh';script='#!/bin/bash\n'
            if args.profile:
                profiler=WS/'build/profiler-speed/usr/lib/x86_64-linux-gnu/libprofiler.so.0'
                if not profiler.exists():raise RuntimeError('Missing isolated libprofiler')
                script+=f'export LD_PRELOAD={profiler}:${{LD_PRELOAD:-}}\nexport CPUPROFILE={shlex.quote(str(out))}/cpu-$$.prof\n'
            if args.ordered_fusion:script+='shift\nexec '+str(ordered_binary)+' "$@"\n'
            elif args.serialized_fusion:script+='shift\nexec '+str(WS/'build/mimosa-speed/mimosa_serial_node')+' "$@"\n'
            else:script+='exec "$@"\n'
            wrapper.write_text(script);wrapper.chmod(0o755)
            launch=ET.parse(launch_path);launch.getroot().find('node').set('launch-prefix',str(wrapper))
            launch_path=out/'mimosa_serial.launch';launch.write(launch_path)
        start('mimosa',['roslaunch',launch_path,f'config:={out}/mimosa.yaml',f'logs:={out}/mimosa','external_odometry:=/rovio/odometry'],True,
              extra=f'export LD_LIBRARY_PATH={BUILD}:{WS}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}'+fast_extra)
        start('planner',['python3',HERE/'planner_node.py','--output',out,'--library',BUILD/'libfif_active.so','--mode',args.mode,'--config',out/'planner.yaml','--lidar-topic','/rmf_unipilot/lidar/points',*(['--fixed-angles-deg',*map(str,args.fixed_angles_deg)] if args.fixed_angles_deg is not None else [])],True)
        if args.validate or args.route:start('monitor',[*monitor,'--output',out,'--truth-topic','/rmf_unipilot/odom'],True)
        if args.paired:
            fixed=out/'fixed';fixed.mkdir();(fixed/'mimosa').mkdir()
            for filename in ('cam0.yaml','raw_cam0.yaml','rovio.info','mimosa.yaml','camera_manifest.json','protocol.json'):
                shutil.copy2(out/filename,fixed/filename)
            manifest=json.loads((fixed/'camera_manifest.json').read_text());manifest['cameras'][0]['topic']='/rmf_unipilot/cam_fixed/image'
            (fixed/'camera_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
            fixed_config=yaml.safe_load((fixed/'mimosa.yaml').read_text());fixed_config['logs_directory']=str(fixed/'mimosa')+'/'
            (fixed/'mimosa.yaml').write_text(yaml.safe_dump(fixed_config))
            proto=json.loads((fixed/'protocol.json').read_text());proto.update(mode='fixed',view_only_drpm_diagnostics=False,shared_flight=str(out))
            (fixed/'protocol.json').write_text(json.dumps(proto,indent=2)+'\n')
            for filename in ('route','route_tracking.csv','route_status.json'):
                (fixed/filename).symlink_to(out/filename,target_is_directory=filename=='route')
            remaps=[f'{topic}:=/fixed{topic}' for topic in ('/active/image','/active/extrinsic','/active/encoder','/rovio/odometry','/rovio/pcl','/slam/odometry')]
            start('fixed_image_encoder',[*encoder,'--calibration',fixed,'--fixed','--reliable'])
            start('fixed_rovio',[rovio_binary,f'_config:={fixed}/rovio.info',f'_calibration:={fixed}/cam0.yaml',
                f'_stationary_initialization_duration:={args.stationary_duration}',f'_stationary_mode:={args.stationary_mode}',*([f'_input_record:={fixed}/vio_input.bin'] if args.record_vio_input else []),*([f'_diagnostic_csv:={fixed}/vio_diagnostic.csv'] if args.record_vio_input or args.diagnostic_vio else []),'__name:=fixed_rovio',*remaps,*imu_remap],True)
            launch=ET.parse(launch_path);node=launch.getroot().find('node');node.set('name','fixed_mimosa')
            for remap in node.findall('remap'):
                topic=remap.get('to')
                if topic in ('/slam/odometry','/slam/map','/slam/path'):remap.set('to','/fixed'+topic)
            ET.SubElement(node,'remap',{'from':'/active_lidar_diagnostics','to':'/fixed/active_lidar_diagnostics'})
            ET.SubElement(node,'remap',{'from':'/active/lidar_information','to':'/fixed/active/lidar_information'})
            launch.write(fixed/'mimosa.launch')
            start('fixed_mimosa',['roslaunch',fixed/'mimosa.launch',f'config:={fixed}/mimosa.yaml',f'logs:={fixed}/mimosa','external_odometry:=/fixed/rovio/odometry'],True,
                extra=f'export LD_LIBRARY_PATH={BUILD}:{WS}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}'+fast_extra)
            start('fixed_monitor',[*monitor,'--output',fixed,'--truth-topic','/rmf_unipilot/odom','__name:=fixed_monitor',*remaps],True)
        if args.fixed_gravity_imu:start('gravity_state',['python3',HERE/'record_gravity_state.py','--output',out,'--gravity',str(config['imu']['preintegration']['gravity_magnitude']),*(['--paired'] if args.paired else [])],True)
        if args.route:start('route',['python3',HERE/'unipilot_route.py','--output',out,'--route',out/'route','--end',str(args.route_end),'--velocity-feedback',str(args.velocity_feedback),*(['--velocity-feedback-xy-only'] if args.velocity_feedback_xy_only else []),*(['--smooth-tracking-rate'] if args.smooth_tracking_rate else []),*(['--corner-speed-limit'] if args.corner_speed_limit else []),*(['--initialization-excitation'] if args.initialization_excitation else []),*(['--paired'] if args.paired else []),*(['--airborne-init'] if args.airborne_init else [])],once=True)
        elif args.takeoff:start('hover',['python3',HERE/'hover.py','--output',out],once=True)
        if args.paced:start('pacing',['python3',HERE/'pace_simulation.py','--output',out,*(['--paired'] if args.paired else []),*(['--planner','--planner-period',str(planner_period)] if pace_planner else []),*(['--lidar-stop-lag','.4','--lidar-resume-lag','.26'] if args.ordered_fusion else []),*[part for topic in args.pacing_extra_lidar for part in ['--extra-lidar-topic',topic]]],True)
        time.sleep(4)
        start('unpause',['gz','service','-s','/world/zab/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: false'],once=True)
        start('factor',['gz','service','-s','/world/zab/set_physics','--reqtype','gz.msgs.Physics','--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'real_time_factor: {args.factor}'],once=True)
        while True:
            if Path('/mnt/c').exists() and shutil.disk_usage('/mnt/c').free<args.min_host_free_gb*1e9:
                raise RuntimeError('Windows free space below safety reserve; stopping owned flight')
            for name,p in processes:
                code=p.poll()
                if code is not None and not (name in oneshots and code==0):raise RuntimeError(f'{name} exited {code}; see {out}/{name}.log')
            if args.route and (out/'route_status.json').exists():
                report=json.loads((out/'route_status.json').read_text())
                if report['failure']:raise RuntimeError('Flight failed: '+report['failure'])
                if report['stage']=='complete':
                    for name,proc in processes:
                        if name=='route':proc.wait(timeout=10)
                    # Finish every tracking-window image and LiDAR update before
                    # stopping child nodes. Previously the last image could be
                    # lost during shutdown despite zero INTERNAL timestamp gaps.
                    if args.paced:
                        target=float(report['sim_time']);deadline=time.monotonic()+20
                        required={k:(int((target+1e-7)/(0.05 if 'vio' in k else .1))*(0.05 if 'vio' in k else .1))
                                  for k in (['lidar','vio','fixed_lidar','fixed_vio'] if args.paired else ['lidar','vio'])}
                        while True:
                            try:
                                state=json.loads((out/'pacing.json').read_text())
                                completed={k:state['sim_time']-state['ages'].get(k,float('inf')) for k in required}
                                if all(completed[k]>=t-1e-6 for k,t in required.items()):break
                            except (FileNotFoundError,json.JSONDecodeError):pass
                            if time.monotonic()>deadline:raise RuntimeError('Tracking-window estimator drain timed out')
                            time.sleep(.1)
                        (out/'drain.json').write_text(json.dumps(dict(tracking_end=target,required=required,completed=completed),indent=2)+'\n')
                    status='route_complete';break
            if args.duration and time.monotonic()-started>=args.duration:status='duration_complete';break
            time.sleep(.5)
    except KeyboardInterrupt:pass
    except Exception:
        status='failed'
        raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        # A paused pacer sends a final resume request on exit. It must finish
        # while the world-control service is still alive, before stopping Gazebo.
        world_clients=stop_world_clients(processes)
        # Gazebo's signal callback stops sensors synchronously; if the signal
        # interrupts a held sensor mutex, shutdown can hang. The server-control
        # service performs Stop on its own thread, outside the signal handler.
        # Keep the launch wrapper alive until its Gazebo child has actually exited.
        simulation=next((p for name,p in processes if name=='simulation'),None)
        shutdown_request=dict(world_clients=world_clients)
        if args.safe_shutdown and simulation is not None and simulation.poll() is None:
            command=f'source {shlex.quote(str(NATIVE / "env_dragon.sh"))}\nexport GZ_PARTITION=uas_speed_{args.domain}\nexec gz service -s /server_control --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean --timeout 2000 --req '+shlex.quote('stop: true')
            try:
                reply=subprocess.run(['bash','--noprofile','--norc','-c',command],env=CLEAN,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=4)
                shutdown_request.update(returncode=reply.returncode,reply=reply.stdout[-2000:])
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    log=out/'simulation.log'
                    if log.exists() and gazebo_exit_status(log.read_text(errors='replace'))['all_clean']:break
                    if simulation.poll() is not None:break
                    time.sleep(.1)
            except subprocess.TimeoutExpired:
                shutdown_request.update(timeout=True)
        # OnProcessExit already emits ROS launch Shutdown after Gazebo exits.
        # Do not race that event with a second SIGINT to the launch service.
        auto_launch_shutdown=False
        if args.safe_shutdown and simulation is not None:
            log=out/'simulation.log'
            auto_launch_shutdown=log.exists() and gazebo_exit_status(log.read_text(errors='replace'))['all_clean']
        shutdown_request['launch_stop_mode']='automatic_server_exit' if auto_launch_shutdown else 'signal'
        (out/'shutdown_request.json').write_text(json.dumps(shutdown_request,indent=2)+'\n')
        for name,p in reversed(processes):
            if p.poll() is not None:continue
            try:
                # ros2 launch forwards SIGINT itself. Signalling its group
                # also hits Gazebo directly, causing duplicate stop requests.
                if name=='simulation':
                    if not auto_launch_shutdown:p.send_signal(signal.SIGINT)
                else:os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+15
        for name,p in reversed(processes):
            try:p.wait(timeout=max(.05,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                p.wait()
        sim_log=out/'simulation.log'
        (out/'runtime.json').write_text(json.dumps(dict(status=status,stopped=True,wall_s=time.monotonic()-started,
            processes={name:p.returncode for name,p in processes},
            launch_shutdown=launch_exit_status(sim_log.read_text(errors='replace')) if sim_log.exists() else None,
            gazebo_shutdown=gazebo_exit_status(sim_log.read_text(errors='replace')) if sim_log.exists() else None),indent=2)+'\n')
        print(f'Stopped owned processes. Results: {out}',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['active','fixed'],default='active')
    p.add_argument('--fixed-angles-deg',nargs=2,type=float,metavar=('YAW','PITCH'))
    p.add_argument('--world',type=Path);p.add_argument('--spawn',nargs=3,type=float,default=[40,5,.5])
    p.add_argument('--output',type=Path);p.add_argument('--takeoff',action='store_true')
    p.add_argument('--validate',action='store_true',help='Record estimator/GT CSV for isolated verification only')
    p.add_argument('--no-drpm',action='store_true',help='Disable DRPM factors; retain view-only diagnosis')
    p.add_argument('--route',type=Path);p.add_argument('--route-end',type=float,default=0)
    p.add_argument('--seed',type=int,default=None)
    p.add_argument('--duration',type=float,default=0.,help='Optional wall-time test limit; 0 runs until Ctrl+C')
    p.add_argument('--factor',type=float,default=1.)
    p.add_argument('--serial',action='store_true')
    p.add_argument('--inline-render',action='store_true')
    p.add_argument('--batch-render',action='store_true')
    p.add_argument('--airborne-init',action='store_true')
    p.add_argument('--paced',action='store_true')
    p.add_argument('--pacing-extra-lidar',action='append',default=[],help='Diagnostic LiDAR completion topic to include in simulation pacing')
    p.add_argument('--fast-map',action='store_true')
    p.add_argument('--fast-icp',action='store_true')
    p.add_argument('--compact-map',action='store_true',help='Use the isolated exact contiguous voxel index with parallel ICP')
    p.add_argument('--map-variant',choices=['compact','scratch','pruned','cached','neighbors','prefetch'],default='compact')
    p.add_argument('--profile',action='store_true',help='CPU sampling for diagnosis; exclude from final timing comparisons')
    p.add_argument('--native-adapters',action='store_true',help='Use verified C++ image and sensor adapters')
    p.add_argument('--rovio-noise-profile',choices=['sensor','dragon'],default='sensor')
    p.add_argument('--imu-sampling',choices=['resolved','legacy'],default='resolved')
    p.add_argument('--sensor-profile',choices=['native','dragon'],default='native')
    p.add_argument('--camera-far',type=float,default=None)
    p.add_argument('--diagnostic-sensors',action='store_true')
    p.add_argument('--diagnostic-imu-only',action='store_true',help='With diagnostic sensors, omit reference cameras to save storage and rendering')
    p.add_argument('--record-vio-input',action='store_true',help='Opt-in diagnostic replay inputs')
    p.add_argument('--diagnostic-vio',action='store_true',help='Small feature/bias CSV without images or point clouds')
    p.add_argument('--planner-config',type=Path,help='Override scoring parameters after sensor calibration')
    p.add_argument('--fusion-calibration',type=Path,help='Frozen independent relative-odometry calibration JSON')
    p.add_argument('--fusion-imu-noise',choices=['legacy','sensor','sensor_gyro'],default='legacy',help='MIMOSA measurement noise from existing config or simulated sensor density (both or gyro only); bias settings unchanged')
    p.add_argument('--fusion-initial-velocity-sigma',type=float,help='Initial zero-velocity prior uncertainty in m/s; omitted retains source configuration')
    p.add_argument('--min-host-free-gb',type=float,default=2.)
    p.add_argument('--imu-model',choices=['sensor','legacy'],default='sensor',help='Use SDF bias dynamics and world gravity; legacy reproduces old random walk and 9.81')
    p.add_argument('--legacy-slam-start',action='store_true',help='Reproduce MIMOSA initialization overlapping takeoff')
    p.add_argument('--stationary-duration',type=float,default=3.,help='Zero disables static IMU mean/bias initialization')
    p.add_argument('--stationary-mode',choices=['legacy','mean','wait'],default='wait')
    p.add_argument('--native-monitor',action='store_true')
    p.add_argument('--smooth-tracking-rate',action='store_true')
    p.add_argument('--corner-speed-limit',action='store_true')
    p.add_argument('--initialization-excitation',action='store_true',help='Common 26 s preparation motion, excluded from tracking evaluation')
    p.add_argument('--velocity-feedback-xy-only',action='store_true')
    p.add_argument('--velocity-feedback',type=float,default=0.)
    p.add_argument('--clock-hz',type=int,default=0,help='Optional native clock relay; 1000 removes only duplicate paused times')
    p.add_argument('--serialized-fusion',action='store_true')
    p.add_argument('--ordered-fusion',action='store_true',help='Experimental timestamp-ordered LiDAR/VIO scheduling; preserves sensor samples')
    p.add_argument('--safe-shutdown',action='store_true',help='Isolated Gazebo stop serialization and process-local Windows core-dump guard')
    p.add_argument('--temporal-fusion',action='store_true',help='Experimental AR(1) external-odometry likelihood')
    p.add_argument('--velocity-fusion',action='store_true',help='Fuse reported body velocity and covariance at 1 Hz, plus gravity direction')
    p.add_argument('--fixed-gravity-imu',action='store_true',help='Honor the fixed-gravity configuration in IMU factors and prediction')
    p.add_argument('--sensor-bias-model',action='store_true',help='Use the generated IMU sensor bias decay and driving noise')
    p.add_argument('--batched-fusion',action='store_true',help='Apply coincident vision and LiDAR before publishing or inserting map points')
    p.add_argument('--joint-rovio-information',action='store_true',help='Use read-only ROVIO velocity/landmark joint covariance publisher')
    p.add_argument('--geometry-rovio-information',action='store_true')
    p.add_argument('--lidar-four-dof',action='store_true',help='Use existing MIMOSA registration for position and yaw, leaving tilt to IMU/VIO')
    p.add_argument('--tilt-translation',action='store_true',help='Use visual displacement without visual rotation, plus sparse gravity direction')
    p.add_argument('--tilt-fusion',action='store_true',help='Add sparse visual-inertial gravity direction factors')
    p.add_argument('--translation-fusion',action='store_true',help='Experimental visual displacement constraints only')
    p.add_argument('--tbb-threads',type=int,default=0)
    p.add_argument('--paired',action='store_true',help='Independent fixed and active estimators on the same physical flight')
    p.add_argument('--shared-initialization',action='store_true',help='Paired experiment only: share unchanged reference frames until measured views separate')
    p.add_argument('--port',type=int,default=11381)
    p.add_argument('--domain',type=int,default=81)
    run(p.parse_args())
