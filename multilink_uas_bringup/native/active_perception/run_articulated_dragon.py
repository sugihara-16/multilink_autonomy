#!/usr/bin/env python3
"""Own a finite DRAGON articulation/LIVO smoke test and all children."""
import argparse,hashlib,json,os,re,shlex,signal,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]
sys.path.insert(0,str(NATIVE))
from manager import CLEAN
from unipilot_calibration import prepare,imu_runtime_parameters
from sensor_bias_parameters import parameters
from dragon_articulated import Kinematics

def calibration(out,world):
    fusion,variance=prepare(out)
    manifest=json.loads((out/'articulated_manifest.json').read_text());T=np.array(manifest['T_FC_camera'])
    from dragon_cameras import OPTICAL
    q=Rotation.from_matrix(T[:3,:3]@OPTICAL).inv().as_quat()
    text=(out/'rovio.info').read_text()
    for name,value in zip(('qCM_x','qCM_y','qCM_z','qCM_w','MrMC_x','MrMC_y','MrMC_z'),[*q,*T[:3,3]]):
        text=re.sub(rf'({name}\s+)[^;\s]+;',lambda m:m[1]+format(value,'.16g')+';',text)
    (out/'rovio.info').write_text(text)
    c=json.loads((out/'camera_manifest.json').read_text());c.update(robot='dragon',expected_joints={})
    c['cameras']=[dict(T_FC_camera=T.tolist(),topic='/dragon/end_camera/image')]
    (out/'camera_manifest.json').write_text(json.dumps(c,indent=2))
    actual_lidar=ET.parse(out/'dragon_slam.urdf').getroot().find(".//sensor[@name='end_lidar']")
    pose=np.fromstring(actual_lidar.findtext('pose'),sep=' ')
    fusion['lidar']['T_B_S']=[*map(float,pose[:3]),*map(float,Rotation.from_euler('xyz',pose[3:]).as_quat())]
    fusion['imu']['preintegration'].update(parameters(out/'unipilot_gimbal.sdf'))
    fusion['graph']['manager']['smoother']['initial_velocity_sigma']=1.
    cal=json.loads((NATIVE/'simulation_inputs/calibration/native_fusion_velocity_gravity_calibration.json').read_text())
    fusion['odometry']['manager'].update(relative_pose_covariance=np.array(cal['covariance']).ravel().tolist(),
        velocity_covariance_scale=cal['velocity_covariance_scale'],velocity_period_s=cal['velocity_period_s'],
        vio_gravity_aligned=True,tilt_sigma_rad=cal['gravity_sigma_rad'],tilt_period_s=1.)
    fusion['lidar']['geometric']['scan_to_map'].update(drpm_enabled=False,reg_4_dof=True)
    fusion['logs_directory']=str(out/'mimosa')+'/'
    (out/'mimosa').mkdir();(out/'mimosa.yaml').write_text(yaml.safe_dump(fusion))
    (out/'rovio_runtime.yaml').write_text(yaml.safe_dump({'rovio_gimbal':imu_runtime_parameters(out/'unipilot_gimbal.sdf',world)}))
    launch=ET.parse(NATIVE/'mimosa_unipilot.launch')
    wrapper=out/'fusion.sh';wrapper.write_text('#!/bin/bash\nshift\nexec '+str(WS/'build/mimosa-batched-vision/mimosa_batched_node')+' "$@"\n');wrapper.chmod(0o755)
    launch.getroot().find('node').set('launch-prefix',str(wrapper));launch.write(out/'mimosa.launch')

def run(a):
    import shutil,socket,fcntl
    if shutil.disk_usage('/mnt/c').free<8e9:raise RuntimeError('Less than 8GB Windows reserve')
    if not np.isfinite(a.distance) or not 0<=a.distance<=90:raise ValueError('This validation supports 0 to 90 metres')
    if not np.isfinite(a.speed) or a.speed<=0:raise ValueError('Positive finite flight speed required')
    if a.inspect_gimbal and not a.gimbal:raise ValueError('Gimbal inspection requires --gimbal')
    if a.unipilot_matched and (not a.gimbal or a.inspect_gimbal):raise ValueError('Matched experiment requires gimbal without inspection sweep')
    common_runtime=a.unipilot_matched or a.mechanism_comparison
    if a.mechanism_comparison and (a.unipilot_matched or a.inspect_gimbal):raise ValueError('Mechanism comparison is separate from legacy/matched-gimbal runs')
    rovio_binary=WS/('build/rovio-joint-geometry/rovio_gimbal' if a.unipilot_matched else 'build/rovio-articulated/rovio_articulated')
    fusion_library=WS/('build/mimosa-batched-vision' if a.fusion_profile=='baseline' else 'build/mimosa-batched-drpm')
    if a.fusion_profile=='drpm_isotropic20':fusion_library=WS/'build/mimosa-batched-drpm-support'
    if a.fusion_profile in ('drpm_isotropic20_view','drpm_isotropic20_calibrated'):fusion_library=WS/'build/mimosa-batched-drpm-support-view'
    if a.fusion_diagnostics and a.diagnostic_suite=='view' and a.fusion_profile!='drpm_isotropic20_view':
        raise ValueError('View diagnostic requires drpm_isotropic20_view profile')
    if not (fusion_library/'libmimosa_lib.so').exists():raise RuntimeError('Missing fusion library: '+str(fusion_library))
    if not rovio_binary.exists():raise RuntimeError('Missing ROVIO binary: '+str(rovio_binary))
    lock=(NATIVE/'.articulated.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('ROS master already running on test port')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    simulation_world=a.world
    if common_runtime:
        from matched_dragon import prepare_world
        simulation_world=prepare_world(out,a.world)
    gimbal_profile=a.gimbal_profile or ('unipilot' if common_runtime else 'legacy')
    bindir=out/'bin';bindir.mkdir()
    real_gz=shutil.which('gz')
    wrapper=bindir/'gz';wrapper.write_text('#!/bin/bash\nif [[ "$1" == sim ]]; then\nshift\nexec '+shlex.quote(real_gz)+' sim --seed '+str(a.seed)+' "$@"\nfi\nexec '+shlex.quote(real_gz)+' "$@"\n');wrapper.chmod(0o755)
    sources=list(HERE.glob('*articulated*'))+list(HERE.glob('*dragon_gimbal*'))+[NATIVE/'dragon_articulated.py',NATIVE/'dragon_gimbal.py',NATIVE/'dragon_sim.launch.py',HERE/'initialization_motion.py']
    if common_runtime:sources+=list(HERE.glob('*.py'))+[HERE/'motion_view_v10.yaml',HERE/'config.yaml']
    binaries=[rovio_binary,WS/'build/mimosa-batched-vision/mimosa_batched_node',fusion_library/'libmimosa_lib.so']
    (out/'protocol.json').write_text(json.dumps(dict(mode=a.mode,world=str(a.world),distance_m=a.distance,speed_mps=a.speed,
        sensor_profile='native',imu_sampling='resolved',camera_far_m=1000,preparation_cycles=2,seed=a.seed,unipilot_matched=a.unipilot_matched,mechanism_comparison=a.mechanism_comparison,
        estimator=('UniPilot ROVIO rotation-only binary' if a.unipilot_matched else 'ROVIO with full SE3 camera transform')+' + MIMOSA',fusion_profile=a.fusion_profile,
        truth_usage='flight control and offline evaluation only',record_bag=False,headless=a.headless,camera_actuation='gimbal' if a.gimbal else 'body_joints',inspection_sweep=a.inspect_gimbal,
        sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources+binaries if p.is_file()}),indent=2))
    snapshot=out/'implementation';snapshot.mkdir()
    for source in sources:
        if source.is_file():shutil.copy2(source,snapshot/source.name)
    children=[];started=time.monotonic()
    def start(name,cmd,ros1=False,extra=''):
        script='source '+shlex.quote(str(NATIVE/('env_ros1.sh' if ros1 else 'env.sh')))+'\n'
        if ros1:script+=f'source {WS}/slam_ros1_ws/devel/setup.bash\nsource {WS}/comparison_ws/devel/setup.bash\n'
        script+=f'export ROS_MASTER_URI=http://127.0.0.1:{a.port} ROS_DOMAIN_ID={a.domain} GZ_PARTITION=dragon_articulated_{a.domain} OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1\n'
        script+=f'export PATH={shlex.quote(str(bindir))}:$PATH\nexport PYTHONPATH={shlex.quote(str(HERE))}:${{PYTHONPATH:-}}\n'
        if common_runtime:
            from matched_dragon import runtime_environment
            script+=runtime_environment(name)
        script+=extra+'\nexec '+shlex.join(map(str,cmd))
        with (out/(name+'.log')).open('w') as f:p=subprocess.Popen(['bash','--noprofile','--norc','-c',script],env=CLEAN,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,p));(out/'owners.json').write_text(json.dumps({k:v.pid for k,v in children},indent=2));return p
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    success=False
    try:
        start('master',['roscore','-p',a.port],True);time.sleep(2)
        sim=start('simulation',['ros2','launch',NATIVE/'dragon_sim.launch.py','articulated:=true',f'end_gimbal:={str(a.gimbal).lower()}',f'gimbal_profile:={gimbal_profile}',f'matched_joint_mount:={str(a.mechanism_comparison and not a.gimbal).lower()}',f'headless:={str(a.headless).lower()}',f'world_file:={simulation_world}',f'output:={out}','spawn_x:=4.6755','spawn_y:=0.001','spawn_z:=0.4','takeoff_height:=2.0'],
            extra=f'export GZ_SIM_RESOURCE_PATH={shlex.quote(str(a.world.parent/"models"))}:${{GZ_SIM_RESOURCE_PATH:-}}')
        deadline=time.monotonic()+90
        while not (out/'dragon_slam.urdf').exists():
            if sim.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Model generation failed')
            time.sleep(.2)
        calibration(out,a.world)
        if a.fusion_profile!='baseline':
            cfg=yaml.safe_load((out/'mimosa.yaml').read_text())
            cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=1,
                drpm_noise_model=0 if a.fusion_profile.startswith('drpm_isotropic') else 2)
            if a.fusion_profile.startswith('drpm_isotropic20'):cfg['lidar']['geometric']['scan_to_map']['num_corres_points']=20
            if a.fusion_profile=='drpm_isotropic20_calibrated':
                from unipilot_calibration import fusion_white_noise
                from numerical_relinearization import apply
                (out/'fusion_noise_model.json').write_text(json.dumps(fusion_white_noise(cfg,out/'unipilot_gimbal.sdf','sensor'),indent=2))
                apply(cfg)
            (out/'mimosa.yaml').write_text(yaml.safe_dump(cfg))
            (out/'fusion_library_manifest.json').write_text((fusion_library/'manifest.json').read_text())
        if a.fusion_diagnostics:
            diagnostic_binary=WS/'build/mimosa-fusion-trace/mimosa_trace_node'
            if not diagnostic_binary.exists():raise RuntimeError('Build fusion trace first')
            wrapper=out/'fusion.sh'
            wrapper.write_text('#!/bin/bash\nshift\nexec '+str(diagnostic_binary)+' "$@"\n')
            (out/'fusion_trace_manifest.json').write_text((diagnostic_binary.parent/'manifest.json').read_text())
            variants={'basic':('duplicate','no_vision'),
                      'solver':('duplicate','full_solver','sensor_noise','sensor_full'),
                      'constraints':('duplicate','no_lidar','standard_degeneracy','drpm_weight'),
                      'drpm':('duplicate','no_lidar','drpm_official'),
                      'planes':('duplicate','plane20','plane50','drpm_isotropic'),
                      'support':('duplicate','no_lidar','drpm_support20','drpm_support50'),
                      'view':('duplicate','view_masked','sensor_noise','sensor_full')}[a.diagnostic_suite]
            for variant in variants:
                directory=out/variant;directory.mkdir()
                cfg=yaml.safe_load((out/'mimosa.yaml').read_text())
                cfg['logs_directory']=str(directory)+'/'
                cfg['diagnostic_no_vision']=variant=='no_vision'
                if variant=='no_lidar':cfg['lidar']['geometric']['enabled']=False
                if variant=='standard_degeneracy':cfg['lidar']['geometric']['scan_to_map']['project_on_degneneracy']=True
                if variant=='drpm_weight':cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=0)
                if variant=='drpm_official':cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=1)
                if variant in ('plane20','plane50','drpm_plane50'):
                    cfg['lidar']['geometric']['scan_to_map']['num_corres_points']=20 if variant=='plane20' else 50
                if variant=='drpm_plane50':cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=1)
                if variant=='drpm_isotropic':cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=1,drpm_noise_model=0)
                if variant.startswith('drpm_support'):
                    cfg['lidar']['geometric']['scan_to_map'].update(drpm_enabled=True,drpm_registration_mode=1,drpm_noise_model=0,
                        num_corres_points=int(variant.removeprefix('drpm_support')))
                if variant in ('sensor_noise','sensor_full'):
                    from unipilot_calibration import fusion_white_noise
                    (directory/'noise_model.json').write_text(json.dumps(fusion_white_noise(cfg,out/'unipilot_gimbal.sdf','sensor'),indent=2))
                if variant in ('full_solver','sensor_full'):
                    from numerical_relinearization import apply
                    apply(cfg)
                (directory/'mimosa.yaml').write_text(yaml.safe_dump(cfg))
                launch=ET.parse(out/'mimosa.launch');node=launch.getroot().find('node')
                node.set('name','trace_'+variant)
                for e in node.findall('remap'):
                    if e.get('to','').startswith('/slam/') and not e.get('to').startswith('/slam/input/'):
                        e.set('to','/trace/'+variant+e.get('to'))
                for topic in ('/active_lidar_diagnostics','/active/lidar_information','/tf','/tf_static'):
                    ET.SubElement(node,'remap',{'from':topic,'to':'/trace/'+variant+topic})
                launch.write(directory/'mimosa.launch')
        if common_runtime:
            from matched_dragon import prepare_runtime
            prepare_runtime(out,a.world,mechanism_comparison=a.mechanism_comparison)
        if a.gimbal:
            from audit_dragon_gimbal import audit
            audit_result=audit(ET.parse(out/'dragon_slam.urdf').getroot())
            (out/'occlusion_audit.json').write_text(json.dumps(audit_result,indent=2))
            if not audit_result['passed']:raise RuntimeError('Camera frustum intersects robot bounds')
        topics={'/clock':'rosgraph_msgs/msg/Clock','/dragon/imu/raw':'sensor_msgs/msg/Imu','/dragon/lidar/points':'sensor_msgs/msg/PointCloud2',
                '/active/image':'sensor_msgs/msg/Image','/active/extrinsic':'geometry_msgs/msg/PoseStamped','/active/initialize_vio':'std_msgs/msg/Bool',
                '/articulated/enable':'std_msgs/msg/Bool','/articulated/target':'sensor_msgs/msg/JointState','/dragon/articulated/states':'sensor_msgs/msg/JointState',
                '/dragon/ground_truth':'nav_msgs/msg/Odometry'}
        if a.unipilot_matched:topics.update({'/active/encoder':'std_msgs/msg/Float64MultiArray',
              '/active/gimbal/yaw':'std_msgs/msg/Float64','/active/gimbal/pitch':'std_msgs/msg/Float64'})
        config={'active_lidar_diagnostics':True,'use_sim_time':True,'topics':[dict(topic=k,type=v,queue_size=2000 if k.endswith(('/raw','/states','/encoder')) else 30) for k,v in topics.items()],'services_1_to_2':[],'services_2_to_1':[]}
        (out/'bridge.yaml').write_text(yaml.safe_dump(config))
        for name,file in [('bridge_params','bridge.yaml'),('rovio_params','rovio_runtime.yaml')]:
            if start(name,['rosparam','load',out/file],True).wait(20):raise RuntimeError('Parameter setup failed')
        start('bridge',[WS/'build/reliable-bridge/reliable_bridge'])
        start('encoder',['python3',HERE/'articulated_encoder.py','--output',out])
        if not a.unipilot_matched:start('actuator',['python3',HERE/('dragon_gimbal_actuator.py' if a.gimbal else 'articulated_actuator.py'),'--output',out])
        start('monitor',['python3',HERE/'monitor.py','--output',out],True)
        if a.fusion_diagnostics:start('input_probe',['python3',HERE/'fusion_input_probe.py','--output',out/'input_probe'],True)
        start('inputs',[WS/'build/native-adapters/native_slam_inputs','_imu_topic:=/dragon/imu/raw','_points_topic:=/dragon/lidar/points','_gate_slam:=true','_imu_rate:=1000',f'_audit_path:={out}/input_audit.json'],True)
        start('rovio',[rovio_binary,f'_config:={out}/rovio.info',f'_calibration:={out}/cam0.yaml','_stationary_initialization_duration:=3','_stationary_mode:=wait',f'_diagnostic_csv:={out}/vio.csv'],True)
        start('mimosa',['roslaunch',out/'mimosa.launch',f'config:={out}/mimosa.yaml',f'logs:={out}/mimosa','external_odometry:=/rovio/odometry'],True,
              extra=f'export LD_LIBRARY_PATH={fusion_library}:{WS}/build/active_perception:{WS}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}')
        if a.fusion_diagnostics:
            for variant in variants:
                directory=out/variant
                libdir=WS/'build/mimosa-batched-drpm' if variant in ('drpm_official','drpm_plane50','drpm_isotropic') else fusion_library
                if variant.startswith('drpm_support'):libdir=WS/'build/mimosa-batched-drpm-support'
                if variant=='view_masked':libdir=WS/'build/mimosa-batched-drpm-support'
                start(variant,['roslaunch',directory/'mimosa.launch',f'config:={directory}/mimosa.yaml',f'logs:={directory}','external_odometry:=/rovio/odometry'],True,
                    extra=f'export UAS_TBB_THREADS=1 LD_PRELOAD={WS}/build/mimosa-speed/libtbb_limit.so:${{LD_PRELOAD:-}}\nexport LD_LIBRARY_PATH={libdir}:{WS}/build/active_perception:{WS}/comparison_ws/deps/lib:${{LD_LIBRARY_PATH:-}}')
        if a.unipilot_matched:
            start('planner',['python3',HERE/'ablation_planner_node.py','--output',out,'--mode',a.mode,'--config',out/'planner.yaml','--library',WS/'build/active_perception/libfif_active.so'],True)
            start('pacing',['python3',out/'matched_pacer.py','--output',out,'--planner','--planner-period','.2','--lidar-stop-lag','.4','--lidar-resume-lag','.26'],True)
        else:
            start('planner',['python3',HERE/'articulated_planner.py','--output',out,'--mode',a.mode],True)
            if a.mechanism_comparison:start('pacing',['python3',out/'matched_pacer.py','--output',out,'--planner','--planner-period','.2','--lidar-stop-lag','.4','--lidar-resume-lag','.26'],True)
        flight=start('flight',['python3',HERE/'articulated_hover_test.py','--output',out,'--distance',a.distance,'--speed',a.speed]+(['--gimbal'] if a.gimbal else [])+(['--inspect-gimbal'] if a.inspect_gimbal else [])+(['--unipilot-matched'] if common_runtime else []))
        deadline=time.monotonic()+1250
        while flight.poll() is None:
            failed=[n for n,p in children if n not in ('flight','bridge_params','rovio_params') and p.poll() is not None]
            if failed:raise RuntimeError('Unexpected process exit: '+str(failed))
            sim_log=(out/'simulation.log').read_text(errors='replace')
            if 'process has died' in sim_log:raise RuntimeError('A required simulation process failed; see simulation.log')
            if time.monotonic()-started>90 and not (out/'flight_status.json').exists():raise RuntimeError('Simulation clock did not start')
            if time.monotonic()>deadline:raise RuntimeError('Flight timeout')
            time.sleep(1)
        if flight.returncode:raise RuntimeError('Flight validation failed')
        if a.fusion_diagnostics:
            events={e['stage']:e['time'] for e in json.loads((out/'flight_events.json').read_text())}
            target=events['active']+a.distance/a.speed+2.
            drain_deadline=time.monotonic()+90
            while True:
                pending=[]
                for variant in variants:
                    path=out/variant/'sensor_states.csv'
                    with path.open('rb') as f:
                        f.seek(max(0,path.stat().st_size-4096));lines=f.read().decode().splitlines()
                    stamps=[float(l.split(',')[2]) for l in lines[1:] if len(l.split(','))==15 and l.split(',')[2]!='state_t']
                    if not stamps or max(stamps)<target-1e-6:pending.append(variant)
                if not pending:break
                if time.monotonic()>drain_deadline:raise RuntimeError('Diagnostic incomplete: '+str(pending))
                time.sleep(.2)
        success=True
    finally:
        for name,p in reversed(children):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        deadline=time.monotonic()+15
        while any(p.poll() is None for _,p in children) and time.monotonic()<deadline:time.sleep(.2)
        for name,p in children:
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGKILL)
                except ProcessLookupError:pass
            p.wait()
        (out/'completion.json').write_text(json.dumps(dict(flight_completed=success,wall_s=time.monotonic()-started,processes={n:p.returncode for n,p in children}),indent=2))
    if success:
        from report_articulated_dragon import report
        result=report(out)
        # Fixed mode is a comparison baseline, so no movement decision required.
        checks={k:v for k,v in result['checks'].items() if a.mode!='fixed' or k!='autonomous_view_selection'}
        if not all(checks.values()):raise RuntimeError('Integration validation failed; see validation.json')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--world',type=Path,default=WS/'assets/weathered_holdout_20261026/24301/world.sdf')
    p.add_argument('--domain',type=int,default=89);p.add_argument('--port',type=int,default=11489)
    p.add_argument('--distance',type=float,default=90.)
    p.add_argument('--speed',type=float,default=.5)
    p.add_argument('--gimbal',action='store_true')
    p.add_argument('--inspect-gimbal',action='store_true')
    p.add_argument('--unipilot-matched',action='store_true')
    p.add_argument('--mechanism-comparison',action='store_true',help='Common forward mount, SE3 ROVIO, preparation and articulated planner for both mechanisms')
    p.add_argument('--fusion-profile',choices=['baseline','drpm_isotropic','drpm_isotropic20','drpm_isotropic20_view','drpm_isotropic20_calibrated','drpm_radial'],default='baseline')
    p.add_argument('--fusion-diagnostics',action='store_true',help='Trace post-sensor states and run duplicate/no-vision controls on the identical inputs')
    p.add_argument('--diagnostic-suite',choices=['basic','solver','constraints','drpm','planes','support','view'],default='basic')
    p.add_argument('--gimbal-profile',choices=['legacy','unipilot'])
    p.add_argument('--seed',type=int,default=20262801)
    p.add_argument('--headless',action=argparse.BooleanOptionalAction,default=True,
                   help='Disable Gazebo GUI (the ROS launch defaults to --no-headless)')
    p.add_argument('--mode',choices=['active','fixed'],default='active');a=p.parse_args();run(a)
