#!/usr/bin/env python3
"""Independent complete flights in isolated Gazebo/ROS namespaces; resumable."""
import argparse,json,signal,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--labels',nargs='+',default=['run1_prefix','run1_tail_a1','run1_tail_b1'])
    p.add_argument('--modes',nargs='+',default=['fixed','active']);p.add_argument('--jobs',type=int,default=3)
    p.add_argument('--fixed-angles-deg',nargs=2,type=float,metavar=('YAW','PITCH'))
    p.add_argument('--end',type=float,default=0);p.add_argument('--factor',type=float,default=3)
    p.add_argument('--domain-base',type=int,default=84);p.add_argument('--wall-limit',type=float,default=3600)
    p.add_argument('--paired',action='store_true');p.add_argument('--fast',action='store_true')
    p.add_argument('--shared-initialization',action='store_true')
    p.add_argument('--pacing-extra-lidar',action='append',default=[])
    p.add_argument('--paced',action='store_true');p.add_argument('--fast-map',action='store_true');p.add_argument('--fast-icp',action='store_true');p.add_argument('--serialized-fusion',action='store_true');p.add_argument('--tbb-threads',type=int,default=0)
    p.add_argument('--airborne-init',action='store_true');p.add_argument('--gpu-keepalive',action='store_true')
    p.add_argument('--compact-map',action='store_true');p.add_argument('--profile',action='store_true')
    p.add_argument('--ordered-fusion',action='store_true')
    p.add_argument('--safe-shutdown',action='store_true')
    p.add_argument('--temporal-fusion',action='store_true',help='Experimental correlated external-odometry factors')
    p.add_argument('--velocity-fusion',action='store_true')
    p.add_argument('--fixed-gravity-imu',action='store_true')
    p.add_argument('--sensor-bias-model',action='store_true')
    p.add_argument('--batched-fusion',action='store_true')
    p.add_argument('--joint-rovio-information',action='store_true')
    p.add_argument('--geometry-rovio-information',action='store_true')
    p.add_argument('--lidar-four-dof',action='store_true')
    p.add_argument('--tilt-translation',action='store_true')
    p.add_argument('--tilt-fusion',action='store_true')
    p.add_argument('--translation-fusion',action='store_true',help='Fuse visual displacement without a visual rotation measurement')
    p.add_argument('--drpm-fusion',action='store_true',help='Enable the adopted radial-noise DRPM registration in both paired estimators')
    p.add_argument('--map-variant',choices=['compact','scratch','pruned','cached','neighbors','prefetch'],default='compact')
    p.add_argument('--rovio-noise-profile',choices=['sensor','dragon'],default='sensor')
    p.add_argument('--imu-sampling',choices=['resolved','legacy'],default='resolved')
    p.add_argument('--sensor-profile',choices=['native','dragon'],default='native')
    p.add_argument('--camera-far',type=float,default=None)
    p.add_argument('--diagnostic-sensors',action='store_true')
    p.add_argument('--diagnostic-imu-only',action='store_true',help='With diagnostic sensors, omit reference cameras to save storage and rendering')
    p.add_argument('--record-vio-input',action='store_true')
    p.add_argument('--diagnostic-vio',action='store_true')
    p.add_argument('--planner-config',type=Path)
    p.add_argument('--fusion-calibration',type=Path)
    p.add_argument('--fusion-imu-noise',choices=['legacy','sensor','sensor_gyro'],default='legacy')
    p.add_argument('--fusion-initial-velocity-sigma',type=float)
    p.add_argument('--seed',type=int,default=20261013)
    p.add_argument('--min-host-free-gb',type=float,default=2.)
    p.add_argument('--imu-model',choices=['sensor','legacy'],default='sensor')
    p.add_argument('--legacy-slam-start',action='store_true')
    p.add_argument('--stationary-duration',type=float,default=3.,help='Zero disables static IMU mean/bias initialization')
    p.add_argument('--stationary-mode',choices=['legacy','mean','wait'],default='wait')
    p.add_argument('--native-adapters',action='store_true');p.add_argument('--native-monitor',action='store_true')
    p.add_argument('--smooth-tracking-rate',action='store_true')
    p.add_argument('--corner-speed-limit',action='store_true')
    p.add_argument('--initialization-excitation',action='store_true',help='Common 26 s preparation motion, excluded from tracking evaluation')
    p.add_argument('--velocity-feedback-xy-only',action='store_true')
    p.add_argument('--velocity-feedback',type=float,default=0.)
    p.add_argument('--clock-hz',type=int,default=0,help='Optional native clock relay; 1000 removes only duplicate paused times')
    p.add_argument('--gpu-interval',type=float,default=.001);p.add_argument('--gpu-cycles',type=int,default=1000000)
    p.add_argument('--continue-on-error',action='store_true',help='Finish other independent flights if one fails')
    a=p.parse_args()
    if a.pacing_extra_lidar and not a.paced:p.error('Diagnostic completion pacing requires --paced')
    if a.temporal_fusion and (not a.ordered_fusion or not a.fusion_calibration):
        p.error('--temporal-fusion requires --ordered-fusion and --fusion-calibration')
    if a.translation_fusion and (a.temporal_fusion or not a.ordered_fusion or not a.fusion_calibration):
        p.error('--translation-fusion requires ordered fusion and calibration; excludes temporal fusion')
    if a.shared_initialization and not a.paired:p.error('--shared-initialization requires --paired')
    if a.fixed_angles_deg is not None and a.modes!=['fixed']:p.error('Fixed-angle campaign requires --modes fixed')
    a.output.mkdir(parents=True,exist_ok=True)
    refs=NATIVE/'logs/active-fixed-three-20261013';original=json.loads((NATIVE/'logs/zab-full-campaign/results.json').read_text())
    if a.tilt_fusion and not a.fixed_gravity_imu:p.error('--tilt-fusion requires --fixed-gravity-imu')
    if a.sensor_bias_model and not a.fixed_gravity_imu:p.error('--sensor-bias-model requires --fixed-gravity-imu')
    if a.batched_fusion and (not a.velocity_fusion or not a.sensor_bias_model):p.error('Batched fusion requires body velocity and sensor bias model')
    if a.lidar_four_dof and (not a.fixed_gravity_imu or not a.tilt_fusion or a.drpm_fusion):p.error('Four-DoF LiDAR requires fixed gravity, VIO tilt, and DRPM disabled')
    if a.velocity_fusion and (not a.tilt_fusion or a.tilt_translation):p.error('Velocity fusion requires tilt, excludes displacement')
    if a.tilt_translation and not a.tilt_fusion:p.error('--tilt-translation requires --tilt-fusion')
    if a.tilt_fusion and (a.temporal_fusion or a.translation_fusion or not a.ordered_fusion or not a.fusion_calibration):
        p.error('--tilt-fusion requires ordered fusion and calibration, excludes other variants')
    manifest=dict(platform='unipilot',fixed_angles_deg=a.fixed_angles_deg,fusion_imu_noise=a.fusion_imu_noise,scope=('fixed/active monocular estimators share each complete physical flight' if a.paired else 'independent complete physical flights'),
        rovio_noise_profile=a.rovio_noise_profile,sensor_profile=a.sensor_profile,imu_sampling=a.imu_sampling,imu_model=a.imu_model,legacy_slam_start=a.legacy_slam_start,camera_far=a.camera_far,stationary_duration=a.stationary_duration,stationary_mode=a.stationary_mode,diagnostic_sensors=a.diagnostic_sensors,diagnostic_imu_only=a.diagnostic_imu_only,record_vio_input=a.record_vio_input,drpm_fusion=False,speed_mps=.5,factor_target=a.factor,jobs=a.jobs,paired=a.paired,fast=a.fast,
        airborne_initialization=a.airborne_init,gpu_keepalive=a.gpu_keepalive,gpu_interval=a.gpu_interval,gpu_cycles=a.gpu_cycles,paced=a.paced,fast_map=a.fast_map,fast_icp=a.fast_icp,compact_map=a.compact_map,map_variant=a.map_variant,native_adapters=a.native_adapters,native_monitor=a.native_monitor,velocity_feedback=a.velocity_feedback,velocity_feedback_xy_only=a.velocity_feedback_xy_only,smooth_tracking_rate=a.smooth_tracking_rate,corner_speed_limit=a.corner_speed_limit,initialization_excitation=a.initialization_excitation,clock_hz=a.clock_hz,continue_on_error=a.continue_on_error,profile=a.profile,serialized_fusion=a.serialized_fusion,ordered_fusion=a.ordered_fusion,tbb_threads=a.tbb_threads,status='running',runs={})
    manifest['drpm_fusion']=a.drpm_fusion
    manifest['temporal_fusion']=a.temporal_fusion
    manifest['translation_fusion']=a.translation_fusion
    manifest['tilt_fusion']=a.tilt_fusion
    manifest['tilt_translation']=a.tilt_translation
    manifest['fixed_gravity_imu']=a.fixed_gravity_imu
    manifest['sensor_bias_model']=a.sensor_bias_model
    manifest['batched_fusion']=a.batched_fusion
    manifest['joint_rovio_information']=a.joint_rovio_information
    manifest['geometry_rovio_information']=a.geometry_rovio_information
    manifest['lidar_four_dof']=a.lidar_four_dof
    manifest['velocity_fusion']=a.velocity_fusion
    manifest['safe_shutdown']=a.safe_shutdown
    manifest['fusion_initial_velocity_sigma']=a.fusion_initial_velocity_sigma
    manifest['diagnostic_extra_lidar_topics']=a.pacing_extra_lidar
    path=a.output/'campaign.json'
    if path.exists():manifest['runs']=json.loads(path.read_text())['runs']
    queue=[];running={};started=time.monotonic();gpu=None;last_save=started
    def save():
        manifest['wall_s']=time.monotonic()-started
        tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(manifest,indent=2)+'\n');tmp.replace(path)
    def stop(signum,frame):
        manifest['stop_signal']=int(signum)
        print('Campaign received stop signal',signum,flush=True)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    for label in a.labels:
        route=refs/(label+'_fixed')/'route';data=json.loads((route/'segments.json').read_text())['segments']
        end=1070. if label=='run1_prefix' else data[-1]['t0']+data[-1]['duration']
        if a.end:end=min(end,a.end)
        world=json.loads((Path(original[label]['path'])/'world.json').read_text())
        first=json.loads((route/'route.json').read_text())['waypoints'][0]
        spawn=[first[0],first[1],world['spawn_xyz'][2]]
        world['spawn']=spawn
        for mode in ((['fixed'] if a.fixed_angles_deg is not None else ['active']) if a.paired else a.modes):
            # In a paired viewpoint diagnostic the first estimator remains in
            # the usual *_active directory, but its protocol explicitly says
            # fixed and records the requested angle. The reference stays at 0.
            key=label+('_active' if a.paired else '_'+mode);out=a.output/key
            if (out/'runtime.json').exists() and json.loads((out/'runtime.json').read_text())['status']=='route_complete':continue
            if out.exists():raise RuntimeError(f'Incomplete output retained; use a new output directory: {out}')
            queue.append((key,mode,route,world,end,out))
    try:
        if a.gpu_keepalive:
            with (a.output/'gpu_keepalive.log').open('w') as log:
                gpu=subprocess.Popen([sys.executable,str(HERE/'gpu_keepalive.py'),'--duration',str(a.wall_limit+30),'--blocks','32','--interval',str(a.gpu_interval),'--cycles',str(a.gpu_cycles)],stdout=log,stderr=subprocess.STDOUT)
        while queue or running:
            if gpu and gpu.poll() is not None:raise RuntimeError('GPU keepalive exited early')
            for slot in range(a.jobs):
                if slot in running or not queue:continue
                key,mode,route,world,end,out=queue.pop(0);domain=a.domain_base+slot;port=11400+domain
                cmd=[sys.executable,str(HERE/'unipilot_run.py'),'--mode',mode,'--world',world['world_file'] if 'world_file' in world else world['world'],
                    '--spawn',*map(str,world['spawn']), '--route',str(route),'--route-end',str(end),'--validate','--serial',
                    '--seed',str(a.seed),'--factor',str(a.factor),'--domain',str(domain),'--port',str(port),'--output',str(out),'--duration',str(a.wall_limit),'--min-host-free-gb',str(a.min_host_free_gb)]
                if not a.drpm_fusion:cmd.append('--no-drpm')
                if a.planner_config:cmd.extend(['--planner-config',str(a.planner_config.resolve())])
                if a.fusion_calibration:cmd.extend(['--fusion-calibration',str(a.fusion_calibration.resolve())])
                if a.temporal_fusion:cmd.append('--temporal-fusion')
                if a.translation_fusion:cmd.append('--translation-fusion')
                if a.tilt_fusion:cmd.append('--tilt-fusion')
                if a.tilt_translation:cmd.append('--tilt-translation')
                if a.fixed_gravity_imu:cmd.append('--fixed-gravity-imu')
                if a.sensor_bias_model:cmd.append('--sensor-bias-model')
                if a.batched_fusion:cmd.append('--batched-fusion')
                if a.joint_rovio_information:cmd.append('--joint-rovio-information')
                if a.geometry_rovio_information:cmd.append('--geometry-rovio-information')
                if a.lidar_four_dof:cmd.append('--lidar-four-dof')
                if a.velocity_fusion:cmd.append('--velocity-fusion')
                cmd.extend(['--fusion-imu-noise',a.fusion_imu_noise])
                if a.fusion_initial_velocity_sigma is not None:cmd.extend(['--fusion-initial-velocity-sigma',str(a.fusion_initial_velocity_sigma)])
                if a.diagnostic_vio:cmd.append('--diagnostic-vio')
                cmd.extend(['--stationary-duration',str(a.stationary_duration),'--stationary-mode',a.stationary_mode,'--imu-model',a.imu_model,'--sensor-profile',a.sensor_profile,'--imu-sampling',a.imu_sampling,'--rovio-noise-profile',a.rovio_noise_profile])
                if a.fixed_angles_deg is not None:cmd.extend(['--fixed-angles-deg',*map(str,a.fixed_angles_deg)])
                if a.legacy_slam_start:cmd.append('--legacy-slam-start')
                if a.diagnostic_sensors:cmd.append('--diagnostic-sensors')
                if a.diagnostic_imu_only:cmd.append('--diagnostic-imu-only')
                if a.record_vio_input:cmd.append('--record-vio-input')
                if a.camera_far is not None:cmd.extend(['--camera-far',str(a.camera_far)])
                if a.paired:cmd.append('--paired')
                if a.shared_initialization:cmd.append('--shared-initialization')
                if a.fast:cmd.extend(['--inline-render','--batch-render'])
                if a.airborne_init:cmd.append('--airborne-init')
                if a.paced:cmd.append('--paced')
                if a.fast_map:cmd.append('--fast-map')
                if a.fast_icp:cmd.append('--fast-icp')
                if a.compact_map:cmd.extend(['--compact-map','--map-variant',a.map_variant])
                if a.profile:cmd.append('--profile')
                if a.native_adapters:cmd.append('--native-adapters')
                if a.native_monitor:cmd.append('--native-monitor')
                if a.corner_speed_limit:cmd.append('--corner-speed-limit')
                if a.initialization_excitation:cmd.append('--initialization-excitation')
                if a.smooth_tracking_rate:cmd.append('--smooth-tracking-rate')
                if a.velocity_feedback_xy_only:cmd.append('--velocity-feedback-xy-only')
                if a.clock_hz:cmd.extend(['--clock-hz',str(a.clock_hz)])
                if a.velocity_feedback:cmd.extend(['--velocity-feedback',str(a.velocity_feedback)])
                if a.serialized_fusion:cmd.append('--serialized-fusion')
                if a.ordered_fusion:cmd.append('--ordered-fusion')
                for topic in a.pacing_extra_lidar:cmd.extend(['--pacing-extra-lidar',topic])
                if a.safe_shutdown:cmd.append('--safe-shutdown')
                if a.tbb_threads:cmd.extend(['--tbb-threads',str(a.tbb_threads)])
                log=(a.output/(key+'.log')).open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT);log.close()
                running[slot]=(proc,key,time.monotonic());manifest['runs'][key]=dict(status='running',path=str(out),command=cmd,domain=domain,port=port,pid=proc.pid);save()
                print('Started',key,'domain',domain,flush=True)
            for slot,(proc,key,start) in list(running.items()):
                if proc.poll() is None:continue
                r=manifest['runs'][key];runtime=Path(r['path'])/'runtime.json'
                r.update(exit_code=proc.returncode,wall_s=time.monotonic()-start,status=json.loads(runtime.read_text())['status'] if runtime.exists() else 'failed')
                del running[slot];save();print('Finished',key,r['status'],round(r['wall_s'],1),flush=True)
                if r['status']!='route_complete' and not a.continue_on_error:raise RuntimeError('Flight incomplete: '+key)
            if time.monotonic()-started>a.wall_limit:raise RuntimeError('Campaign wall-time limit')
            if time.monotonic()-last_save>5:save();last_save=time.monotonic()
            time.sleep(.5)
        manifest['status']='complete' if all(r['status']=='route_complete' for r in manifest['runs'].values()) else 'failed'
    except KeyboardInterrupt:manifest['status']='interrupted'
    except Exception:
        manifest['status']='failed';raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for proc,_,_ in running.values():proc.send_signal(signal.SIGINT)
        for proc,key,start in running.values():
            proc.wait(timeout=35)
            runtime=Path(manifest['runs'][key]['path'])/'runtime.json'
            manifest['runs'][key].update(exit_code=proc.returncode,wall_s=time.monotonic()-start,
                status=json.loads(runtime.read_text())['status'] if runtime.exists() else 'interrupted')
        if gpu and gpu.poll() is None:gpu.send_signal(signal.SIGINT);gpu.wait(timeout=10)
        save()
if __name__=='__main__':main()
