#!/usr/bin/env python3
"""Finish one declared six-pair development campaign, preserving every result."""
import argparse,hashlib,json,os,signal,subprocess,sys,time,yaml
from pathlib import Path
from continue_gimbal_validation import next_prefix_domain

HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent
def read(p):
    try:return json.loads(p.read_text())
    except (FileNotFoundError,json.JSONDecodeError):return {}

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--candidate',required=True);p.add_argument('--seeds',nargs=2,type=int,required=True)
    p.add_argument('--prior-prefix',required=True)
    p.add_argument('--resume',action='store_true',help='Resume a declaration whose scheduler has not started any child campaign')
    p.add_argument('--max-concurrent',type=int,choices=[1,2],default=1)
    p.add_argument('--drpm-fusion',action='store_true')
    p.add_argument('--safe-shutdown',action='store_true')
    p.add_argument('--fusion-calibration',type=Path)
    p.add_argument('--tilt-fusion',action='store_true')
    p.add_argument('--tilt-translation',action='store_true')
    p.add_argument('--velocity-fusion',action='store_true')
    p.add_argument('--fixed-gravity-imu',action='store_true')
    p.add_argument('--sensor-bias-model',action='store_true')
    p.add_argument('--batched-fusion',action='store_true')
    p.add_argument('--joint-rovio-information',action='store_true')
    p.add_argument('--geometry-rovio-information',action='store_true')
    p.add_argument('--lidar-four-dof',action='store_true')
    p.add_argument('--early-stop',action='store_true',help='Stop between completed campaigns once the declared six-pair criteria are impossible')
    p.add_argument('--fusion-imu-noise',choices=['legacy','sensor','sensor_gyro'],default='legacy')
    p.add_argument('--fusion-initial-velocity-sigma',type=float)
    a=p.parse_args();a.root=a.root.resolve();first,second=a.seeds
    def folder(seed,prefix=False):return a.root/f'{a.candidate}-{"prefix" if prefix else "short"}-seed{seed-20261000}'
    def complete(path,labels,require_clean=True):
        for label in labels:
            r=read(path/(label+'_active')/'runtime.json')
            if not r.get('stopped'):return False
            if require_clean and (r.get('status')!='route_complete' or any(x!=0 for x in r['processes'].values())):
                raise RuntimeError('Incomplete prior flight preserved: '+str(path/label))
        return True
    old=read(a.root/'frozen_candidate_v17.json')
    files=[f for f in old['files'] if not f.endswith('tracking_aware_v17.yaml')]
    calibration=(a.fusion_calibration or a.root/'native_fusion_calibration.json').resolve()
    if a.fusion_calibration:
        files=[f for f in files if not f.endswith('native_fusion_calibration.json')]
        files.extend([str(calibration),str(HERE/'calibrate_unipilot_fusion.py')])
    files.append(str(HERE/f'tracking_aware_{a.candidate}.yaml'))
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('track_lidar_depth',False):
        files.extend(str(HERE/f) for f in ['tracking_lidar_depth.py','test_tracking_lidar_depth.py','test_planner_lidar_depth.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('match_actuator_reachability',False):
        files.extend(str(HERE/f) for f in ['actuator_reachability.py','test_actuator_reachability.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_lidar_conditioned_velocity',False):
        files.extend(str(HERE/f) for f in ['lidar_conditioned_tracking_view.py','test_lidar_conditioned_tracking_view.py','lidar_conditioned_velocity_gain.py','test_lidar_conditioned_velocity_gain.py'])
    if a.safe_shutdown:
        files.extend(str(HERE/f) for f in ['build_serial_stop.py','build_no_crash_dump.py','performance/no_crash_dump.cpp','test_world_client_shutdown.py'])
        files.extend(str(NATIVE.parents[3]/f) for f in ['build/gz-stop/lib/libgz-sim8.so.8.15.0','build/no-crash-dump/libno_crash_dump.so'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_image_stabilization',False):
        files.append(str(HERE/'image_stabilization.py'))
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_parallax_fallback',False):
        files.extend(str(HERE/f) for f in ['parallax_fixation.py','test_parallax_fixation.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_spatial_parallax',False):
        files.extend(str(HERE/f) for f in ['spatial_parallax_fixation.py','test_spatial_parallax_fixation.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_joint_information',False):
        files.extend(str(HERE/f) for f in ['joint_information_view.py','test_joint_information_view.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_translation_information',False):
        files.extend(str(HERE/f) for f in ['translation_information_gain.py','test_translation_information_gain.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_stable_view',False):
        files.extend(str(HERE/f) for f in ['stable_view.py','test_stable_view.py'])
    if a.tilt_fusion:
        files=[f for f in files if not f.endswith('mimosa_ordered_node')]
        files.extend(str(HERE/f) for f in ['build_tilt_fusion.py','performance/tilt_odometry_manager.hpp','performance/test_tilt_odometry.cpp'])
        files.append(str(NATIVE.parents[3]/'build/mimosa-tilt/mimosa_tilt_node'))
    if a.tilt_translation:
        if not a.tilt_fusion:raise ValueError('--tilt-translation requires --tilt-fusion')
        files=[f for f in files if not f.endswith('mimosa_tilt_node')]
        files.extend(str(HERE/f) for f in ['build_gravity_translation.py','calibrate_translation_fusion.py','performance/gravity_translation_manager.hpp','performance/translation_odometry_factor.hpp','performance/test_translation_odometry_factor.cpp'])
        files.append(str(NATIVE.parents[3]/'build/mimosa-gravity-translation/mimosa_gravity_translation_node'))
    if a.fixed_gravity_imu:
        files.extend(str(HERE/f) for f in ['build_fixed_gravity.py','performance/test_fixed_gravity.cpp','record_gravity_state.py'])
        files.append(str(NATIVE.parents[3]/'build/mimosa-fixed-gravity/libmimosa_lib.so'))
    if a.velocity_fusion:
        if not a.tilt_fusion or a.tilt_translation:raise ValueError('Velocity fusion requires tilt, excludes displacement')
        files=[f for f in files if not f.endswith('mimosa_tilt_node')]
        files.extend(str(HERE/f) for f in ['build_velocity_fusion.py','calibrate_velocity_fusion.py','performance/body_velocity_factor.hpp','performance/velocity_odometry_manager.hpp','performance/test_body_velocity.cpp'])
        files.append(str(NATIVE.parents[3]/'build/mimosa-velocity/mimosa_velocity_node'))
    if a.sensor_bias_model:
        if not a.fixed_gravity_imu:raise ValueError('Sensor bias model requires fixed gravity')
        files=[f for f in files if not f.endswith('mimosa-fixed-gravity/libmimosa_lib.so')]
        files.extend(str(HERE/f) for f in ['build_sensor_bias_fusion.py','sensor_bias_parameters.py','performance/mean_reverting_bias_factor.hpp','performance/sensor_bias_model.hpp','performance/test_mean_reverting_bias.cpp'])
        files.append(str(NATIVE.parents[3]/'build/mimosa-sensor-bias/libmimosa_lib.so'))
    if a.batched_fusion:
        if not a.velocity_fusion or not a.sensor_bias_model:raise ValueError('Batched fusion requires body velocity and sensor bias model')
        files=[f for f in files if not f.endswith('mimosa_velocity_node') and not f.endswith('mimosa-sensor-bias/libmimosa_lib.so')]
        files.extend(str(HERE/f) for f in ['build_batched_fusion.py','performance/test_same_stamp_estimate.cpp','performance/test_batched_publication.cpp'])
        files.extend(str(NATIVE.parents[3]/'build/mimosa-batched-vision'/f) for f in ['libmimosa_lib.so','mimosa_batched_node','manifest.json','graph_manager.cpp','ordered_mimosa_node.cpp'])
    if a.joint_rovio_information:
        files.extend(str(HERE/f) for f in ['joint_velocity_view.py','joint_velocity_gain.py','rovio_joint_information.hpp','build_rovio_joint_information.py','test_joint_velocity_view.py','test_joint_velocity_gain.py','audit_joint_information_live.py'])
        files.extend(str(NATIVE.parents[3]/('build/rovio-joint-geometry' if a.geometry_rovio_information else 'build/rovio-joint-information')/f) for f in ['rovio_gimbal','manifest.json','rovio_gimbal.cpp'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_correlated_tracking',False):
        if not a.joint_rovio_information:raise ValueError('Correlated tracking requires joint ROVIO covariance')
        files.extend(str(HERE/f) for f in ['correlated_tracking_view.py','test_correlated_tracking_view.py'])
    if a.geometry_rovio_information:
        if not a.joint_rovio_information:raise ValueError('Geometry requires joint information')
        files.extend(str(HERE/f) for f in ['rovio_joint_geometry.hpp','build_rovio_joint_geometry.py','test_joint_geometry.cpp','audit_joint_geometry_live.py','joint_geometry.py','test_joint_geometry.py','predictive_bearing_covariance.py','test_predictive_bearing_covariance.py','predictive_tracking_view.py','test_predictive_tracking_view.py'])
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_predictive_geometry',False) and not a.geometry_rovio_information:
        raise ValueError('Predictive scoring requires geometry information')
    if yaml.safe_load((HERE/f'tracking_aware_{a.candidate}.yaml').read_text()).get('fixation_retinal_candidates',False):
        files.extend(str(HERE/f) for f in ['retinal_motion_candidates.py','test_retinal_motion_candidates.py','retinal_tracking_view.py','test_retinal_tracking_view.py'])
    hashes={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files}
    declaration=a.root/f'development_{a.candidate}.json'
    payload=dict(candidate=a.candidate,seeds=a.seeds,
        campaigns=[str(folder(s,p).relative_to(a.root)) for s in a.seeds for p in (False,True)],
        files=hashes,criterion=read(a.root/'validation_protocol.json')['success'],
        policy='Six development pairs fixed in advance; no adding repetitions to pass. Independent validation still required.')
    if a.drpm_fusion:payload['drpm_fusion']=True
    if a.safe_shutdown:payload['safe_shutdown']=True
    if a.tilt_fusion:payload['tilt_fusion']=True
    if a.tilt_translation:payload['tilt_translation']=True
    if a.fixed_gravity_imu:payload['fixed_gravity_imu']=True
    if a.sensor_bias_model:payload['sensor_bias_model']=True
    if a.batched_fusion:payload['batched_fusion']=True
    if a.joint_rovio_information:payload['joint_rovio_information']=True
    if a.geometry_rovio_information:payload['geometry_rovio_information']=True
    if a.lidar_four_dof:payload['lidar_four_dof']=True
    if a.fusion_imu_noise!='legacy':payload['fusion_imu_noise']=a.fusion_imu_noise
    if a.fusion_initial_velocity_sigma is not None:payload['fusion_initial_velocity_sigma']=a.fusion_initial_velocity_sigma
    if a.velocity_fusion:payload['velocity_fusion']=True
    if a.early_stop:payload['early_stop']=True
    if a.fusion_calibration:payload['fusion_calibration']=str(calibration)
    hashes[str(HERE/'simulation_shutdown.py')]=hashlib.sha256((HERE/'simulation_shutdown.py').read_bytes()).hexdigest()
    if declaration.exists():
        saved=read(declaration);saved.pop('bootstrap_equivalence',None)
        if not a.resume or saved!=payload:raise RuntimeError('Existing declaration must match exactly')
        if read(a.root/f'development_{a.candidate}_progress.json').get('started'):
            raise RuntimeError('Resume after children have started requires explicit process reconciliation')
    else:declaration.write_text(json.dumps(payload,indent=2)+'\n')
    def verify():
        for f,h in hashes.items():
            if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h:raise RuntimeError('Candidate changed during development: '+f)
    started={};finished=set();reported=set()
    def reject_impossible():
        # Only called between physical flights; never erase or trim a run.
        rows=[]
        for seed in a.seeds:
            path=folder(seed)
            if not complete(path,['run1_tail_a1','run1_tail_b1']):continue
            if path not in reported:
                subprocess.run([sys.executable,str(HERE/'report_gimbal_campaign.py'),str(path),'--plots',*(['--drpm-fusion'] if a.drpm_fusion else [])],check=True)
                reported.add(path)
            for label,r in read(path/'results.json').items():
                if not r['valid']:raise RuntimeError('Invalid completed evidence: '+str(path/label))
                m=r['metrics'];rows.append(dict(route=label.removesuffix('_active'),seed=seed,
                    ratio=m['active']['mimosa']['position_rmse_m']/m['fixed']['mimosa']['position_rmse_m']))
        criterion=payload['criterion'];reasons=[]
        if any(r['ratio']>criterion['max_single_pair_ratio'] for r in rows):reasons.append('A completed pair exceeds the maximum allowed ratio')
        if sum(r['ratio']<1 for r in rows)+(6-len(rows))<criterion['min_wins']:reasons.append('Too many losses to reach the required number of wins')
        for label in ['run1_tail_a1','run1_tail_b1']:
            pairs=[r for r in rows if r['route']==label]
            if len(pairs)==2 and sum(r['ratio'] for r in pairs)/2>criterion['per_route_mean_ratio_max']:
                reasons.append(label+' completed mean ratio fails the declared threshold')
        if reasons:
            (a.root/f'{a.candidate}_early_rejection.json').write_text(json.dumps(dict(
                candidate=a.candidate,reasons=reasons,completed_pairs=rows,criterion=criterion,
                status='rejected; unstarted flights omitted because passing is impossible; no results removed'),indent=2)+'\n')
            print('REJECTED',reasons,flush=True)
        return bool(reasons)
    def stop(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def launch(key,seed,prefix,domain):
        verify();labels=['run1_prefix'] if prefix else ['run1_tail_a1','run1_tail_b1']
        cmd=[sys.executable,str(HERE/'unipilot_campaign.py'),'--output',str(folder(seed,prefix)),
            '--labels',*labels,'--paired','--shared-initialization','--initialization-excitation',
            '--fast','--airborne-init','--gpu-keepalive','--jobs','1','--factor','1.2','--paced',
            '--fast-icp','--compact-map','--map-variant','prefetch','--native-adapters','--native-monitor',
            '--clock-hz','1000','--ordered-fusion','--tbb-threads','1','--wall-limit','2700',
            '--min-host-free-gb','8',
            '--domain-base',str(domain),'--planner-config',str(HERE/f'tracking_aware_{a.candidate}.yaml'),
            '--diagnostic-vio','--seed',str(seed),'--fusion-calibration',str(calibration),
            '--corner-speed-limit','--camera-far','1000']
        if a.drpm_fusion:cmd.append('--drpm-fusion')
        if a.safe_shutdown:cmd.append('--safe-shutdown')
        if a.tilt_fusion:cmd.append('--tilt-fusion')
        if a.tilt_translation:cmd.append('--tilt-translation')
        if a.fixed_gravity_imu:cmd.append('--fixed-gravity-imu')
        if a.sensor_bias_model:cmd.append('--sensor-bias-model')
        if a.batched_fusion:cmd.append('--batched-fusion')
        if a.joint_rovio_information:cmd.append('--joint-rovio-information')
        if a.geometry_rovio_information:cmd.append('--geometry-rovio-information')
        if a.lidar_four_dof:cmd.append('--lidar-four-dof')
        cmd.extend(['--fusion-imu-noise',a.fusion_imu_noise])
        if a.fusion_initial_velocity_sigma is not None:
            cmd.extend(['--fusion-initial-velocity-sigma',str(a.fusion_initial_velocity_sigma)])
        if a.velocity_fusion:cmd.append('--velocity-fusion')
        with (a.root/f'development_{a.candidate}_{key}.log').open('w') as log:
            started[key]=subprocess.Popen(cmd,cwd=NATIVE,stdout=log,stderr=subprocess.STDOUT)
        print('START',key,started[key].pid,flush=True)
    try:
        while len(finished)<3:
            for key,proc in started.items():
                if key in finished or proc.poll() is None:continue
                if proc.returncode:raise RuntimeError('Campaign failed: '+key)
                finished.add(key);print('DONE',key,flush=True)
            external_done=complete(folder(first),['run1_tail_a1','run1_tail_b1'])
            if a.early_stop and external_done and not (set(started)-finished) and reject_impossible():
                return
            def slot_available():return len(set(started)-finished)+(not external_done)<a.max_concurrent
            if 'second_short' not in started and external_done and slot_available():
                launch('second_short',second,False,86)
            # The previous candidate only owns the resource slot. Its accuracy
            # or shutdown validity cannot qualify/disqualify this new candidate.
            if 'first_prefix' not in started and slot_available() and complete(a.root/a.prior_prefix,['run1_prefix'],require_clean=False):
                launch('first_prefix',first,True,85)
            if 'first_prefix' in started and 'second_prefix' not in started and slot_available():
                status=read(folder(first,True)/'run1_prefix_active/route_status.json')
                memory=dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
                available=int(memory['MemAvailable'].strip().split()[0])*1024
                domain=next_prefix_domain('first_prefix' in finished,status.get('route_time_s',0),
                    'second_short' in finished,available,True)
                if domain is not None:launch('second_prefix',second,True,domain)
            tmp=a.root/f'development_{a.candidate}_progress.tmp'
            tmp.write_text(json.dumps(dict(started={k:v.pid for k,v in started.items()},finished=sorted(finished)),indent=2)+'\n')
            tmp.replace(tmp.with_suffix('.json'))
            time.sleep(3)
        verify()
        for seed in a.seeds:
            for prefix in (False,True):
                subprocess.run([sys.executable,str(HERE/'report_gimbal_campaign.py'),str(folder(seed,prefix)),'--plots',*(['--drpm-fusion'] if a.drpm_fusion else [])],check=True)
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for proc in started.values():
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
        for proc in started.values():
            if proc.poll() is None:proc.wait(timeout=45)
if __name__=='__main__':main()
