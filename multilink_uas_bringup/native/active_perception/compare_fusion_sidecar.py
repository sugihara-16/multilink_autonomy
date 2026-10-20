#!/usr/bin/env python3
"""Observe one future flight with additional fusion nodes, without driving it.

Uses existing binaries and previously frozen calibration. A duplicate of the
main active estimator checks that the additional nodes received the same data.
Only numeric logs are saved; no image or point-cloud recording. These results
are conditional on the main controller's camera motion, not a full controller
ablation. A later online comparison is required before adopting another fusion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import yaml

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]


def read(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def owner_environment(port):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc/'cmdline').read_bytes().split(b'\0')
            if not argv or Path(os.fsdecode(argv[0])).name not in ('mimosa_velocity_node','mimosa_batched_node'):
                continue
            env = dict(os.fsdecode(v).split('=', 1) for v in
                       (proc/'environ').read_bytes().split(b'\0') if b'=' in v)
            if env.get('ROS_MASTER_URI') == f'http://127.0.0.1:{port}':
                return env
        except (OSError, ValueError):
            continue
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--comparison', choices=['displacement','tilt_only','information_weighting','drpm_four_dof','initial_velocity','batched_fusion','sensor_white','numerical_imu','full_solver_noise','temporal_velocity'], default='displacement')
    parser.add_argument('--temporal-calibration', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    implementation = args.output/'implementation'
    implementation.mkdir()
    for name in ['compare_fusion_sidecar.py','report_fusion_sidecar.py','report_sidecar_partial.py']:
        shutil.copy2(HERE/name,implementation/name)
    if args.comparison in ('initial_velocity','sensor_white','numerical_imu','full_solver_noise'):
        shutil.copy2(HERE/'unipilot_calibration.py',implementation/'unipilot_calibration.py')
    if args.comparison in ('numerical_imu','full_solver_noise'):
        for name in ['numerical_relinearization.py','report_initial_velocity_diagnostic.py']:
            shutil.copy2(HERE/name,implementation/name)
    if args.comparison == 'batched_fusion':
        for name in ['manifest.json','graph_manager.cpp','ordered_mimosa_node.cpp']:
            shutil.copy2(WS/'build/mimosa-batched-vision'/name,implementation/name)
        shutil.copy2(HERE/'build_batched_fusion.py',implementation/'build_batched_fusion.py')
    if args.comparison == 'tilt_only':
        shutil.copy2(HERE/'build_tilt_only_diagnostic.py',implementation/'build_tilt_only_diagnostic.py')
        for name in ['manifest.json','velocity_odometry_manager.hpp','ordered_mimosa_node.cpp']:
            shutil.copy2(WS/'build/mimosa-tilt-only-diagnostic'/name,implementation/name)
    if args.comparison == 'drpm_four_dof':
        build=WS/'build/mimosa-drpm-four-dof'
        for name in ['manifest.json','drpm_registration.cpp']:
            shutil.copy2(build/name,implementation/name)
        for name in ['build_drpm_four_dof.py','performance/drpm_four_dof_kernel.hpp','performance/test_drpm_four_dof.cpp']:
            shutil.copy2(HERE/name,implementation/Path(name).name)
    calibration = read(args.calibration)
    temporal = None
    if args.comparison == 'temporal_velocity':
        if args.temporal_calibration is None:raise ValueError('Independent temporal calibration required')
        temporal=read(args.temporal_calibration)
        if temporal['source_calibration_sha256']!=hashlib.sha256(args.calibration.read_bytes()).hexdigest():
            raise ValueError('Temporal calibration uses a different covariance calibration')
        shutil.copy2(args.temporal_calibration,args.output/'temporal_calibration.json')
        for name in ['build_temporal_velocity.py','calibrate_velocity_temporal.py','performance/temporal_velocity_factor.hpp','performance/test_temporal_velocity.cpp']:
            shutil.copy2(HERE/name,implementation/Path(name).name)
        for name in ['manifest.json','ordered_mimosa_node.cpp','temporal_velocity_manager.hpp']:
            shutil.copy2(WS/'build/mimosa-temporal-velocity'/name,implementation/name)
    if args.comparison == 'displacement' and calibration.get('factor_interval_s') != .05:
        raise ValueError('Existing 20-Hz displacement calibration required')
    processes = {}
    manifest = dict(scope=__doc__, flight=str(args.flight), port=args.port,
                    calibration_sha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
                    status='waiting', truth_input=False, image_or_cloud_recording=False,
                    comparison_kind=args.comparison)
    if args.comparison == 'information_weighting':
        manifest['variant_meaning'] = ('Existing mode-0 DRPM probability weighting of LiDAR information, followed by '
            'the existing four-DoF projection. This is NOT the official DRPM update rule. '
            'No VIO weights, sensor noise, or other estimator settings changed.')
    if args.comparison == 'drpm_four_dof':
        manifest['variant_meaning'] = ('Four-variable restriction of the official probability-scaled update: '
            'world-vertical heading and translation only. Excluded tilt has zero information. '
            'This constrained extension is not the unmodified official six-DoF implementation. '
            'VIO observations and calibration are unchanged; the diagnostic does not drive the camera.')
    if args.comparison == 'initial_velocity':
        manifest['variant_meaning'] = ('Two-by-two diagnostic of initial velocity prior and IMU white noise. '
            'Initial velocity remains zero, but its standard deviation is either the original 1e-6 m/s '
            'or 1 m/s, matching the existing ROVIO initialization prior. Noise is either the original '
            'MIMOSA setting or derived from the actual simulated IMU. No truth input or performance tuning. '
            'All variants use the same active camera; they never drive it.')
    if args.comparison == 'batched_fusion':
        manifest['variant_meaning'] = ('After initialization, coincident vision and LiDAR updates form one transaction: '
            'vision first, LiDAR second, fresh optimized values after either update, final graph output after both. '
            'Thus map insertion and published pose include both sensors. All sensor timestamps, IMU prefixes, '
            'factor weights and startup order are preserved. Diagnostic estimates do not drive the camera.')
    if args.comparison == 'sensor_white':
        manifest['variant_meaning'] = ('Same corrected coincident-update implementation in every estimator. '
            'Only IMU white-noise densities are replaced by values derived from the unchanged physical SDF. '
            'Initialization, bias dynamics, vision observations and camera control are unchanged. '
            'This repeats the previously failed sensor-noise diagnostic after an independently verified implementation fix.')
    if args.comparison == 'numerical_imu':
        manifest['variant_meaning'] = ('Two-by-two same-active-input diagnostic after the verified batched fusion fix: '
            'legacy/sensor-derived IMU white noise and original/zero incremental solver cutoffs. '
            'Zero cutoffs relinearize all changed variables and fully back-substitute; the same three additional '
            'iterations, objective and sensor measurements are retained. No claim of global convergence. '
            'Diagnostic estimates never drive the camera; the experiment is not gimbal acceptance.')
    if args.comparison == 'full_solver_noise':
        manifest['variant_meaning'] = ('Complete-route conditional fixed/active comparison with zero incremental solver '
            'cutoffs, using either unchanged legacy or physical-sensor-derived IMU white noise. '
            'Original batched fixed/active estimators still drive the existing camera policy. '
            'All additional estimates are diagnostic only; any promising variant requires a later closed-loop trial.')
    if args.comparison == 'temporal_velocity':
        manifest['variant_meaning'] = ('Independent versus AR(1) body velocity errors with identical two-second smoother history. '
            'A disabled one-second duplicate verifies the new binary against the unchanged main estimator. '
            'Persistence comes only from the original independent fixed-camera calibration flights. '
            'Tilt, reported velocity/covariance, sensors, camera control and other factor weights stay unchanged. '
            'This approximation does not remove shared IMU correlation. No diagnostic estimate drives the camera.')
    def save():
        temp = args.output/'manifest.tmp'
        temp.write_text(json.dumps(manifest, indent=2)+'\n')
        temp.replace(args.output/'manifest.json')
    def interrupt(*unused):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    save()
    try:
        waiting_since = time.monotonic()
        while True:
            if time.monotonic()-waiting_since > 1800.:
                raise RuntimeError('Reference flight did not start within 30 minutes')
            if (args.flight/'runtime.json').exists():
                raise RuntimeError('Reference flight already ended')
            stage = read(args.flight/'route_status.json')
            if stage.get('sim_time', 0.) > 15.:
                raise RuntimeError('Too late to observe identical initialization')
            env = owner_environment(args.port)
            if env and (args.flight/'fixed/mimosa.launch').exists():
                break
            time.sleep(.2)
        manifest['status'] = 'running'
        manifest['variants'] = {}
        package = Path(subprocess.check_output(['rospack', 'find', 'mimosa'], env=env, text=True).strip())
        reference_batched = read(args.flight/'protocol.json').get('batched_fusion',False)
        if args.comparison in ('sensor_white','numerical_imu','full_solver_noise','temporal_velocity') and not reference_batched:
            raise ValueError('Sensor white-noise diagnostic requires the verified batched reference')
        variants = [('duplicate_active', 'active', 'velocity'),
                    (args.comparison+'_active', 'active', args.comparison),
                    (args.comparison+'_fixed', 'fixed', args.comparison)]
        if args.comparison == 'initial_velocity':
            variants = [('duplicate_active','active','velocity'),
                        ('loose_initial_legacy','active','loose_initial_legacy'),
                        ('tight_initial_sensor','active','tight_initial_sensor'),
                        ('loose_initial_sensor','active','loose_initial_sensor')]
        if args.comparison == 'numerical_imu':
            variants = [('duplicate_active','active','velocity'),
                        ('default_sensor','active','default_sensor'),
                        ('full_relinearization_legacy','active','full_relinearization_legacy'),
                        ('full_relinearization_sensor','active','full_relinearization_sensor')]
        if args.comparison == 'full_solver_noise':
            variants = [('duplicate_active','active','velocity')]+[
                ('full_relinearization_'+noise+'_'+mode,mode,'full_relinearization_'+noise)
                for noise in ['legacy','sensor'] for mode in ['active','fixed']]
        if args.comparison == 'temporal_velocity':
            variants=[('duplicate_active','active','temporal_disabled')]+[
                (kind+'_'+mode,mode,kind) for kind in ['independent_history','temporal_velocity'] for mode in ['active','fixed']]
        for name, mode, kind in variants:
            directory = args.output/name
            directory.mkdir()
            logdir = directory/'mimosa'
            logdir.mkdir()
            source = args.flight/('fixed' if mode == 'fixed' else '')
            config = yaml.safe_load((source/'mimosa.yaml').read_text())
            config['logs_directory'] = str(logdir)+'/'
            if args.comparison == 'temporal_velocity':
                if kind!='temporal_disabled':config['graph']['manager']['smoother']['lag']=2*temporal['period_s']
                if kind=='temporal_velocity':
                    manager=config['odometry']['manager']
                    if manager['velocity_covariance_scale']!=temporal['velocity_covariance_scale'] or manager['velocity_period_s']!=temporal['period_s']:
                        raise ValueError('Temporal calibration differs from reference observations')
                    manager['temporal_velocity_enabled']=True
                    manager['temporal_velocity_rho']=temporal['rho']
                    manager['temporal_velocity_innovation_variance']=temporal['innovation_variance']
            if kind == 'displacement':
                config['odometry']['manager']['translation_covariance'] = [
                    v for row in calibration['translation_covariance'] for v in row]
                config['odometry']['manager']['relative_pose_covariance'] = [
                    v for row in calibration['covariance'] for v in row]
                if abs(config['odometry']['manager']['tilt_sigma_rad']-calibration['gravity_sigma_rad']) > 1e-12:
                    raise ValueError('Tilt calibration must match the main flight')
            if kind == 'tilt_only':
                config['odometry']['manager']['diagnostic_disable_velocity'] = True
            if kind == 'sensor_white':
                from unipilot_calibration import fusion_white_noise
                noise = fusion_white_noise(config, args.flight/'unipilot_gimbal.sdf', 'sensor')
                (directory/'sensor_noise.json').write_text(json.dumps(noise,indent=2)+'\n')
            if args.comparison in ('numerical_imu','full_solver_noise'):
                if kind.startswith('full_relinearization'):
                    from numerical_relinearization import apply
                    apply(config)
                if kind.endswith('_sensor'):
                    from unipilot_calibration import fusion_white_noise
                    noise = fusion_white_noise(config, args.flight/'unipilot_gimbal.sdf', 'sensor')
                    (directory/'sensor_noise.json').write_text(json.dumps(noise,indent=2)+'\n')
            if args.comparison == 'initial_velocity' and kind != 'velocity':
                if config['graph']['manager']['smoother']['initial_velocity_sigma'] != 1e-6:
                    raise ValueError('Expected original near-zero initial velocity uncertainty')
                if kind.startswith('loose_initial'):
                    config['graph']['manager']['smoother']['initial_velocity_sigma'] = 1.
                if kind.endswith('_sensor'):
                    from unipilot_calibration import fusion_white_noise
                    noise = fusion_white_noise(config, args.flight/'unipilot_gimbal.sdf', 'sensor')
                    (directory/'sensor_noise.json').write_text(json.dumps(noise,indent=2)+'\n')
            if kind in ('information_weighting','drpm_four_dof'):
                scan = config['lidar']['geometric']['scan_to_map']
                if not scan['reg_4_dof'] or scan['project_on_degneneracy'] or scan['drpm_enabled']:
                    raise ValueError('Expected ordinary four-DoF reference without degeneration projection')
                scan['drpm_enabled'] = True
                scan['drpm_registration_mode'] = 0 if kind=='information_weighting' else 1
            config_path = directory/'mimosa.yaml'
            config_path.write_text(yaml.safe_dump(config))
            binary = WS/{'velocity':'build/mimosa-velocity/mimosa_velocity_node',
                         'temporal_disabled':'build/mimosa-temporal-velocity/mimosa_temporal_velocity_node',
                         'independent_history':'build/mimosa-temporal-velocity/mimosa_temporal_velocity_node',
                         'temporal_velocity':'build/mimosa-temporal-velocity/mimosa_temporal_velocity_node',
                         'batched_fusion':'build/mimosa-batched-vision/mimosa_batched_node',
                         'sensor_white':'build/mimosa-batched-vision/mimosa_batched_node',
                         'default_sensor':'build/mimosa-batched-vision/mimosa_batched_node',
                         'full_relinearization_legacy':'build/mimosa-batched-vision/mimosa_batched_node',
                         'full_relinearization_sensor':'build/mimosa-batched-vision/mimosa_batched_node',
                         'information_weighting':'build/mimosa-velocity/mimosa_velocity_node',
                         'drpm_four_dof':'build/mimosa-velocity/mimosa_velocity_node',
                         'loose_initial_legacy':'build/mimosa-velocity/mimosa_velocity_node',
                         'tight_initial_sensor':'build/mimosa-velocity/mimosa_velocity_node',
                         'loose_initial_sensor':'build/mimosa-velocity/mimosa_velocity_node',
                         'displacement':'build/mimosa-gravity-translation/mimosa_gravity_translation_node',
                         'tilt_only':'build/mimosa-tilt-only-diagnostic/mimosa_tilt_only_node'}[kind]
            if kind=='velocity' and reference_batched:
                binary=WS/'build/mimosa-batched-vision/mimosa_batched_node'
            node_name = 'sidecar_'+name
            namespace = '/sidecar/'+name
            subprocess.run(['rosparam', 'load', str(package/'config/os_dummy.json'),
                            '/'+node_name+'/lidar/sensor'], env=env, check=True,
                           stdout=subprocess.DEVNULL)
            subprocess.run(['rosparam', 'load', str(config_path), '/'+node_name],
                           env=env, check=True, stdout=subprocess.DEVNULL)
            remaps = {
                'imu/manager/imu_in': '/slam/input/imu',
                'lidar/manager/lidar_in': '/slam/input/points',
                'odometry/manager/odometry_in': ('/fixed' if mode == 'fixed' else '')+'/rovio/odometry',
                'graph/odometry': namespace+'/slam/odometry',
                'graph/path': namespace+'/slam/path',
                'lidar/geometric/map': namespace+'/slam/map'}
            command = [str(binary), '__name:='+node_name]
            command.extend('/'+node_name+'/'+source+':='+target for source, target in remaps.items())
            for topic in ['/active_lidar_diagnostics', '/active/lidar_information', '/tf', '/tf_static']:
                command.append(topic+':='+namespace+topic)
            (directory/'command.json').write_text(json.dumps(command, indent=2)+'\n')
            node_env = dict(env)
            expected_library = None
            if kind in ('drpm_four_dof', 'batched_fusion','sensor_white') or (kind=='velocity' and reference_batched) or args.comparison in ('numerical_imu','full_solver_noise','temporal_velocity'):
                library_directory = WS/('build/mimosa-drpm-four-dof' if kind=='drpm_four_dof' else 'build/mimosa-batched-vision')
                node_env['LD_LIBRARY_PATH'] = str(library_directory)+':'+env.get('LD_LIBRARY_PATH','')
                expected_library = hashlib.sha256((library_directory/'libmimosa_lib.so').read_bytes()).hexdigest()
            with (directory/'node.log').open('w') as log:
                processes[name] = subprocess.Popen(command, env=node_env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            manifest['variants'][name] = dict(mode=mode, kind=kind, pid=processes[name].pid,
                binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest())
            if expected_library:
                manifest['variants'][name]['library_sha256_override'] = expected_library
            save()
        libraries = {}
        for name, process in processes.items():
            start = time.monotonic()
            while True:
                maps = Path(f'/proc/{process.pid}/maps')
                if process.poll() is not None:raise RuntimeError('Diagnostic exited during library verification')
                paths = {line.split()[-1] for line in maps.read_text().splitlines()
                         if 'libmimosa_lib.so' in line}
                if paths:break
                if time.monotonic()-start>5.:raise RuntimeError('Cannot verify loaded MIMOSA library')
                time.sleep(.02)
            libraries[name] = {path:hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in paths}
        (args.output/'loaded_libraries.json').write_text(json.dumps(libraries,indent=2)+'\n')
        # Keep receiving until the owner has stopped Gazebo and its estimators.
        # A fixed delay after route completion can truncate the final input tail.
        while not (args.flight/'runtime.json').exists():
            available = next(int(line.split()[1])*1024 for line in
                             Path('/proc/meminfo').read_text().splitlines()
                             if line.startswith('MemAvailable:'))
            if available < 2.5e9:
                raise RuntimeError('Stopping diagnostic nodes to retain main-flight memory reserve')
            for name, process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited before reference completion: {process.returncode}')
            stage = read(args.flight/'route_status.json')
            if stage.get('failure'):
                raise RuntimeError('Reference flight failed: '+stage['failure'])
            time.sleep(.25)
        manifest['status'] = 'reference_ended'
    except BaseException as exc:
        manifest['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
        manifest['error'] = str(exc)
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for process in processes.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes.values():
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        manifest['processes'] = {name: p.returncode for name, p in processes.items()}
        manifest['stopped'] = True
        save()


if __name__ == '__main__':
    main()
