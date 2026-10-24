#!/usr/bin/env python3
"""Compare pure LIO and a fused duplicate on one shared live sensor stream.

Pure LIO uses the selected executable/library and all LiDAR/IMU settings.
Only the external odometry manager is disabled and its input is disconnected.
This observer never publishes control commands or estimator measurements.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import yaml
from compare_fusion_sidecar import owner_environment, read

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flight', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--record-geometry', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    impl = args.output/'implementation'
    impl.mkdir()
    for name in ['observe_pure_lio.py', 'compare_fusion_sidecar.py']:
        shutil.copy2(HERE/name, impl/name)
    processes = {}
    manifest = dict(scope=__doc__, flight=str(args.flight.resolve()), port=args.port,
                    status='waiting', truth_input=False, image_or_cloud_recording=False,
                    variants={}, four_dof_kept=True)

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
        start = time.monotonic()
        while True:
            if time.monotonic()-start > 180:
                raise RuntimeError('Reference did not start in 180 seconds')
            if (args.flight/'runtime.json').exists():
                raise RuntimeError('Reference flight already ended')
            if read(args.flight/'route_status.json').get('sim_time', 0) > 15:
                raise RuntimeError('Too late to observe initialization')
            env = owner_environment(args.port)
            if env and (args.flight/'fixed/mimosa.launch').exists():
                break
            time.sleep(.1)
        protocol = read(args.flight/'protocol.json')
        if not protocol['batched_fusion'] or protocol['drpm_fusion']:
            raise ValueError('Expected selected batched fusion with DRPM disabled')
        binary = WS/'build/mimosa-batched-vision/mimosa_batched_node'
        library = binary.parent/'libmimosa_lib.so'
        sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
        if sha(binary) != protocol['binary_sha256']['mimosa_node']:
            raise ValueError('Executable differs from reference')
        if sha(library) != protocol['binary_sha256']['libmimosa_lib.so']:
            raise ValueError('Library differs from reference')
        package = Path(subprocess.check_output(['rospack', 'find', 'mimosa'], env=env, text=True).strip())
        if args.record_geometry:
            subprocess.run(['rosparam','set','/lio_ablation/pure_lio/active_lidar_diagnostics','true'],env=env,check=True)
        for name in ['duplicate_active', 'pure_lio']:
            directory = args.output/name
            logdir = directory/'mimosa'
            logdir.mkdir(parents=True)
            config = yaml.safe_load((args.flight/'mimosa.yaml').read_text())
            config['logs_directory'] = str(logdir)+'/'
            if name == 'pure_lio':
                config['odometry']['manager']['enabled'] = False
            path = directory/'mimosa.yaml'
            path.write_text(yaml.safe_dump(config))
            node = 'lio_ablation_'+name
            namespace = '/lio_ablation/'+name
            subprocess.run(['rosparam', 'load', str(package/'config/os_dummy.json'),
                            '/'+node+'/lidar/sensor'], env=env, check=True)
            subprocess.run(['rosparam', 'load', str(path), '/'+node], env=env, check=True)
            remaps = {
                'imu/manager/imu_in': '/slam/input/imu',
                'lidar/manager/lidar_in': '/slam/input/points',
                'odometry/manager/odometry_in': '/rovio/odometry' if name=='duplicate_active' else namespace+'/unused_external_odometry',
                'graph/odometry': namespace+'/slam/odometry',
                'graph/path': namespace+'/slam/path',
                'lidar/geometric/map': namespace+'/slam/map',
            }
            command = [str(binary), '__name:='+node]
            command.extend('/'+node+'/'+source+':='+target for source, target in remaps.items())
            command.extend(topic+':='+namespace+topic for topic in
                           ['/active_lidar_diagnostics', '/active/lidar_information', '/tf', '/tf_static'])
            (directory/'command.json').write_text(json.dumps(command, indent=2)+'\n')
            node_env = dict(env)
            node_env['LD_LIBRARY_PATH'] = str(binary.parent)+':'+env.get('LD_LIBRARY_PATH', '')
            with (directory/'node.log').open('w') as log:
                processes[name] = subprocess.Popen(command, env=node_env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            manifest['variants'][name] = dict(pid=processes[name].pid,
                binary_sha256=sha(binary), library_sha256=sha(library),
                config_sha256=sha(path), vio_enabled=name!='pure_lio', remaps=remaps)
            save()
        libraries = {}
        for name, process in processes.items():
            start = time.monotonic()
            while True:
                if process.poll() is not None:
                    raise RuntimeError(name+' exited during initialization')
                paths = {line.split()[-1] for line in Path(f'/proc/{process.pid}/maps').read_text().splitlines()
                         if 'libmimosa_lib.so' in line}
                if paths:
                    break
                if time.monotonic()-start > 5:
                    raise RuntimeError('Cannot verify loaded library')
                time.sleep(.02)
            libraries[name] = {path:sha(Path(path)) for path in paths}
            if set(libraries[name].values()) != {sha(library)}:
                raise RuntimeError('Unexpected loaded library')
        (args.output/'loaded_libraries.json').write_text(json.dumps(libraries, indent=2)+'\n')
        if args.record_geometry:
            with (args.output/'geometry_recorder.log').open('w') as log:
                processes['geometry_recorder']=subprocess.Popen(['/usr/bin/python3',str(HERE/'record_common_lidar.py'),'--output',str(args.output/'common_lidar.jsonl.gz')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            manifest['geometry_diagnostic_only']=True
        manifest['status'] = 'running'
        save()
        while not (args.flight/'runtime.json').exists():
            free = os.statvfs('/mnt/c')
            if free.f_bavail*free.f_frsize < 8e9:
                raise RuntimeError('Windows free space below 8 GB')
            for name, process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited early: {process.returncode}')
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
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        manifest['processes'] = {name:p.returncode for name,p in processes.items()}
        manifest['stopped'] = True
        save()


if __name__ == '__main__':
    main()
