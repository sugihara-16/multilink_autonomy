#!/usr/bin/env python3
"""Read-only geometry records and conditional velocity/tilt crossover estimators.

All additional estimators share the actual flight but never control it.
No images or dense LiDAR clouds are recorded. No truth is an estimator input.
"""
import argparse
import copy
import gzip
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import yaml
from compare_fusion_sidecar import owner_environment, read

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]
VARIANTS = ['duplicate_active', 'pure_lio',
            'fixed_velocity_active_tilt', 'active_velocity_fixed_tilt']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--flight', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--port', type=int, required=True)
    p.add_argument('--attached', action='store_true')
    p.add_argument('--mode', choices=['geometry', 'covariance'], default='geometry')
    a = p.parse_args()
    variants = VARIANTS if a.mode == 'geometry' else [
        'duplicate_active', 'active_velocity_fixed_cov',
        'fixed_velocity_active_cov', 'fixed_velocity_active_tilt']
    if not a.attached:
        start = time.monotonic()
        while True:
            env = owner_environment(a.port)
            if env and (a.flight/'fixed/mimosa.launch').exists():
                break
            if time.monotonic()-start > 180:
                raise RuntimeError('Reference startup timed out')
            time.sleep(.05)
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()),
                                  *sys.argv[1:], '--attached'], env)
    import rospy
    import sensor_msgs.point_cloud2 as pc2
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float64MultiArray

    a.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('directional_complementarity_observer', disable_signals=True)
    processes = {}
    subscribers = []
    counts = {}
    errors = []
    cache = {'fixed': {}, 'active': {}}
    histories = {'fixed': {}, 'active': {}}
    lock = threading.RLock()
    records = gzip.open(a.output/'geometry.jsonl.gz', 'xt', compresslevel=2)
    manifest = dict(scope=__doc__, status='starting', variants={}, truth_input=False, mode=a.mode)

    def save():
        (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')

    def write(kind, stamp, data):
        with lock:
            records.write(json.dumps(dict(kind=kind, stamp_ns=int(stamp), data=data),
                                     separators=(',', ':'))+'\n')
            counts[kind] = counts.get(kind, 0)+1

    publishers = {name: rospy.Publisher('/direction_diagnostic/'+name,
                                       Odometry, queue_size=200)
                  for name in variants if name not in ['duplicate_active', 'pure_lio']}

    def odometry(msg, side):
        stamp = msg.header.stamp.to_nsec()
        with lock:
            cache[side][stamp] = msg
            for t in sorted(cache['fixed'].keys() & cache['active'].keys()):
                fixed = cache['fixed'].pop(t)
                active = cache['active'].pop(t)
                combinations = [
                        ('fixed_velocity_active_tilt', fixed, active),
                        ('active_velocity_fixed_tilt', active, fixed)]
                if a.mode == 'covariance':
                    combinations = [
                        ('fixed_velocity_active_tilt', fixed, active),
                        ('active_velocity_fixed_cov', active, active),
                        ('fixed_velocity_active_cov', fixed, active)]
                for name, velocity, tilt in combinations:
                    mixed = copy.deepcopy(velocity)
                    # Position and pose covariance remain with the velocity
                    # source. The actual factor uses only the copied tilt,
                    # body velocity, and its covariance.
                    mixed.pose.pose.orientation = copy.deepcopy(tilt.pose.pose.orientation)
                    if name == 'active_velocity_fixed_cov':
                        mixed.twist.covariance = copy.deepcopy(fixed.twist.covariance)
                    if name == 'fixed_velocity_active_cov':
                        mixed.twist.covariance = copy.deepcopy(active.twist.covariance)
                    publishers[name].publish(mixed)
                    counts[name+'_published'] = counts.get(name+'_published', 0)+1
            if len(cache[side]) > 500:
                errors.append('Unmatched odometry queue exceeded 500')

    def information(msg, side):
        if len(msg.data) != 96:
            errors.append('Unexpected LiDAR information length')
            return
        write('lidar_'+side, round(msg.data[1]*1e9), list(msg.data))

    def geometry(msg, side):
        stamp = round(msg.data[1])
        if stamp % 500000000 == 0:
            write('geometry_'+side, stamp, list(msg.data))

    def extrinsic(msg, side):
        stamp = msg.header.stamp.to_nsec()
        if stamp % 500000000:
            return
        t = msg.pose.position
        q = msg.pose.orientation
        write('extrinsic_'+side, stamp, [t.x, t.y, t.z, q.x, q.y, q.z, q.w])

    def features(msg, side):
        stamp = msg.header.stamp.to_nsec()
        rows = list(pc2.read_points(msg, field_names=['id', 'status'],
                                   skip_nans=True))
        tracked = {int(r[0]) for r in rows if int(r[1]) == 4}
        with lock:
            history = histories[side]
            for ident in tracked:
                born, last = history.get(ident, (stamp, stamp))
                history[ident] = (stamp if stamp-last > 250000000 else born, stamp)
            histories[side] = {k: v for k, v in history.items()
                               if stamp-v[1] <= 250000000}
            if stamp % 500000000 == 0:
                write('tracks_'+side, stamp,
                      dict(valid=len(rows), tracked=len(tracked),
                           ages_s=[(stamp-history[i][0])*1e-9 for i in sorted(tracked)]))

    def interrupt(*unused):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        for side, prefix, private in [('active', '', '/rovio_gimbal'),
                                      ('fixed', '/fixed', '/fixed_rovio')]:
            subscribers += [
                rospy.Subscriber(prefix+'/rovio/odometry', Odometry,
                                 lambda m, s=side: odometry(m, s), queue_size=200,
                                 tcp_nodelay=True),
                rospy.Subscriber(private+'/joint_geometry', Float64MultiArray,
                                 lambda m, s=side: geometry(m, s), queue_size=100),
                rospy.Subscriber(prefix+'/active/extrinsic', PoseStamped,
                                 lambda m, s=side: extrinsic(m, s), queue_size=100),
                rospy.Subscriber(prefix+'/rovio/pcl', PointCloud2,
                                 lambda m, s=side: features(m, s), queue_size=100)]
        subscribers.append(rospy.Subscriber(
            '/active/lidar_information', Float64MultiArray,
            lambda m: information(m, 'active'), queue_size=100))
        subscribers.append(rospy.Subscriber(
            '/direction_diagnostic/pure_lio/lidar_information', Float64MultiArray,
            lambda m: information(m, 'pure'), queue_size=100))

        binary = WS/'build/mimosa-batched-vision/mimosa_batched_node'
        library = binary.parent/'libmimosa_lib.so'
        proto = read(a.flight/'protocol.json')
        sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        assert sha(binary) == proto['binary_sha256']['mimosa_node']
        assert sha(library) == proto['binary_sha256']['libmimosa_lib.so']
        package = Path(subprocess.check_output(['rospack', 'find', 'mimosa'],
                                               text=True).strip())
        for name in variants:
            directory = a.output/name
            logdir = directory/'mimosa'
            logdir.mkdir(parents=True)
            config = yaml.safe_load((a.flight/'mimosa.yaml').read_text())
            config['logs_directory'] = str(logdir)+'/'
            if name == 'pure_lio':
                config['odometry']['manager']['enabled'] = False
            path = directory/'mimosa.yaml'
            path.write_text(yaml.safe_dump(config))
            node = 'direction_'+name
            namespace = '/direction_diagnostic/'+name
            subprocess.run(['rosparam', 'load', str(package/'config/os_dummy.json'),
                            '/'+node+'/lidar/sensor'], check=True)
            subprocess.run(['rosparam', 'load', str(path), '/'+node], check=True)
            rospy.set_param(namespace+'/active_lidar_diagnostics', name == 'pure_lio')
            source = ('/rovio/odometry' if name == 'duplicate_active' else
                      namespace if name != 'pure_lio' else namespace+'/unused')
            remaps = {
                'imu/manager/imu_in': '/slam/input/imu',
                'lidar/manager/lidar_in': '/slam/input/points',
                'odometry/manager/odometry_in': source,
                'graph/odometry': namespace+'/slam/odometry',
                'graph/path': namespace+'/slam/path',
                'lidar/geometric/map': namespace+'/slam/map'}
            command = [str(binary), '__name:='+node]
            command += ['/'+node+'/'+s+':='+t for s, t in remaps.items()]
            command += ['/active_lidar_diagnostics:='+namespace+'/active_lidar_diagnostics',
                        '/active/lidar_information:='+namespace+'/lidar_information',
                        '/tf:='+namespace+'/tf', '/tf_static:='+namespace+'/tf_static']
            env = dict(os.environ)
            env['LD_LIBRARY_PATH'] = str(binary.parent)+':'+env.get('LD_LIBRARY_PATH', '')
            (directory/'command.json').write_text(json.dumps(command, indent=2)+'\n')
            with (directory/'node.log').open('w') as log:
                processes[name] = subprocess.Popen(command, env=env, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            manifest['variants'][name] = dict(pid=processes[name].pid,
                binary_sha256=sha(binary), library_sha256=sha(library),
                odometry_source=source, config_sha256=sha(path))
            save()
        loaded = {}
        for name, process in processes.items():
            deadline = time.monotonic()+10
            while True:
                mappings = Path(f'/proc/{process.pid}/maps').read_text()
                paths = {line.split()[-1] for line in mappings.splitlines()
                         if 'libmimosa_lib.so' in line}
                if paths:
                    loaded[name] = {p: sha(Path(p)) for p in paths}
                    assert set(loaded[name].values()) == {sha(library)}
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('Could not verify loaded MIMOSA library')
                time.sleep(.05)
        (a.output/'loaded_libraries.json').write_text(json.dumps(loaded, indent=2)+'\n')
        manifest['status'] = 'running'
        save()
        while not (a.flight/'runtime.json').exists():
            if errors:
                raise RuntimeError(errors[-1])
            if any(p.poll() is not None for p in processes.values()):
                raise RuntimeError('Diagnostic estimator exited early')
            free = os.statvfs('/mnt/c')
            if free.f_bavail*free.f_frsize < 8e9:
                raise RuntimeError('Windows free space below 8 GB')
            time.sleep(.2)
        manifest['status'] = 'reference_ended'
    except BaseException as exc:
        manifest.update(status='failed', error=str(exc))
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        for sub in subscribers:
            sub.unregister()
        rospy.signal_shutdown('Diagnostic finished')
        for proc in processes.values():
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes.values():
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        with lock:
            records.close()
            manifest.update(counts=counts, errors=errors,
                unmatched_odometry={k: len(v) for k, v in cache.items()},
                stopped=True, processes={k: p.returncode for k, p in processes.items()})
            save()


if __name__ == '__main__':
    main()
