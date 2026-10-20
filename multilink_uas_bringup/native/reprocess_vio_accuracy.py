#!/usr/bin/env python3
"""Bounded source-MCAP replay for the validated, simulation-matched ROVIO model.

Run in the project's ROS2 environment with a private ROS1 master on port 11375.
Ground truth is never delivered to ROVIO. Previous results are read-only.
"""
import concurrent.futures
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

ROOT = Path(__file__).resolve().parent
BASE = ROOT / 'logs/zab-offline'
OLD = BASE / 'results'
OUT = BASE / 'vio_recomputed'
AUDIT = BASE / 'vio_audit'
CAM = [f'/dragon/cam_link{i}/image' for i in (1, 3, 4)]
TOPICS = ['/dragon/imu/raw', *CAM]
REUSE = {'run1_tail_a2_prefix', 'run1_tail_b2', 'run2a_end'}
VARIANT = 'gyro_matched_depth10'
cv2.setNumThreads(1)


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2) + '\n')


def ns(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for b in iter(lambda: stream.read(2**20), b''):
            h.update(b)
    return h.hexdigest()


def matched_config(text):
    for pattern, value, count in [
        (r'(gyb_[012]\s+)3e-4;', '1e-10', 3),
        (r'(gyb_[012]\s+)3.8e-7;', '1e-16', 3),
        (r'(att_[012]\s+)7.6e-7;', '4e-11', 3),
        (r'(initDepth\s+)0.5;', '0.1', 1),
    ]:
        text, n = re.subn(pattern, rf'\g<1>{value};', text)
        assert n == count, (pattern, n)
    return text


def replay(entry, dest, window):
    label = entry['label']
    src = ROOT / 'logs' / entry['run']
    end = window['end_ns'] + 200000000
    original_log = (BASE / label / 'rovio.log').read_text()
    initializations = re.findall(r'-- Filter: Initialized at t = ([0-9.]+)', original_log)
    assert len(initializations) == 1
    initialization_ns = round(float(initializations[0]) * 1e9)
    maps = []
    for i in range(3):
        raw = yaml.safe_load((src / f'raw_cam{i}.yaml').read_text())
        rect = yaml.safe_load((src / f'cam{i}.yaml').read_text())
        maps.append(cv2.initUndistortRectifyMap(
            np.array(raw['camera_matrix']['data']).reshape(3, 3),
            np.array(raw['distortion_coefficients']['data']), np.eye(3),
            np.array(rect['camera_matrix']['data']).reshape(3, 3),
            (480, 360), cv2.CV_32FC1))
    receipts = {}
    names = yaml.safe_load((src / 'cloud_metadata.yaml').read_text())['rosbag2_bagfile_information']['relative_file_paths']
    for n in names:
        receipts[n] = json.loads((src / 'cloud_chunks' / (Path(n).stem + '.json')).read_text())
    names = [n for n in names if receipts[n]['file_metadata']['starting_time']['nanoseconds_since_epoch'] <= end]
    cache = OUT / 'cache'
    cache.mkdir(exist_ok=True)

    def fetch(n):
        # Windows is the physical backing store, unlike WSL's virtual free space.
        if shutil.disk_usage('/mnt/c').free < 4 * 2**30:
            raise RuntimeError('Windows backing store has less than 4 GiB free')
        path = cache / (label + '_' + n)
        subprocess.run(['rclone', 'copyto', entry['remote'] + '/' + n,
                        str(path), '--retries', '4'], check=True)
        h = hashlib.md5()
        with path.open('rb') as stream:
            for b in iter(lambda: stream.read(2**20), b''):
                h.update(b)
        assert h.hexdigest() == receipts[n]['md5'], n
        return path

    command = ['env', '-i', 'HOME=/root', 'USER=root',
               'PATH=/usr/local/bin:/usr/bin:/bin', 'LANG=C.UTF-8',
               'bash', '-c',
               'source /root/uas_native/env_ros1.sh\n'
               'source /root/multilink_autonomy_ws/slam_ros1_ws/devel/setup.bash\n'
               'export ROS_MASTER_URI=http://127.0.0.1:11375\n'
               'exec "$@"', 'replay', str(AUDIT / 'audit25'),
               str(dest / 'rovio.info'), str(src), str(dest / 'result'),
               f'__name:=recompute_{label}']
    counts = [0] * 4
    camera_stamps = [[], [], []]
    first_imu = None
    started = time.monotonic()
    with (dest / 'replay.log').open('w') as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
                pending = {i: pool.submit(fetch, n) for i, n in enumerate(names[:4])}
                for i, n in enumerate(names):
                    path = pending.pop(i).result()
                    if i + 4 < len(names):
                        pending[i + 4] = pool.submit(fetch, names[i + 4])
                    reader = rosbag2_py.SequentialReader()
                    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                                rosbag2_py.ConverterOptions('', ''))
                    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
                    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
                    while reader.has_next():
                        topic, raw, stored = reader.read_next()
                        m = deserialize_message(raw, types[topic])
                        stamp = ns(m)
                        if stamp > end:
                            continue
                        kind = TOPICS.index(topic)
                        if kind == 0:
                            if first_imu is None:
                                first_imu = stamp
                            if stamp < initialization_ns:
                                continue
                            a, w = m.linear_acceleration, m.angular_velocity
                            payload = struct.pack('<6d', a.x, a.y, a.z, w.x, w.y, w.z)
                        else:
                            assert m.encoding in ('rgb8', 'bgr8'), m.encoding
                            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                            gray = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
                            payload = cv2.remap(gray, *maps[kind - 1], interpolation=cv2.INTER_LINEAR).tobytes()
                            camera_stamps[kind - 1].append(stamp)
                        process.stdin.write(struct.pack('<IQ', kind, stamp))
                        process.stdin.write(payload)
                        counts[kind] += 1
                    del reader
                    path.unlink()
                    progress = dict(label=label, completed_chunks=i + 1, total_chunks=len(names),
                                    wall_s=time.monotonic() - started, counts=counts)
                    dump(OUT / 'progress.json', progress)
                    print(json.dumps(progress), flush=True)
            process.stdin.close()
            code = process.wait(timeout=60)
            # Incomplete final camera sets can exist only beyond the retained window.
            sets = [set(a) for a in camera_stamps]
            common = set.intersection(*sets)
            incomplete = sorted(set.union(*sets) - common)
            assert all(t > window['end_ns'] for t in incomplete), incomplete
            assert code == (7 if incomplete else 0), code
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
    log = (dest / 'replay.log').read_text()
    report = re.search(r'AUDIT_INPUT (\d+) (\d+) (\d+) (\d+) sets (\d+) incomplete (\d+)', log)
    assert report and list(map(int, report.groups()[:4])) == counts
    assert int(report[5]) == len(common) and int(report[6]) == len(incomplete)
    expected = sorted(t for t in common if window['start_ns'] <= t <= window['end_ns'])
    dump(dest / 'run.json', dict(label=label, variant=VARIANT, counts=counts,
         wall_s=time.monotonic() - started, source=entry,
         first_source_imu_ns=first_imu, initialization_ns=initialization_ns,
         source_chunks=names, source_md5={n: receipts[n]['md5'] for n in names},
         tracking_camera_stamps=expected, incomplete_tail_stamps=incomplete,
         return_code=code, method='Direct callbacks in source storage order; all 3 images grouped by exact stamp; unchanged IMU and image rectification; no truth supplied'))


def retain(entry, dest, old_run):
    label = entry['label']
    window = old_run['window']
    with (dest / 'result_vio.csv').open() as stream:
        rows = [r for r in csv.DictReader(stream) if window['start_ns'] <= int(r['stamp_ns']) <= window['end_ns']]
    old_rows = list(csv.DictReader((OLD / label / 'vio.csv').open()))
    old_stamps = [int(r['stamp_ns']) for r in old_rows]
    stamps = [int(r['stamp_ns']) for r in rows]
    assert stamps == old_stamps, (label, len(stamps), len(old_stamps), sorted(set(stamps) ^ set(old_stamps))[:20])
    assert all(a < b for a, b in zip(stamps, stamps[1:]))
    values = np.array([[float(r[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz')] for r in rows])
    assert np.isfinite(values).all()
    assert np.max(np.abs(np.linalg.norm(values[:, 3:7], axis=1) - 1)) < 1e-8
    fields = list(old_rows[0])
    with (dest / 'vio.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            stamp = int(r['stamp_ns'])
            r.update(sec=stamp // 1000000000, nanosec=stamp % 1000000000,
                     frame_id=old_rows[0]['frame_id'], child_frame_id=old_rows[0]['child_frame_id'])
            writer.writerow(r)
    hashes = {}
    for name in ('truth.csv', 'lio.csv', 'window.json'):
        shutil.copy2(OLD / label / name, dest / name)
        assert digest(OLD / label / name) == digest(dest / name)
        hashes[name] = digest(dest / name)
    hashes['vio.csv'] = digest(dest / 'vio.csv')
    metadata = dict(label=label, source_run=entry['run'], window=window,
                    counts=dict(old_run['counts'], vio=len(rows)), hashes=hashes,
                    variant=VARIANT, reused_prior_verified_experiment=label in REUSE,
                    exact_original_output_stamps=True)
    dump(dest / 'validation.json', metadata)
    return metadata


def main():
    OUT.mkdir(exist_ok=True)
    entries = {x['label']: x for x in json.loads((ROOT / 'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags']}
    old_manifest = json.loads((OLD / 'manifest.json').read_text())
    runs = []
    for old_run in old_manifest['runs']:
        label = old_run['label']
        dest = OUT / label
        if (dest / 'validation.json').exists():
            runs.append(json.loads((dest / 'validation.json').read_text()))
            continue
        dest.mkdir(exist_ok=True)
        entry = entries[label]
        config = matched_config((ROOT / 'logs' / entry['run'] / 'rovio.info').read_text())
        (dest / 'rovio.info').write_text(config)
        if label in REUSE:
            source = AUDIT / 'experiments' / label / VARIANT
            assert (source / 'rovio.info').read_text() == config
            for name in ('result_vio.csv', 'run.json', 'replay.log'):
                shutil.copy2(source / name, dest / name)
        else:
            replay(entry, dest, old_run['window'])
        runs.append(retain(entry, dest, old_run))
        dump(OUT / 'manifest.json', dict(status='running', variant=VARIANT, runs=runs))
        print('FLIGHT COMPLETE', label, flush=True)
    dump(OUT / 'manifest.json', dict(status='complete', variant=VARIANT, runs=runs,
         original_results=str(OLD), format='CSV pose and twist; original MCAP bags remain unchanged at original_results',
         settings='25 features, three cameras; gyb initial variance 1e-10, gyb process variance 1e-16, attitude prediction variance 4e-11, initial inverse depth 0.1. Fixed across all flights.'))
    print('ALL 14 COMPLETE', flush=True)


if __name__ == '__main__':
    main()
