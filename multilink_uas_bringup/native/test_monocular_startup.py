#!/usr/bin/env python3
"""Replay local recordings to isolate monocular initialization, without GT input.

Run under the ROS2 environment; only the estimator subprocess uses ROS1.
Original header timestamps, storage order, calibration and image conversion
match replay_monocular_comparison.py. No image/point-cloud cache is created.
"""
import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rovio_initialization import match_accelerometer_bias_prior

ROOT = Path(__file__).resolve().parent
BASE = ROOT / 'logs/zab-offline'
OUT = BASE / 'monocular_startup'
OLD = BASE / 'drpm_fast_failures'
ENV = dict(HOME='/root', USER='root', PATH='/usr/local/bin:/usr/bin:/bin',
           LANG='C.UTF-8', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
cv2.setNumThreads(1)


def configuration(text, variant, urdf_path):
    if variant == 'baseline':
        return text
    if variant in ('zupt', 'bias_zupt'):
        text, count = re.subn(r'(isEnabled\s+)0;', r'\g<1>1;', text)
        assert count == 2
    if variant in ('bias', 'bias_zupt'):
        text, _ = match_accelerometer_bias_prior(text, urdf_path)
    if variant in ('zupt', 'bias', 'bias_zupt'):
        return text
    raise ValueError(variant)


def replay(label, variants, end_s=None):
    manifest = json.loads((OLD / 'inputs' / label / 'manifest.json').read_text())
    source = ROOT / 'logs' / manifest['source']['run']
    end = round(end_s * 1e9) if end_s else manifest['window']['end_ns'] + 200000000
    raw = yaml.safe_load((source / 'raw_cam1.yaml').read_text())
    rect = yaml.safe_load((source / 'cam1.yaml').read_text())
    maps = cv2.initUndistortRectifyMap(
        np.array(raw['camera_matrix']['data']).reshape(3, 3),
        np.array(raw['distortion_coefficients']['data']), np.eye(3),
        np.array(rect['camera_matrix']['data']).reshape(3, 3),
        (480, 360), cv2.CV_32FC1)
    binary = BASE / 'livo_comparison/audit25'
    runs = []
    start = time.monotonic()
    counts = [0, 0]
    digest = hashlib.sha256()
    try:
        for variant in variants:
            dest = OUT / variant / label
            dest.mkdir(parents=True, exist_ok=True)
            assert not (dest / 'summary.json').exists(), 'Do not overwrite completed results'
            cfg = configuration((OLD / 'rovio' / label / 'rovio.info').read_text(), variant,
                                source / 'dragon_slam.urdf')
            (dest / 'rovio.info').write_text(cfg)
            shutil.copyfile(source / 'cam1.yaml', dest / 'cam0.yaml')
            log = (dest / 'replay.log').open('w')
            command = ['bash', str(ROOT / 'comparison_ros1.sh'), str(binary),
                       str(dest / 'rovio.info'), str(dest), str(dest / 'result'),
                       '__name:=startup_' + variant]
            proc = subprocess.Popen(command, env=ENV, stdin=subprocess.PIPE,
                                    stdout=log, stderr=subprocess.STDOUT)
            runs.append((variant, dest, proc, log))
        names = yaml.safe_load((source / 'cloud_metadata.yaml').read_text())['rosbag2_bagfile_information']['relative_file_paths']
        for name in names:
            metadata = json.loads((source / 'cloud_chunks' / (Path(name).stem + '.json')).read_text())
            if metadata['file_metadata']['starting_time']['nanoseconds_since_epoch'] > end:
                break
            path = ROOT / 'logs/zab-local-bags' / label / 'rosbag' / name
            assert path.is_file()
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            topics = ['/dragon/imu/raw', '/dragon/cam_link3/image']
            reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
            types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
            while reader.has_next():
                topic, data, stored = reader.read_next()
                m = deserialize_message(data, types[topic])
                stamp = m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec
                if stamp < 2003000000 or stamp > end:
                    continue
                if topic.endswith('imu/raw'):
                    v, w = m.linear_acceleration, m.angular_velocity
                    kind = 0
                    payload = struct.pack('<6d', v.x, v.y, v.z, w.x, w.y, w.z)
                else:
                    kind = 1
                    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
                    gray = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
                    payload = cv2.remap(gray, *maps, interpolation=cv2.INTER_LINEAR).tobytes()
                packet = struct.pack('<IQ', kind, stamp) + payload
                digest.update(packet)
                counts[kind] += 1
                for _, _, proc, _ in runs:
                    proc.stdin.write(packet)
            del reader
            print(label, name, counts, round(time.monotonic() - start, 1), flush=True)
        for variant, dest, proc, log in runs:
            proc.stdin.close()
            assert proc.wait(timeout=120) == 0
            log.close()
            summary = dict(label=label, variant=variant, counts=counts, input_sha256=digest.hexdigest(),
                           binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                           wall_s=time.monotonic() - start, window=manifest['window'], end_ns=end,
                           truth_used=False, camera='link3', initialization_stamp_ns=2003000000)
            (dest / 'summary.json').write_text(json.dumps(summary, indent=2))
    finally:
        for _, _, proc, log in runs:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=10)
            log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('label')
    parser.add_argument('--variants', nargs='+', default=['zupt'])
    parser.add_argument('--end-s', type=float)
    args = parser.parse_args()
    replay(args.label, args.variants, args.end_s)
