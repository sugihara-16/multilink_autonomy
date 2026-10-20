#!/usr/bin/env python3
"""Disable DRPM with byte-identical inputs to the corrected monocular run."""
import argparse
import concurrent.futures
import json
import os
import shutil
import signal
import socket
import subprocess
import time
import xmlrpc.client

import run_corrected_mono_drpm as runner
import run_three_camera_no_drpm as off

base = runner.base
ROOT = base.ROOT
REF = base.BASE / 'drpm_corrected_monocular'
OUT = base.BASE / 'corrected_monocular_no_drpm'
runner.OUT = off.OUT = base.OUT = OUT
runner.REF = off.REF = REF


def stage(phase, label, port):
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    marker = OUT / 'stage_checks' / f'{label}_{phase}.json'
    if marker.exists():
        return
    if phase == 'prepare':
        runner.prepare(label)
    elif phase == 'bag':
        # Refuse to estimate unless serialized IMU, LiDAR and VIO input bag
        # is exactly identical to the previous DRPM-enabled experiment.
        off.bag(label)
    elif phase == 'fusion':
        off.fusion(label)
        path = OUT / 'fusion' / label / 'summary.json'
        summary = json.loads(path.read_text())
        summary.pop('cameras')  # Shared runner's old three-camera description.
        summary['camera'] = 'link3'
        summary['rovio_initialization'] = 'sensor_bias_second_moment'
        base.save(path, summary)
    elif phase == 'evaluate':
        off.evaluate(label)
    else:
        raise ValueError(phase)
    base.save(marker, dict(complete=True))


def campaign():
    OUT.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(REF / 'calibration.json', OUT / 'calibration.json')
    base.save(OUT / 'protocol.json', dict(labels=base.LABELS, camera='link3',
        reference=str(REF), only_setting_change='drpm_enabled: true -> false',
        native_mimosa_projection=False, target_truth_tuning=False,
        rovio='Reuse exact corrected single-camera output',
        input_verification='Require complete ROS1 bag SHA256 to match DRPM ON',
        note='Conventional MIMOSA point-to-plane factors vs adopted DRPM independent ICP pose factor; not a probability-only ablation'))
    masters = []
    try:
        for port in [11376, 11377]:
            with socket.socket() as sock:
                assert sock.connect_ex(('127.0.0.1', port)) != 0, 'Port is already in use'
            log = (OUT / f'master_{port}.log').open('w')
            proc = subprocess.Popen(['bash', str(ROOT / 'comparison_ros1.sh'), 'roscore', '-p', str(port)],
                                    env=base.ENV, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            masters.append((proc, log))
            for _ in range(100):
                assert proc.poll() is None
                try:
                    assert xmlrpc.client.ServerProxy(f'http://127.0.0.1:{port}').getPid('/startup')[0] == 1
                    break
                except (ConnectionError, AssertionError):
                    time.sleep(.1)
            else:
                raise RuntimeError('ROS master did not become ready')
        def shorter():
            for label in ['run1_tail_b1', 'run1_tail_a1']:
                runner.flight(label, 11377, script=__file__)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            tasks = [pool.submit(runner.flight, 'run1_prefix', 11376, __file__), pool.submit(shorter)]
            for task in tasks:
                task.result()
        base.save(OUT / 'runtime.json', dict(status='measurements_complete'))
    finally:
        for proc, log in masters:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=30)
            log.close()
        base.save(OUT / 'cleanup.json', dict(owned_masters_stopped=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('phase')
    parser.add_argument('label', nargs='?')
    parser.add_argument('--port', type=int, default=11376)
    args = parser.parse_args()
    if args.phase == 'campaign':
        campaign()
    else:
        stage(args.phase, args.label, args.port)
