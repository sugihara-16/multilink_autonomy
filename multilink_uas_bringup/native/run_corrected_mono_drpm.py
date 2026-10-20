#!/usr/bin/env python3
"""Matched DRPM fusion with the validated single-camera initialization fix."""
import argparse
import concurrent.futures
import json
import os
import shutil
import signal
import subprocess
import time
import xmlrpc.client
from pathlib import Path

import numpy as np
import run_drpm_failure_comparison as base

ROOT = base.ROOT
REF = base.BASE / 'drpm_fast_failures'
VIO = base.BASE / 'monocular_startup/bias'
OUT = base.BASE / 'drpm_corrected_monocular'
base.OUT = OUT


def prepare(label):
    import prepare_livo_comparison as p
    p.OUT = OUT
    p.TOPICS = ['/dragon/lidar/points', '/dragon/imu/raw',
                '/dragon/ground_truth', '/dragon/joint_states']
    p.np.savez_compressed = p.np.savez
    p.main(label)
    dest = OUT / 'inputs' / label
    manifest = json.loads((dest / 'manifest.json').read_text())
    manifest['image_conversion'] = 'No images processed; reuse corrected single-link3 ROVIO CSV'
    base.save(dest / 'manifest.json', manifest)
    old = json.loads((REF / 'inputs' / label / 'input_audit.json').read_text())
    checks = {}
    for key, period in [('imu_t', 1000000), ('cloud_t', 100000000)]:
        arrays = []
        for chunk in manifest['chunks']:
            with np.load(dest / chunk) as a:
                arrays.append(a[key])
        t = np.concatenate(arrays)
        t = t[t >= 2003000000]
        assert np.all(np.diff(t) == period)
        checks[key] = dict(count=len(t), first_ns=int(t[0]), last_ns=int(t[-1]), period_ns=period)
        assert checks[key] == old['streams'][key]
    base.save(dest / 'input_audit.json', dict(streams=checks,source_md5_verified=True))
    out = OUT / 'rovio' / label
    out.mkdir(parents=True, exist_ok=True)
    for name in ['result_vio.csv', 'rovio.info', 'cam0.yaml', 'summary.json']:
        shutil.copyfile(VIO / label / name, out / name)
    assert base.sha(out / 'result_vio.csv') == base.sha(VIO / label / 'result_vio.csv')


def stage(phase, label, port):
    os.environ['ROS_MASTER_URI'] = f'http://127.0.0.1:{port}'
    marker = OUT / 'stage_checks' / f'{label}_{phase}.json'
    if marker.exists():
        return
    if phase == 'prepare':
        prepare(label)
    elif phase == 'bag':
        base.stage_impl('bag', label)
        dest = OUT / 'inputs' / label
        assert (dest / 'truth.csv').read_bytes() == (REF / 'inputs' / label / 'truth.csv').read_bytes()
        current = json.loads((dest / 'bag_summary.json').read_text())
        previous = json.loads((REF / 'inputs' / label / 'bag_summary.json').read_text())
        assert current['counts'] == previous['counts'] and current['vio_count'] == previous['vio_count']
    elif phase == 'fusion':
        old = json.loads((REF / 'fusion' / label / 'summary.json').read_text())
        for path, digest in old['artifacts'].items():
            assert base.sha(path) == digest
        base.stage_impl('fusion', label)
        import yaml
        new_cfg = yaml.safe_load((OUT / 'fusion' / label / 'config.yaml').read_text())
        old_cfg = yaml.safe_load((REF / 'fusion' / label / 'config.yaml').read_text())
        new_cfg['logs_directory'] = old_cfg['logs_directory']
        assert new_cfg == old_cfg
    elif phase == 'evaluate':
        import evaluate_drpm_failure_comparison as e
        e.OUT = OUT
        m = e.evaluate(label)
        assert all(c['missing_20hz_stamps'] == 0 for c in m['checks']['native_cadence'].values())
        m['checks']['fusion_config_identical_to_previous_monocular'] = True
        m['checks']['corrected_vio_csv_sha256'] = base.sha(VIO / label / 'result_vio.csv')
        base.save(OUT / 'fusion' / label / 'metrics.json', m)
    else:
        raise ValueError(phase)
    base.save(marker, dict(complete=True))


def flight(label, port, script=None):
    for phase in ['prepare', 'bag', 'fusion', 'evaluate']:
        base.save(OUT / f'{label}_runtime.json', dict(status='running', phase=phase))
        print('START', label, phase, flush=True)
        args = [str(Path(script or __file__).resolve()), phase, label, '--port', str(port)]
        if phase == 'prepare':
            command = ['bash', '--noprofile', '--norc', '-c',
                       'source /opt/ros/humble/setup.bash\nexec python3 "$@"', 'prepare', *args]
        else:
            command = ['bash', str(ROOT / 'comparison_ros1.sh'), 'python3', *args]
        with (OUT / f'{label}_{phase}.log').open('a') as log:
            subprocess.run(command, env=base.ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
        print('DONE', label, phase, flush=True)
    for path in (OUT / 'inputs' / label).glob('*.npz'):
        path.unlink()
    (OUT / 'inputs' / f'{label}.bag').unlink(missing_ok=True)
    base.save(OUT / f'{label}_runtime.json', dict(status='complete', temporary_sensor_inputs_removed=True))


def campaign():
    OUT.mkdir(parents=True, exist_ok=True)
    base.fixed_calibration()
    assert (OUT / 'calibration.json').read_bytes() == (REF / 'calibration.json').read_bytes()
    base.save(OUT / 'protocol.json', dict(labels=base.LABELS, camera='link3',
        change='Only ROVIO initial accelerometer bias covariance; reuse validated corrected ROVIO CSVs',
        fusion_config='Identical to previous monocular DRPM',
        covariance='Frozen previous calibration; no new target-ground-truth tuning',
        input_stamps='Original integer header timestamps', source=str(VIO),
        evaluation='Full tracking window; initialize using preceding recorded sensors',
        parallelism='Two independent ROS masters; prefix and the two shorter flights'))
    masters = []
    try:
        for port in [11376, 11377]:
            import socket
            with socket.socket() as sock:
                assert sock.connect_ex(('127.0.0.1', port)) != 0, 'Port is already in use'
            log = (OUT / f'master_{port}.log').open('w')
            proc = subprocess.Popen(['bash', str(ROOT / 'comparison_ros1.sh'), 'roscore', '-p', str(port)],
                                    env=base.ENV, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
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
                flight(label, 11377)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(flight, 'run1_prefix', 11376), pool.submit(shorter)]
            for future in futures:
                future.result()
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
