#!/usr/bin/env python3
"""Enable native translational degeneracy handling with DRPM disabled."""
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
CONTROL = base.BASE / 'corrected_monocular_no_drpm'
OUT = base.BASE / 'corrected_monocular_native_degen'
runner.OUT = off.OUT = base.OUT = OUT
runner.REF = off.REF = REF



def fusion(label):
    import rospy, yaml
    rospy.init_node('native_translation_degeneracy_driver', anonymous=True, disable_signals=True)
    dest = OUT / 'fusion' / label
    dest.mkdir(parents=True, exist_ok=True)
    old = json.loads((CONTROL / 'fusion' / label / 'summary.json').read_text())
    for path, digest in old['artifacts'].items():
        assert base.sha(path) == digest
    cfg = yaml.safe_load((CONTROL / 'fusion' / label / 'config.yaml').read_text())
    scan = cfg['lidar']['geometric']['scan_to_map']
    assert not scan['drpm_enabled'] and not scan['project_on_degneneracy']
    assert scan['degen_thresh_trans'] == 40 and scan['degen_thresh_rot'] == 0
    scan['project_on_degneneracy'] = True
    cfg['logs_directory'] = str(dest) + '/'
    (dest / 'config.yaml').write_text(yaml.safe_dump(cfg))
    rospy.set_param('/mimosa', cfg)
    rospy.set_param('/mimosa/lidar/sensor', json.loads((base.WS / 'src/mimosa/config/os_dummy.json').read_text()))
    rospy.set_param('/mimosa/bag_name', str(OUT / 'inputs' / f'{label}.bag'))
    command = json.loads((REF / 'fusion' / label / 'summary.json').read_text())['command']
    start = time.monotonic()
    with (dest / 'run.log').open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=3600)
    base.save(dest / 'summary.json', dict(returncode=result.returncode, wall_s=time.monotonic()-start,
        window=old['window'], artifacts=old['artifacts'], input_sha256=old['input_sha256'],
        camera='link3', drpm_enabled=False, project_on_degneneracy=True, command=command))
    assert result.returncode == 0


def evaluate(label):
    import csv, yaml, numpy as np
    from plot_fast_camera_comparison import read, errors
    from evaluate_drpm_failure_comparison import export
    dest = OUT / 'fusion' / label
    control = CONTROL / 'fusion' / label
    previous = json.loads((control / 'metrics.json').read_text())
    w = previous['window']; start = previous['alignment_stamp_ns']; end = previous['full_grid_end_ns']
    grid = np.arange(start, end+1, 100000000, dtype=np.int64)
    estimate = read(dest / 'odometry.csv')
    truth = read(OUT / 'inputs' / label / 'truth.csv')
    expected = np.arange((w['start_ns']+49999999)//50000000*50000000, w['end_ns']+1, 50000000, dtype=np.int64)
    stamps = estimate[0][(estimate[0]>=w['start_ns']) & (estimate[0]<=w['end_ns'])]
    assert np.array_equal(stamps, expected)
    e, metrics = errors(estimate, truth, start, grid)
    export(dest / 'native_full_errors.csv', grid, e)
    cfg = yaml.safe_load((dest / 'config.yaml').read_text())
    old = yaml.safe_load((control / 'config.yaml').read_text())
    cfg['logs_directory'] = old['logs_directory']
    cfg['lidar']['geometric']['scan_to_map']['project_on_degneneracy'] = False
    assert cfg == old
    log = (dest / 'run.log').read_text()
    assert 'Using calibrated full relative-pose covariance' in log
    assert not (dest / 'drpm_registration.csv').exists()
    count = log.count('*********************DEGEN Handling*********************')
    with (dest / 'odometry.csv').open() as src, (dest / 'odometry_tracking.csv').open('w') as output:
        reader = csv.DictReader(src); writer = csv.DictWriter(output, fieldnames=reader.fieldnames); writer.writeheader()
        rows = {int(r['stamp_ns']):r for r in reader if w['start_ns']<=int(r['stamp_ns'])<=w['end_ns']}
        writer.writerows(rows[t] for t in sorted(rows))
    for name in ['truth_tracking.csv', 'rovio_tracking.csv']:
        shutil.copyfile(control/name, dest/name)
    # Include an exact-output check: enabling the flag need not activate handling.
    old_estimate = read(control / 'odometry.csv')
    identical = (np.array_equal(estimate[0], old_estimate[0]) and
                 np.array_equal(estimate[1], old_estimate[1]) and
                 np.array_equal(estimate[2].as_quat(), old_estimate[2].as_quat()))
    metrics_out = dict(label=label, window=w, alignment_stamp_ns=start, full_grid_end_ns=end,
        native=metrics, untreated=previous['off'], drpm=previous['on'], rovio=previous['rovio'],
        native_handling_linearization_count=count, output_identical_to_untreated=bool(identical),
        checks=dict(same_bag_bytes=True, same_rovio=True, same_binary=True,
                    only_native_projection_switch_and_log_path_changed=True, all_20hz_stamps_present=True,
                    drpm_disabled=True, rotation_threshold_zero=True, translation_threshold_40=True,
                    vio_uncertainty_rejections=log.count('is higher than threshold')))
    base.save(dest / 'metrics.json', metrics_out)
    print('RESULT', label, json.dumps(metrics_out), flush=True)


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
        check = json.loads((OUT / 'inputs' / label / 'bag_match.json').read_text())
        control = json.loads((CONTROL / 'fusion' / label / 'summary.json').read_text())
        assert check['sha256'] == control['input_sha256']
    elif phase == 'fusion':
        fusion(label)
    elif phase == 'evaluate':
        evaluate(label)
    else:
        raise ValueError(phase)
    base.save(marker, dict(complete=True))


def campaign():
    OUT.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(REF / 'calibration.json', OUT / 'calibration.json')
    base.save(OUT / 'protocol.json', dict(labels=base.LABELS, camera='link3',
        reference=str(CONTROL), only_setting_change='project_on_degneneracy: false -> true',
        native_mimosa_projection=True, drpm_enabled=False, translation_threshold=40, rotation_threshold=0, target_truth_tuning=False,
        rovio='Reuse exact corrected single-camera output',
        input_verification='Require complete ROS1 bag SHA256 to match DRPM ON',
        note='Same conventional MIMOSA factors, native translation projection enabled; no threshold tuning'))
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
