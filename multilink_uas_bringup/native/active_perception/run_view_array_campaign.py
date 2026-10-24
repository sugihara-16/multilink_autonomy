#!/usr/bin/env python3
"""Five independent fixed monocular views plus selected gimbal on shared flights."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
from run_lio_failure_comparisons import HERE, NATIVE, INPUTS, read, run_one, verify_selected
from run_zab_additional_conditions import ROUTES, SEEDS
from run_zab_additional_conditions import save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--only', help='Run just this predeclared trial for integration validation')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    path = root/'campaign.json'
    if path.exists():
        manifest = read(path)
    else:
        catalog = read(INPUTS/'zab_catalog.json')
        manifest = dict(status='prepared', routes={k: catalog[k] for k in ROUTES},
                        order=[f'{label}_seed{seed}' for seed in SEEDS for label in ROUTES],
                        seeds=SEEDS, runs={}, source_checks=verify_selected(),
                        fixed_angles_deg=[[-50, 0], [0, 0], [50, 0], [0, 90], [0, -90]],
                        protocol='All five fixed directions use independent monocular ROVIO and MIMOSA; '
                        'selected active camera runs concurrently. Same physical flight/LiDAR/IMU. '
                        'Camera pose is the only per-fixed-view estimator setting difference. '
                        'Own-view initialization; no cross-camera fusion. Frozen v57; DRPM OFF. '
                        'Three previously specified short routes, two requested seeds each. '
                        'No tuning or outcome-based selection. This new multiview flight is not '
                        'assumed to reproduce sensor noise from the earlier two-camera flight.')
        manifest['extension_sha256'] = {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [HERE/'fixed_view_array.py', HERE/'run_view_array.py',
                      HERE/'audit_view_array_inputs.py', NATIVE/'unipilot_view_array.launch.py']}
        save(path, manifest)
    if args.prepare_only:
        save(path, manifest)
        return
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    template = read(INPUTS/'templates/paired_flight.json')
    template[1] = str(HERE/'run_view_array.py')
    manifest['status'] = 'running'
    save(path, manifest)
    try:
        for key in manifest['order']:
            if args.only and key != args.only:
                continue
            if key in manifest['runs']:
                if manifest['runs'][key]['status'] == 'complete':
                    continue
                raise RuntimeError('Retained incomplete trial requires inspection: '+key)
            for p, sha in manifest['extension_sha256'].items():
                assert hashlib.sha256(Path(p).read_bytes()).hexdigest() == sha, p
            route, seed = key.rsplit('_seed', 1)
            entry = manifest['routes'][route]
            for p, sha in entry['inputs_sha256'].items():
                assert hashlib.sha256(Path(p).read_bytes()).hexdigest() == sha, p
            manifest['runs'][key] = dict(status='running', route=route, seed=int(seed))
            save(path, manifest)
            print('START', key, flush=True)
            result = run_one(root, key, entry, template, int(seed))
            if not result['valid']:
                raise RuntimeError('Reference paired audit failed: '+key)
            subprocess.run([sys.executable, str(HERE/'report_view_array.py'),
                            '--trial', str(root/key)], check=True)
            array = read(root/key/'array_comparison.json')
            result.update(route=route, seed=int(seed), array_valid=array['valid'])
            manifest['runs'][key] = result
            save(path, manifest)
            if not array['valid']:
                raise RuntimeError('Array comparison audit failed: '+key)
            print('FINISHED', key, flush=True)
        manifest['status'] = 'complete' if len(manifest['runs']) == len(manifest['order']) else 'prepared'
        manifest['final_source_checks'] = verify_selected()
    except BaseException as exc:
        manifest['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
        manifest['error'] = str(exc)
        raise
    finally:
        save(path, manifest)


if __name__ == '__main__':
    main()
