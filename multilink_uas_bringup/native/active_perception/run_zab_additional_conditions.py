#!/usr/bin/env python3
"""Frozen-policy evaluation on three previously untested paired flight routes."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import numpy as np
from run_lio_failure_comparisons import NATIVE, INPUTS, read, run_one, verify_selected

ROUTES = ['run3', 'run4', 'run1_tail_branch']
SEEDS = [20261101, 20261102]


def save(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2) + '\n')
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=True)
    path = root / 'campaign.json'
    if path.exists():
        manifest = read(path)
    else:
        catalog = read(INPUTS / 'zab_catalog.json')
        entries = {}
        for label in ROUTES:
            entries[label] = catalog[label].copy()
        manifest = dict(
            status='prepared', seeds=SEEDS, routes=entries, runs={},
            selection='All three routes selected before new results: run3 short connector, '
                      'run4 rising connector, run1_tail_branch descending branch with return. '
                      'Not used in the preceding nine-route paired comparison. '
                      'This is not a claim of spatial disjointness from all historical routes.',
            protocol='Frozen v57, DRPM OFF, UniPilot native sensors, 0.5 m/s cruise. '
                     'Same-flight pure LIO / fixed fusion / active fusion. Two fresh requested '
                     'seeds per route; stochastic inputs need not reproduce exactly across flights. '
                     'Evaluate full tracking period only, initial rigid alignment, no scale fit. '
                     'Retain every result; no parameter tuning or success-based selection.',
            order=[f'{label}_seed{seed}' for seed in SEEDS for label in ROUTES],
            source_checks=verify_selected())
        save(path, manifest)
    if args.prepare_only:
        print(json.dumps(manifest, indent=2))
        return
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    template = read(INPUTS/'templates/paired_flight.json')
    manifest['status'] = 'running'
    save(path, manifest)
    try:
        for key in manifest['order']:
            if key in manifest['runs']:
                if manifest['runs'][key]['status'] == 'complete':
                    continue
                raise RuntimeError('Retained incomplete attempt requires inspection: ' + key)
            label, raw_seed = key.rsplit('_seed', 1)
            seed = int(raw_seed)
            entry = manifest['routes'][label]
            for p, expected in entry['inputs_sha256'].items():
                assert hashlib.sha256(Path(p).read_bytes()).hexdigest() == expected, p
            manifest['runs'][key] = dict(status='running', route=label, seed=seed)
            save(path, manifest)
            print('START', key, flush=True)
            result = run_one(root, key, entry, template, seed)
            result.update(route=label, seed=seed)
            manifest['runs'][key] = result
            save(path, manifest)
            print('FINISHED', key, result['status'], flush=True)
            if not result['valid']:
                raise RuntimeError('Invalid comparison: ' + key)
        manifest['status'] = 'complete'
        manifest['final_source_checks'] = verify_selected()
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
        raise
    except Exception as exc:
        manifest['status'] = 'failed'
        manifest['error'] = str(exc)
        raise
    finally:
        save(path, manifest)


if __name__ == '__main__':
    main()
