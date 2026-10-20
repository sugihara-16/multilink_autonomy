#!/usr/bin/env python3
"""Audit the same-input duplicate before evaluating diagnostic fusion variants."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import numpy as np
import yaml
from report_routes import csv_path, load
from report_estimator_validation import evaluate


def coincident_input_signature(rows):
    """Equal sensor multiset and IMU prefix at every timestamp, allowing tie order only."""
    grouped = {}
    for row in rows:
        stamp = int(row['stamp_ns'])
        item = grouped.setdefault(stamp, dict(sources=[], imu_count=0, imu_prefix=[]))
        item['sources'].append(row['source'])
        item['imu_count'] += int(row['imu_count'])
        item['imu_prefix'].append(int(row['imu_upto_ns']))
    return [(stamp, tuple(sorted(item['sources'])), item['imu_count'], tuple(sorted(item['imu_prefix'])))
            for stamp,item in sorted(grouped.items())]


def tum(path):
    return np.loadtxt(csv_path(path), ndmin=2)


def ordered(path, lower, upper):
    path = csv_path(path)
    with (gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()) as stream:
        rows = list(csv.DictReader(stream))
    rows = [r for r in rows if lower <= int(r['stamp_ns']) <= upper]
    stamps = np.array([int(r['stamp_ns']) for r in rows], dtype=np.int64)
    sources = {}
    for source, period in [('lidar', 100000000), ('vio', 50000000)]:
        t = np.array([int(r['stamp_ns']) for r in rows if r['source'] == source], dtype=np.int64)
        sources[source] = dict(count=len(t), continuous=bool(len(t)>1 and np.all(np.abs(np.diff(t)-period)<=2)))
    imu = np.array([int(r['imu_upto_ns']) for r in rows], dtype=np.int64)
    initialized = imu > 0
    return rows, dict(chronological=bool(len(stamps)>1 and np.all(np.diff(stamps)>=0)),
                     imu_monotonic=bool(len(imu)>1 and np.all(np.diff(imu)>=0)),
                     imu_brackets_events=bool(initialized.any() and np.all(
                         (imu[initialized]-stamps[initialized]>=0)&(imu[initialized]-stamps[initialized]<=1000000))),
                     sources=sources)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.output/'manifest.json').read_text())
    flight = Path(manifest['flight'])
    runtime = json.loads((flight/'runtime.json').read_text())
    checks = dict(sidecars_stopped=manifest.get('stopped', False),
                  sidecars_completed=manifest['status']=='reference_ended',
                  sidecar_processes_zero=bool(manifest.get('processes')) and all(v==0 for v in manifest['processes'].values()),
                  reference_complete=runtime['status']=='route_complete' and runtime['stopped'],
                  reference_processes_zero=all(v==0 for v in runtime['processes'].values()))
    expected_library = json.loads((flight/'protocol.json').read_text())['binary_sha256']['libmimosa_lib.so']
    libraries = json.loads((args.output/'loaded_libraries.json').read_text())
    library_key = ('loaded_mimosa_libraries_match_declared_variants'
                   if manifest.get('comparison_kind') in ('drpm_four_dof','batched_fusion') else 'same_loaded_mimosa_library')
    checks[library_key] = all(
        set(libraries[name].values()) == {variant.get('library_sha256_override',expected_library)}
        for name,variant in manifest['variants'].items())
    main_pose = tum(flight/'mimosa/graph_manager_odometry.tum')
    duplicate = tum(args.output/'duplicate_active/mimosa/graph_manager_odometry.tum')
    common, mi, di = np.intersect1d(np.rint(main_pose[:, 0]*1e9).astype(np.int64),
                                   np.rint(duplicate[:, 0]*1e9).astype(np.int64), return_indices=True)
    equal = bool(len(common)>1 and np.array_equal(main_pose[mi, 1:], duplicate[di, 1:]))
    checks['duplicate_poses_exactly_equal'] = equal
    checks['duplicate_covers_all_reference_poses'] = len(common) == len(main_pose)
    duplicate_report = dict(common_samples=len(common), exactly_equal=equal,
        maximum_numeric_difference=float(np.max(np.abs(main_pose[mi, 1:]-duplicate[di, 1:]))) if len(common) else None)
    # Audit the whole reference, not just a possibly truncated intersection.
    lower, upper = int(round(main_pose[0, 0]*1e9)), int(round(main_pose[-1, 0]*1e9))
    input_audits = {}
    for name in manifest['variants']:
        directory = args.output/name
        mode = manifest['variants'][name]['mode']
        reference_rows, _ = ordered(flight/('fixed' if mode=='fixed' else '')/'mimosa/ordered_inputs.csv', lower, upper)
        rows, audit = ordered(directory/'mimosa/ordered_inputs.csv', lower, upper)
        input_audits[name] = audit
        checks[name+'_continuous_input'] = (audit['chronological'] and audit['imu_monotonic']
            and audit['imu_brackets_events'] and all(v['continuous'] for v in audit['sources'].values()))
        def sensor_rows(stream):
            return [tuple(row[k] for k in ['stamp_ns', 'source', 'imu_count', 'imu_upto_ns']) for row in stream]
        if manifest['variants'][name]['kind'] == 'batched_fusion':
            checks[name+'_same_input_stamps_and_imu_prefix'] = coincident_input_signature(rows) == coincident_input_signature(reference_rows)
            audit['equality_rule'] = 'Exact multiset of sensors, total IMU count and per-event IMU prefix at every timestamp; only coincident ordering may differ.'
        else:
            checks[name+'_same_input_stamps_and_imu_prefix'] = sensor_rows(rows) == sensor_rows(reference_rows)
    alternative = manifest.get('comparison_kind', 'displacement')
    metrics = {'velocity': {}, alternative: {}}
    tracking_covered = False
    if checks['reference_complete']:
        route = load(flight/'route_tracking.csv')
        mask = route['route_t'] > 0
        rt, sim = route['route_t'][mask], route['sim_t'][mask]
        keep = np.r_[True, np.diff(rt)>1e-8]
        end = json.loads((flight/'protocol.json').read_text())['route_end']
        grid = np.arange(.1, end-.001, .1)
        limits = np.interp(grid[[0,-1]], rt[keep], sim[keep])
        for name in manifest['variants']:
            data = tum(args.output/name/'mimosa/graph_manager_odometry.tum')
            checks[name+'_spans_declared_tracking_interval'] = bool(
                data[0,0] <= limits[0] and data[-1,0] >= limits[1])
        tracking_covered = all(checks[name+'_spans_declared_tracking_interval'] for name in manifest['variants'])
    # Never extend the evaluation interval or rewrite the declared route end
    # to turn an interrupted experiment into a completed accuracy comparison.
    modes = ['fixed', 'active'] if checks['reference_complete'] and tracking_covered else []
    for mode in modes:
        metrics['velocity'][mode] = evaluate(flight/('fixed' if mode=='fixed' else ''), 'mimosa')[0]
        directory = args.output/(alternative+'_'+mode)
        data = tum(directory/'mimosa/graph_manager_odometry.tum')
        with tempfile.TemporaryDirectory(prefix='evaluation_', dir=args.output) as temporary:
            proxy = Path(temporary)
            (proxy/'protocol.json').symlink_to(flight/'protocol.json')
            for name in ['truth.csv', 'route_tracking.csv']:
                source = csv_path(flight/name)
                (proxy/source.name).symlink_to(source)
            data[:, 0] = np.rint(data[:, 0]*1e9)
            np.savetxt(proxy/'mimosa.csv', data, delimiter=',', header='stamp_ns,x,y,z,qx,qy,qz,qw',
                       comments='', fmt=['%.0f']+['%.17g']*7)
            metrics[alternative][mode] = evaluate(proxy, 'mimosa')[0]
        checks[mode+'_'+alternative+'_output_continuous'] = metrics[alternative][mode]['max_output_gap_s'] <= .10001
        if alternative == 'tilt_only':
            reference = flight/('fixed' if mode=='fixed' else '')
            original = np.loadtxt(csv_path(reference/'mimosa/tilt_factors.csv'), delimiter=',', skiprows=1)
            ablated = np.loadtxt(csv_path(directory/'mimosa/tilt_factors.csv'), delimiter=',', skiprows=1)
            checks[mode+'_tilt_observations_unchanged'] = np.array_equal(original, ablated)
            checks[mode+'_velocity_factors_omitted'] = not csv_path(directory/'mimosa/velocity_factors.csv').exists()
        if alternative in ('information_weighting','drpm_four_dof','batched_fusion','sensor_white'):
            reference = flight/('fixed' if mode=='fixed' else '')
            for observation in ['tilt_factors.csv', 'velocity_factors.csv']:
                original = np.loadtxt(csv_path(reference/'mimosa'/observation), delimiter=',', skiprows=1)
                weighted = np.loadtxt(csv_path(directory/'mimosa'/observation), delimiter=',', skiprows=1)
                checks[mode+'_'+observation.removesuffix('.csv')+'_unchanged'] = np.array_equal(original, weighted)
            original = yaml.safe_load((reference/'mimosa.yaml').read_text())
            weighted = yaml.safe_load((directory/'mimosa.yaml').read_text())
            for config in [original, weighted]: config.pop('logs_directory')
            if alternative == 'sensor_white':
                from unipilot_calibration import fusion_white_noise
                fusion_white_noise(original,flight/'unipilot_gimbal.sdf','sensor')
            elif alternative != 'batched_fusion':
                scan = original['lidar']['geometric']['scan_to_map']
                scan['drpm_enabled'] = True
                scan['drpm_registration_mode'] = 0 if alternative=='information_weighting' else 1
            checks[mode+'_only_declared_weighting_settings_changed'] = original == weighted
    result = dict(scope=manifest['scope'], valid=all(checks.values()), checks=checks,
                  complete_accuracy_available=checks['reference_complete'] and tracking_covered,
                  duplicate=duplicate_report, input_audits=input_audits, metrics=metrics,
                  variant_meaning=manifest.get('variant_meaning'),
                  report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    analysis_implementation = args.output/'analysis_implementation'
    analysis_implementation.mkdir(exist_ok=True)
    (analysis_implementation/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    (args.output/'comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(valid=result['valid'], checks=checks, duplicate=duplicate_report,
                         metrics=metrics), indent=2))


if __name__ == '__main__':
    main()
