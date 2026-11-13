#!/usr/bin/env python3
"""Audit a completed campaign in its runtime container, retaining failed checks."""
import argparse
import datetime
import hashlib
import json
import os
import re
from pathlib import Path
import numpy as np
import yaml
from report_dragon_fusion_fix import registration
from compact_record_io import record_path


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    protocol = read(root / 'comparison_protocol.json')
    state = read(root / 'status.json')
    duration = protocol['distance_m']/protocol['speed_mps']+2
    changed = [name for name, digest in protocol['sha256'].items()
               if not Path(name).exists() or hashlib.sha256(Path(name).read_bytes()).hexdigest() != digest]
    report_only_changes = [name for name in changed if Path(name).name == 'report_practical_comparison.py']
    execution_changes = [name for name in changed if name not in report_only_changes]
    trials = {}
    for arm, seed in protocol['order']:
        name = f'{arm}_seed{seed}'
        p = root / name
        result = trials[name] = dict(checks={})
        checks = result['checks']
        try:
            inputs = read(p / 'input_audit.json')
            manifest = read(p / 'protocol.json')
            loaded = read(p / 'loaded_fusion_library.json')
            for sensor in ('imu', 'lidar'):
                sample = inputs[sensor]
                checks[sensor + '_continuous'] = sample['count'] > 0 and all(sample[k] == 0 for k in ('missing_periods', 'duplicates', 'reordered'))
            checks['startup_valid'] = read(p / 'evaluation_status.json')['startup']['valid']
            library = 'mimosa-batched-vision' if arm == 'unipilot' else 'mimosa-batched-drpm-support-view'
            checks['correct_loaded_library'] = bool(loaded['maps']) and all('/' + library + '/libmimosa_lib.so' in line for line in loaded['maps'])
            checks['seed'] = manifest['seed'] == seed
            if arm == 'unipilot':
                route = read(p / 'route_status.json')
                runtime = read(p / 'runtime.json')
                checks['route_complete'] = route['stage'] == 'complete' and route['failure'] is None and route['route_time_s'] >= duration
                checks['clean_shutdown'] = runtime['stopped'] and all(v == 0 for v in runtime['processes'].values()) and runtime['launch_shutdown']['all_clean'] and runtime['gazebo_shutdown']['all_clean']
                image = read(p / 'array_input_audit.json')['front']
                stamps = np.array(image['image_stamps'], dtype=np.int64)
                extrinsics = np.array(image['extrinsic_stamps'], dtype=np.int64)
                checks['images_continuous'] = len(stamps) > 0 and bool(np.all(np.diff(stamps) == 50000000))
                checks['camera_transform_paired'] = np.array_equal(stamps, extrinsics) and image['rotation_error_max_rad'] < 1e-8 and image['position_error_max_m'] < 1e-8
                checks['reference_configuration'] = not manifest['drpm_fusion'] and manifest['joint_rovio_information'] and manifest['rovio_noise_profile'] == 'sensor'
                records = np.genfromtxt(record_path(p / 'route_tracking.csv'), delimiter=',', names=True)
                moving = records[records['route_t'] > 0]
                start = float(moving['sim_t'][0] - moving['route_t'][0])
                result['image_count'] = len(stamps)
                checks['images_cover_tracking'] = stamps[0]*1e-9 <= start and stamps[-1]*1e-9 >= start + duration
            else:
                validation = read(p / 'validation.json')
                result['motion_and_acquisition'] = validation['checks']
                checks['all_flight_checks'] = validation['passed']
                done = read(p / 'completion.json')
                checks['clean_shutdown'] = done['flight_completed'] and all(v == 0 for v in done['processes'].values())
                checks['adopted_configuration'] = manifest['fusion_profile'] == 'drpm_isotropic20_view' and manifest['unipilot_matched'] == (arm == 'gimbal') and manifest['mechanism_comparison'] == (arm == 'joint')
                start = next(e['time'] for e in read(p / 'flight_events.json') if e['stage'] == 'active')
                result['registration'] = registration(p / 'mimosa/drpm_registration.csv', start, start + duration)
                checks['lidar_corrections_retained'] = result['registration']['passed']
                owners = read(p / 'owners.json')
                checks['owned_pids_stopped'] = not any(Path('/proc', str(pid)).exists() for pid in [*owners.values(), loaded['pid']])
                checks['recorded_hashes_match_campaign'] = all(digest == protocol['sha256'][filename] for filename, digest in manifest['sha256'].items() if filename in protocol['sha256'])
                encoder = read(p / 'encoder_audit.json')
                result['encoder'] = encoder
                checks['images_cover_tracking'] = encoder['first_ns']*1e-9 <= start and encoder['last_ns']*1e-9 >= start + duration
            checks['input_covers_tracking'] = all(s['first_stamp_ns']*1e-9 <= start and s['last_stamp_ns']*1e-9 >= start + duration for s in inputs.values())
            result['input_summary'] = inputs
        except (OSError, KeyError, ValueError, IndexError, StopIteration) as error:
            result['error'] = str(error)
            checks['complete_records'] = False
        for key in checks:
            checks[key] = bool(checks[key])
        result['passed'] = bool(checks) and all(checks.values())
    config_checks = {}
    config_errors = []
    try:
        # Single-arm reproduction campaigns still require configuration checks.
        reference_arm, reference_seed = next(
            ((arm, seed) for arm, seed in protocol['order'] if arm == 'gimbal'),
            protocol['order'][0])
        reference = root / f'{reference_arm}_seed{reference_seed}'
        fusion = yaml.safe_load((reference / 'mimosa.yaml').read_text())
        fusion.pop('logs_directory', None)
        for arm, seed in protocol['order']:
            name = f'{arm}_seed{seed}'
            trial = root / name
            config_checks[name + '_intrinsics'] = yaml.safe_load((trial/'cam0.yaml').read_text()) == yaml.safe_load((reference/'cam0.yaml').read_text())
            rv = (trial/'rovio.info').read_text()
            ref = (reference/'rovio.info').read_text()
            if arm != reference_arm and 'unipilot' in (arm, reference_arm):
                # Mount position differs by design; compare all other filter settings.
                pattern = r'^\s*(?:qCM_[xyzw]|MrMC_[xyz])\s+[^;]+;\s*$'
                rv = re.sub(pattern, '', rv, flags=re.MULTILINE)
                ref = re.sub(pattern, '', ref, flags=re.MULTILINE)
            config_checks[name + '_rovio_filter_parameters'] = rv == ref
            if arm != 'unipilot' or reference_arm == 'unipilot':
                cfg = yaml.safe_load((trial/'mimosa.yaml').read_text())
                cfg.pop('logs_directory', None)
                config_checks[name + '_fusion'] = cfg == fusion
    except (OSError, KeyError, ValueError) as error:
        config_errors.append(str(error))
        config_checks['complete_configuration_records'] = False
    live = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            environment = (proc / 'environ').read_bytes().split(b'\0')
            if any(f'ROS_MASTER_URI=http://127.0.0.1:{protocol.get("port_base",11651)+i}'.encode() in environment or f'ROS_DOMAIN_ID={protocol.get("domain_base",151)+i}'.encode() in environment for i in range(len(protocol['order']))):
                live.append(dict(pid=int(proc.name), command=(proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    checks = dict(common_configurations_match=bool(config_checks) and all(config_checks.values()), campaign_complete=state.get('status') == 'complete', all_trial_returncodes_zero=len(state['completed']) == len(protocol['order']) and all(r['returncode'] == 0 for r in state['completed']), frozen_execution_and_metrics_unchanged=not execution_changes, all_trials_passed=all(r['passed'] for r in trials.values()), experiment_processes_stopped=not live)
    result = dict(timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), checks=checks, passed=all(checks.values()), changed_frozen_files=changed, report_only_changes=report_only_changes, configuration_checks=config_checks, configuration_errors=config_errors, live_processes=live, trials=trials, note='Only the report presentation script may be updated after acquisition; execution and the common-clock numerical evaluator remain frozen. Raw matching.json in joint trials describes its legacy mechanism-comparison mode. Campaign protocol defines the actual asymmetric adopted-configuration comparison.')
    (root / 'final_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(checks=checks, trial_failures={name: [k for k,v in r['checks'].items() if not v] for name,r in trials.items() if not r['passed']}), indent=2))


if __name__ == '__main__':
    main()
