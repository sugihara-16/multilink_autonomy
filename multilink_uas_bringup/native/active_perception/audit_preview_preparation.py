#!/usr/bin/env python3
"""Check the executed common preview and keep its view mismatch visible."""
import argparse
import json
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from report_routes import load
from preview_motion import reference
from initialization_motion import DURATION


def audit(root):
    campaign = json.loads((root / 'campaign.json').read_text())
    result = {}
    for key in campaign['order']:
        if campaign['runs'].get(key, {}).get('status') != 'complete':
            continue
        flight = root / key / 'campaign' / (key + '_active')
        stages = re.findall(r'Stage: (\w+)', (flight / 'route.log').read_text())
        expected = ['excitation', 'preview_out', 'preview_return', 'excitation_hold', 'tracking', 'complete']
        stage_ok = stages[stages.index('excitation'):] == expected if 'excitation' in stages else False
        rows = [json.loads(line) for line in (flight / 'decisions.jsonl').read_text().splitlines()]
        held = [row for row in rows if row['reason'] == 'hold_view_for_common_preparation']
        if not held:
            raise RuntimeError('No held preview/preparation decisions: ' + key)
        times = np.array([row['stamp_ns'] * 1e-9 for row in held])
        current = np.array([row['encoder'] for row in held])
        target = np.array([row['target'] for row in held])
        enc = load(flight / 'encoders.csv')
        mask = (enc['t'] >= times[0] + 1.) & (enc['t'] <= times[-1])
        angles = np.column_stack([enc['yaw'][mask], enc['pitch'][mask]])
        angular_range = np.ptp(angles, axis=0)
        route = load(flight / 'route_tracking.csv')
        tracking = route['route_t'] > 0
        start = float(route['sim_t'][tracking][0] - route['route_t'][tracking][0])
        route_mask = (route['sim_t'] >= times[0]) & (route['sim_t'] <= start)
        segment = json.loads((flight / 'route/segments.json').read_text())['segments'][0]
        return_duration = reference(segment, 0)[2]
        cruise = (enc['t'] >= start + 60) & (enc['t'] < start + 120)
        median = np.median(np.column_stack([enc['yaw'][cruise], enc['pitch'][cruise]]), axis=0)
        held_median = np.median(angles, axis=0)
        held_rotation = Rotation.from_euler('z', held_median[0]) * Rotation.from_euler('y', held_median[1])
        cruise_rotation = Rotation.from_euler('z', median[0]) * Rotation.from_euler('y', median[1])
        cosine = held_rotation.apply([1., 0., 0.]) @ cruise_rotation.apply([1., 0., 0.])
        checks = dict(
            stage_order=stage_ok,
            command_equals_measured_angle=bool(np.array_equal(current, target)),
            held_angle_stable=bool(np.all(angular_range < 1e-6)),
            return_plus_preparation_observed=bool(times[-1] - times[0] >= return_duration + DURATION - 2.),
            held_motion_excluded_from_evaluation=bool(route_mask.any() and np.all(route['route_t'][route_mask] == 0)),
            full_body_preparation_before_evaluation=bool(start - times[0] >= return_duration + DURATION - 2.),
            camera_hold_handoff_within_update=bool(times[-1] - start < .5),
        )
        result[key] = dict(checks=checks, valid=all(checks.values()), held_interval_s=[float(times[0]), float(times[-1])],
                           held_range_deg=np.rad2deg(angular_range).tolist(),
                           held_angle_deg=np.rad2deg(held_median).tolist(),
                           cruise_60_120_median_deg=np.rad2deg(median).tolist(),
                           optical_axis_change_deg=float(np.rad2deg(np.arccos(np.clip(cosine, -1., 1.)))),
                           camera_rotation_change_deg=float(np.rad2deg((held_rotation.inv() * cruise_rotation).magnitude())),
                           evaluation_start_s=start, expected_return_duration_s=return_duration,
                           last_camera_hold_minus_evaluation_start_s=float(times[-1] - start))
    out = dict(trials=result, all_valid=bool(result) and all(row['valid'] for row in result.values()),
               note='Cruise-versus-held view mismatch is reported, not used to discard a trial. All fixed views receive the same body motion. Body preparation ends at the route clock start; a final held camera command may overlap that transition by one planning update.')
    (root / 'preview_preparation_audit.json').write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out, indent=2))
    if not out['all_valid']:
        raise RuntimeError('Preview/preparation execution check failed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('campaign', type=Path)
    audit(parser.parse_args().campaign.resolve())
