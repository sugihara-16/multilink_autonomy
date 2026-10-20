#!/usr/bin/env python3
"""Posthoc velocity-error directions; travel direction is NOT LiDAR degeneracy.

Uses truth only for offline evaluation. Includes every completed declared flight;
the moving subset is diagnostic and never replaces full-route acceptance.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from report_routes import load, poses, interp


def diagnose(flight):
    runtime = json.loads((flight/'runtime.json').read_text())
    if not runtime['stopped'] or runtime['status'] != 'route_complete':
        raise ValueError('Only complete stopped flights: '+str(flight))
    tracking = load(flight/'route_tracking.csv')
    tracking = tracking['sim_t'][tracking['route_t'] > 0]
    gt = poses(flight/'truth.csv')
    inputs = {side: load(base/'mimosa/velocity_factors.csv')
              for side, base in [('fixed', flight/'fixed'), ('active', flight)]}
    if not np.array_equal(inputs['fixed']['stamp_ns'], inputs['active']['stamp_ns']):
        raise ValueError('Sparse fusion timestamps differ')
    t = inputs['fixed']['stamp_ns']
    mask = ((t >= tracking[0]*1e9) & (t <= tracking[-1]*1e9)
            & (t > gt[0, 0]+2e7) & (t < gt[-1, 0]-2e7))
    t = t[mask]
    before, _ = interp(gt, t-1e7)
    after, _ = interp(gt, t+1e7)
    velocity = (after-before)/.02
    _, q = interp(gt, t)
    horizontal_speed = np.linalg.norm(velocity[:, :2], axis=1)
    moving = horizontal_speed >= .2
    forward = np.column_stack([velocity[:, :2]/np.maximum(horizontal_speed[:, None], 1e-30),
                               np.zeros(len(t))])
    up = np.tile([0., 0., 1.], (len(t), 1))
    left = np.cross(up, forward)
    basis = np.stack([forward, left, up], axis=1)
    result = dict(flight=str(flight), samples=len(t), moving_samples=int(moving.sum()),
                  moving_min_horizontal_speed_mps=.2,
                  axes=['horizontal_travel', 'horizontal_cross_travel', 'vertical'],
                  scope='Moving-only direction statistics. Travel direction is not a measured LiDAR weak direction.')
    for side, data in inputs.items():
        v = np.column_stack([data['v'+c][mask] for c in 'xyz'])
        C = np.array([data[f'c{i}{j}'][mask] for i in range(3) for j in range(3)]).T.reshape(-1, 3, 3)
        R = q.as_matrix()
        world_error = q.apply(v)-velocity
        world_covariance = R@C@R.transpose(0, 2, 1)
        error = np.einsum('nij,nj->ni', basis[moving], world_error[moving])
        covariance = basis[moving]@world_covariance[moving]@basis[moving].transpose(0, 2, 1)
        norm_difference = np.abs((error**2).sum(axis=1)-(world_error[moving]**2).sum(axis=1))
        if norm_difference.max() > 1e-10:
            raise ValueError('Direction transform did not preserve error norm')
        result[side] = dict(
            all_tracking_rmse_mps=float(np.sqrt(np.mean((world_error**2).sum(axis=1)))),
            moving_rmse_mps=float(np.sqrt(np.mean((error**2).sum(axis=1)))),
            moving_axis_rmse_mps=np.sqrt(np.mean(error**2, axis=0)).tolist(),
            moving_axis_bias_mps=error.mean(axis=0).tolist(),
            moving_axis_sigma_rms_mps=np.sqrt(np.mean(np.diagonal(covariance, axis1=1, axis2=2), axis=0)).tolist(),
            error_norm_preservation_max=float(norm_difference.max()))
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('campaigns', nargs='+', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('Preserve previous diagnosis')
    result = dict(scope=__doc__, flights=[diagnose(flight)
                  for campaign in args.campaigns for flight in sorted(campaign.glob('*_active'))])
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for row in result['flights']:
        print(row['flight'], {side: row[side]['moving_axis_rmse_mps'] for side in ['fixed', 'active']})


if __name__ == '__main__':
    main()
