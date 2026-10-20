#!/usr/bin/env python3
"""Posthoc exact decomposition of accumulated pose error; not an estimator input.

Separate body-relative displacement error from the error introduced when that
displacement is rotated into the navigation frame. The components interact;
their norms are not independent causal contributions or additive RMSEs.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from report_routes import load, poses, interp


def decompose(truth, tq, position, q):
    alignment = tq[0]*q[0].inv()
    position = alignment.apply(position-position[0])+truth[0]
    q = alignment*q
    true_steps = np.diff(truth, axis=0)
    measured_body_steps = q[:-1].inv().apply(np.diff(position, axis=0))
    true_body_steps = tq[:-1].inv().apply(true_steps)
    local = q[:-1].apply(measured_body_steps-true_body_steps)
    orientation = q[:-1].apply(true_body_steps)-true_steps
    local = np.vstack([np.zeros(3), np.cumsum(local, axis=0)])
    orientation = np.vstack([np.zeros(3), np.cumsum(orientation, axis=0)])
    error = position-truth
    closure = float(np.max(np.abs(local+orientation-error)))
    if closure > 1e-8:
        raise ValueError('Decomposition does not reconstruct original error')
    def stats(a):
        return dict(rmse_m=float(np.sqrt(np.mean(np.sum(a*a, axis=1)))),
                    axis_rmse_m=np.sqrt(np.mean(a*a, axis=0)).tolist(),
                    final_world_m=a[-1].tolist())
    return dict(total=stats(error), body_displacement=stats(local),
                orientation=stats(orientation),
                cross_mean_m2=float(np.mean(np.sum(local*orientation, axis=1))),
                reconstruction_max_error_m=closure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--candidate', required=True)
    args = parser.parse_args()
    result = {}
    for campaign in sorted(args.root.glob(args.candidate+'-*-seed*')):
        for flight in sorted(campaign.glob('*_active')):
            runtime = flight/'runtime.json'
            if not runtime.exists() or json.loads(runtime.read_text())['status'] != 'route_complete':
                continue
            route = load(flight/'route_tracking.csv')
            keep = (route['route_t'] > 0) & np.r_[True, np.diff(route['route_t']) > 1e-8]
            rt, sim = route['route_t'][keep], route['sim_t'][keep]
            grid = np.arange(.1, rt[-1]-.02, .1)
            stamps = np.rint(np.interp(grid, rt, sim)*1e9)
            gp, gq = interp(poses(flight/'truth.csv'), stamps)
            values = {}
            for side, base in [('fixed', flight/'fixed'), ('active', flight)]:
                values[side] = {}
                for method in ['rovio', 'mimosa']:
                    data = poses(base/(method+'.csv'))
                    if stamps[0] < data[0, 0] or stamps[-1] > data[-1, 0]:
                        raise ValueError('Missing full tracking coverage')
                    p, q = interp(data, stamps)
                    values[side][method] = decompose(gp, gq, p, q)
            key = campaign.name+'/'+flight.name
            result[key] = dict(duration_s=float(grid[-1]), values=values)
            print(key, {s: {k: round(v['rmse_m'], 4) for k, v in m['mimosa'].items()
                            if isinstance(v, dict)} for s, m in values.items()}, flush=True)
    output = dict(scope=__doc__, candidate=args.candidate, flights=result)
    (args.root/(args.candidate+'_integrated_drift_diagnosis.json')).write_text(json.dumps(output, indent=2)+'\n')


if __name__ == '__main__':
    main()
