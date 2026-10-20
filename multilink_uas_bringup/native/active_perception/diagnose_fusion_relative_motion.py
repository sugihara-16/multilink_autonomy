#!/usr/bin/env python3
"""Offline comparison of VIO/fusion motion errors over matching one-second windows."""
import argparse
import json
from pathlib import Path
import numpy as np
from report_routes import load, poses, interp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--candidate', required=True)
    args = parser.parse_args()
    results = {}
    for campaign in sorted(args.root.glob(args.candidate+'-*-seed*')):
        for flight in sorted(campaign.glob('*_active')):
            runtime = flight/'runtime.json'
            if not runtime.exists() or json.loads(runtime.read_text())['status'] != 'route_complete':
                continue
            route = load(flight/'route_tracking.csv')
            inside = route['route_t'] > 0
            start, end = route['sim_t'][inside][[0, -1]]
            stamps = np.rint(np.arange(start+1., end-.02, 1.)*1e9)
            truth = poses(flight/'truth.csv')
            gp, gq = interp(truth, stamps)
            g0, gq0 = interp(truth, stamps-1e9)
            reference = gq0.inv().apply(gp-g0)
            result = {}
            for mode, directory in [('fixed', flight/'fixed'), ('active', flight)]:
                result[mode] = {}
                for estimator in ['rovio', 'mimosa']:
                    data = poses(directory/(estimator+'.csv'))
                    ep, eq = interp(data, stamps)
                    e0, eq0 = interp(data, stamps-1e9)
                    error = eq0.inv().apply(ep-e0)-reference
                    angle = np.rad2deg(((gq0.inv()*gq).inv()*(eq0.inv()*eq)).magnitude())
                    def stats(indices):
                        e = error[indices]
                        return dict(samples=len(e), translation_rmse_m=float(np.sqrt(np.mean(np.sum(e*e, axis=1)))),
                                    translation_axis_rmse_m=np.sqrt(np.mean(e*e, axis=0)).tolist(),
                                    translation_bias_m=e.mean(axis=0).tolist(),
                                    rotation_rmse_deg=float(np.sqrt(np.mean(angle[indices]**2))))
                    indices = np.arange(len(stamps))
                    result[mode][estimator] = dict(stats(indices),
                        thirds=[stats(i) for i in np.array_split(indices, 3)])
            results[campaign.name+'/'+flight.name] = result
            print(campaign.name, flight.name,
                  {mode: {name: round(m['translation_rmse_m'], 5) for name, m in values.items()}
                   for mode, values in result.items()})
    output = dict(scope='Offline diagnostic only. One-second body-relative displacement and rotation, '
                        'sampled at common one-Hz timestamps during tracking. No global alignment or scale fit. '
                        'MIMOSA results already include VIO: this is not a LiDAR-only ablation.', results=results)
    (args.root/(args.candidate+'_relative_motion_diagnosis.json')).write_text(json.dumps(output, indent=2)+'\n')


if __name__ == '__main__':
    main()
