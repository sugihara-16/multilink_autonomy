#!/usr/bin/env python3
"""Summarize measured gaze and planner health in complete, declared flights."""
import argparse
import collections
import gzip
import json
from pathlib import Path

import numpy as np
from report_routes import load


def describe(base):
    runtime = base/'runtime.json'
    if not runtime.exists():
        return {'complete': False}
    status = json.loads(runtime.read_text())
    if not status.get('stopped') or status.get('status') != 'route_complete':
        return {'complete': False, 'status': status.get('status')}
    route = load(base/'route_tracking.csv')
    times = route['sim_t'][route['route_t'] > 0]
    path = base/'decisions.jsonl'
    if path.exists():
        stream = path.open()
    else:
        stream = gzip.open(str(path)+'.gz', 'rt')
    with stream:
        decisions = [d for line in stream if (d := json.loads(line))
                     and times[0]*1e9 <= d['stamp_ns'] <= times[-1]*1e9]
    encoder = load(base/'encoders.csv')
    # Use logged encoder records, not desired commands, to describe the view.
    names = list(encoder)
    if not {'t', 'yaw', 'pitch'}.issubset(names):
        raise ValueError('Unexpected encoder columns: '+str(names))
    inside = (encoder['t'] >= times[0]) & (encoder['t'] <= times[-1])
    angles = np.rad2deg(np.column_stack([encoder[n][inside] for n in ['yaw', 'pitch']]))
    tracks = [d['image_tracks'] for d in decisions if d.get('image_tracks') is not None]
    selected = [d for d in decisions if d.get('reason') == 'evaluated']
    return dict(complete=True, decisions=len(decisions),
        reason_counts=dict(collections.Counter(d['reason'] for d in decisions)),
        encoder_samples=len(angles), yaw_pitch_quantiles_deg=np.percentile(angles, [10, 50, 90], axis=0).tolist(),
        steep_pitch_fraction=float(np.mean(np.abs(angles[:, 1]) > 60)),
        image_tracks_quantiles=np.percentile(tracks, [10, 50, 90]).tolist() if tracks else [],
        low_image_tracks_fraction=float(np.mean(np.asarray(tracks) < 4)) if tracks else None,
        map_points_quantiles=np.percentile([d['map_points'] for d in decisions], [10, 50, 90]).tolist(),
        retention_infeasible_decisions=sum(not d['retention_constraint_feasible'] for d in selected))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('declaration', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    declaration = json.loads(a.declaration.read_text())
    campaigns = declaration.get('campaigns', declaration.get('validation_campaigns'))
    result = dict(scope=__doc__, declaration=str(a.declaration), flights={})
    for campaign in campaigns:
        for base in sorted((a.declaration.parent/campaign).glob('*_active')):
            result['flights'][campaign+'/'+base.name] = describe(base)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
