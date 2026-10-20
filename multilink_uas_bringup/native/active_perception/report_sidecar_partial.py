#!/usr/bin/env python3
"""Read complete log lines only; report common partial tracking, never acceptance."""
import argparse
import io
import gzip
import json
from pathlib import Path
import numpy as np
from report_routes import interp, csv_path


def read_text(path):
    path = csv_path(path)
    return gzip.open(path, 'rt').read() if path.suffix == '.gz' else path.read_text()


def complete(path, csv=False):
    text = read_text(path)
    text = text[:text.rfind('\n')+1]
    return np.loadtxt(io.StringIO(text), delimiter=',' if csv else None,
                      skiprows=1 if csv else 0, ndmin=2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.output/'manifest.json').read_text())
    flight = Path(manifest['flight'])
    route = complete(flight/'route_tracking.csv', csv=True)
    header = read_text(flight/'route_tracking.csv').splitlines()[0].split(',')
    rt = route[:, header.index('route_t')]
    sim = route[:, header.index('sim_t')]
    keep = (rt>0)&np.r_[True, np.diff(rt)>1e-8]
    rt, sim = rt[keep], sim[keep]
    if len(rt)<3:
        raise RuntimeError('Tracking has not started')
    data = {'truth': complete(flight/'truth.csv', csv=True)}
    for mode in ['fixed', 'active']:
        source = flight/('fixed' if mode=='fixed' else '')
        data['velocity_'+mode] = complete(source/'mimosa.csv', csv=True)
        data['rovio_'+mode] = complete(source/'rovio.csv', csv=True)
    for name in manifest['variants']:
        array = complete(args.output/name/'mimosa/graph_manager_odometry.tum')
        array[:, 0] = np.rint(array[:, 0]*1e9)
        data[name] = array
    for name, array in data.items():
        data[name] = array[np.r_[True, np.diff(array[:, 0])>0]]
    end = min(sim[-1], *[a[-1, 0]*1e-9 for a in data.values()])-.02
    grid = np.arange(.1, float(np.interp(end, sim, rt)), .1)
    stamps = np.rint(np.interp(grid, rt, sim)*1e9)
    if len(stamps)<3:
        raise RuntimeError('Insufficient common tracking output')
    truth, tq = interp(data.pop('truth'), stamps)
    metrics = {}
    for name, array in data.items():
        p, q = interp(array, stamps)
        alignment = tq[0]*q[0].inv()
        error = alignment.apply(p-p[0])+truth[0]-truth
        angle = np.rad2deg((tq.inv()*alignment*q).magnitude())
        metrics[name] = dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
                             rotation_rmse_deg=float(np.sqrt(np.mean(angle**2))))
    result = dict(scope='Common partial tracking interval only; not complete accuracy or acceptance.',
                  route_time_s=float(grid[-1]), metrics=metrics)
    (args.output/'partial.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
