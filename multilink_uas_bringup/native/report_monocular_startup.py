#!/usr/bin/env python3
"""Validate and plot the sensor-derived single-camera initialization fix."""
import csv
import hashlib
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from plot_fast_camera_comparison import read, errors
from rovio_initialization import match_accelerometer_bias_prior

ROOT = Path(__file__).resolve().parent
BASE = ROOT / 'logs/zab-offline'
OUT = BASE / 'monocular_startup'
LABELS = ['run1_prefix', 'run1_tail_a1', 'run1_tail_b1']


def table(path):
    return np.genfromtxt(path, delimiter=',', names=True)


def main():
    plots = OUT / 'plots'
    plots.mkdir(exist_ok=True)
    results, html_rows, html_images = {}, [], []
    with PdfPages(plots / 'comparison.pdf') as pdf:
        for label in LABELS:
            old = BASE / 'drpm_fast_failures/rovio' / label
            new = OUT / 'bias' / label
            summary = json.loads((new / 'summary.json').read_text())
            previous = json.loads((old / 'summary.json').read_text())
            assert summary['counts'] == previous['counts']
            assert summary['binary_sha256'] == previous['binary_sha256']
            source = ROOT / 'logs' / previous['source']['run']
            cfg, variances = match_accelerometer_bias_prior((old / 'rovio.info').read_text(), source / 'dragon_slam.urdf')
            assert cfg == (new / 'rovio.info').read_text()
            assert (new / 'cam0.yaml').read_bytes() == (old / 'cam0.yaml').read_bytes()
            log = (new / 'replay.log').read_text()
            assert len(re.findall(r'-- Filter: Initialized at t =', log)) == 1
            assert 'AUDIT_INPUT ' + ' '.join(map(str, summary['counts'])) + ' 0 0' in log
            assert 'incomplete 0' in log
            truth_file = BASE / 'drpm_fast_failures/inputs' / label / 'truth.csv'
            truth = read(truth_file)
            gt = table(truth_file)
            stationary = (gt['stamp_ns'] >= 3e9) & (gt['stamp_ns'] <= 10e9)
            true_speed = np.linalg.norm(np.c_[gt['vx'], gt['vy'], gt['vz']][stationary], axis=1)
            assert true_speed.max() < .01
            w = summary['window']
            anchor = (w['start_ns'] + 99999999) // 100000000 * 100000000
            grid = np.arange(anchor, w['end_ns'] + 1, 100000000, dtype=np.int64)
            data, metrics, estimates = {}, {}, {}
            for name, folder in [('Previous', old), ('Corrected prior', new)]:
                pose = read(folder / 'result_vio.csv')
                a = table(folder / 'result_vio.csv')
                estimates[name] = a
                assert np.all(np.diff(pose[0]) == 50000000)
                assert np.isin(grid, pose[0]).all()
                if name == 'Previous':
                    original_stamps = pose[0]
                else:
                    assert np.array_equal(pose[0], original_stamps)
                e, m = errors(pose, truth, anchor, grid)
                k = (a['stamp_ns'] >= 3e9) & (a['stamp_ns'] <= 10e9)
                speed = np.linalg.norm(np.c_[a['vx'], a['vy'], a['vz']], axis=1)
                m['stationary_3to10s_speed_rmse_mps'] = float(np.sqrt(np.mean(speed[k] ** 2)))
                m['stationary_3to10s_speed_max_mps'] = float(speed[k].max())
                i = np.flatnonzero(a['stamp_ns'] == 10000000000)[0]
                m['speed_at_10s_mps'] = float(speed[i])
                m['position_increment_speed_at_10s_mps'] = float(np.linalg.norm(pose[1][i] - pose[1][i - 1]) / .05)
                depths = []
                with (folder / 'result_features.csv').open() as stream:
                    for row in csv.DictReader(stream):
                        if int(row['stamp_ns']) == 10000000000:
                            depths.append(float(row['depth']))
                m['median_feature_depth_at_10s_m'] = float(np.median(depths))
                m['feature_count_at_10s'] = len(depths)
                metrics[name] = m
                data[name] = e
                np.savetxt(plots / f'{label}_{name.replace(" ", "_")}_errors.csv',
                           np.column_stack([grid, e]), delimiter=',', fmt=['%d'] + ['%.12g'] * 6,
                           header='stamp_ns,ex_m,ey_m,ez_m,eroll_deg,epitch_deg,eyaw_deg', comments='')
            fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
            for name, a in estimates.items():
                k = (a['stamp_ns'] >= 2.05e9) & (a['stamp_ns'] <= 10e9)
                speed = np.linalg.norm(np.c_[a['vx'], a['vy'], a['vz']], axis=1)
                axes[0].plot(a['stamp_ns'][k] * 1e-9, speed[k], label=name)
                state_path = (old if name == 'Previous' else new) / 'result_state.csv'
                state = table(state_path)
                k = (state['stamp_ns'] >= 2.05e9) & (state['stamp_ns'] <= 10e9)
                axes[1].plot(state['stamp_ns'][k] * 1e-9, state['acb_z'][k], label=name)
            k = (gt['stamp_ns'] >= 2.05e9) & (gt['stamp_ns'] <= 10e9)
            axes[0].plot(gt['stamp_ns'][k] * 1e-9, np.linalg.norm(np.c_[gt['vx'], gt['vy'], gt['vz']][k], axis=1), label='Truth', color='black', lw=.8)
            axes[0].set_ylabel('Estimated speed [m/s]')
            axes[1].set_ylabel('Estimated Z accel. bias [m/s²]')
            axes[1].set_xlabel('Original simulation time [s]')
            for ax in axes:
                ax.legend(); ax.grid(alpha=.25)
            fig.suptitle(label + ' | Stationary startup: camera + IMU only')
            fig.tight_layout(); fig.savefig(plots / f'{label}_startup.png', dpi=130); pdf.savefig(fig); plt.close(fig)
            html_images.append(f'<h2>{label}</h2><img src="{label}_startup.png">')
            # Separate full-scale views retain the old divergence without hiding
            # corrected errors behind its much larger axis range.
            for name, e in data.items():
                fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)
                for i, ax in enumerate(axes):
                    ax.plot((grid - anchor) * 1e-9, e[:, i]); ax.grid(alpha=.25)
                    ax.set_ylabel(['X [m]', 'Y [m]', 'Z [m]', 'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'][i])
                axes[-1].set_xlabel('Tracking time [s]')
                fig.suptitle(label + ' | ' + name + ' | Full tracking window')
                fig.tight_layout(); filename = f'{label}_{name.replace(" ", "_")}.png'
                fig.savefig(plots / filename, dpi=110); pdf.savefig(fig); plt.close(fig)
                html_images.append(f'<details><summary>{name}: six-axis tracking errors</summary><img src="{filename}"></details>')
            for name, m in metrics.items():
                html_rows.append(f'<tr><td>{label}</td><td>{name}</td><td>{m["stationary_3to10s_speed_rmse_mps"]:.6f}</td><td>{m["position_rmse_m"]:.6f}</td><td>{m["position_max_m"]:.6f}</td><td>{m["rotation_rmse_deg"]:.6f}</td></tr>')
            results[label] = dict(metrics=metrics, window=w, alignment_stamp_ns=anchor,
                                 true_stationary_speed_max_mps=float(true_speed.max()),
                                 initial_accelerometer_bias_variance=variances,
                                 checks=dict(same_binary=True, same_calibration=True, input_counts_identical=True,
                                             output_stamps_identical=True, complete_tracking=True,
                                             only_three_initial_bias_variances_changed=True, truth_used_only_for_evaluation=True),
                                 result_sha256=hashlib.sha256((new / 'result_vio.csv').read_bytes()).hexdigest())
    assert (OUT / 'baseline/run1_tail_b1/result_vio.csv').read_bytes() == (BASE / 'drpm_fast_failures/rovio/run1_tail_b1/result_vio.csv').read_bytes()
    (OUT / 'evaluation.json').write_text(json.dumps(results, indent=2))
    note = ('Single link3 camera; same 25-feature ROVIO executable and same recorded IMU. '
            'Only initial accelerometer bias variance changes: 0.0004 → 0.010001 (m/s²)², '
            'derived from recorded sensor bias specification 0.1 ± 0.001 m/s². '
            'Bias mean remains zero. No ground truth, LiDAR, stationary update, reset or extra camera is supplied to the estimator. '
            'Initialization at the original 2.003 s; all measurement timestamps unchanged. '
            'Tracking errors use one rigid alignment at tracking start, no subsequent alignment or scale fit. '
            'Stationary startup is evaluated separately before that alignment, so an initial failure cannot be hidden. '
            'Old/new tracking plots use separate full, unbroken axes. Old result files remain unchanged. '
            'These three full flights test initialization; remaining monocular drift is not claimed solved.')
    (plots / 'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Single-camera ROVIO startup</title>'
        '<style>body{font:16px sans-serif;max-width:1250px;margin:30px auto;padding:15px}img{max-width:100%}td,th{border:1px solid #bbb;padding:8px}table{border-collapse:collapse}p{line-height:1.6}</style>'
        '<h1>Single-camera ROVIO: corrected startup bias uncertainty</h1><p>' + note + '</p>'
        '<p><a href="comparison.pdf">All plots PDF</a> · <a href="../evaluation.json">Metrics and validation</a></p>'
        '<table><tr><th>Flight</th><th>Configuration</th><th>Stationary speed RMS [m/s]</th><th>Position RMSE [m]</th><th>Position max [m]</th><th>Rotation RMSE [deg]</th></tr>'
        + ''.join(html_rows) + '</table>' + ''.join(html_images))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
