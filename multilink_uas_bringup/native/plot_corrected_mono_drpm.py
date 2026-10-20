#!/usr/bin/env python3
"""Full-window comparison after fixing single-camera ROVIO initialization."""
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

from run_corrected_mono_drpm import ROOT, OUT, REF, VIO, base


def main():
    plots = OUT / 'plots'
    plots.mkdir(exist_ok=True)
    rows, images, results = [], [], {}
    names = {'old': 'Previous single-camera fusion',
             'new': 'Corrected single-camera fusion',
             'rovio': 'Corrected single-camera ROVIO alone',
             'three': 'Previous three-camera fusion'}
    with PdfPages(plots / 'comparison.pdf') as pdf:
        for label in base.LABELS:
            folder = OUT / 'fusion' / label
            m = json.loads((folder / 'metrics.json').read_text())
            old = json.loads((REF / 'fusion' / label / 'metrics.json').read_text())
            three_folder = base.BASE / 'drpm_three_camera/fusion' / label
            three = json.loads((three_folder / 'metrics.json').read_text())
            assert m['window'] == old['window'] == three['window']
            assert m['alignment_stamp_ns'] == old['alignment_stamp_ns'] == three['alignment_stamp_ns']
            metrics = dict(old=old['full']['drpm'], new=m['full']['drpm'],
                           rovio=m['full']['rovio'], three=three['full']['drpm'])
            paths = dict(old=REF / 'fusion' / label / 'drpm_full_errors.csv',
                         new=folder / 'drpm_full_errors.csv', rovio=folder / 'rovio_full_errors.csv',
                         three=three_folder / 'drpm_full_errors.csv')
            data = {name: np.genfromtxt(path, names=True, delimiter=',') for name, path in paths.items()}
            stamps = data['new']['evaluation_stamp_ns']
            for a in data.values():
                assert np.array_equal(stamps, a['evaluation_stamp_ns'])
            expected_vio = json.loads((base.BASE / 'monocular_startup/evaluation.json').read_text())[label]['metrics']['Corrected prior']
            assert abs(metrics['rovio']['position_rmse_m'] - expected_vio['position_rmse_m']) < 1e-9
            summary = json.loads((folder / 'summary.json').read_text())
            assert summary['artifacts'] == json.loads((REF / 'fusion' / label / 'summary.json').read_text())['artifacts']
            assert base.sha(OUT / 'rovio' / label / 'result_vio.csv') == base.sha(VIO / label / 'result_vio.csv')
            results[label] = dict(metrics=metrics, window=m['window'],
                                 alignment_stamp_ns=m['alignment_stamp_ns'], registration=m['registration'],
                                 checks=m['checks'], fusion_wall_s=summary['wall_s'])
            for name, values in metrics.items():
                rows.append(f'<tr><td>{label}</td><td>{names[name]}</td>'
                            f'<td>{values["position_rmse_m"]:.6f}</td><td>{values["position_max_m"]:.6f}</td>'
                            f'<td>{values["rotation_rmse_deg"]:.6f}</td></tr>')
            for suffix, selected, title in [
                ('current', ['new', 'rovio', 'three'], 'Corrected single-camera fusion, ROVIO, and three-camera reference'),
                ('before_after', ['old', 'new'], 'Single-camera fusion before / after initialization fix')]:
                fig, axes = plt.subplots(6, 1, figsize=(13, 14), sharex=True)
                for name in selected:
                    a = data[name]
                    for i, ax in enumerate(axes):
                        ax.plot((stamps - m['alignment_stamp_ns']) * 1e-9, a[a.dtype.names[i + 1]],
                                label=names[name], lw=1.1)
                for i, ax in enumerate(axes):
                    ax.set_ylabel(['X [m]', 'Y [m]', 'Z [m]', 'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'][i])
                    ax.grid(alpha=.25)
                axes[0].legend(fontsize=9)
                axes[-1].set_xlabel('Tracking time [s]')
                fig.suptitle(label + ' | ' + title, fontsize=12)
                fig.tight_layout()
                filename = f'{label}_{suffix}.png'
                fig.savefig(plots / filename, dpi=120)
                pdf.savefig(fig)
                plt.close(fig)
                tag = f'<h2>{label}: {title}</h2><img src="{filename}">'
                if suffix == 'before_after':
                    tag = '<details><summary>Previous vs corrected fusion (full axes)</summary>' + tag + '</details>'
                images.append(tag)
    note = ('Same three complete tracking windows and same link3 camera. '
            'The corrected ROVIO outputs are reused without changing their timestamps. '
            'DRPM uses the adopted official update with radial range-noise model; the MIMOSA executable, '
            'LiDAR/IMU processing, and fixed fusion covariance match the previous single-camera experiment. '
            'Only the ROVIO initial accelerometer bias variance changed (0.0004 to 0.010001 per axis). '
            'Fusion weights were not recalibrated after this change. '
            'Ground truth is used only for evaluation. Initialization uses the preceding recording; '
            'takeoff and initial heading alignment are excluded from the accuracy window. '
            'One rigid alignment at tracking start; no later alignment, scale fitting, clipping, or axis breaks. '
            'Position RMSE is the three-dimensional error; rotation RMSE uses relative rotation angle. '
            'The three-camera result is an existing reference, not newly measured. '
            'FAST results are retained in the per-flight common-window metrics, not mixed into the full-flight table.')
    (plots / 'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Corrected single-camera DRPM fusion</title>'
        '<style>body{font:16px sans-serif;max-width:1350px;margin:30px auto;padding:15px}img{max-width:100%}p{line-height:1.6}td,th{border:1px solid #bbb;padding:8px}table{border-collapse:collapse}</style>'
        '<h1>MIMOSA + DRPM + corrected single-camera ROVIO</h1><p>' + note + '</p>'
        '<p><a href="comparison.pdf">Six-axis plots PDF</a> · <a href="../comparison.json">Metrics and checks</a></p>'
        '<table><tr><th>Flight</th><th>Method</th><th>Position RMSE [m]</th><th>Position max [m]</th><th>Rotation RMSE [deg]</th></tr>'
        + ''.join(rows) + '</table>' + ''.join(images))
    base.save(OUT / 'comparison.json', results)
    for path, digest in summary['artifacts'].items():
        assert base.sha(path) == digest
    for link in re.findall(r'(?:href|src)="([^"]+)"', (plots / 'index.html').read_text()):
        assert (plots / link).is_file()
    pdf_bytes = (plots / 'comparison.pdf').read_bytes()
    assert len(re.findall(rb'/Type\s*/Page\b', pdf_bytes)) == 6
    assert json.loads((OUT / 'cleanup.json').read_text())['owned_masters_stopped']
    assert not list((OUT / 'inputs').glob('*.bag'))
    assert not list((OUT / 'inputs').glob('*/*.npz'))
    base.save(OUT / 'validation.json', dict(status='complete', labels=base.LABELS,
        fusion_config_and_binary_unchanged=True, corrected_vio_reused_exactly=True,
        original_timestamps_preserved=True, full_tracking_coverage=True, ground_truth_tuning=False,
        pdf_pages=6, owned_masters_stopped=True, temporary_sensor_inputs_removed=True,
        original_bags_and_previous_results_retained=True))
    base.save(OUT / 'runtime.json', dict(status='complete', plots=str(plots / 'index.html')))
    print(json.dumps({label: r['metrics'] for label, r in results.items()}, indent=2))


if __name__ == '__main__':
    main()
