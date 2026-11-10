"""Score all frozen validation flights and verify that LiDAR remains in use."""
import argparse
import copy
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from report_matched_dragon import NATIVE, flight


def registration(path, start, end):
    with path.open() as stream:
        rows = [r for r in csv.DictReader(stream)
                if start <= int(r['stamp_ns']) * 1e-9 <= end]
    ok = [r for r in rows if r['status'] == 'ok']
    result = dict(scans=len(rows), accepted=len(ok),
                  accepted_fraction=len(ok) / len(rows) if rows else 0)
    if ok:
        probs = np.array([[float(r[f'p{i}']) for i in range(2, 6)] for r in ok])
        result.update(probability_medians=np.median(probs, axis=0).tolist(),
                      min_retained_modes=int((probs > .5).sum(axis=1).min()),
                      median_correspondences=float(np.median([float(r['correspondences']) for r in ok])))
    result['passed'] = result['accepted_fraction'] >= .95 and result.get('min_retained_modes', 0) >= 2
    return result


def differences(a, b, prefix=''):
    if isinstance(a, dict) and isinstance(b, dict):
        out = {}
        for key in sorted(a.keys() | b.keys()):
            out.update(differences(a.get(key), b.get(key), prefix + '.' + key if prefix else key))
        return out
    return {} if a == b else {prefix: [a, b]}


def report(root, final=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    protocol = json.loads((root / ('validation_view_protocol.json' if final else 'validation20_protocol.json')).read_text())
    prefix = 'view_seed' if final else 'validated20_seed'
    library_dir = 'mimosa-batched-drpm-support-view' if final else 'mimosa-batched-drpm-support'
    oldroot = NATIVE / 'logs/dragon-unipilot-matched-20261110'
    out = root / ('report' if final else 'report_drpm20')
    out.mkdir(exist_ok=True)
    results, curves, audits = {}, {}, {}
    for seed in protocol['seeds']:
        key = f'layout24301_lidar_conditioned_seed{seed}'
        paths = {
            'UniPilot': NATIVE / 'logs/information-holdout-20261026' / key / 'campaign' / (key + '_active'),
            'DRAGON before': oldroot / f'active_seed{seed}',
            'DRAGON corrected': root / f'{prefix}{seed}',
        }
        for label, path in paths.items():
            k = f'{label} {seed}'
            results[k], curves[k] = flight(path, 'unipilot' if label == 'UniPilot' else 'dragon')
        new, old = paths['DRAGON corrected'], paths['DRAGON before']
        row = results[f'DRAGON corrected {seed}']
        reg = registration(new / 'mimosa/drpm_registration.csv', row['start_s'], row['end_s'])
        before = yaml.safe_load((old / 'mimosa.yaml').read_text())
        after = yaml.safe_load((new / 'mimosa.yaml').read_text())
        for c in (before, after):
            c.pop('logs_directory', None)
        delta = differences(before, after)
        expected = copy.deepcopy(before)
        expected['lidar']['geometric']['scan_to_map'].update(
            drpm_enabled=True, drpm_registration_mode=1, drpm_noise_model=0, num_corres_points=20)
        completion = json.loads((new / 'completion.json').read_text())
        valid = json.loads((new / 'validation.json').read_text())
        manifest = json.loads((new / 'protocol.json').read_text())
        loaded = json.loads((new / 'loaded_fusion_library.json').read_text())
        owners = json.loads((new / 'owners.json').read_text())
        checks = dict(acquisition_passed=valid['passed'], lidar_retained=reg['passed'],
                      intended_configuration_only=after == expected,
                      same_planner=(old / 'planner.yaml').read_bytes() == (new / 'planner.yaml').read_bytes(),
                      same_intrinsics=(old / 'cam0.yaml').read_bytes() == (new / 'cam0.yaml').read_bytes(),
                      same_rovio=(old / 'rovio.info').read_bytes() == (new / 'rovio.info').read_bytes(),
                      all_exited_zero=all(v == 0 for v in completion['processes'].values()),
                      owned_processes_stopped=not any(Path('/proc', str(pid)).exists() for pid in [*owners.values(), loaded['pid']]),
                      actual_library=bool(loaded['maps']) and all('/' + library_dir + '/libmimosa_lib.so' in line for line in loaded['maps']),
                      frozen_runner=next(v for p, v in manifest['sha256'].items() if p.endswith('/run_articulated_dragon.py')) == next(v for p, v in protocol['sha256'].items() if p.endswith('/run_articulated_dragon.py')),
                      frozen_library=next(v for p, v in manifest['sha256'].items() if p.endswith('/' + library_dir + '/libmimosa_lib.so')) == next(v for p, v in protocol['sha256'].items() if p.endswith('/libmimosa_lib.so')))
        audits[str(seed)] = dict(checks=checks, passed=all(checks.values()), registration=reg,
                                configuration_changes=delta, acquisition=valid)
    aggregates = {}
    for group in ('UniPilot', 'DRAGON before', 'DRAGON corrected'):
        selected = [r for k, r in results.items() if k.startswith(group + ' ')]
        aggregates[group] = {m: dict(mean_position_rmse_m=float(np.mean([r[m]['position_rmse_m'] for r in selected])),
                                    maximum_position_rmse_m=max(r[m]['position_rmse_m'] for r in selected))
                             for m in ('rovio', 'mimosa')}
    data = dict(results=results, aggregates=aggregates, audits=audits,
                evaluation='All three 90m flights retained; first tracking SE3 alignment, no scale fit. Independent simulations, not identical raw input.',
                method_change='DRPM four-DoF official update; isotropic nominal sigma 0.03m; plane support 20. Physical sensor noise, VIO weights, IMU settings and planner algorithm unchanged.' + (' Full six-DoF measured geometry to planner, without the factor variable mask.' if final else ''),
                scope='This tests correction of DRAGON fusion failure, not a fresh active-vs-fixed experiment.',
                rejected='N=5 isotropic candidate rejected: no accepted LiDAR corrections during tracking. Sensor-calibrated IMU plus zero numerical cutoffs also rejected after an independent closed-loop divergence. Both interrupted trials retained.')
    (out / 'summary.json').write_text(json.dumps(data, indent=2) + '\n')
    with PdfPages(out / 'comparison.pdf') as pdf:
        for seed in protocol['seeds']:
            fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
            for label in ('UniPilot', 'DRAGON before', 'DRAGON corrected'):
                color = {'UniPilot': 'C0', 'DRAGON before': 'C1', 'DRAGON corrected': 'C2'}[label]
                for ax, method in zip(axes, ('rovio', 'mimosa')):
                    t, err = curves[f'{label} {seed}'][method]
                    ax.plot(t, np.linalg.norm(err, axis=1), label=label, color=color)
                if label != 'DRAGON before':
                    t, err = curves[f'{label} {seed}']['mimosa']
                    axes[2].plot(t, np.linalg.norm(err, axis=1), label=label, color=color)
            for ax, title in zip(axes, ('ROVIO', 'ROVIO + MIMOSA')):
                ax.set(title=title, ylabel='Position error [m]'); ax.grid(alpha=.3); ax.legend()
            axes[2].set(title='Fusion: reference vs corrected (full range)', ylabel='Position error [m]')
            axes[2].grid(alpha=.3); axes[2].legend()
            axes[-1].set_xlabel('Tracking elapsed [s]')
            fig.suptitle(f'90m / 0.5m/s / seed {seed}')
            fig.text(.5, .005, 'Corrected: DRPM 4DoF / isotropic nominal 3cm / 20-point planes. Sensor, VIO and IMU settings unchanged.', ha='center', fontsize=8)
            fig.tight_layout(rect=(0, .02, 1, 1)); fig.savefig(out / f'{seed}.png', dpi=130); pdf.savefig(fig); plt.close(fig)
    html = '''<!doctype html><meta charset="utf-8"><title>DRAGON融合の修正検証</title>
<style>body{font:16px sans-serif;max-width:1100px;margin:32px auto;line-height:1.7}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}img{max-width:100%}</style>
<h1>DRAGON＋ジンバル：融合の修正検証</h1>
<p>90m、0.5m/s、模様24301、3回すべて掲載。追従開始姿勢のみで整列し、尺度補正なし。過去のUniPilot・DRAGONとは別飛行。</p>
<p>LiDARの誤った進行方向拘束を抑えるため、DRPMの4自由度更新、等方的な公称3cmの不確かさ、20点の平面推定を使用。センサ設定、VIO重み、視野選択処理は変更していない。</p>
<p>これは融合不良の修正試験であり、修正後の固定対偏向の優劣を新しく検証する試験ではない。</p>
<table><tr><th>条件</th><th>ROVIO位置RMSE [m]</th><th>融合位置RMSE [m]</th></tr>'''
    for label, r in results.items():
        html += f'<tr><td>{label}</td><td>{r["rovio"]["position_rmse_m"]:.3f}</td><td>{r["mimosa"]["position_rmse_m"]:.3f}</td></tr>'
    html += '</table>'
    html += '<h2>3試行の平均位置RMSE</h2><table><tr><th>条件</th><th>ROVIO [m]</th><th>融合 [m]</th></tr>'
    for label, r in aggregates.items():
        html += f'<tr><td>{label}</td><td>{r["rovio"]["mean_position_rmse_m"]:.3f}</td><td>{r["mimosa"]["mean_position_rmse_m"]:.3f}</td></tr>'
    html += '</table>'
    if final:
        html += '<p>追加修正：視野選択には全6方向のLiDAR幾何を渡す。融合で更新しない傾き2方向を、観測不能と誤認させない。IMU設定・視覚の重み・物理的なセンサノイズは変更なし。</p>'
    html += '<h2>LiDARを維持したことの検査</h2><ul>'
    for seed, a in audits.items():
        r = a['registration']
        html += f'<li>{seed}: {r["accepted"]}/{r["scans"]}回のLiDAR補正成立、常時{r["min_retained_modes"]}方向以上を保持。全検査: {a["passed"]}</li>'
    html += '</ul><p>初期の5点候補は飛行中のLiDAR補正が0回で、実質的にVIOのみとなるため不採用。今回の3飛行に含めていない。診断データと中断記録は保存。</p>'
    html += '<p>IMUの不確かさを小さくし更新省略も解除した候補は、同一入力対照では改善したが、独立飛行で融合が発散したため不採用。中断した全データと棄却理由を保存。今回の3本にはこの変更を含めていない。</p>'
    for seed in protocol['seeds']:
        html += f'<img src="{seed}.png">'
    html += '<p><a href="comparison.pdf">PDF</a> / <a href="summary.json">全数値・設定差・検査</a></p>'
    if final:
        html += '<p><a href="../report_drpm20/index.html">第1修正の全3試行</a> / <a href="../view90/diagnostic_report.json">同一入力による情報渡し・IMU設定の対照</a></p>'
    (out / 'index.html').write_text(html)
    print(json.dumps(dict(aggregates=aggregates, checks={k: a['checks'] for k, a in audits.items()}), indent=2))
    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('root', type=Path)
    parser.add_argument('--final', action='store_true')
    args = parser.parse_args(); report(args.root, args.final)
