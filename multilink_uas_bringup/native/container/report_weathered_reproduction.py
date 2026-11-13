#!/usr/bin/env python3
"""Compare all three fresh weathered24201 flights with the recorded old results."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'active_perception'))
from compact_record_io import record_path
from report_practical_comparison import read_pose
from practical_comparison_metrics import evaluate
from report_estimator_validation import evaluate as historical_evaluate
from report_routes import poses, interp
from window_metrics import local_displacement_error


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    protocol = json.loads((root / 'comparison_protocol.json').read_text())
    design = json.loads(root.with_name(root.name + '.design.json').read_text())
    assert protocol['seeds'] == design['seeds']
    assert protocol['order'] == [['unipilot', s] for s in design['seeds']]
    out = root / 'report'
    out.mkdir(exist_ok=True)
    rows = []
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    for i, seed in enumerate(design['seeds']):
        trial = root / f'unipilot_seed{seed}'
        row = dict(seed=seed, historical_position_rmse_m=design['historical_fused_position_rmse_m'][i])
        rows.append(row)
        try:
            route = np.genfromtxt(record_path(trial / 'route_tracking.csv'), delimiter=',', names=True)
            moving = route[route['route_t'] > 0]
            start = float(moving['sim_t'][0] - moving['route_t'][0])
            truth = read_pose(trial / 'truth.csv')
            row.update(tracking_start_s=start, estimators={})
            for estimator in ('mimosa', 'rovio'):
                t, errors, _, metrics = evaluate(truth, read_pose(trial / f'{estimator}.csv'), start, 182.)
                old, *_ = historical_evaluate(trial, estimator)
                metrics['difference_from_historical_evaluator_m'] = metrics['position_rmse_m'] - old['position_rmse_m']
                gt, est = poses(trial/'truth.csv'), poses(trial/f'{estimator}.csv')
                local = []
                for lo in np.arange(10., 161., 10.):
                    stamps = (start + np.array([lo, lo+10.])) * 1e9
                    gp, gq = interp(gt, stamps)
                    ep, eq = interp(est, stamps)
                    delta = local_displacement_error(gp[0], gp[1], gq[0].as_matrix(), ep[0], ep[1], eq[0].as_matrix())
                    local.append(float(delta[0]))
                metrics['historical_window_axial_10s_rms_m'] = float(np.sqrt(np.mean(np.square(local))))
                row['estimators'][estimator] = metrics
                axes[i, 0].plot(t, np.linalg.norm(errors, axis=1), label=estimator)
            enc = np.loadtxt(record_path(trial/'encoders.csv'), delimiter=',', skiprows=1, ndmin=2)
            enc = enc[(enc[:, 0] >= start) & (enc[:, 0] <= start+182)]
            row['actual_view_span_deg'] = np.rad2deg(np.ptp(enc[:, 1:3], axis=0)).tolist()
            for j, label in enumerate(('yaw', 'pitch')):
                axes[i, 1].plot(enc[::10, 0]-start, np.rad2deg(enc[::10, j+1]), label=label)
            row['full_interval_available'] = True
        except (OSError, ValueError, KeyError, IndexError, AssertionError) as error:
            row.update(full_interval_available=False, error=str(error))
        axes[i, 0].set_ylabel(f'{seed}\nPosition error [m]')
        axes[i, 1].set_ylabel(f'{seed}\nCamera angle [deg]')
        for ax in axes[i]:
            ax.grid(alpha=.3)
            if ax.lines:
                ax.legend()
    for ax in axes[-1]:
        ax.set_xlabel('Flight time [s]')
    fig.tight_layout()
    fig.savefig(out/'trajectory_and_view.png', dpi=130)
    plt.close(fig)
    complete = all(r['full_interval_available'] for r in rows)
    previous = np.array(design['historical_fused_position_rmse_m'])
    values = [r.get('estimators', {}).get('mimosa', {}).get('position_rmse_m') if r['full_interval_available'] else None for r in rows]
    result = dict(design=design, trials=rows, all_full_intervals_available=complete,
                  historical_mean_m=float(previous.mean()), historical_sample_sd_m=float(previous.std(ddof=1)),
                  current_values_m=values, current_mean_m=float(np.mean(values)) if complete else None,
                  current_sample_sd_m=float(np.std(values, ddof=1)) if complete else None,
                  mean_change_percent=float((np.mean(values)/previous.mean()-1)*100) if complete else None,
                  historical_raw_records_available=False,
                  scope='Descriptive same-environment reproduction; not statistical equivalence or a new five-fixed-camera comparison.')
    audit_path = root/'final_audit.json'
    if audit_path.exists():
        result['acquisition_audit'] = json.loads(audit_path.read_text())
    (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.scatter(np.zeros(3)+np.linspace(-.08, .08, 3), previous, label='Historical runs', color='#555555')
    ax.plot([-.15, .15], [previous.mean()]*2, color='#555555', linewidth=3)
    valid = [(i, v) for i, v in enumerate(values) if v is not None]
    ax.scatter([1+(i-1)*.08 for i, _ in valid], [v for _, v in valid], label='Current runs', color='#2474b5')
    if complete:
        ax.plot([.85, 1.15], [np.mean(values)]*2, color='#2474b5', linewidth=3)
    ax.set_xticks([0, 1], ['2026-10-24', '2026-11-12'])
    ax.set_ylabel('Fused position RMSE [m]')
    ax.set_title('UniPilot gimbal | weathered24201 | 90 m | 3 runs')
    ax.grid(axis='y', alpha=.3)
    fig.tight_layout()
    fig.savefig(out/'historical_comparison.png', dpi=140)
    plt.close(fig)
    fmt = lambda value: '未完了' if value is None else f'{value:.3f}'
    page = '''<!doctype html><meta charset="utf-8"><title>UniPilot：以前の環境での再検証</title>
<style>body{font:16px sans-serif;line-height:1.8;max-width:1100px;margin:32px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}img{max-width:100%}</style>
<h1>UniPilotジンバル：以前の汚れ・ひびのトンネルで再検証</h1>
<p>模様配置24201、長さ100 m・幅5 m・高さ4.5 m。高さ2 m、90 mを毎秒0.5 mで直進。以前と同じ3つの乱数初期値、v10視線選択、DRPM OFF。離陸・準備運動は採点から除外。全行程を10 Hzで採点し、開始時の位置・姿勢だけを合わせる。距離尺度の補正なし。</p>
<p>過去の数値は作業記録の保存値から引用。過去の生データはこのPCにない。今回はLinuxで再構築した実行環境を使用し、通信ライブラリと補助推定器数に違いがある。6カメラの描画と正面参照は維持。過去との完全な入力一致や統計的同等性は主張しない。</p>
<table><tr><th>乱数初期値</th><th>以前の位置誤差 [m]</th><th>今回の位置誤差 [m]</th></tr>'''
    for row, value in zip(rows, values):
        page += f'<tr><td>{row["seed"]}</td><td>{fmt(row["historical_position_rmse_m"])}</td><td>{fmt(value)}</td></tr>'
    page += f'<tr><th>3回平均</th><td>{fmt(result["historical_mean_m"])}</td><td>{fmt(result["current_mean_m"])}</td></tr></table>'
    page += '<p>誤差が小さいほど良い。全試行を保持し、未完了があれば3回平均を出さない。以前との採点方法の照合、補助の10秒移動誤差、取得・終了検査は数値記録を参照。</p>'
    page += '<p><a href="summary.json">全数値・条件</a> / <a href="../final_audit.json">取得・動作・終了検査</a></p><img src="historical_comparison.png"><img src="trajectory_and_view.png">'
    (out/'index.html').write_text(page)
    print(json.dumps({k:result[k] for k in ('historical_mean_m', 'current_values_m', 'current_mean_m', 'current_sample_sd_m', 'mean_change_percent', 'all_full_intervals_available')}, indent=2))


if __name__ == '__main__':
    main()
