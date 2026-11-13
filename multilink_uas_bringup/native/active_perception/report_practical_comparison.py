#!/usr/bin/env python3
"""Report every requested trial on a full shared 10 Hz evaluation interval."""
import argparse
import html
import json
from pathlib import Path
import numpy as np
from practical_comparison_metrics import evaluate
from compact_record_io import record_path


def read_pose(path):
    values = np.loadtxt(record_path(path), delimiter=',', skiprows=1, ndmin=2)[:, :8]
    values[:, 0] *= 1e-9
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    protocol = json.loads((root / 'comparison_protocol.json').read_text())
    rows, curves = {}, {}
    for arm, seed in protocol['order']:
        name = f'{arm}_seed{seed}'
        directory = root / name
        record = rows[name] = dict(arm=arm, seed=seed, estimators={})
        try:
            if arm == 'unipilot':
                route = np.genfromtxt(record_path(directory / 'route_tracking.csv'), delimiter=',', names=True)
                moving = route[route['route_t'] > 0]
                start = float(moving['sim_t'][0] - moving['route_t'][0])
            else:
                events = json.loads((directory / 'flight_events.json').read_text())
                start = next(e['time'] for e in events if e['stage'] == 'active')
            record['tracking_start_s'] = start
            truth = read_pose(directory / 'truth.csv')
            for estimator in ('mimosa', 'rovio'):
                t, error, angular, metrics = evaluate(truth, read_pose(directory / (estimator + '.csv')),
                                                      start, protocol['distance_m'] / protocol['speed_mps'] + 2)
                record['estimators'][estimator] = metrics
                curves[name, estimator] = (t, error, angular)
            record['full_interval_available'] = True
            for audit in ('validation', 'completion', 'runtime', 'route_status', 'input_audit', 'encoder_audit'):
                path = directory / (audit + '.json')
                if path.exists():
                    record[audit] = json.loads(path.read_text())
        except (OSError, ValueError, KeyError, IndexError, StopIteration) as error:
            record.update(full_interval_available=False, error=str(error))
    arms = list(dict.fromkeys(arm for arm, _ in protocol['order']))
    means = {}
    for arm in arms:
        trials = [r for r in rows.values() if r['arm'] == arm]
        means[arm] = {estimator: float(np.mean([r['estimators'][estimator]['position_rmse_m'] for r in trials]))
                      if all(r['full_interval_available'] for r in trials) else None
                      for estimator in ('mimosa', 'rovio')}
    audit_path = root / 'final_audit.json'
    audit = json.loads(audit_path.read_text()) if audit_path.exists() else None
    result = dict(audit=audit, protocol=protocol, results=rows, mean_position_rmse_m=means,
                  all_full_intervals_available=all(r['full_interval_available'] for r in rows.values()),
                  interpretation='Practical adopted configurations. All trials retained; acquisition and motion failures remain explicit. Independent sensor realizations, not identical raw input.')
    out = root / 'report'
    out.mkdir(exist_ok=True)
    (out / 'summary.json').write_text(json.dumps(result, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    images = []
    for seed in protocol['seeds']:
        for estimator in ('mimosa', 'rovio'):
            fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
            for arm in arms:
                key = (f'{arm}_seed{seed}', estimator)
                if key not in curves:
                    continue
                t, error, angular = curves[key]
                for i in range(3):
                    axes[i, 0].plot(t, error[:, i], label=arm)
                    axes[i, 1].plot(t, angular[:, i], label=arm)
            for i in range(3):
                axes[i, 0].set_ylabel('XYZ'[i] + ' error [m]')
                axes[i, 1].set_ylabel('XYZ'[i] + ' rotvec error [deg]')
            for ax in axes.flat:
                ax.grid(alpha=.3)
            if axes[0, 0].lines:
                axes[0, 0].legend()
            for ax in axes[-1]:
                ax.set_xlabel('Tracking time [s]')
            fig.suptitle(f'{estimator.upper()} | seed {seed} | adopted configurations')
            fig.tight_layout()
            filename = f'{seed}_{estimator}.png'
            fig.savefig(out / filename, dpi=140)
            plt.close(fig)
            images.append(filename)
    page = '<!doctype html><meta charset="utf-8"><title>DRAGON比較</title><style>body{font:16px sans-serif;max-width:1200px;margin:30px auto;line-height:1.7}td,th{padding:8px;border:1px solid #aaa}table{border-collapse:collapse}img{width:100%}</style>'
    page += '<h1>DRAGON：採用設定によるジンバル版と関節版の比較</h1><p>新規実行のUniPilotを参照。全区間を同じ10 Hzで評価し、最初の位置と姿勢だけを合わせる。距離の尺度補正なし。設定全体の比較であり、機構だけを変えた比較ではない。</p>'
    page += '<p>融合（MIMOSA）は画像・慣性・距離センサーを組み合わせた最終推定。視覚（ROVIO）は画像・慣性による推定。</p>'
    page += '<p>Linux環境への移植時に、描画ライブラリの版の不一致、慣性データ受信待ち容量不足、一時停止中にも進む制御時計を修正。両方式の10m予備試験に合格してから本比較を開始した。失敗した予備試験は別フォルダに保持している。</p>'
    page += '<p>専用GPUコンテナで実行。過去環境と同じバイナリであるとは主張しない。実行条件は数値記録に保存。</p>'
    page += '<p>位置RMSEは、各時刻の位置ずれを二乗して平均し平方根を取った値。小さいほど良い。表の平均は3試行のRMSEの算術平均。</p>'
    if audit is not None:
        page += '<p>終了時検査：' + ('全項目合格' if audit['passed'] else '不合格項目あり。精度と分けて要確認') + '。<a href="../final_audit.json">取得・動作・終了・固定条件の検査</a></p>'
    page += '<table><tr><th>方式</th><th>融合位置RMSE平均 [m]</th><th>視覚位置RMSE平均 [m]</th></tr>'
    for arm, values in means.items():
        page += '<tr><td>' + html.escape(arm) + '</td>' + ''.join('<td>' + ('—' if values[k] is None else f'{values[k]:.3f}') + '</td>' for k in ('mimosa', 'rovio')) + '</tr>'
    page += '</table>'
    page += '<p>ジンバル：採用済みv10の視野選択＋回転対応の視覚推定。関節：関節用の視野探索＋カメラの並進も扱う視覚推定。両者の融合設定は同一。UniPilotは従来の融合設定で新規実行。</p>'
    page += f'<p>単一の合成トンネル、{protocol["distance_m"]:g}m、0.5m/s、各3試行。要求する乱数seedが同じでも、機体と動作が異なるため同一のセンサー入力列ではない。過去実験との数値を混ぜていない。</p>'
    page += '<p><a href="summary.json">全数値・取得検査・条件</a></p><table><tr><th>試行</th><th>融合位置RMSE [m]</th><th>視覚位置RMSE [m]</th><th>全区間取得</th></tr>'
    for name, row in rows.items():
        page += '<tr><td>' + html.escape(name) + '</td>'
        for estimator in ('mimosa', 'rovio'):
            value = row['estimators'].get(estimator, {}).get('position_rmse_m')
            page += '<td>' + ('—' if value is None else f'{value:.3f}') + '</td>'
        page += '<td>' + str(row['full_interval_available']) + '</td></tr>'
    page += '</table><p>未完了試行がある方式の平均は表示しない。取得検査の不合格は数値ファイルに保持し、精度の結果と分けて確認する。</p>'
    page += ''.join(f'<img src="{name}">' for name in images)
    (out / 'index.html').write_text(page)
    print(json.dumps(means, indent=2))


if __name__ == '__main__':
    main()
