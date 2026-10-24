#!/usr/bin/env python3
"""Report every prespecified additional ZaB flight, with unbroken six-axis plots."""
import argparse
import gzip
import html
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

METHODS = ['pure_lio', 'fusion_fixed', 'fusion_active']
NAMES = ['MIMOSA only', 'MIMOSA + ROVIO fixed', 'MIMOSA + ROVIO gimbal']
FIELDS = ['x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    manifest = json.loads((root/'campaign.json').read_text())
    records = {}
    rows = []
    figures = []
    with PdfPages(root/'comparison.pdf') as pdf:
        for key in manifest['order']:
            entry = manifest['runs'].get(key, {})
            path = root/key
            if not (path/'comparison.json').exists():
                rows.append(f'<tr><td>{html.escape(key)}</td><td colspan="7">{entry.get("status", "pending")}</td></tr>')
                continue
            data = json.loads((path/'comparison.json').read_text())
            assert data['route'] == key
            assert data['seed'] == entry['seed']
            m = data['metrics']
            ratio = m['fusion_active']['position_rmse_m']/m['fusion_fixed']['position_rmse_m']
            record = dict(valid=data['valid'], metrics=m, position_active_fixed_ratio=ratio)
            flight = path/'campaign'/(key+'_active')
            decision_path = flight/'decisions.jsonl'
            if not decision_path.exists():
                decision_path = decision_path.with_suffix('.jsonl.gz')
            opener = gzip.open if decision_path.suffix == '.gz' else open
            with opener(decision_path, 'rt') as stream:
                decisions = [json.loads(line) for line in stream if line.strip()]
            # Full planner lifetime, explicitly distinct from tracking-only errors.
            enc = np.array([d['encoder'] for d in decisions if 'encoder' in d])
            record['planner_lifetime'] = dict(
                decisions=len(decisions),
                reasons={reason: sum(d.get('reason') == reason for d in decisions)
                    for reason in sorted({d.get('reason', '') for d in decisions})},
                encoder_min_deg=np.rad2deg(enc.min(0)).tolist() if len(enc) else [],
                encoder_max_deg=np.rad2deg(enc.max(0)).tolist() if len(enc) else [])
            evaluation = np.genfromtxt(path/'fusion_active_errors.csv', delimiter=',', names=True)
            first, last = evaluation['stamp_ns'][[0, -1]]
            tracking = [d for d in decisions if first <= d['stamp_ns'] <= last]
            if tracking:
                angles = np.rad2deg(np.asarray([d['encoder'] for d in tracking]))
                weakness = np.asarray([d['lidar_weakness'] for d in tracking])
                record['tracking_planner'] = dict(
                    decisions=len(tracking), encoder_min_deg=angles.min(0).tolist(),
                    encoder_max_deg=angles.max(0).tolist(),
                    encoder_median_deg=np.median(angles, axis=0).tolist(),
                    predicted_weakness_max=float(weakness.max()),
                    predicted_weakness_mean_of_max=float(weakness.max(axis=1).mean()),
                    note='Planner diagnostic prediction, not ground-truth observability or an enabled DRPM correction.')
            records[key] = record
            cells = ''.join(f'<td>{m[k][metric]:.4f}</td>'
                for metric in ['position_rmse_m', 'rotation_rmse_deg'] for k in METHODS)
            rows.append(f'<tr><td><a href="{key}/index.html">{key}</a></td>{cells}'
                        f'<td>{100*(1-ratio):+.1f}%</td></tr>')
            fig, axs = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
            for method, name, color in zip(METHODS, NAMES, ['#444444', '#2474b5', '#df6619']):
                errors = np.genfromtxt(path/(method+'_errors.csv'), delimiter=',', names=True)
                for n, ax in enumerate([*axs[:, 0], *axs[:, 1]]):
                    ax.plot(errors['route_t'], errors[FIELDS[n]], label=name, color=color, lw=1)
                    ax.set_ylabel(FIELDS[n])
                    ax.grid(alpha=.3)
            axs[0, 0].legend()
            for ax in axs[-1]:
                ax.set_xlabel('Tracking time [s]')
            fig.suptitle(key+' / same flight / initial rigid alignment, no scale fitting')
            fig.tight_layout()
            png = key+'_six_axes.png'
            fig.savefig(root/png, dpi=130)
            pdf.savefig(fig)
            plt.close(fig)
            figures.append(f'<h2>{key}</h2><img src="{png}">')
    summary = dict(
        complete=manifest['status'] == 'complete' and len(records) == len(manifest['order'])
                 and all(v['valid'] for v in records.values()),
        protocol=manifest['protocol'], selection=manifest['selection'], runs=records,
        wins=sum(r['position_active_fixed_ratio'] < 1 for r in records.values()),
        comparisons=len(records))
    summary['per_route'] = {}
    for route in manifest['routes']:
        paired = [r for key, r in records.items() if manifest['runs'][key]['route'] == route]
        if paired:
            summary['per_route'][route] = dict(
                trials=len(paired),
                wins=sum(r['position_active_fixed_ratio'] < 1 for r in paired),
                mean_position_rmse_m={method: float(np.mean([
                    r['metrics'][method]['position_rmse_m'] for r in paired])) for method in METHODS},
                mean_rotation_rmse_deg={method: float(np.mean([
                    r['metrics'][method]['rotation_rmse_deg'] for r in paired])) for method in METHODS},
                meaning='Arithmetic mean of per-flight RMSE; not a pooled trajectory RMSE.')
    (root/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    page = '''<!doctype html><meta charset="utf-8"><title>ZaB 追加3条件</title>
<style>body{font:16px sans-serif;line-height:1.7;max-width:1300px;margin:30px auto;padding:0 20px}
table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:7px;text-align:right}
td:first-child{text-align:left}img{width:100%}</style>
<h1>ZaB内の追加3条件：固定と偏向の比較</h1>
<p>採用v57を調整せず、直前9経路の比較対象外だった3経路を各2回実行。
run3は約17mの接続通路、run4は約15m・高さ0.6m上昇の接続通路、
run1_tail_branchは下降と折り返しを含む往復経路。過去の全飛行と空間的に重複しないとは主張しない。</p>
<p>UniPilot単眼、DRPM OFF、巡航0.5m/s。同一物理飛行のLiDAR・IMUを3構成で共有。
離陸・初期化を除く全追従期間を評価し、開始位置と姿勢のみ整合、尺度補正なし。
純LIOはROVIOの全拘束を初期化から除外。要求seedを変えた2試行であり、
同じseedでも入力の厳密再現を保証しない。短区間なので長時間ドリフトの一般化検証とは異なる。</p>
<p>RMSEは誤差の二乗平均の平方根。位置は3軸の合計、回転は姿勢の差の回転角。
改善率は固定に対する偏向の位置RMSE低減率（負値は悪化）。結果に基づく除外や設定調整なし。</p>'''
    page += f'<p>全比較完了・検査合格：{summary["complete"]}。位置改善：{summary["wins"]}/{len(records)}組。</p>'
    if (root/'route_context.png').exists():
        page += '<p>灰色はworld衝突メッシュの面中心、青は計画経路、緑は開始点。視覚特徴点の密度ではありません。</p><img src="route_context.png">'
        page += '<p>メッシュの3断面で左右の壁までの距離を合計すると、run3は4.64～5.19m、run4は4.59～4.74m。branchは側方に15～19m開いた場所で下降・折り返します。これは指定方向へのレイ計測で、通路の最小幅を保証する値ではありません。<a href="cross_sections.json">断面計測</a> / <a href="world_context.json">モデル出典</a></p>'
    page += '<p><a href="comparison.pdf">全6軸PDF</a> / <a href="summary.json">数値</a> / <a href="campaign.json">事前条件・実行記録</a></p>'
    page += '<table><tr><th rowspan="2">経路・seed</th><th colspan="3">位置RMSE [m]</th><th colspan="3">回転RMSE [deg]</th><th rowspan="2">位置改善率</th></tr><tr>'+''.join('<th>'+n+'</th>' for n in ['単体', '固定融合', '偏向融合']*2)+'</tr>'+''.join(rows)+'</table>'
    page += ''.join(figures)
    (root/'index.html').write_text(page)
    print(json.dumps({k: summary[k] for k in ['complete', 'wins', 'comparisons']}))


if __name__ == '__main__':
    main()
