#!/usr/bin/env python3
"""Audit and plot independent five-direction monocular comparisons."""
import argparse
import json
import re
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from fixed_view_array import VIEWS
from report_estimator_validation import evaluate
from report_fusion_sidecar import ordered, coincident_input_signature
from report_routes import load

ORDER = ['yaw_m50', 'front', 'yaw_p50', 'pitch_p90', 'pitch_m90', 'active']
LABEL = dict(yaw_m50='(-50, 0)', front='(0, 0)', yaw_p50='(+50, 0)',
             pitch_p90='(0, +90) down', pitch_m90='(0, -90) up', active='Active gimbal')
COLORS = ['#8a4bb0', '#2474b5', '#3d954b', '#b58a10', '#c64d72', '#111111']


def plot(root, key, values, pdf=None):
    fig, axs = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    for name, color in zip(ORDER, COLORS):
        grid, errors = values[name]
        for i, ax in enumerate([*axs[:, 0], *axs[:, 1]]):
            ax.plot(grid, errors[:, i], label=LABEL[name], color=color, lw=1)
            ax.set_ylabel(['x [m]', 'y [m]', 'z [m]', 'roll [deg]', 'pitch [deg]', 'yaw [deg]'][i])
            ax.grid(alpha=.3)
    axs[0, 0].legend(ncol=2, fontsize=9)
    for ax in axs[-1]:
        ax.set_xlabel('Tracking time [s]')
    fig.suptitle(key+' / six independent monocular estimators, same physical flight')
    fig.tight_layout()
    fig.savefig(root/'six_axes.png', dpi=130)
    fig.savefig(root/'six_axes.pdf')
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)


def trial(root):
    paired = json.loads((root/'comparison.json').read_text())
    key = paired['route']
    flight = root/'campaign'/(key+'_active')
    runtime = json.loads((flight/'runtime.json').read_text())
    audit = json.loads((flight/'array_input_audit.json').read_text())
    paths = dict(front=flight/'fixed', active=flight)
    paths.update({name: flight/'views'/name for name in VIEWS})
    checks = dict(reference_paired_audit=paired['valid'],
                  all_processes_zero=all(v == 0 for v in runtime['processes'].values()),
                  route_complete=runtime['status'] == 'route_complete')
    metrics, plots = {}, {}
    reference, _ = ordered(flight/'fixed/mimosa/ordered_inputs.csv', 0, 2**63-1)
    reference_config = yaml.safe_load((flight/'fixed/mimosa.yaml').read_text())
    reference_config.pop('logs_directory')
    base_info = (flight/'fixed/rovio.info').read_text()
    normalize = lambda text: re.sub(r'(qCM_[xyzw]\s+)[^;\s]+;', r'\1POSE;', text)
    for name in ORDER:
        directory = paths[name]
        metrics[name] = {}
        for estimator in ['mimosa', 'rovio']:
            metric, grid, errors, stamps = evaluate(directory, estimator)
            metrics[name][estimator] = metric
            checks[name+'_'+estimator+'_continuous'] = metric['max_output_gap_s'] <= (.100001 if estimator == 'mimosa' else .050001)
            if estimator == 'mimosa':
                plots[name] = (grid, errors)
                np.savetxt(root/(name+'_errors.csv'), np.column_stack([grid, stamps, errors]),
                           delimiter=',', header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',
                           comments='', fmt=['%.6f', '%.0f']+['%.12g']*6)
        rows, event_audit = ordered(directory/'mimosa/ordered_inputs.csv', 0, 2**63-1)
        # Compare the complete tracking window, including cumulative IMU prefixes
        # since initialization. Terminal extra scans remain separately recorded.
        end = int(stamps[-1])
        ref_cut = [r for r in reference if int(r['stamp_ns']) <= end]
        view_cut = [r for r in rows if int(r['stamp_ns']) <= end]
        checks[name+'_same_event_and_imu_inputs'] = coincident_input_signature(ref_cut) == coincident_input_signature(view_cut)
        checks[name+'_ordered_imu'] = all(event_audit[k] for k in ['chronological', 'imu_monotonic', 'imu_brackets_events'])
        config = yaml.safe_load((directory/'mimosa.yaml').read_text())
        config.pop('logs_directory')
        checks[name+'_same_fusion_parameters'] = config == reference_config
        checks[name+'_same_rovio_except_extrinsic'] = normalize((directory/'rovio.info').read_text()) == normalize(base_info)
        checks[name+'_same_camera_intrinsics'] = (directory/'cam0.yaml').read_bytes() == (flight/'fixed/cam0.yaml').read_bytes()
        if name != 'active':
            a = audit[name]
            checks[name+'_constant_correct_extrinsic'] = a['rotation_error_max_rad'] < 1e-10 and a['position_error_max_m'] < 1e-12
            # The independent auditor may attach after the first messages; only
            # require coverage of every exposure in the complete evaluation window.
            frames = np.asarray(a['image_stamps'], dtype=np.int64)
            extrinsics = set(a['extrinsic_stamps'])
            lo = int(np.ceil(stamps[0]/50000000)*50000000)
            hi = int(np.floor(stamps[-1]/50000000)*50000000)
            expected = np.arange(lo, hi+1, 50000000, dtype=np.int64)
            checks[name+'_every_tracking_image_observed'] = np.array_equal(frames[(frames >= lo)&(frames <= hi)], expected)
            checks[name+'_every_tracking_extrinsic_observed'] = all(int(t) in extrinsics for t in expected)
            output = load(directory/'rovio.csv')['stamp_ns']
            checks[name+'_rovio_outputs_every_exposure'] = np.array_equal(output[(output >= lo)&(output <= hi)], expected)
    result = dict(valid=all(checks.values()), checks=checks, metrics=metrics,
                  pure_lio=paired['metrics']['pure_lio'], seed=paired['seed'], route=key,
                  meaning='Independent monocular estimators, no cross-camera fusion. '
                          'Same flight and all LiDAR/IMU events through evaluation end. '
                          'Initial rigid alignment, no scale adjustment. '
                          'Fixed cameras see their own direction during initialization.')
    decisions = [json.loads(line) for line in (flight/'decisions.jsonl').read_text().splitlines() if line.strip()]
    active_errors = np.genfromtxt(root/'active_errors.csv', delimiter=',', names=True)
    first, last = active_errors['stamp_ns'][[0, -1]]
    angles = np.asarray([d['encoder'] for d in decisions if first <= d['stamp_ns'] <= last])
    result['active_tracking_angles_deg'] = dict(min=np.rad2deg(angles.min(0)).tolist(),
                                                max=np.rad2deg(angles.max(0)).tolist())
    (root/'array_comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    plot(root, key, plots)
    print(json.dumps(dict(valid=result['valid'], failed=[k for k, v in checks.items() if not v],
                         position_m={n: metrics[n]['mimosa']['position_rmse_m'] for n in ORDER})), flush=True)
    return result


def campaign(root):
    manifest = json.loads((root/'campaign.json').read_text())
    records = {}
    rows = []
    figures = []
    with PdfPages(root/'comparison.pdf') as pdf:
        for key in manifest['order']:
            path = root/key
            if not (path/'array_comparison.json').exists():
                continue
            data = json.loads((path/'array_comparison.json').read_text())
            records[key] = data
            rows.append('<tr><td>'+key+'</td>'+''.join(
                f'<td>{data["metrics"][n]["mimosa"]["position_rmse_m"]:.4f}</td>' for n in ORDER)+'</tr>')
            values = {}
            for name in ORDER:
                e = np.genfromtxt(path/(name+'_errors.csv'), delimiter=',', names=True)
                values[name] = (e['route_t'], np.column_stack([e[k] for k in
                    ['x_m', 'y_m', 'z_m', 'roll_deg', 'pitch_deg', 'yaw_deg']]))
            plot(path, key, values, pdf)
            figures.append(f'<h2>{key}</h2><img src="{key}/six_axes.png">')
        # All raw per-trial metrics retained even if a direction performs poorly.
    means = {}
    for route in manifest['routes']:
        subset = [r for k, r in records.items() if k.startswith(route+'_seed')]
        if subset:
            means[route] = {name: float(np.mean([r['metrics'][name]['mimosa']['position_rmse_m'] for r in subset]))
                            for name in ORDER}
    summary = dict(complete=manifest['status'] == 'complete' and len(records) == len(manifest['order'])
                   and all(r['valid'] for r in records.values()),
                   per_route_mean_position_rmse_m=means, runs=records,
                   protocol=manifest['protocol'],
                   warning='Best direction chosen on these evaluation results is retrospective, not a deployable preselected baseline.')
    (root/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    text = '''<!doctype html><meta charset="utf-8"><title>ZaB 固定5方向比較</title>
<style>body{font:16px sans-serif;line-height:1.7;max-width:1300px;margin:30px auto;padding:0 20px}
table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:7px}img{width:100%}</style>
<h1>固定5方向と動的偏向の同時比較</h1>
<p>3経路×2回。5方向の固定カメラと偏向カメラを同じ飛行で描画し、それぞれ独立した単眼ROVIO＋MIMOSAで処理。
複数カメラ間の融合なし。位置・画質・フレームレート・IMU・LiDARは共通。角度以外の推定器設定は同一、採用v57・DRPM OFF。
固定方向は初期化時からその方向を観測します。全方向の処理完了を待って時刻を進め、品質を落とす間引きは行いません。</p>
<p>pitch +90°は真下、−90°は真上。カメラ本体の重複形状・質量は追加せず、既存機体による遮蔽は残しています。
飛行は前回とは新しく取得したものです。前回の二眼構成と今回の六眼構成で、同じseedでも画像・IMUノイズの厳密再現は仮定しません。</p>
<p>位置RMSE[m]は誤差の二乗平均平方根。追従全区間を評価、初回位置姿勢整合のみ・尺度補正なし。
この評価結果から最良固定方向を選ぶのは事後比較であり、事前選択した固定方向の性能とは区別します。</p>'''
    text += f'<p>全比較合格：{summary["complete"]}</p><p><a href="comparison.pdf">六軸PDF</a> / <a href="summary.json">位置・回転・ROVIO単体の数値と監査</a></p>'
    text += '<h2>各経路の平均位置RMSE [m]</h2><p>完了した各試行のRMSEの算術平均です。全軌跡をつないで計算したRMSEとは異なります。</p>'
    text += '<table><tr><th>経路</th>'+''.join('<th>'+LABEL[n]+'</th>' for n in ORDER)+'</tr>'
    text += ''.join('<tr><td>'+route+'</td>'+''.join(f'<td>{values[n]:.4f}</td>' for n in ORDER)+'</tr>'
                    for route, values in means.items())+'</table><h2>全試行</h2>'
    text += '<table><tr><th>経路/seed</th>'+''.join('<th>'+LABEL[n]+'</th>' for n in ORDER)+'</tr>'+''.join(rows)+'</table>'
    text += '<h2>固定方向の画像例</h2><p>各経路の1回目、追従開始後約3秒の入力画像です。連続画像は保存していません。</p>'
    for route in manifest['routes']:
        key = next((k for k in records if k.startswith(route+'_seed')), None)
        if key:
            text += '<h3>'+route+'</h3><div style="display:flex;gap:8px">'
            for name in ORDER[:-1]:
                image = f'{key}/campaign/{key}_active/view_snapshots/{name}.png'
                text += f'<div style="width:20%">{LABEL[name]}<img src="{image}"></div>'
            text += '</div>'
    text += ''.join(figures)
    (root/'index.html').write_text(text)
    print(json.dumps(dict(complete=summary['complete'], means=means), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial', type=Path)
    parser.add_argument('--campaign', type=Path)
    args = parser.parse_args()
    if args.trial:
        trial(args.trial)
    if args.campaign:
        campaign(args.campaign)
