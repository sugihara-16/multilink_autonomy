#!/usr/bin/env python3
"""Add presentation-only plots to the frozen numerical evaluator's report."""
import argparse
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from report_direction_repeats import ORDER, LABEL, COLORS
from report_routes import load


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    out = root / 'report_sparse'
    results = json.loads((out / 'summary.json').read_text())
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for ax, (layout, result) in zip(axes, results['conditions'].items()):
        for j, name in enumerate(ORDER[:6]):
            values = result['methods'][name]['complete_axial_velocity_rms_mps']['values']
            for offset, value in zip(np.linspace(-.12, .12, len(values)), values):
                if value is not None:
                    ax.scatter(j + offset, value, color=COLORS[j])
            if all(value is not None for value in values):
                ax.plot([j-.2, j+.2], [np.mean(values)]*2, color=COLORS[j])
            else:
                coverage = result['methods'][name]['coverage']['mean']
                ax.text(j, .03, f'{coverage:.0%}\ncoverage', ha='center',
                        va='bottom', transform=ax.get_xaxis_transform(), fontsize=9)
        ax.set_xticks(range(6), [LABEL[name] for name in ORDER[:6]], rotation=35)
        ax.set_title(layout)
        ax.set_ylabel('Accepted VIO axial velocity RMS [m/s]')
        ax.set_ylim(bottom=0)
        ax.grid(alpha=.25)
    fig.suptitle('Primary metric: full coverage only; missing factors are not filled')
    fig.tight_layout()
    fig.savefig(out / 'primary_velocity.png', dpi=150)
    fig.savefig(out / 'primary_velocity.pdf')
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    rich = {'alternating': ['left', 'roof', 'right', 'floor'],
            'reversed': ['floor', 'right', 'roof', 'left'],
            'uniform': ['all surfaces'] * 4}
    for row, (layout, result) in enumerate(results['conditions'].items()):
        for key in result['trials']:
            flight = root / key / 'campaign' / (key + '_active')
            encoder = load(flight / 'encoders.csv')
            t0 = results['trials'][key]['tracking_start_s']
            mask = (encoder['t'] >= t0) & (encoder['t'] <= t0 + 262)
            route = load(flight / 'route_tracking.csv')
            # Use the commanded route distance: identical for all paired methods.
            x = np.interp(encoder['t'][mask], route['sim_t'], route['ref_x'])
            for col, name in enumerate(['yaw', 'pitch']):
                axes[row, col].plot(x, np.degrees(encoder[name][mask]),
                                    label=key.rsplit('seed', 1)[1], linewidth=.8)
        for col, name in enumerate(['yaw', 'pitch']):
            ax = axes[row, col]
            for i, label in enumerate(rich[layout]):
                left = 24 + 32*i
                ax.axvspan(left, left+32, color=['#d5e8fc', '#e2f0d9'][i % 2], alpha=.4)
                ax.text(left+16, 1.01, label, ha='center', transform=ax.get_xaxis_transform(), fontsize=9)
            ax.set_title(layout + ' / ' + name, pad=25)
            ax.set_ylabel(name + ' [deg]')
            ax.set_ylim((-55, 55) if name == 'yaw' else (-95, 95))
            ax.set_xlim(20, 150)
            ax.grid(alpha=.25)
            if col == 0:
                ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel('Position along tunnel [m]')
    fig.tight_layout()
    fig.savefig(out / 'gimbal_angles.png', dpi=150)
    fig.savefig(out / 'gimbal_angles.pdf')
    plt.close(fig)

    html = (out / 'index.html').read_text()
    findings = '<section id="key-findings"><h2>結果</h2><p>特徴の多い面が切り替わる2配置では、計6飛行すべてで偏向が固定5方向を上回りました。VIO自身の10秒軸方向移動と、融合後の全行程位置の両方で確認した結果です。全周に特徴がある対照条件では3飛行とも最良固定が勝ちました。</p>'
    findings += '<table><tr><th>配置</th><th>VIO 10秒移動：最良固定 [m]</th><th>VIO 10秒移動：偏向 [m]</th><th>融合全位置：最良固定 [m]</th><th>融合全位置：偏向 [m]</th></tr>'
    for layout, result in results['conditions'].items():
        methods = result['methods']
        values = []
        for field in ['vio_local_10s_axial_rms_m', 'full_fused_position_rmse_m']:
            best = min(ORDER[:5], key=lambda name: methods[name][field]['mean'])
            values.extend([f"{methods[best][field]['mean']:.3f} ({LABEL[best]})",
                           f"{methods['active'][field]['mean']:.3f}"])
        findings += '<tr><td>' + layout + '</td>' + ''.join('<td>'+v+'</td>' for v in values) + '</tr>'
    findings += '</table><p>数値は各飛行の誤差RMSを計算した後の3回平均。「最良固定」は各指標で結果を見て選んだ一つの方向で、途中で方向を切り替える方式ではありません。偏向は各飛行の最良固定と比べても偏在配置で6/6回優位でした。VIOの10秒指標は各領域中央40秒の共通窓、融合全位置は130mの全経路を対象とします。</p>'
    findings += '<p>主指標である完全利用時の軸速度誤差も、偏在2配置で最良固定の平均0.270→0.130m/s、0.459→0.223m/sへ改善しました。上下固定は拘束利用率が約50%/75%なので完全利用値を算出せず、下の表に利用時誤差と利用率を分けて表示します。画像・odometry欠落ではなく、ROVIOの不確かさが増した入力をMIMOSAが見送ったものです。読取診断では判定閾値を十分超えた148時刻すべてで近傍の採用拘束がありませんでした。</p>'
    findings += '<p>偏向のVIO誤差自体が減っているため、今回の改善を融合誤差の相殺だけで説明する必要はありません。絶対精度はなお偏在配置で全位置RMSE 8.6〜13.2mあり、高精度SLAMが完成したという結果ではありません。この人工環境での固定との比較であり、実環境への汎化や単純な特徴追従方式に対する優位性は別の検証です。</p></section>'
    html = re.sub(r'<section id="key-findings">.*?</section>', '', html, flags=re.S)
    html = html.replace('</h1>', '</h1>' + findings, 1)
    marker = '<!-- supplementary-illustrations -->'
    html = html.split(marker)[0]
    html += marker + '''<h2>環境と主指標</h2>
<p><a href="../../environment_overview.pdf">環境の断面・特徴配置</a> /
<a href="primary_velocity.pdf">軸方向速度の主指標PDF</a> /
<a href="gimbal_angles.pdf">実際の偏向角PDF</a> /
<a href="final_audit.json">取得・再現性の検査結果</a></p>
<img src="../../environment_overview.png">
<p>速度の図は全評価時間の拘束が揃う方法だけ数値表示し、欠ける方法は利用率を示します。
上の表の「利用時RMS」は欠けなかった時刻だけの値なので、全時間の精度と混同しないでください。</p>
<img src="primary_velocity.png">
<p>下は実際に記録したジンバル角です。背景は事後評価用の模様配置で、カメラ制御には渡していません。
pitch +90度は下向き、-90度は上向きです。</p><img src="gimbal_angles.png">
<p>完了した全9飛行を掲載。最初のuniform試行はシミュレーションの処理待ちが解消せず途中終了したため、
同じ凍結条件・seedで再取得しました。未完了試行もinvalid_trialsに保存しています。
固定方向の入力欠落は推定結果として残しており、精度不良の飛行を除外したものではありません。</p>
<p>全条件で推定器・偏向方式・重みを固定。標準採用方式v57は変更していません。
今回の偏向は実験候補v10です。各条件3飛行で、同一飛行内の窓は独立試行として数えません。</p>'''
    (out / 'index.html').write_text(html)
    print(out / 'index.html')


if __name__ == '__main__':
    main()
