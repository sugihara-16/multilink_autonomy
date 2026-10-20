#!/usr/bin/env python3
"""Present all frozen validation pairs, using the existing accuracy evaluator."""
import argparse
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

from report_estimator_validation import evaluate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('--candidate', required=True)
    args = parser.parse_args()
    summary = json.loads((args.root / f'validation_summary_{args.candidate}.json').read_text())
    if not summary['complete']:
        raise ValueError('All six valid, frozen-matching pairs must be complete before this report')
    rows = sorted(summary['comparisons'], key=lambda row: (row['route'], row['seed']))
    output = args.root / f'validation_report_{args.candidate}'
    output.mkdir(exist_ok=True)
    figures = []
    with PdfPages(output / 'comparison.pdf') as pdf:
        for estimator in ('mimosa', 'rovio'):
            fig, axes = plt.subplots(1, 2, figsize=(13, 6))
            y = np.arange(len(rows))
            labels = [r['route'].removeprefix('run1_') + '\nseed ' + str(r['seed']) for r in rows]
            for axis, metric, unit in zip(axes, ('position_rmse_m', 'rotation_rmse_deg'), ('m', 'deg')):
                for mode, offset, color in [('fixed', -.19, '#2474b5'), ('active', .19, '#df6619')]:
                    values = [r['metrics'][mode][estimator][metric] for r in rows]
                    bars = axis.barh(y + offset, values, .36, label=mode, color=color)
                    axis.bar_label(bars, fmt='%.3f', padding=3, fontsize=8)
                axis.set_yticks(y, labels)
                axis.invert_yaxis()
                axis.set_xlabel('RMSE [' + unit + ']')
                axis.margins(x=.2)
                axis.grid(axis='x', alpha=.25)
            axes[0].legend()
            fig.suptitle(('ROVIO + MIMOSA' if estimator == 'mimosa' else 'ROVIO alone') +
                         ' / all held-out pairs / first-pose alignment, no scale fit')
            fig.tight_layout()
            name = estimator + '_summary.png'
            fig.savefig(output / name, dpi=140)
            pdf.savefig(fig)
            plt.close(fig)
            figures.append(name)
        for row in rows:
            base = args.root / row['campaign'] / (row['route'] + '_active')
            for estimator in ('mimosa', 'rovio'):
                fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
                for mode, path, color in [('fixed', base / 'fixed', '#2474b5'), ('active', base, '#df6619')]:
                    metrics, grid, error, _ = evaluate(path, estimator)
                    assert metrics == row['metrics'][mode][estimator], 'Saved and recomputed metrics differ'
                    for index, axis in enumerate([*axes[:, 0], *axes[:, 1]]):
                        axis.plot(grid, error[:, index], label=mode, color=color, lw=.8)
                        axis.set_ylabel(['x [m]', 'y [m]', 'z [m]', 'roll [deg]', 'pitch [deg]', 'yaw [deg]'][index])
                        axis.grid(alpha=.3)
                axes[0, 0].legend()
                for axis in axes[-1]:
                    axis.set_xlabel('Trajectory time [s]')
                fig.suptitle(row['route'] + ' / seed ' + str(row['seed']) + ' / ' + estimator +
                             '\nSame physical flight; first-pose alignment, no scale fit')
                fig.tight_layout()
                name = row['route'] + '_' + str(row['seed']) + '_' + estimator + '.png'
                fig.savefig(output / name, dpi=120)
                pdf.savefig(fig)
                plt.close(fig)
                figures.append(name)
    state = '事前に定めた改善基準を満たしました。' if summary['passed'] else '事前に定めた改善基準を満たしませんでした。'
    body = '<!doctype html><meta charset="utf-8"><title>固定・ジンバル比較</title>'
    body += '<style>body{font-family:sans-serif;max-width:1250px;margin:30px auto;line-height:1.7}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style>'
    body += '<h1>ROVIO＋MIMOSA：固定カメラとジンバルの比較</h1><p><strong>' + state + '</strong></p>'
    body += '<p>実装・設定の固定後、調整に使っていない乱数条件2つで3経路を比較。各組は同じ飛行・IMU・初期状態を使用し、推定器設定も同一です。DRPMによる融合補正はOFF。軌道追従中だけを評価し、最初の位置・姿勢を揃えます。尺度の補正はしません。</p>'
    body += '<p>prefixは長い経路の先頭1,070秒、a1/b1は各区間全長です。3経路・2反復の結果であり、未検証の環境すべてでの改善を意味しません。ROVIO単体の悪化も下表・図に含みます。</p>'
    body += '<p>位置・回転の値は二乗平均平方根誤差（RMSE）。青が固定、橙がジンバルです。縦軸の省略はありません。</p>'
    body += '<p><a href="comparison.pdf">全14ページのPDF</a> / <a href="../validation_summary_' + html.escape(args.candidate) + '.json">数値・検証記録</a> / <a href="../index.html">開発中の試行も含む記録</a></p>'
    body += '<table><tr><th>経路 / seed</th><th>融合位置 m</th><th>融合回転 deg</th><th>ROVIO位置 m</th><th>ROVIO回転 deg</th></tr>'
    for row in rows:
        cells = [html.escape(row['route']) + ' / ' + str(row['seed'])]
        for estimator in ('mimosa', 'rovio'):
            for metric in ('position_rmse_m', 'rotation_rmse_deg'):
                cells.append(' → '.join(f'{row["metrics"][mode][estimator][metric]:.3f}' for mode in ('fixed', 'active')))
        body += '<tr>' + ''.join('<td>' + value + '</td>' for value in cells) + '</tr>'
    body += '</table>' + ''.join('<img alt="' + name + '" src="' + name + '">' for name in figures)
    (output / 'index.html').write_text(body + '\n')
    print(output)


if __name__ == '__main__':
    main()
