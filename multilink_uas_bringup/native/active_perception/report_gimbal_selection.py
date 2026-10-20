#!/usr/bin/env python3
"""Summarize the selected configuration without changing acceptance criteria."""
import argparse
import html
import json
from pathlib import Path


def table(rows):
    body = []
    for row in rows:
        fixed = row['metrics']['fixed']['mimosa']['position_rmse_m']
        active = row['metrics']['active']['mimosa']['position_rmse_m']
        cells = [f'<a href="{html.escape(row["report"], quote=True)}">{html.escape(row["route"])}</a>',
                 str(row['seed']), f'{fixed:.4f}', f'{active:.4f}', f'{100*(1-active/fixed):+.1f}%']
        body.append('<tr>'+''.join('<td>'+cell+'</td>' for cell in cells)+'</tr>')
    return '<table><tr><th>飛行区間</th><th>乱数条件</th><th>固定 m</th><th>可動 m</th><th>誤差低減率</th></tr>'+''.join(body)+'</table>'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    selected = json.loads((Path(__file__).parent/'selected_gimbal.json').read_text())
    result = json.loads((args.root/f'validation_summary_{selected["candidate"]}.json').read_text())
    rejected = json.loads((args.root/'validation_summary_v59.json').read_text())
    stop = json.loads((args.root/'v59_independent_futility_stop.json').read_text())
    assert result['complete'] and result['pairs'] == 6 and result['wins'] == 6
    assert all(row['valid'] for row in result['comparisons'])
    assert result['passed'] is False and selected['original_acceptance_passed'] is False
    reduction = 100*(1-result['mean_ratio'])
    document = '''<!doctype html><html lang="ja"><meta charset="utf-8"><title>UniPilot ジンバル比較：採用構成</title>
<style>body{font:17px sans-serif;line-height:1.7;max-width:1100px;margin:35px auto;padding:0 20px}table{width:100%;border-collapse:collapse}td,th{border:1px solid #ccc;padding:8px;text-align:right}td:first-child{text-align:left}code{background:#eee;padding:2px 5px} .note{padding:12px;background:#fff2cf}</style>'''
    document += f'<h1>採用構成：固定カメラに対して6比較すべてで改善</h1><p>ROVIO＋MIMOSAの融合位置誤差は、3経路×2乱数条件で平均<b>{reduction:.1f}%低減</b>しました。採用するのはv57です。設定を固定した後の別乱数条件による結果です。</p>'
    document += '<p class="note">事前に置いた平均20%以上の低減目標には未達です。元の合否判定はfalseのまま保持しています。6組全改善、経路別平均10%以上改善、最大悪化の条件は満たしています。この3経路以外の改善を保証する結果ではありません。</p>'
    document += '<p>同じ物理飛行・IMU・LiDARを共有し、カメラを固定する場合と動かす場合を比較しました。DRPMによる融合補正はOFF。軌道追従中だけを評価し、始点の位置・姿勢を合わせています。距離の尺度の事後補正はしていません。表は位置の二乗平均平方根誤差です。平均低減率は6組の誤差比の平均から求めています。</p>'
    document += table(result['comparisons'])
    document += '<p>カメラの向きは、観測済みの画像特徴とLiDAR形状から選びます。追跡中の特徴を視野に残し、機体の回転と実際のジンバル速度制限を考慮します。yawは±50°、pitchは±90°です。</p>'
    document += '<p>ROVIO単体の位置誤差は6組中4組で悪化しました。融合位置の改善を、ROVIO単体の一様な精度向上とは説明できません。長区間には数メートルの誤差が残っています。</p>'
    document += '<p><a href="validation_v57.html">六軸プロットへの一覧・詳細監査</a> / <a href="validation_summary_v57.json">元の数値と合否</a> / <a href="selected_v57_source_verification.json">採用ソースの一致確認</a></p>'
    document += '<h2>追加した距離補正は不採用</h2><p>v59は開発6組で平均20.7%低減しましたが、独立検証の最初の3組で2組悪化しました。6組中5組以上の改善が不可能になったため、実行中の4組目を完走して終了し、未開始の長区間2組は実行していません。実施済みの結果は削除していません。</p>'
    document += table(rejected['comparisons'])
    document += '<p><a href="validation_v59.html">追加候補の詳細</a> / <a href="v59_independent_futility_stop.json">追加試験を止めた理由</a> / <a href="frozen_v59_sources.zip">追加候補の保存実装</a></p>'
    document += '<h2>再実行</h2><p>新しい保存先を指定して、次のコマンドをユーザーの端末から実行できます。実行前に採用ソースの一致を検査します。終了はCtrl+Cです。</p><pre><code>python3 /root/uas_native/active_perception/run_selected_gimbal.py \\\n  --output /root/uas_native/logs/gimbal-repeat-new</code></pre><p>既定は3経路の固定・可動比較です。<code>--labels run1_tail_a1</code>で1経路、<code>--dry-run</code>で起動せずコマンド確認ができます。GUIとrosbagは起動・収録しません。Windowsの空きが8GBを下回ると停止します。</p></html>'
    (args.root/'adopted_gimbal.html').write_text(document)
    evidence = dict(selected=selected['candidate'],original_acceptance_passed=result['passed'],
                    observed_pairs=result['pairs'],observed_wins=result['wins'],mean_reduction_percent=reduction,
                    rejected_candidate='v59',rejected_observed_pairs=rejected['pairs'],futility_stop=stop)
    (args.root/'gimbal_selection.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,ensure_ascii=False))


if __name__ == '__main__':
    main()
