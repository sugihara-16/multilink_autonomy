"""Offline, retrospective tests of preserved flight-level UniPilot summaries.

No simulator, estimator, trajectory replay, or new measurements are started.
The JSON input explicitly separates available paired values from missing data.
"""
import argparse
import hashlib
import html
import itertools
import json
from pathlib import Path

import numpy as np
import scipy
from scipy import stats


def sign_flip_mean(differences):
    d = np.asarray(differences, dtype=float)
    assert d.ndim == 1 and len(d) > 0 and np.isfinite(d).all()
    null = np.array([np.mean(d * s) for s in itertools.product((-1, 1), repeat=len(d))])
    observed = float(d.mean())
    tolerance = 1e-12 * max(1., abs(observed))
    return dict(mean_fixed_minus_gimbal_m=observed,
                one_sided_p=float(np.mean(null >= observed - tolerance)),
                two_sided_p=float(np.mean(abs(null) >= abs(observed) - tolerance)),
                enumerated_assignments=len(null))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    source = json.loads(args.input.read_text())
    out = args.input.parent
    gimbal = np.asarray(source['weathered']['gimbal_values_m'])
    rows = []
    for fixed in source['weathered']['fixed']:
        row = dict(fixed)
        values = fixed.get('values_m')
        if values is None:
            row.update(status='missing_paired_values', one_sided_p=None,
                       minimum_possible_exact_one_sided_p=2 ** -len(gimbal))
        else:
            d = np.asarray(values) - gimbal
            row.update(status='tested_preserved_rounded_values', differences_m=d.tolist(),
                       **sign_flip_mean(d))
            row['bonferroni_five_directions_p'] = min(1., 5 * row['one_sided_p'])
            # Independent implementation check against SciPy's paired swap test.
            reference = stats.permutation_test(
                (np.asarray(values), gimbal), lambda a, b: np.mean(a-b),
                permutation_type='samples', vectorized=False,
                n_resamples=np.inf, alternative='greater')
            assert row['one_sided_p'] == float(reference.pvalue)
            t = stats.ttest_rel(values, gimbal, alternative='greater')
            row['sensitivity_paired_t_one_sided_p'] = float(t.pvalue)
            two = stats.ttest_rel(values, gimbal)
            ci = two.confidence_interval()
            row['sensitivity_paired_t_mean_difference_95_ci_m'] = [float(ci.low), float(ci.high)]
            row['gimbal_better_flights'] = int(sum(d > 0))
            row['sign_test_one_sided_p'] = float(stats.binomtest(int(sum(d > 0)), len(d), .5, alternative='greater').pvalue)
        rows.append(row)
    assert sign_flip_mean([1., 2., 3.])['one_sided_p'] == .125
    assert sign_flip_mean([1., 2., 3.])['two_sided_p'] == .25
    # All-positive differences attain the maximum signed sum, irrespective of
    # their unknown magnitudes. Do not fabricate per-flight numeric differences.
    fixed_average = dict(available_evidence='Recorded 3/3 strict wins; individual magnitudes unavailable',
                         sign_test_one_sided_p=.125, sign_flip_mean_one_sided_p_from_all_positive_signs=.125,
                         two_sided_p=.25)
    switching = dict(available_evidence=source['switching'],
                     assuming_six_independent_flights_one_sided_p=1/64,
                     assuming_six_independent_flights_two_sided_p=2/64,
                     bonferroni_five_direction_individual_claims_p=5/64,
                     intersection_union_all_five_under_six_independent_flights_p=1/64,
                     three_shared_seed_blocks_one_sided_p=1/8,
                     two_layout_blocks_one_sided_p=1/4,
                     warning='Conditional calculations from recorded strict wins, not recovered raw numeric effects. Reused seeds do not by themselves prove dependence, but flight independence across layouts is unverified. Do not equate six flights with six independent environments.')
    result = dict(analysis='Retrospective; no new experiment', scipy_version=scipy.__version__,
                  input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  metric='Historical whole-route fused position RMSE per physical flight',
                  independent_unit='Flight within the single weathered layout; never camera streams or time samples',
                  primary_test='Mean paired difference sign flips; one-sided fixed minus gimbal > 0, all 2^3 assignments',
                  assumptions='Independent flight pairs and exchangeable signs under the null (e.g. symmetric differences). Not an assumption-free test of arbitrary population means.',
                  t_test_scope='Sensitivity analysis only: normally distributed paired differences, uncheckable with n=3.',
                  multiplicity='Bonferroni across the five fixed-direction individual comparisons; unavailable directions are not silently removed from the family.',
                  gimbal_mean_m=float(gimbal.mean()), rows=rows,
                  fixed_average_reference=fixed_average, switching_conditional_appendix=switching,
                  conclusion='No 5% significance established for the natural-weathered three-flight comparison. This does not establish equivalence or no benefit.',
                  verification='Two independently implemented SciPy paired permutation checks and analytic all-positive three-pair checks passed.')
    (out/'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    page = '''<!doctype html><meta charset="utf-8"><title>UniPilot過去データの有意性検定</title>
<style>body{font:16px sans-serif;line-height:1.7;max-width:1100px;margin:30px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}pre{white-space:pre-wrap}</style>
<h1>UniPilot：固定方向に対するジンバルの過去データ検定</h1>
<p><strong>自然な汚れ・ひびのトンネルの3反復では、5%水準の有意な優位性は確認できなかった。</strong>平均が良いことや、改善がないこととは別の判断。</p>
<p>再実験・推定器再実行なし。2026-10-24の作業記録に残る、同じ飛行の固定カメラとジンバルの試行別集計値を使用した。元の時系列記録はこのPCでは見つからず、3方向は平均値のみ。小数5桁の保存値からの事後検定であり、過去に事前登録された検定ではない。</p>
<p>指標は全90mの融合位置RMSE。配置24201、seed20262601/602/603、単眼ROVIO＋MIMOSA、DRPM OFF。同じ飛行内の対応を保ち、画像フレームや5カメラを独立試行として水増ししない。別の環境や今回の再現試験の値は混ぜない。</p>
<h2>残っている試行別の値</h2><table><tr><th>方式</th><th>601</th><th>602</th><th>603</th><th>平均 [m]</th></tr>'''
    page += '<tr><td>ジンバル</td>'+''.join(f'<td>{v:.5f}</td>' for v in gimbal)+f'<td>{gimbal.mean():.5f}</td></tr>'
    for row in rows:
        cells = ''.join(f'<td>{v:.5f}</td>' for v in row['values_m']) if row.get('values_m') else '<td colspan="3">試行別値は未確認</td>'
        page += f'<tr><td>{html.escape(row["label"])}</td>{cells}<td>{row["reported_mean_m"]:.5f}</td></tr>'
    page += '</table><h2>検定結果</h2><p>各飛行の「固定の誤差−ジンバルの誤差」を使い、8通りの符号反転を全列挙。正の平均差に対する片側検定を行い、両側も併記した。差の符号交換が許される仮定が必要。対応のあるt検定は差の正規分布を仮定した参考値で、3例では分布を検証できない。</p><table><tr><th>比較対象</th><th>平均改善 [m]</th><th>片側p</th><th>両側p</th><th>5方向補正p</th><th>参考：対応t片側p</th></tr>'
    for row in rows:
        if row['one_sided_p'] is not None:
            page += f'<tr><td>{row["label"]}</td><td>{row["mean_fixed_minus_gimbal_m"]:.5f}</td><td>{row["one_sided_p"]:.3f}</td><td>{row["two_sided_p"]:.3f}</td><td>{row["bonferroni_five_directions_p"]:.3f}</td><td>{row["sensitivity_paired_t_one_sided_p"]:.6f}</td></tr>'
    page += '''</table><p>正面と上向きは未補正でも5%未達。固定5方向平均に3/3回勝った記録からは、符号検定の片側p=0.125（両側0.25）。全差が正なので平均差の符号反転検定も同値だが、効果量の分散やt検定は復元できない。</p>
<p>3反復の全列挙符号反転検定では、片側pの最小値自体が1/8=0.125。残る3方向の試行別値があっても、この検定では5%に達しない。他の強い分布仮定を置く検定まで不可能という意味ではない。5方向の平均値から架空の個別値や分散は生成しなかった。</p>
<h2>別実験：明確な模様切替の2配置×3回</h2>
<p>作業記録には、正順・逆順の計6飛行で全固定5方向より融合全位置誤差が小さかったとある。この結果は自然な汚れ配置の3飛行とは別に扱う。</p>
<ul><li>6飛行を独立と仮定：全勝から各方向の片側符号検定/平均差符号反転p=1/64=0.015625、両側0.03125。個々の5方向比較のBonferroni補正は0.078125。</li>
<li>両配置に同じ3個のseedを再使用している。seedごとに両配置をまとめて符号反転すると片側p=0.125。seed再使用だけで依存が確定するわけではないが、残った記録だけでは6飛行の独立性を確認できない。</li>
<li>配置を独立単位とする場合は2配置しかなく、片側p=0.25。同種の環境全体への平均優位を示す6環境分の証拠ではない。</li></ul>
<p>「5方向すべてに優位」という一つの仮説を、5方向との比較がすべて棄却された場合だけ採択する検定では、Bonferroni補正は不要。この積集合の検定も、独立6飛行・符号交換の仮定の下では片側p=0.015625となる。上記の個別比較補正を、この限定した主張を退ける必須条件として使わない。</p>
<p>したがって、独立6飛行という条件付きでは有意となる計算はあるが、自然な汚れトンネルの結果や、環境全体に対する確定した有意差として報告しない。全周に特徴がある対照では最良固定が3/3回良好だった記録も保持する。</p>
<h2>出典と再現</h2><p><a href="input.json">転記した値・条件・出典</a>、<a href="source_excerpt.txt">元の作業記録の該当箇所</a>、<a href="results.json">全検定結果</a>。</p>
<p>検定仕様：<a href="https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.permutation_test.html">SciPy paired permutation test</a>、<a href="https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.ttest_rel.html">SciPy paired t-test</a>。独自の全列挙実装をSciPyと照合済み。</p>'''
    (out/'index.html').write_text(page)
    print(json.dumps(dict(conclusion=result['conclusion'], rows=rows, switching=switching), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
