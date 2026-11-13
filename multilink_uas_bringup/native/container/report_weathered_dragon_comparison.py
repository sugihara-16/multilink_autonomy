#!/usr/bin/env python3
"""Recompute all nine retained flights on the same full 182-second interval."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'active_perception'))
from practical_comparison_metrics import evaluate
from report_practical_comparison import read_pose
from compact_record_io import record_path

LABELS = {'unipilot': 'UniPilot gimbal', 'gimbal': 'DRAGON gimbal', 'joint': 'DRAGON joint'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    protocol = json.loads((root/'protocol.json').read_text())
    reference = Path(protocol['reference'])
    reference_summary = json.loads((reference/'report/summary.json').read_text())
    assert hashlib.sha256((reference/'report/summary.json').read_bytes()).hexdigest() == protocol['reference_summary_sha256']
    out = root/'report'
    out.mkdir(exist_ok=True)
    trials, audits, curves = {}, {}, {}
    amendment_path = root/'acquisition_amendment.json'
    amendment = json.loads(amendment_path.read_text()) if amendment_path.exists() else None
    def directory_for(arm, seed):
        name = f'{arm}_seed{seed}'
        if arm == 'unipilot':return reference/name
        if amendment and name == amendment['replacement_trial']:
            return root/amendment['replacement_campaign']/name
        return root/f'seed{seed}'/name
    for seed in protocol['seeds']:
        audit = json.loads((root/f'seed{seed}/final_audit.json').read_text())
        audits[str(seed)] = audit
        for arm in LABELS:
            name = f'{arm}_seed{seed}'
            directory = directory_for(arm,seed)
            row = trials[name] = dict(arm=arm, seed=seed, path=str(directory), estimators={})
            if arm == 'unipilot':
                row['audit'] = reference_summary['acquisition_audit']['trials'][name]
            else:
                row['audit'] = audit['trials'][name]
                if amendment and name == amendment['replacement_trial']:
                    retry_audit = json.loads((directory.parent/'final_audit.json').read_text())
                    row['original_startup_attempt_audit'] = row['audit']
                    row['audit'] = retry_audit['trials'][name]
                    audits['startup_retry'] = retry_audit
                for filename in ('flight_result.json','completion.json'):
                    if (directory/filename).exists():row[filename] = json.loads((directory/filename).read_text())
            try:
                if arm == 'unipilot':
                    route = np.genfromtxt(record_path(directory/'route_tracking.csv'),delimiter=',',names=True)
                    moving = route[route['route_t'] > 0]
                    start = float(moving['sim_t'][0]-moving['route_t'][0])
                else:
                    events = json.loads((directory/'flight_events.json').read_text())
                    start = next(e['time'] for e in events if e['stage'] == 'active')
                row['start_s'] = start
                truth = read_pose(directory/'truth.csv')
                for estimator in ('mimosa', 'rovio'):
                    t, error, _, metrics = evaluate(truth,read_pose(directory/f'{estimator}.csv'),start,182.)
                    row['estimators'][estimator] = metrics
                    curves[name,estimator] = (t, np.linalg.norm(error,axis=1))
                row['full_interval_available'] = True
            except (OSError, ValueError, KeyError, IndexError, StopIteration) as exc:
                row.update(full_interval_available=False,error=str(exc))
    groups = {}
    for arm in LABELS:
        selected = [v for v in trials.values() if v['arm'] == arm]
        complete = all(v['full_interval_available'] for v in selected)
        group = groups[arm] = dict(full_flights=sum(v['full_interval_available'] for v in selected),
                                  passed_audits=sum(v['audit']['passed'] for v in selected), estimators={})
        for estimator in ('mimosa','rovio'):
            values = [v['estimators'][estimator]['position_rmse_m'] if v['full_interval_available'] else None for v in selected]
            group['estimators'][estimator] = dict(values=values,mean=float(np.mean(values)) if complete else None,
                sample_sd=float(np.std(values,ddof=1)) if complete else None)
    # Across-seed consistency supplements each pair's acquisition/configuration audit.
    config_checks = {}
    for arm in ('gimbal','joint'):
        base = directory_for(arm,protocol['seeds'][0])
        for seed in protocol['seeds']:
            directory = directory_for(arm,seed)
            for name in ('planner.yaml','cam0.yaml','rovio.info','mimosa.yaml'):
                try:
                    if name.endswith('.yaml'):
                        left,right = [yaml.safe_load((d/name).read_text()) for d in (base,directory)]
                        if name == 'mimosa.yaml':
                            left.pop('logs_directory',None);right.pop('logs_directory',None)
                    else:
                        left,right = [(d/name).read_text() for d in (base,directory)]
                    config_checks[f'{arm}_{seed}_{name}'] = left == right
                except OSError:
                    config_checks[f'{arm}_{seed}_{name}'] = False
    result = dict(protocol=protocol, acquisition_amendment=amendment, trials=trials, groups=groups, seed_audits=audits,
                  cross_seed_configuration_checks=config_checks,
                  all_dragon_audits_passed=all(a['passed'] for a in audits.values()),
                  all_cross_seed_configurations_match=all(config_checks.values()),
                  evaluation='Full 182 s at 10 Hz, first common pose rigid alignment, no scale fit. No failed trial exclusions.',
                  scope='UniPilot is the preceding 3-run campaign. Six new DRAGON flights. Same environment and route, independent sensor realizations and differing adopted configurations.')
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    fig, axes = plt.subplots(3,2,figsize=(12,9),sharex=True)
    for i,seed in enumerate(protocol['seeds']):
        for j,estimator in enumerate(('mimosa','rovio')):
            for arm in LABELS:
                key=(f'{arm}_seed{seed}',estimator)
                if key in curves:
                    axes[i,j].plot(*curves[key],label=LABELS[arm])
            axes[i,j].set_ylabel(f'{seed}\nPosition error [m]')
            axes[i,j].grid(alpha=.3)
    axes[0,0].set_title('Fusion (MIMOSA)')
    axes[0,1].set_title('Visual-inertial (ROVIO)')
    axes[0,0].legend()
    for ax in axes[-1]:ax.set_xlabel('Tracking time [s]')
    fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=130);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4))
    for i,arm in enumerate(LABELS):
        data=groups[arm]['estimators']['mimosa']
        vals=[(j,v) for j,v in enumerate(data['values']) if v is not None]
        ax.scatter([i+(j-1)*.08 for j,v in vals],[v for j,v in vals])
        if data['mean'] is not None:ax.plot([i-.16,i+.16],[data['mean']]*2,linewidth=3)
    ax.set_xticks(range(3),list(LABELS.values()));ax.set_ylabel('Fusion position RMSE [m]')
    ax.set_title('weathered24201 | 90 m | individual runs and 3-run mean');ax.grid(axis='y',alpha=.3)
    fig.tight_layout();fig.savefig(out/'individual_runs.png',dpi=140);plt.close(fig)
    fmt=lambda x:'—' if x is None else f'{x:.3f}'
    page='''<!doctype html><meta charset="utf-8"><title>汚れ・ひびのトンネル：DRAGON比較</title><style>body{font:16px sans-serif;line-height:1.8;max-width:1200px;margin:32px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}img{max-width:100%}</style><h1>以前の汚れ・ひびのトンネルでDRAGONを比較</h1><p>模様配置24201、90 m・毎秒0.5 m・高さ2 m、各3回。同じ乱数初期値20262601/602/603。UniPilotは直前の完了済み3回を再集計し、DRAGONのジンバル版・関節版を新たに各3回実行した。失敗を除外せず、全区間が3回そろわない場合は平均を表示しない。</p><p>位置誤差は小さいほど良い。全行程を共通10 Hzで評価し、開始時の位置と姿勢だけを合わせる。尺度補正なし。平均は3試行の位置RMSEの算術平均、ばらつきは標本標準偏差。</p><p>採用済み設定全体の比較。DRAGONの両方式は同じ融合設定だが視線選択と移動カメラへの対応が異なる。UniPilotは従来の融合設定（DRPM OFF）。同一センサー入力や機構だけの効果を切り分けた試験ではない。</p><table><tr><th>方式</th><th>位置誤差平均 ± ばらつき [m]</th><th>全区間取得</th><th>取得・動作・終了検査合格</th></tr>'''
    for arm,label in LABELS.items():
        g=groups[arm];m=g['estimators']['mimosa']
        page+=f'<tr><td>{label}</td><td>{fmt(m["mean"])} ± {fmt(m["sample_sd"])}</td><td>{g["full_flights"]}/3</td><td>{g["passed_audits"]}/3</td></tr>'
    page+='</table><h2>全試行</h2><table><tr><th>方式</th><th>乱数初期値</th><th>位置誤差 [m]</th><th>検査</th></tr>'
    for row in trials.values():
        failed=[k for k,v in row['audit']['checks'].items() if not v]
        value=row['estimators'].get('mimosa',{}).get('position_rmse_m') if row['full_interval_available'] else None
        failure = row.get('flight_result.json',{}).get('failed')
        detail = failure or (', '.join(failed) if failed else '合格')
        page+=f'<tr><td>{LABELS[row["arm"]]}</td><td>{row["seed"]}</td><td>{fmt(value)}</td><td>{detail}</td></tr>'
    page+='</table><p><a href="summary.json">全数値・条件・取得監査</a></p><img src="individual_runs.png"><img src="trajectories.png">'
    page+=f'<p><a href="{os.path.relpath(reference/"report/index.html",out)}">先行するUniPilotと過去の比較</a></p>'
    if amendment:
        page+='<p>関節20262603は最初の起動時に機体モデルの通信待ちが時間切れとなり、センサー取得・飛行開始とも0件だった。起動失敗を保存し、設定を変えず1回だけ再実行した。表には事前に指定した再実行を用い、元の失敗も全数値に保持。飛行中に失敗した試行は再実行・除外していない。<a href="../acquisition_amendment.json">起動再試行の記録</a></p>'
    page+='<p><a href="../diagnostics/findings.json">ジンバルの設定・入力・距離補正の検査と、関節版の中断記録</a></p>'
    for figure in sorted((root/'diagnostics').glob('*_abort.png')):
        page+=f'<img src="{os.path.relpath(figure,out)}">'
    (out/'index.html').write_text(page)
    print(json.dumps(dict(groups=groups,all_dragon_audits_passed=result['all_dragon_audits_passed'],all_cross_seed_configurations_match=result['all_cross_seed_configurations_match']),indent=2))


if __name__ == '__main__':
    main()
