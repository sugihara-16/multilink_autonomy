#!/usr/bin/env python3
"""Report all 45 trials, with complete-case means and visible audit failures."""
import argparse
import html
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ARMS = [('unipilot','UniPilot gimbal'),('gimbal','DRAGON gimbal'),('joint','DRAGON joints')]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', type=Path)
    a = p.parse_args()
    root = a.root
    protocol = json.loads((root/'protocol.json').read_text())
    rows = {}
    count = len(protocol['design']['conditions'])
    distance = protocol['design'].get('distance_m',90)
    description = protocol['design'].get('description','基準環境に対し、模様配置・コントラスト・照明・幅をそれぞれ変更。')
    for c in protocol['design']['conditions']:
        name = c['name']
        directory = root/name
        subprocess.run([sys.executable,str(HERE/'audit_practical_comparison.py'),str(directory)],check=True)
        subprocess.run([sys.executable,str(HERE/'report_practical_comparison.py'),str(directory)],check=True)
        data = json.loads((directory/'report/summary.json').read_text())
        stats = {}
        for arm,_ in ARMS:
            trials = [data['results'][f'{arm}_seed{seed}'] for seed in protocol['run_seeds']]
            values = [t['estimators'].get('mimosa',{}).get('position_rmse_m') for t in trials]
            full = all(t['full_interval_available'] for t in trials)
            stats[arm] = dict(values=values, mean=float(np.mean(values)) if full else None,
                sample_std=float(np.std(values,ddof=1)) if full else None,
                min=float(min(values)) if full else None, max=float(max(values)) if full else None,
                complete_intervals=sum(t['full_interval_available'] for t in trials))
        rows[name] = dict(condition=c, fusion_position_rmse_m=stats,
            all_intervals_available=data['all_full_intervals_available'], audit=data['audit'], trials=data['results'])
    out = root/'report';out.mkdir(exist_ok=True)
    result = dict(protocol=protocol,conditions=rows,
        all_intervals_available=all(r['all_intervals_available'] for r in rows.values()),
        all_audits_passed=all(r['audit']['passed'] for r in rows.values()),
        interpretation=f'{count} controlled synthetic conditions, three independent runs per arm and condition. Practical adopted configurations, not a mechanism-only causal comparison. No result selection; missing complete intervals prevent a 3-run mean.')
    (out/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(12,5))
    xx=np.arange(len(rows));width=.25
    for i,(arm,label) in enumerate(ARMS):
        means=[r['fusion_position_rmse_m'][arm]['mean'] for r in rows.values()]
        std=[r['fusion_position_rmse_m'][arm]['sample_std'] for r in rows.values()]
        ax.bar(xx+(i-1)*width,[np.nan if v is None else v for v in means],width,
            yerr=[0 if v is None else v for v in std],capsize=3,label=label)
    ax.set_xticks(xx,list(rows));ax.set_ylabel('Fusion position RMSE [m]')
    ax.set_title(f'{distance:g} m / 0.5 m/s | mean of 3 runs; error bars: sample standard deviation')
    ax.legend();ax.grid(axis='y',alpha=.25);ax.set_axisbelow(True);fig.tight_layout()
    fig.savefig(out/'comparison.png',dpi=160);plt.close(fig)
    page=f'<!doctype html><meta charset="utf-8"><title>{count}環境条件の比較</title><style>body{{font:16px sans-serif;max-width:1200px;margin:30px auto;line-height:1.7}}table{{border-collapse:collapse}}td,th{{padding:8px;border:1px solid #aaa}}img{{max-width:100%}}</style>'
    page+=f'<h1>{count}環境条件 × 3方式 × 3回</h1><p>各試行を新規実行。{distance:g}m・0.5m/s、共通10Hzの評価時刻、開始位置・姿勢のみ整列、尺度補正なし。位置RMSEは位置ずれを二乗して平均し平方根を取った値で、小さいほど良い。融合は画像・慣性・距離センサーを組み合わせた最終推定。</p>'
    page+='<p>'+html.escape(description)+'各条件内では3方式の環境ファイルが共通。機構に応じた視線制御・推定処理を含む実用構成の比較。</p>'
    page+='<p>全評価区間取得：'+str(result['all_intervals_available'])+'／全検査合格：'+str(result['all_audits_passed'])+'。<a href="summary.json">全試行・条件・監査</a></p>'
    page+='<table><tr><th>条件</th>'+''.join('<th>'+label+'</th>' for _,label in ARMS)+'<th>取得／検査</th></tr>'
    for name,row in rows.items():
        page+='<tr><td><a href="../'+name+'/report/index.html">'+html.escape(row['condition']['label'])+'</a></td>'
        for arm,_ in ARMS:
            s=row['fusion_position_rmse_m'][arm]
            page+='<td>'+('未完了' if s['mean'] is None else f'{s["mean"]:.3f} ± {s["sample_std"]:.3f} m')+'</td>'
        page+='<td>'+str(row['all_intervals_available'])+' / '+str(row['audit']['passed'])+'</td></tr>'
    page+='</table><p>表は3回平均 ± 標本標準偏差。未完了区間がある方式の平均は表示しない。検査不合格を精度に基づいて除外しない。各条件のリンク先に全試行の値と位置・姿勢の誤差グラフを示す。</p><img src="comparison.png">'
    (out/'index.html').write_text(page)
    print(json.dumps({k:v['fusion_position_rmse_m'] for k,v in rows.items()},indent=2))


if __name__ == '__main__':
    main()
