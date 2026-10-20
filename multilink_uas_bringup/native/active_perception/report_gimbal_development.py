#!/usr/bin/env python3
"""Small live development index; keep preregistered and observed results separate."""
import argparse, html, json
from datetime import datetime, timezone
from pathlib import Path
from report_gimbal_improvement import assess, frozen_evidence


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--candidate',required=True)
    a=p.parse_args();declaration=json.loads((a.root/f'development_{a.candidate}.json').read_text())
    rows=[];pending=[]
    for name in declaration['campaigns']:
        c=a.root/name;result=c/'results.json'
        found=json.loads(result.read_text()) if result.exists() else {}
        for label,r in found.items():
            if 'metrics' not in r:continue
            checks=frozen_evidence(c/label,declaration)
            rows.append(dict(r,campaign=name,route=label.removesuffix('_active'),
                             saved_conditions_match=all(checks.values()),valid=r['valid'] and all(checks.values())))
        for d in c.glob('*_active'):
            status=d/'route_status.json'
            if d.name not in found and status.exists():
                s=json.loads(status.read_text());runtime=d/'runtime.json'
                done=json.loads(runtime.read_text()) if runtime.exists() else {}
                pending.append(dict(campaign=name,route=d.name,stage=done.get('status',s['stage']),route_time_s=s['route_time_s'],stopped=done.get('stopped',False)))
    verdict=assess(rows,declaration['criterion'],['run1_tail_a1','run1_tail_b1','run1_prefix'],declaration['seeds'])
    data=dict(candidate=a.candidate,updated_utc=datetime.now(timezone.utc).isoformat(),
              scope='Development evidence only. No claim of independent validation.',
              original_criterion_verdict=verdict,completed=rows,pending=pending)
    (a.root/f'development_summary_{a.candidate}.json').write_text(json.dumps(data,indent=2)+'\n')
    body=[]
    for r in rows:
        f=r['metrics']['fixed'];g=r['metrics']['active']
        cells=[f'<a href="{html.escape(r["campaign"])}/index.html">{html.escape(r["route"])} / {r["seed"]}</a>',
               str(r['valid'])]
        for est in ['mimosa','rovio']:
            for metric in ['position_rmse_m','rotation_rmse_deg']:
                cells.append(f'{f[est][metric]:.3f} → {g[est][metric]:.3f}')
        body.append('<tr>'+''.join('<td>'+c+'</td>' for c in cells)+'</tr>')
    state='開発比較の事前基準に合格。独立検証は別途必要。' if verdict['passed'] else '安定した改善の確認は未完了。事前基準を満たしたとは扱いません。'
    if (a.root/f'{a.candidate}_early_rejection.json').exists():state='この候補は不採用。結果と途中停止の記録を保持しています。'
    text='''<!doctype html><meta charset="utf-8"><title>Gimbal development evidence</title>
<style>body{font:16px sans-serif;max-width:1300px;margin:30px auto;line-height:1.6}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:8px;text-align:right}td:first-child{text-align:left}</style>'''
    text+=f'<h1>{html.escape(a.candidate)}：固定カメラと可動カメラの比較</h1><p><b>{state}</b></p>'
    text+='<p>同一の物理飛行・IMU・LiDAR・推定器設定を共有。ROVIO＋MIMOSA、DRPM '+('ON' if declaration.get('drpm_fusion',False) else 'OFF')+'。評価前の準備運動を揃え、軌道追従中のみ比較。開始時の位置・姿勢だけ合わせ、距離の尺度は補正しません。矢印は固定→可動。数値は二乗平均平方根誤差です。</p>'
    text+='<table><tr><th>区間 / 乱数条件</th><th>入力・終了・条件監査</th><th>融合位置 m</th><th>融合回転 deg</th><th>ROVIO位置 m</th><th>ROVIO回転 deg</th></tr>'+''.join(body)+'</table>'
    text+='<p>元の事前基準：全6比較が有効、5勝以上、各経路の平均誤差比≤0.9、全比較の平均比≤0.8、最悪比≤1.5。改善率が小さい結果も悪化も隠さず表示します。</p>'
    text+='<p>完了 '+str(len(rows))+'/6。'+html.escape(str(verdict['checks']))+'</p>'
    text+='<p>進行中：'+html.escape(json.dumps(pending,ensure_ascii=False))+'</p>'
    text+=f'<p><a href="development_summary_{a.candidate}.json">数値と監査結果</a> / <a href="development_{a.candidate}.json">事前に固定した条件</a></p><p>更新 UTC {data["updated_utc"]}</p>'
    (a.root/f'{a.candidate}_progress.html').write_text(text)
    print(json.dumps(dict(completed=len(rows),pending=pending,verdict=verdict)))


if __name__=='__main__':main()
