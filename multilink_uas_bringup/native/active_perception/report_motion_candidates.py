#!/usr/bin/env python3
"""Cross-candidate numerical report with development and validation separated."""
import argparse,json,html
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
NAMES=['yaw_m50','front','yaw_p50','pitch_p90','pitch_m90','active']
LABEL=['Yaw -50','Front','Yaw +50','Down','Up','Active']
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('campaigns',type=Path,nargs='+');a=p.parse_args();a.output.mkdir(exist_ok=True,parents=True)
 text='''<!doctype html><meta charset="utf-8"><title>偏向手法の改良比較</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1400px;margin:30px auto;padding:0 20px}table{border-collapse:collapse}td,th{padding:7px;border:1px solid #aaa}img{width:100%}p{max-width:1100px}</style><h1>偏向手法の改良比較</h1><p>主指標は、MIMOSAへ入力したVIO速度の誤差を、全手法共通のLiDAR弱並進方向へ投影したRMSです。小さい方が正確な視覚拘束です。方向を10秒ごとに更新し、60秒区間で集計します。融合後の10秒移動誤差は別指標です。真値は共通の機体飛行制御と事後採点に使用し、視野選択・推定の補正には使いません。ここでの弱方向は純LIOの情報行列内で相対的に弱い並進方向であり、環境そのものの退化判定とは区別します。</p><p>各飛行で固定5方向と偏向を独立の単眼推定器で同時比較。各行は飛行ごとのRMSの平均±試行間標準偏差です。固定平均は5方向を等確率で選ぶ場合の平均、最良固定は試験結果から事後に選んだ参考値です。開発飛行は候補選択に使っており、独立検証へ混ぜません。欠落による無効飛行は除外理由と共に保持します。</p><p><a href="comparison.pdf">比較PDF</a> / <a href="summary.json">全集計JSON</a></p>'''
 result={}
 with PdfPages(a.output/'comparison.pdf') as pdf:
  for root in a.campaigns:
   m=json.loads((root/'campaign.json').read_text());d=json.loads((root/'report/summary.json').read_text());cfgfile=next((root/'implementation').glob('motion_view_v*.yaml'));cfg=yaml.safe_load(cfgfile.read_text());name=root.name
   description=('ROVIOが追跡中の特徴と移動速度の同時の不確かさを使い、移動後も残る特徴を評価。独立の画像角点地図は視野評価に使いません。' if cfg.get('fixation') else '独立の画像特徴地図から2画像間の相対運動の情報を予測。特徴選別は'+('5×5区画の平均情報。' if cfg.get('feature_budget_model')=='spatial_expected' else '寄与上位25点の近似。'))
   if cfg.get('fixation_retinal_candidates'):description+=' 画像内の特徴の動きを小さくするジンバル候補を追加。'
   if cfg.get('committed_track_recovery'):description+=' 追跡点が少ない状態が続いたら正面への復帰を完了させ、十分な継続追跡点を確認してから視野選択へ戻ります。'
   if cfg.get('preview_preparation'):description+=' 既存経路の冒頭を最大15m往復し、戻りと追加26秒の準備運動中は偏向の実角を保持。固定にも同じ身体運動を与える別プロトコルです。往復の指令時間は最大64秒（整定待ち別）。'
   elif cfg.get('common_preparation_cycles',1)>1:description+=' 準備運動を全固定・偏向に共通で2回実施し、2回目は偏向の実角度を保持。標準比較より準備時間が26秒長い別プロトコルです。'
   text+=f'<h2>{html.escape(name)} / {m["stage"]}</h2><p>状態: {m["status"]}、設定: {cfgfile.name}。{description}'+('巡航前の準備運動から視野選択。' if cfg.get('preflight_view_selection') else '軌道追従開始後に視野選択。')+'</p>'
   record=dict(stage=m['stage'],status=m['status'],config=cfgfile.name,description=description,invalid_trials=m.get('invalid_trials',[]),sections={})
   for key,s in d['sections'].items():
    ts=s['windows']['10']['trials'];n=len(ts);vals={q:np.array([t['methods'][q]['visual']['rmse'] for t in ts]) for q in NAMES};av=vals['active'];fm=np.mean([vals[q] for q in NAMES[:5]],axis=0);ref={'zab_b1':'yaw_m50','futatsugoya':'pitch_m90'}.get(s['route']);best=min(NAMES[:5],key=lambda q:vals[q].mean())
    record['sections'][key]=dict(bounds_s=s['bounds_s'],trials=[t['trial'] for t in ts],methods={q:dict(mean=float(v.mean()),values=v.tolist(),sd=float(v.std(ddof=1)) if n>1 else None,active_wins=int((av<v).sum())) for q,v in vals.items()},fixed_uniform_mean=fm.tolist(),active_wins_uniform=int((av<fm).sum()),preselected_reference=ref,retrospective_best=best)
    text+=f'<h3>{html.escape(key)}: {s["bounds_s"]}秒 / {n}飛行</h3><table><tr><th>方向</th><th>VIO方向速度誤差[m/s]</th><th>全3D速度誤差[m/s]</th><th>融合10秒移動誤差[m]</th><th>偏向の勝数</th></tr>'
    for q in NAMES:
     v=vals[q];value=f'{v.mean():.5f}'+(f' ± {v.std(ddof=1):.5f}' if n>1 else '');vec=np.mean([t['methods'][q]['vector_rmse_mps'] for t in ts]);fused=np.mean([t['methods'][q]['fused_displacement']['rmse'] for t in ts]);wins='—' if q=='active' else f'{(av<v).sum()}/{n}'
     text+=f'<tr><td>{q}</td><td>{value}</td><td>{vec:.5f}</td><td>{fused:.5f}</td><td>{wins}</td></tr>'
    text+=f'</table><p>固定平均 {fm.mean():.5f}m/s。偏向が固定平均より小さい試行 {(av<fm).sum()}/{n}。事前比較方向: {ref or "未指定"}。最良固定（事後）: {best}。</p>'
    fig,axes=plt.subplots(1,2,figsize=(13,4.8))
    for ax,field,title in [(axes[0],('visual','rmse'),'VIO velocity error [m/s]'),(axes[1],('fused_displacement','rmse'),'Fused displacement error / 10 s [m]')]:
     for i,q in enumerate(NAMES):
      x=[t['methods'][q][field[0]][field[1]] for t in ts];ax.scatter(i+np.linspace(-.12,.12,n),x,s=30);ax.plot([i-.2,i+.2],[np.mean(x)]*2,color='black')
     ax.set_xticks(range(6),LABEL,rotation=20);ax.set_yscale('log');ax.set_ylabel(title);ax.grid(alpha=.3)
    fig.suptitle(f'{name}: {key} {s["bounds_s"]} s / n={n} / {m["stage"]}');fig.tight_layout();pdf.savefig(fig);fn=f'{name}_{key}.png';fig.savefig(a.output/fn,dpi=120);plt.close(fig);text+=f'<img src="{fn}">'
   result[name]=record
 (a.output/'index.html').write_text(text);(a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(a.output/'index.html')
if __name__=='__main__':main()
