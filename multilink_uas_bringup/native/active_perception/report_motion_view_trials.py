#!/usr/bin/env python3
"""Candidate improvement report, keeping velocity and fused errors separate."""
import argparse,json,html,yaml
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_direction_repeats import ORDER,LABEL,COLORS


def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
 m=json.loads((root/'campaign.json').read_text());d=json.loads((root/'report/summary.json').read_text())
 out=root/'report';stage=m['stage'];cfg=yaml.safe_load(next((root/'implementation').glob('motion_view_v*.yaml')).read_text());result=dict(stage=stage,complete=m['status']=='complete',sections={})
 text='''<!doctype html><meta charset="utf-8"><title>偏向手法の改良・固定5方向との比較</title><style>body{font:16px sans-serif;line-height:1.65;max-width:1400px;margin:30px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:7px}img{max-width:100%}</style><h1>偏向手法の改良・固定5方向との比較</h1>'''
 text+=f'<p>段階: {stage} / 完了: {result["complete"]}。候補設定は各飛行の前に固定。センサ、ROVIO、MIMOSA、融合重みは従来のまま。DRPM OFF。真値は飛行制御と事後評価だけに使用し、視野選択・推定器に入力していません。</p>'
 text+='<p>改良点: 連続する2画像から移動量を決めるための情報を評価。同じ地図特徴の不確かさを2時刻の視線へ伝播し、LiDARの方向間の相対的な弱さを使用。選択視野を保持する時間と切替え幅を設けます。実ROVIOの将来共分散そのものを計算する方式ではありません。</p>'
 text+='<p>特徴の見積もり: '+('画面を5×5区画に分け、各区画内の情報を平均。幾何的に最良の点をROVIOが選ぶとは仮定しない期待値モデル。' if cfg.get('feature_budget_model')=='spatial_expected' else '弱い並進方向への寄与が高い25点を選べるという計画上の近似。実ROVIOが同じ点を選ぶ保証はない。')+(' 推定器の初期化完了後、巡航前のapproach/excitationから視野を選択。' if cfg.get('preflight_view_selection') else '視野選択は軌道追従開始後から。')+'</p>'
 text+='<p>主指標: 融合へ入力したVIO速度誤差の、全手法共通のLiDAR弱並進方向への投影。10秒ごとに方向を更新し、各60秒区間の誤差RMSを算出。5/20秒も確認。方向は純LIOの情報行列の並進部分から求め、真の環境退化そのものとは区別します。3D速度誤差、傾き、融合後の10秒移動誤差も別に記録。小さな融合誤差だけで視覚拘束改善とは判断しません。</p>'
 text+='<p>各行は独立飛行のRMSを平均。固定平均は指定5方向を等確率で選ぶ場合の平均。以前の試行で選んだ比較方向はZaB b1=(-50,0)、二ツ小屋=(0,-90)。全方向中の最良は事後的な参考値です。試行内の多数の標本を独立試行と数えません。</p>'
 text+='<p><a href="motion_comparison.pdf">比較PDF</a> / <a href="motion_summary.json">比較集計</a> / <a href="summary.json">全数値</a> / <a href="visual_samples.csv">速度誤差CSV</a> / <a href="windows.csv">融合局所移動誤差CSV</a></p>'
 with PdfPages(out/'motion_comparison.pdf') as pdf:
  for key,s in d['sections'].items():
   trials=s['windows']['10']['trials'];N=len(trials)
   values={n:np.array([t['methods'][n]['visual']['rmse'] for t in trials]) for n in ORDER[:6]}
   fixedmean=np.mean([values[n] for n in ORDER[:5]],axis=0)
   chosen={'zab_b1':'yaw_m50','futatsugoya':'pitch_m90'}.get(s['route'])
   stats=dict(bounds_s=s['bounds_s'],trials=[t['trial'] for t in trials],methods={n:dict(values=v.tolist(),mean=float(v.mean()),sd=float(v.std(ddof=1)) if N>1 else None,active_wins=int(np.sum(values['active']<v))) for n,v in values.items()},fixed_mean=fixedmean.tolist(),active_wins_fixed_mean=int(np.sum(values['active']<fixedmean)),preselected_fixed=chosen)
   stats['sensitivity']={W:{n:float(np.mean([t['methods'][n]['visual']['rmse'] for t in w['trials']])) for n in ORDER[:6]} for W,w in s['windows'].items()}
   result['sections'][key]=stats
   title=f"{s['route']} {s['bounds_s']} s / n={N}";text+=f'<h2>{html.escape(title)}</h2><table><tr><th>方向</th><th>VIO方向誤差[m/s]</th><th>3D速度誤差[m/s]</th><th>融合10秒移動誤差[m]</th><th>偏向の勝数</th></tr>'
   for n in ORDER[:6]:
    vec=np.mean([t['methods'][n]['vector_rmse_mps'] for t in trials]);fu=np.mean([t['methods'][n]['fused_displacement']['rmse'] for t in trials]);sd=stats['methods'][n]['sd'];num=f'{values[n].mean():.5f}'+(f' ± {sd:.5f}' if sd is not None else '')
    win_label='—' if n=='active' else f'{stats["methods"][n]["active_wins"]}/{N}'
    text+=f'<tr><td>{LABEL[n]}</td><td>{num}</td><td>{vec:.5f}</td><td>{fu:.5f}</td><td>{win_label}</td></tr>'
   text+=f'</table><p>固定5方向の平均: {fixedmean.mean():.5f} m/s / 偏向が固定平均を下回った試行: {stats["active_wins_fixed_mean"]}/{N}</p>'
   fig,axs=plt.subplots(1,2,figsize=(13,5))
   for ax,field,ylabel in [(axs[0],('visual','rmse'),'VIO directional velocity RMS [m/s]'),(axs[1],('fused_displacement','rmse'),'Fused 10 s displacement RMS [m]')]:
    for i,n in enumerate(ORDER[:6]):
     vals=[t['methods'][n][field[0]][field[1]] for t in trials];ax.scatter(np.full(N,i)+np.linspace(-.12,.12,N),vals,color=COLORS[i]);ax.plot([i-.2,i+.2],[np.mean(vals)]*2,color=COLORS[i],lw=3)
    ax.set_xticks(range(6),[LABEL[n] for n in ORDER[:6]],rotation=25,ha='right');ax.set_ylabel(ylabel);ax.set_yscale('log');ax.grid(alpha=.3)
   fig.suptitle(title+' / '+stage);fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(key+'_motion.png'),dpi=130);plt.close(fig);text+=f'<img src="{key}_motion.png">'
 (out/'motion_summary.json').write_text(json.dumps(result,indent=2)+'\n');(out/'motion_index.html').write_text(text)
 # Correct inherited narrative; the numeric report is reused, not its old experiment claims.
 old=out/'index.html';source=old.read_text();start=source.find('<p>ZaB a1/b1');end=source.index('</p>',start)+4 if start>=0 else 0
 source=(source[:start]+f'<p>本ページは偏向手法改良の{stage}試験の数値詳細です。実行済み反復数は各節に表示。<a href="motion_index.html">実験条件と改良比較</a>を参照してください。旧v57ではなく明示保存された新制御を使用しています。センサ・推定器・融合重みは不変です。</p>'+source[end:]) if start>=0 else source
 source=source.replace('7区間×5方向の探索結果です。','記載した全区間・全方向の結果です。').replace('他の2試行だけで選ぶ評価','他の試行だけで選ぶ評価')
 old.write_text(source)
 print(out/'motion_index.html')
if __name__=='__main__':main()
