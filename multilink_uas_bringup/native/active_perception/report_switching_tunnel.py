#!/usr/bin/env python3
"""Known-axis evaluation for a constant-section, texture-switching tunnel."""
import argparse,csv,json,html,os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_direction_repeats import ORDER,LABEL,COLORS

def rms(x):return float(np.sqrt(np.mean(np.square(x))))

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root;out=r/'report';m=json.loads((r/'campaign.json').read_text())
    with (out/'visual_samples.csv').open() as f:rows=[v for v in csv.DictReader(f) if int(v['window_s'])==10]
    for v in rows:
        v['error_xyz_mps']=json.loads(v['error_xyz_mps'])
        for key in ['elapsed_s','features','yaw_deg','pitch_deg','error_mps']:v[key]=float(v[key])
    result=dict(status=m['status'],policy='v10, unchanged',primary='VIO velocity RMS along world X, the exact axial null direction of the noiseless extruded tunnel.',independent_unit='physical flight; four sections within a flight are correlated',conditions={},sections={})
    doc='''<!doctype html><meta charset="utf-8"><title>特徴が偏在するトンネル：固定と偏向</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1400px;margin:30px auto;padding:0 20px}td,th{border:1px solid #aaa;padding:7px}table{border-collapse:collapse}img{max-width:100%}</style><h1>特徴が偏在するトンネル：固定と偏向</h1><p>同じ幅6m・高さ5mの断面を持つトンネルで、模様の濃い面だけを32mごとに変更。左壁→天井→右壁→床、その逆順、全周に模様がある対照条件を比較します。形状・照明・速度・センサは共通。現実のスキャンではなく、作用を切り分けるための人工環境です。</p><p>偏向は直前に実装したv10を変更せず使用。配置の正解・真値は偏向と推定器へ渡しません。真値による機体制御で同じ軌道を飛行し、固定5方向と偏向の単眼ROVIO＋MIMOSAを同時比較。DRPM OFF。1条件3飛行、1飛行130m・0.5m/s。準備運動は共通で評価から除外します。</p><p>主指標はトンネル軸X方向のVIO速度誤差です。一定断面の壁面は、理想的な点と平面の照合ではこの方向を拘束できません。各面の中央40秒×4区間を統合したRMSを飛行ごとに算出。固定平均、飛行全体で一つ選ぶ最良固定、区間ごとに選び直す参考値を区別します。融合後の位置誤差は別に示します。3試行の結果を統計的有意性とは呼びません。</p><p><a href="switching_comparison.pdf">PDF</a> / <a href="switching_summary.json">集計JSON</a> / <a href="summary.json">従来の推定LiDAR弱方向指標</a></p>'''
    with PdfPages(out/'switching_comparison.pdf') as pdf:
        for condition in m['entries']:
            keys=[k for k in m['order'] if k.rsplit('_seed',1)[0]==condition and m['runs'].get(k,{}).get('status')=='complete']
            pooled={};sections={};position={}
            for key in keys:
                pooled[key]={};sections[key]={};position[key]={}
                ar=json.loads((r/key/'array_comparison.json').read_text())
                for name in ORDER[:6]:
                    vv=[v for v in rows if v['trial']==key and v['method']==name]
                    if len(vv)!=160:raise ValueError(f'Expected 160 primary samples: {key} {name}: {len(vv)}')
                    pooled[key][name]=rms([v['error_xyz_mps'][0] for v in vv])
                    sections[key][name]=[rms([v['error_xyz_mps'][0] for v in vv if int(v['segment'])==i]) for i in range(4)]
                # Full-trajectory fused errors are from the unmodified paired reporter.
                position[key]=ar.get('metrics',{})
            methods={name:dict(values=[pooled[k][name] for k in keys],mean=float(np.mean([pooled[k][name] for k in keys])),sd=float(np.std([pooled[k][name] for k in keys],ddof=1)) if len(keys)>1 else None) for name in ORDER[:6]}
            best=min(ORDER[:5],key=lambda n:methods[n]['mean']);pairs=[]
            for k in keys:
                fixed=[pooled[k][n] for n in ORDER[:5]];active=pooled[k]['active'];oracle=rms([min(sections[k][n][i] for n in ORDER[:5]) for i in range(4)])
                pairs.append(dict(trial=k,active=active,fixed_mean=float(np.mean(fixed)),best_fixed_in_flight=min(fixed),active_wins_fixed_mean=active<np.mean(fixed),active_wins_all_fixed=active<min(fixed),per_zone_oracle_rms=oracle))
            result['conditions'][condition]=dict(trials=keys,methods=methods,best_direction_of_mean=best,paired=pairs,sections=sections,full_fused_metrics=position)
            doc+=f'<h2>{html.escape(condition)} — {len(keys)}飛行</h2><table><tr><th>方向</th><th>軸方向のVIO速度RMS：平均 ± 試行間標準偏差 [m/s]</th><th>各飛行</th></tr>'
            for n,v in methods.items():doc+=f'<tr><td>{LABEL[n]}</td><td>{v["mean"]:.5f} ± {(v["sd"] or 0):.5f}</td><td>'+', '.join(f'{x:.5f}' for x in v['values'])+'</td></tr>'
            doc+=f'</table><p>全区間で一つ選ぶ最良固定（結果を見た参考値）: {LABEL[best]}。偏向が固定平均より小さい飛行 {sum(x["active_wins_fixed_mean"] for x in pairs)}/{len(keys)}、全固定より小さい飛行 {sum(x["active_wins_all_fixed"] for x in pairs)}/{len(keys)}。</p>'
            sheet=r.parent/'optical_check_v2'/f'{condition}_contact_sheet.jpg'
            if sheet.exists():doc+=f'<details><summary>Gazeboでの画像確認（画像上の角の検出数）</summary><img src="{os.path.relpath(sheet,out)}"></details>'
            fig,axs=plt.subplots(1,2,figsize=(14,5))
            for j,n in enumerate(ORDER[:6]):
                values=np.array([sections[k][n] for k in keys]);axs[0].plot(range(1,5),values.mean(axis=0),'-o',color=COLORS[j],label=LABEL[n]);axs[1].scatter(j+np.linspace(-.12,.12,len(keys)),methods[n]['values'],color=COLORS[j]);axs[1].plot([j-.2,j+.2],[methods[n]['mean']]*2,color=COLORS[j])
            axs[0].set_xticks(range(1,5));axs[0].set_xlabel('Texture zone');axs[0].legend(fontsize=8);axs[1].set_xticks(range(6),[LABEL[n] for n in ORDER[:6]],rotation=25)
            for ax in axs:ax.set_yscale('log');ax.set_ylabel('Axial VIO velocity RMS [m/s]');ax.grid(alpha=.3)
            fig.suptitle(condition+' / independent flights n='+str(len(keys)));fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'{condition}_axial.png',dpi=140);plt.close(fig);doc+=f'<img src="{condition}_axial.png">'
    (out/'switching_summary.json').write_text(json.dumps(result,indent=2)+'\n');(out/'switching_index.html').write_text(doc)
    # Correct inherited explanatory text without modifying numerical evaluation.
    legacy=out/'index.html';txt=legacy.read_text();import re
    txt=re.sub(r'<p>ZaB a1/b1.*?</p>','<p>人工トンネルの特徴配置3条件×3飛行。偏向v10、独立した固定5方向、DRPM OFF。正確な条件と既知軸の主指標は<a href="switching_index.html">専用レポート</a>を参照。</p>',txt,count=1,flags=re.S)
    txt=re.sub(r'<p>ZaB b1の180.*?</p>','<p>全条件とも直進0.5m/s。時刻区間は環境生成前に定義し、結果を見て選択していません。</p>',txt,count=1,flags=re.S)
    txt=txt.replace('真値は採点にだけ使用。','真値は共通の機体制御と事後採点に使用。').replace('7区間×5方向の探索結果です。','3条件×4区間×5固定方向の比較です。');legacy.write_text(txt)
    print(out/'switching_index.html')

if __name__=='__main__':main()
