#!/usr/bin/env python3
"""Compare identical tracking windows on the existing common 10Hz grid."""
import csv,json,warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from plot_livo_comparison import read,interpolate,LABELS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_official'
BASE=ROOT/'logs/zab-offline/livo_comparison';CAL=ROOT/'logs/zab-offline/livo_calibrated'
METHODS={
 'official_both':('Official update + official noise','#0072B2'),
 'official_update':('Official update + radial noise','#D55E00'),
 'official_noise':('Weighted-cost update + official noise','#009E73'),
 'neither':('Weighted-cost update + radial noise','#CC79A7'),
 'official_noise_legacy_graph':('Previous direct graph + official noise','#888888'),
 'old_radial':('Previous direct graph + radial noise','#AA4499'),
 'no_drpm':('MIMOSA + ROVIO: no DRPM','#222222'),
 'official_both_30':('Official update + official noise: 30 iterations','#E69F00')}
def output_dir(label,method):
    if method=='official_both_30':return OUT/'iteration_check_30/official_both'/label
    return OUT/method/label
def source(label,method):
    if method=='no_drpm':return CAL/'mimosa_calibrated'/label/'odometry.csv'
    if method=='old_radial':return ROOT/'logs/zab-offline/drpm_noise/radial'/label/'odometry.csv'
    return output_dir(label,method)/'odometry.csv'

def evaluate(label):
    w=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'];start=w['start_ns'];end=w['end_ns']
    grid=np.arange(((start+99999999)//100000000)*100000000,end+1,100000000,dtype=np.int64)
    truth=read(BASE/'inputs'/label/'truth.csv');gp,gq=interpolate(truth,grid);g0,q0=interpolate(truth,[start]);data={};metrics={}
    for method in METHODS:
        a=read(source(label,method));p0,r0=interpolate(a,[start]);R=q0[0]*r0[0].inv();T=g0[0]-R.apply(p0[0])
        p,q=interpolate(a,grid);ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
        with warnings.catch_warnings():warnings.simplefilter('ignore');ea=er.as_euler('xyz',degrees=True)
        angles=np.rad2deg(er.magnitude());errors=np.c_[ep,ea];native=a[0][(a[0]>=start)&(a[0]<=end)]
        metrics[method]=dict(position_rmse_m=float(np.sqrt((ep**2).sum(axis=1).mean())),rotation_rmse_deg=float(np.sqrt((angles**2).mean())),
          position_end_m=float(np.linalg.norm(ep[-1])),position_max_m=float(np.linalg.norm(ep,axis=1).max()),
          common_grid_samples=len(grid),native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9),
          native_first_ns=int(native[0]),native_last_ns=int(native[-1]),
          alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist())
        if method not in ['no_drpm','old_radial']:
            summary=json.loads((output_dir(label,method)/'summary.json').read_text());assert summary['returncode']==0
            metrics[method]['wall_s']=summary['wall_s']
        with (OUT/'plots'/f'{label}_{method}_errors.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*errors.T))
        data[method]=errors
    return grid,data,dict(window=w,metrics=metrics)
def main():
    plots=OUT/'plots';plots.mkdir(exist_ok=True);results={}
    groups={'requested':['official_both','official_update','official_noise'],
            'all_controls':[m for m in METHODS if m!='official_both_30'],
            'iteration_limit':['official_both','official_both_30']}
    with PdfPages(plots/'comparison.pdf') as pdf:
        for label in LABELS:
            grid,data,meta=evaluate(label);results[label]=meta
            for group,methods in groups.items():
                fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
                for j,ax in enumerate(axes):
                    for method in methods:
                        name,color=METHODS[method];ax.plot((grid-meta['window']['start_ns'])*1e-9,data[method][:,j],label=name,color=color,lw=1.2)
                    ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
                axes[0].legend(ncol=2,fontsize=8);axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | DRPM update/noise comparison | '+group)
                fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}_{group}.png',dpi=145);pdf.savefig(fig);plt.close(fig)
    (OUT/'evaluation.json').write_text(json.dumps(results,indent=2))
    verified=json.loads((OUT/'verification.json').read_text())['flights']
    rows=''
    for label,r in results.items():
        for k,m in r['metrics'].items():
            audit=verified[label].get(k,{}).get('tracking')
            valid=f"{audit['status_counts'].get('ok',0)} / {audit['scans']}" if audit else '—'
            converged=f"{audit['converged_scans']} / {audit['status_counts'].get('ok',0)}" if audit else '—'
            rows+=f'<tr><td>{label}</td><td>{METHODS[k][0]}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td><td>{valid}</td><td>{converged}</td></tr>'
    (plots/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>DRPM 公式更新と誤差モデルの比較</title>
<style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style>
<h1>DRPM：公式更新と誤差モデルの比較</h1>
<p>同じ2飛行・単眼link3 ROVIO出力・校正済み融合重み。公式の更新式と等方ノイズモデルの両方、更新だけ、誤差モデルだけを使用した3条件を比較。周辺の点群位置合わせと融合処理を共通化しています。</p>
<p>「公式」は公開DRPM計算部を指します。公式ヘッダは改変せず使用。SLAM全体はMIMOSAへの統合で、論文のCompSLAM全体の再現ではありません。位置合わせは最大10回、面の不確かさによる除外と情報行列の受け渡しは論文の方法に従います。</p>
<p>追加対照として、共通構造で両方変更した条件と、前回の直接グラフ内で重みを変更する条件を掲載。前回構造との違いを更新式だけの効果とは解釈しません。</p>
<p>「両方公式」の反復上限30回は打切りの影響を確認する追加実験です。主比較の10回条件とは分けて掲載し、良い結果への差し替えはしません。</p>
<p>離陸前から初期化し、軌道追従区間のみ評価。共通10Hz、追従開始時に一度だけ位置・姿勢を整合。縦軸省略なし。元の推定時刻は各odometry.csvに保存。</p>
<p>「採用」は位置合わせからLiDAR制約を生成できた回数で、十分な収束を意味しません。「停止条件達成」は更新量が並進1mm・回転0.0001rad未満となった回数です。上限まで反復しても条件を満たさない結果は、最終反復の結果として採用しています。対応点不足などの失敗スキャンではIMU/ROVIOのみで推定を継続します。</p>
<p><a href="comparison.pdf">全図PDF</a> / <a href="../README.md">条件・解釈・検証結果</a> / <a href="../evaluation.json">評価値</a> / <a href="../verification.json">採用回数・時刻・条件の検証</a></p>
<table><tr><th>飛行</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th><th>LiDAR採用／全スキャン</th><th>停止条件達成／採用スキャン</th></tr>'''+rows+'</table>'+''.join(f'<h2>{l} | {g}</h2><img src="{l}_{g}.png">' for l in LABELS for g in groups))
    print(json.dumps({l:{k:[m['position_rmse_m'],m['rotation_rmse_deg']] for k,m in r['metrics'].items()} for l,r in results.items()},indent=2))
if __name__=='__main__':main()
