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
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_noise'
BASE=ROOT/'logs/zab-offline/livo_comparison';CAL=ROOT/'logs/zab-offline/livo_calibrated'
METHODS={'legacy_isotropic':('DRPM: previous isotropic','#888888'),
 'propagated_isotropic':('DRPM: propagated isotropic','#E69F00'),
 'radial':('DRPM: corrected radial','#D55E00'),
 'no_drpm':('MIMOSA + ROVIO: no DRPM','#0072B2')}
def source(label,method):
    return (CAL/'mimosa_calibrated' if method=='no_drpm' else OUT/method)/label/'odometry.csv'
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
        if method!='no_drpm':
            summary=json.loads((OUT/method/label/'summary.json').read_text());assert summary['returncode']==0
            metrics[method]['wall_s']=summary['wall_s']
        with (OUT/'plots'/f'{label}_{method}_errors.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*errors.T))
        data[method]=errors
    return grid,data,dict(window=w,metrics=metrics)
def main():
    plots=OUT/'plots';plots.mkdir(exist_ok=True);results={}
    with PdfPages(plots/'comparison.pdf') as pdf:
        for label in LABELS:
            grid,data,meta=evaluate(label);results[label]=meta
            fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
            for j,ax in enumerate(axes):
                for method,(name,color) in METHODS.items():ax.plot((grid-meta['window']['start_ns'])*1e-9,data[method][:,j],label=name,color=color,lw=1.2)
                ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
            axes[0].legend(ncol=2,fontsize=9);axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | DRPM noise model, fixed ROVIO weights')
            fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}.png',dpi=150);pdf.savefig(fig);plt.close(fig)
    (OUT/'evaluation.json').write_text(json.dumps(results,indent=2))
    rows=''.join(f'<tr><td>{label}</td><td>{METHODS[k][0]}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td></tr>' for label,r in results.items() for k,m in r['metrics'].items())
    (plots/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>DRPM ノイズモデル検証</title>
<style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style>
<h1>DRPM：距離方向ノイズへの修正</h1>
<p><strong>ノイズモデルの不一致は修正しましたが、この2例では位置精度は悪化しました。修正後の位置RMSEは23.97m / 5.88m、従来は4.40m / 4.45mです。</strong></p>
<p>同じ2飛行・同じ単眼ROVIO出力・同じ校正済み融合重み。旧等方近似、面の誤差計算を変更した等方近似、距離方向ノイズの3方式を再実行。DRPMなしは前回結果を参照。</p>
<p>センサの設定と保存済み静止点群から、ノイズは距離方向に約3cmであることを確認。面の向きの不確かさは、その面を構成する各点のノイズから計算。地図の各点には取得時の光線方向を保存しています。</p>
<p>離陸前から推定器へ入力し、軌道追従中だけ評価。共通10Hz、追従開始時に一度だけ位置・姿勢を整合。縦軸の省略なし。元の推定時刻は各odometry.csvに保持。</p>
<p>DRPMはMIMOSAへの情報量の重み付けによる移植で、原論文の独立ICP再現ではありません。点同士・共通IMU・地図姿勢の相関は未補正。詳細は<a href="../README.md">検証報告</a>。</p>
<p><a href="comparison.pdf">全図PDF</a> / <a href="../evaluation.json">数値</a> / <a href="../sensor_noise.json">保存点群でのノイズ確認</a></p>
<table><tr><th>飛行</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'''+rows+'</table>'+''.join(f'<h2>{l}</h2><img src="{l}.png">' for l in LABELS))
    print(json.dumps({l:r['metrics'] for l,r in results.items()},indent=2))
if __name__=='__main__':main()
