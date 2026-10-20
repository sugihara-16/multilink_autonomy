#!/usr/bin/env python3
"""Generate complete, unbroken six-axis error plots for the all-flight campaign."""
import csv,json,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_multicamera_all'
def main():
 plots=OUT/'plots';plots.mkdir(exist_ok=True);entries=json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'];rows=[];allmetrics={};figures=[]
 with PdfPages(plots/'all_flights.pdf') as pdf:
  for entry in entries:
   label=entry['label'];d=OUT/label
   if not (d/'metrics.json').exists():continue
   s=json.loads((d/'summary.json').read_text());m=json.loads((d/'metrics.json').read_text());allmetrics[label]=dict(metrics=m,window=s['window'],reused=s['reused']);a=np.genfromtxt(d/'errors.csv',delimiter=',',names=True);t=(a['evaluation_stamp_ns']-s['window']['start_ns'])*1e-9
   fig,axes=plt.subplots(6,1,figsize=(14,15),sharex=True)
   for j,(k,axis) in enumerate(zip(a.dtype.names[1:],['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'])):
    ax=axes[j];ax.plot(t,a[k],color='#0072B2',lw=1,label='FAST-LIVO2 3 cameras');ax.axhline(0,color='gray',lw=.5);ax.grid(alpha=.25);ax.set_ylabel(axis)
   axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(f'{label} | Position RMSE {m["position_rmse_m"]:.3f} m | Rotation RMSE {m["rotation_rmse_deg"]:.3f} deg');fig.tight_layout(rect=(0,0,1,.97));pdf.savefig(fig);fig.savefig(plots/f'{label}.png',dpi=120);plt.close(fig)
   rows.append(f'<tr><td>{label}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td><td>{m["position_max_m"]:.4f}</td><td>{m["native_samples"]}</td><td>{"既存trial1" if s["reused"] else "新規"}</td></tr>');figures.append(f'<h2>{label}</h2><p><a href="../{label}/odometry.csv">推定CSV</a> / <a href="../{label}/truth.csv">真値CSV</a> / <a href="../{label}/errors.csv">誤差CSV</a></p><img src="{label}.png">')
 (OUT/'evaluation.json').write_text(json.dumps(allmetrics,indent=2))
 (plots/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>FAST-LIVO2 全飛行・3カメラ</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}p{line-height:1.7}</style><h1>FAST-LIVO2 全飛行・3カメラ</h1><p>完了 '+str(len(allmetrics))+'/14run</p><p>link1・link3・link4の順で画像を共通の推定状態に反映。設定は前回の3台カメラ実験と同一。離陸・向き合わせは初期化のみに使用。評価は軌道追従区間に限り、開始時の位置・姿勢を一度だけ整合します。途中の再整合、スケール補正、縦軸の省略なし。元推定値・真値CSVは元のheader stamp（整数ナノ秒）を保持。誤差CSVは共通10Hzへの補間評価時刻です。既存の2飛行は、今回の新規計算前にtrial1の再利用を決定しました。</p><p><a href="all_flights.pdf">全飛行PDF</a> / <a href="../evaluation.json">評価数値</a></p><table><tr><th>飛行</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th><th>最大位置誤差 [m]</th><th>推定数</th><th>取得</th></tr>'+''.join(rows)+'</table>'+''.join(figures))
 print('Plotted',len(allmetrics),'flights')
if __name__=='__main__':main()
