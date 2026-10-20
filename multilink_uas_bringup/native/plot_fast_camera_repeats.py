#!/usr/bin/env python3
"""Report all three repeats; no best-run selection."""
import csv,copy,json,re
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import plot_fast_camera_comparison as p
TRIALS=['','repeat_audit','repeat_b']
def main():
 p.main()
 out=p.OUT;plots=out/'plots';results={};audit={};binaries=set();md='| 飛行 | カメラ | 位置RMSE平均 [最小–最大] m | 姿勢RMSE平均 [最小–最大] deg |\n|---|---|---:|---:|\n';rows=''
 with PdfPages(plots/'comparison.pdf') as pdf:
  for label in p.LABELS:
   w=json.loads((p.BASE/'inputs'/label/'manifest.json').read_text())['window'];s=w['start_ns'];end=w['end_ns'];grid=np.arange((s+99999999)//100000000*100000000,end+1,100000000,dtype=np.int64);truth=p.read(p.BASE/'inputs'/label/'truth.csv');expected=np.arange((s+49999999)//50000000*50000000,end+1,50000000,dtype=np.int64)
   ref=yaml.safe_load((out/'link3'/label/'config.yaml').read_text());data={};results[label]={};audit[label]={}
   for link in (1,3,4):
    trialdata=[];metrics=[];checks=[]
    for i,trial in enumerate(TRIALS):
     d=out/trial/f'link{link}'/label;summary=json.loads((d/'summary.json').read_text());assert summary['returncode']==0 and summary['window']==w;binaries.add(summary['binary_sha256']);cfg=yaml.safe_load((d/'config.yaml').read_text());normalized=copy.deepcopy(cfg)
     for k in ('Rcl','Pcl'):normalized['parameters']['extrin_calib'][k]=ref['parameters']['extrin_calib'][k]
     assert normalized==ref
     a=p.read(d/'odometry.csv');assert np.array_equal(a[0][(a[0]>=s)&(a[0]<=end)],expected)
     e,m=p.errors(a,truth,s,grid)
     norms=np.linalg.norm(e[:,:3],axis=1);m['first_position_error_threshold_crossing_tracking_s']={str(v):float((grid[np.flatnonzero(norms>v)[0]]-s)*1e-9) if np.any(norms>v) else None for v in [1,10,100]}
     trialdata.append(e);metrics.append(m)
     match=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',(d/'run.log').read_text());assert match and int(match[2])>0 and int(match[3])==0
     checks.append(dict(trial=i+1,path=str(d),odometry_sha256=p.sha(d/'odometry.csv'),config_sha256=p.sha(d/'config.yaml'),binary_sha256=summary['binary_sha256'],complete_tracking_stamps=True,lio_updates=int(match[1]),visual_updates=int(match[2]),skipped_visual=int(match[3])))
     with (plots/f'{label}_link{link}_trial{i+1}_errors.csv').open('w') as f:
      writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*e.T))
    stats={k:dict(mean=float(np.mean([m[k] for m in metrics])),min=float(min(m[k] for m in metrics)),max=float(max(m[k] for m in metrics))) for k in ('position_rmse_m','rotation_rmse_deg')}
    representative=int(np.argsort([m['position_rmse_m'] for m in metrics])[1]);results[label][f'link{link}']=dict(trials=metrics,statistics=stats,representative_trial=representative+1,position_over_10m_trials=sum(m['first_position_error_threshold_crossing_tracking_s']['10'] is not None for m in metrics));audit[label][f'link{link}']=checks;data[link]=(trialdata,representative)
    a,b=(stats[k] for k in ('position_rmse_m','rotation_rmse_deg'));pos=f'{a["mean"]:.4f} [{a["min"]:.4f}–{a["max"]:.4f}]';rot=f'{b["mean"]:.4f} [{b["min"]:.4f}–{b["max"]:.4f}]';md+=f'| {label} | link{link} | {pos} | {rot} |\n';rows+=f'<tr><td>{label}</td><td>link{link}</td><td>{pos}</td><td>{rot}</td></tr>'
   fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
   for j,ax in enumerate(axes):
    for link,(curves,representative) in data.items():
     for i,e in enumerate(curves):
      if i!=representative:ax.plot((grid-s)*1e-9,e[:,j],color=p.COLORS[link],lw=.7,alpha=.25)
     ax.plot((grid-s)*1e-9,curves[representative][:,j],color=p.COLORS[link],lw=1.3,label=f'link{link} (trial {representative+1})')
    ax.axhline(0,color='gray',lw=.6);ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
   axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | FAST-LIVO2, 3 repeats per camera\nSolid: median position-RMSE trial; faint: other trials');fig.tight_layout(rect=(0,.015,1,.965));fig.savefig(plots/f'{label}.png',dpi=140);pdf.savefig(fig);plt.close(fig)
   fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True)
   for link,(curves,representative) in data.items():
    for i,e in enumerate(curves):
     norms=[np.linalg.norm(e[:,:3],axis=1),np.rad2deg(Rotation.from_euler('xyz',e[:,3:],degrees=True).magnitude())]
     for j,ax in enumerate(axes):ax.plot((grid-s)*1e-9,norms[j],color=p.COLORS[link],lw=1.3 if i==representative else .7,alpha=1 if i==representative else .25,label=f'link{link}' if i==representative else None)
   for j,ax in enumerate(axes):ax.set_yscale('log');ax.grid(alpha=.25,which='both');ax.set_ylabel(['Position error [m, log scale]','Rotation error [deg, log scale]'][j])
   axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | Error magnitude, logarithmic scale');fig.tight_layout();fig.savefig(plots/f'{label}_magnitude.png',dpi=140);pdf.savefig(fig);plt.close(fig)
 assert len(binaries)==1
 (out/'repeat_evaluation.json').write_text(json.dumps(results,indent=2));(out/'repeat_verification.json').write_text(json.dumps(audit,indent=2))
 description='同じ2飛行を各カメラ3回ずつ、計18回処理。各条件はカメラ1台＋同じLiDAR＋IMU。カメラ画像と記録済みの取り付け位置・向きのみ変更し、他の設定と実行ファイルは共通。軌道追従区間を共通10Hzで評価、開始時の位置・姿勢整合は一度だけ。離陸・向き合わせは初期化に使用し評価から除外。元の推定時刻は保持し、誤差CSVには補間評価時刻を明記。姿勢RMSEは三次元相対回転角。軸の省略・スケール補正・途中の再整合なし。'
 finding='短いrun2a_endは全条件で位置RMSE 0.13m以下。長いrun1_tail_a2_prefixではlink1が3/3試行、link4が2/3試行で発散し、位置誤差が追従開始後128〜137秒に10mを超えました。link4の残る1試行は位置RMSE 0.502m、link3は3試行とも0.353〜0.370m。link4の平均値だけでは成功と発散の混在が分からないため、全試行の範囲・曲線も確認してください。今回の2飛行ではカメラの選択で結果が変わり、link3の結果を他方向のカメラへ一般化できません。原因が特徴不足・三次元点との重なり・初期化・推定器内部のどれかは、今回の精度比較だけでは確定しません。'
 note='同一入力・設定・実行ファイルでも結果にばらつきが出たため、全3試行の平均と最小–最大を掲載。太線は位置RMSEが中央の試行、薄線は残り2試行で、最良試行だけを選んでいません。3回は再実行のばらつきを確認するための回数で、異なる飛行の反復ではありません。過去のlink3値は今回の表に混ぜていません。OpenMPの並列計算を1スレッドに制限した補助試験でも数値は一致せず、ばらつきの原因は未特定です。補助試験は主比較に混ぜていません。'
 (plots/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>FAST-LIVO2 camera comparison</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style><h1>FAST-LIVO2 カメラ比較</h1><p><strong>'+finding+'</strong></p><p>'+description+'</p><p>'+note+'</p><p><a href="comparison.pdf">全図PDF</a> / <a href="../repeat_evaluation.json">全試行の数値</a> / <a href="../repeat_verification.json">実行照合</a> / <a href="../verification.json">入力照合</a></p><table><tr><th>飛行</th><th>カメラ</th><th>位置RMSE 平均 [最小–最大] m</th><th>姿勢RMSE 平均 [最小–最大] deg</th></tr>'+rows+'</table>'+''.join(f'<h2>{l}</h2><img src="{l}.png"><p>誤差の大きさ（対数目盛。軸の省略ではありません）</p><img src="{l}_magnitude.png">' for l in p.LABELS))
 (out/'README.md').write_text('# FAST-LIVO2 カメラ比較\n\n'+finding+'\n\n'+md+'\n'+description+'\n\n'+note+'\n\n再集計: python3 /root/uas_native/plot_fast_camera_repeats.py\n');print(md)
if __name__=='__main__':main()
