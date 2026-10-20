#!/usr/bin/env python3
"""Camera-only FAST-LIVO2 comparison, preserving source odometry timestamps."""
import csv,hashlib,json,re,copy,warnings
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from plot_livo_comparison import read,interpolate
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';BASE=LOG/'livo_comparison';OUT=LOG/'fast_camera_comparison'
LABELS=['run2a_end','run1_tail_a2_prefix'];COLORS={1:'#D55E00',3:'#0072B2',4:'#009E73'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def errors(a,truth,start,grid):
 gp,gq=interpolate(truth,grid);g0,q0=interpolate(truth,[start]);p0,r0=interpolate(a,[start]);R=q0[0]*r0[0].inv();T=g0[0]-R.apply(p0[0]);p,q=interpolate(a,grid)
 ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
 with warnings.catch_warnings():warnings.simplefilter('ignore');e=np.c_[ep,er.as_euler('xyz',degrees=True)]
 assert np.isfinite(e).all()
 m=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(ep**2,axis=1)))),rotation_rmse_deg=float(np.sqrt(np.mean(np.rad2deg(er.magnitude())**2))),axis_rmse=np.sqrt(np.mean(e**2,axis=0)).tolist(),position_end_m=float(np.linalg.norm(ep[-1])),position_max_m=float(np.linalg.norm(ep,axis=1).max()),rotation_max_deg=float(np.rad2deg(er.magnitude()).max()),alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist())
 return e,m

def main():
 plots=OUT/'plots';plots.mkdir(exist_ok=True);result={};audit={};binaries=set();rows='';md='| 飛行 | カメラ | 位置RMSE [m] | 姿勢RMSE [deg] |\n|---|---|---:|---:|\n'
 with PdfPages(plots/'comparison.pdf') as pdf:
  for label in LABELS:
   manifest=json.loads((BASE/'inputs'/label/'manifest.json').read_text());w=manifest['window'];start=w['start_ns'];end=w['end_ns'];truthpath=BASE/'inputs'/label/'truth.csv';truth=read(truthpath);grid=np.arange((start+99999999)//100000000*100000000,end+1,100000000,dtype=np.int64)
   expected=np.arange((start+49999999)//50000000*50000000,end+1,50000000,dtype=np.int64)
   refcfg=yaml.safe_load((OUT/'link3'/label/'config.yaml').read_text());oldcfg=yaml.safe_load((LOG/'livo_calibrated/fast_full'/label/'config.yaml').read_text());assert refcfg==oldcfg
   data={};metrics={};checks={}
   for link in (1,3,4):
    d=OUT/f'link{link}'/label;s=json.loads((d/'summary.json').read_text());assert s['returncode']==0 and s['window']==w;binaries.add(s['binary_sha256']);cfg=yaml.safe_load((d/'config.yaml').read_text());normalized=copy.deepcopy(cfg)
    normalized['parameters']['extrin_calib']['Rcl']=refcfg['parameters']['extrin_calib']['Rcl'];normalized['parameters']['extrin_calib']['Pcl']=refcfg['parameters']['extrin_calib']['Pcl'];assert normalized==refcfg
    match=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',(d/'run.log').read_text());assert match and int(match[2])>0 and int(match[3])==0
    a=read(d/'odometry.csv');native=a[0][(a[0]>=start)&(a[0]<=end)];assert np.array_equal(native,expected)
    e,m=errors(a,truth,start,grid);m.update(native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9),common_grid_samples=len(grid));data[link]=e;metrics[f'link{link}']=m
    checks[f'link{link}']=dict(odometry_sha256=sha(d/'odometry.csv'),config_sha256=sha(d/'config.yaml'),only_camera_extrinsic_config_changed=True,tracking_stamps_complete=True,lio_updates=int(match[1]),visual_updates=int(match[2]),skipped_visual=int(match[3]),binary_sha256=s['binary_sha256'])
    if link!=3:
     check=json.loads((OUT/'inputs'/f'{label}_link{link}.json').read_text());assert check['imu_lidar_serialized_identical'] and check['message_order_and_timestamps_identical'];checks[f'link{link}']['input_verification']=check
    with (plots/f'{label}_link{link}_errors.csv').open('w') as f:
     writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*e.T))
    rows+=f'<tr><td>{label}</td><td>link{link}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td></tr>'
    md+=f'| {label} | link{link} | {m["position_rmse_m"]:.6f} | {m["rotation_rmse_deg"]:.6f} |\n'
   old=read(LOG/'livo_calibrated/fast_full'/label/'odometry.csv');new=read(OUT/'link3'/label/'odometry.csv');assert np.array_equal(old[0],new[0]);_,oldmetrics=errors(old,truth,start,grid)
   checks['old_link3_reproduction']=dict(old_metrics=oldmetrics,current_metrics=metrics['link3'],stamps_identical=True,poses_bit_identical=bool(np.array_equal(old[1],new[1]) and np.array_equal(old[2].as_quat(),new[2].as_quat())),max_unaligned_position_difference_m=float(np.linalg.norm(old[1]-new[1],axis=1).max()),max_rotation_difference_deg=float(np.rad2deg((old[2].inv()*new[2]).magnitude()).max()),old_binary_sha256=json.loads((LOG/'livo_calibrated/fast_full'/label/'summary.json').read_text())['binary_sha256'])
   checks['image_preprocessing']={c:json.loads((OUT/'images'/label/(Path(c).stem+'.json')).read_text()) for c in manifest['chunks']};assert all(c['link3_preprocessing_byte_identical'] for c in checks['image_preprocessing'].values())
   result[label]=dict(window=w,metrics=metrics);audit[label]=dict(truth_sha256=sha(truthpath),checks=checks)
   fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
   for j,ax in enumerate(axes):
    for link in (1,3,4):ax.plot((grid-start)*1e-9,data[link][:,j],label=f'FAST-LIVO2 link{link}',color=COLORS[link],lw=1.2)
    ax.axhline(0,color='gray',lw=.6);ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
   axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | FAST-LIVO2 camera comparison');fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}.png',dpi=140);pdf.savefig(fig);plt.close(fig)
 assert len(binaries)==1
 (OUT/'evaluation.json').write_text(json.dumps(result,indent=2));(OUT/'verification.json').write_text(json.dumps(audit,indent=2))
 description='同じ2飛行の保存済みrosbagを使用。各条件はカメラ1台＋同じLiDAR＋IMU。カメラ画像と、記録済み校正から得た取り付け位置・向きのみを変更し、推定器の設定は共通です。3条件とも今回同じ実行ファイルで計算。軌道追従区間を共通10Hzで評価し、開始時の位置・姿勢を一度だけ整合。離陸等は初期化に使用しますが評価から除外。軸省略・途中の再整合・スケール補正なし。元推定値の時刻は変更せず、誤差CSVには補間評価時刻を記録。姿勢RMSEは三次元の相対回転角、6軸図は相対回転のロール・ピッチ・ヨーです。'
 note='再現性の注意：過去のlink3結果とは数値が完全一致しません。過去と今回の実行ファイルのハッシュも異なりますが、差の原因は未特定です。この比較には過去値を混ぜず、今回のlink3再計算を使用します。旧値との差と入力照合はverification.jsonに保存しています。'
 (plots/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>FAST-LIVO2 camera comparison</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style><h1>FAST-LIVO2 カメラ比較</h1><p>'+description+'</p><p>'+note+'</p><p><a href="comparison.pdf">全図PDF</a> / <a href="../evaluation.json">数値</a> / <a href="../verification.json">照合記録</a></p><table><tr><th>飛行</th><th>カメラ</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'+rows+'</table>'+''.join(f'<h2>{l}</h2><img src="{l}.png">' for l in LABELS))
 (OUT/'README.md').write_text('# FAST-LIVO2 カメラ比較\n\n'+md+'\n'+description+'\n\n'+note+'\n\n再現: prepare_fast_camera_inputs.py → comparison_ros1.sh make_fast_camera_bags.py → comparison_ros1.sh run_fast_camera_comparison.py → plot_fast_camera_comparison.py。画像キャッシュ・比較用bagはローカルのみ。元クラウドbag・過去の評価を変更していません。\n');print(md)
if __name__=='__main__':main()
