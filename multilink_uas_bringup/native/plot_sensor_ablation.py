#!/usr/bin/env python3
"""Compare adopted DRPM LIO against saved FAST LIO; document unsupported FAST VIO."""
import csv,json,hashlib,warnings,re
from pathlib import Path
from collections import Counter
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from plot_livo_comparison import read,interpolate,LABELS
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';OUT=LOG/'sensor_ablation';BASE=LOG/'livo_comparison';CAL=LOG/'livo_calibrated'
METHODS={'drpm_lio':('MIMOSA + adopted DRPM: LiDAR + IMU','#D55E00'), 'fast_lio':('FAST-LIVO2: LiDAR + IMU','#0072B2'),'rovio':('ROVIO: image + IMU','#009E73')}
def source(label,m):
 if m=='drpm_lio':return OUT/m/label/'odometry.csv'
 if m=='fast_lio':return CAL/m/label/'odometry.csv'
 return BASE/'rovio'/label/'result_vio.csv'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 plots=OUT/'plots';plots.mkdir(exist_ok=True);results={};verified={};htmlrows='';mdrows='| 飛行 | 方式 | 位置RMSE [m] | 姿勢RMSE [deg] |\n|---|---|---:|---:|\n'
 reference=json.loads((CAL/'evaluation.json').read_text())
 groups={'lio_comparison':['drpm_lio','fast_lio'],'drpm_lio_detail':['drpm_lio'],'rovio_only':['rovio']}
 with PdfPages(plots/'comparison.pdf') as pdf:
  for label in LABELS:
   d=OUT/'drpm_lio'/label;summary=json.loads((d/'summary.json').read_text());assert summary['returncode']==0
   cfg=yaml.safe_load((d/'config.yaml').read_text());original=yaml.safe_load((LOG/'drpm_official/official_update'/label/'config.yaml').read_text())
   assert not cfg['odometry']['manager']['enabled'] and not cfg['lidar']['photometric']['enabled']
   normalized=json.loads(json.dumps(cfg));normalized['logs_directory']=original['logs_directory'];normalized['odometry']['manager']['enabled']=True;assert normalized==original
   oldsummary=json.loads((LOG/'drpm_official/official_update'/label/'summary.json').read_text());assert summary['input_sha256']==oldsummary['input_sha256']
   for path,checksum in summary['artifacts'].items():assert checksum==oldsummary['artifacts'][path]
   log=(d/'run.log').read_text();topics=log.split('Topics: \n',1)[1].split('Processing bag:',1)[0].split();assert topics==['/comparison/imu','/comparison/points'],topics
   assert 'd opt value' not in log
   fastcfg=yaml.safe_load((CAL/'fast_lio'/label/'config.yaml').read_text());assert fastcfg['parameters']['common']['img_en']==0
   fastsummary=json.loads((CAL/'fast_lio'/label/'summary.json').read_text());assert fastsummary['returncode']==0 and fastsummary['window']==summary['window']
   fastlog=(CAL/'fast_lio'/label/'run.log').read_text();u=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',fastlog);assert u and int(u[2])==0
   w=summary['window'];start=w['start_ns'];end=w['end_ns'];grid=np.arange(((start+99999999)//100000000)*100000000,end+1,100000000,dtype=np.int64)
   truth=read(BASE/'inputs'/label/'truth.csv');gp,gq=interpolate(truth,grid);g0,q0=interpolate(truth,[start]);data={};metrics={};checks={}
   for method,(name,color) in METHODS.items():
    path=source(label,method);a=read(path);p0,r0=interpolate(a,[start]);R=q0[0]*r0[0].inv();T=g0[0]-R.apply(p0[0]);p,q=interpolate(a,grid);ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
    with warnings.catch_warnings():warnings.simplefilter('ignore');ea=er.as_euler('xyz',degrees=True)
    error=np.c_[ep,ea];assert np.isfinite(error).all();data[method]=error;native=a[0][(a[0]>=start)&(a[0]<=end)]
    step=50000000 if method=='rovio' else 100000000;expected=np.arange(((start+step-1)//step)*step,end+1,step,dtype=np.int64);assert np.array_equal(native,expected),(method,len(native),len(expected))
    m=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(ep**2,axis=1)))),rotation_rmse_deg=float(np.sqrt(np.mean(np.rad2deg(er.magnitude())**2))),position_max_m=float(np.linalg.norm(ep,axis=1).max()),rotation_max_deg=float(np.rad2deg(er.magnitude()).max()),native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9),evaluation_samples=len(grid),alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist())
    if method!='drpm_lio':
     for key in ['position_rmse_m','rotation_rmse_deg']:assert np.isclose(m[key],reference[label]['metrics'][method][key],rtol=0,atol=1e-10)
    metrics[method]=m;checks[method]=dict(source=str(path),source_sha256=sha(path),native_stamps_complete=True,previous_result_reused=method!='drpm_lio')
    with (plots/f'{label}_{method}_errors.csv').open('w') as f:
     writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*error.T))
    htmlrows+=f'<tr><td>{label}</td><td>{name}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td></tr>'
    mdrows+=f'| {label} | {name} | {m["position_rmse_m"]:.6f} | {m["rotation_rmse_deg"]:.6f} |\n'
   registrations=[r for r in csv.DictReader((d/'drpm_registration.csv').open()) if start<=int(r['stamp_ns'])<=end];assert len(registrations)==len(grid)
   checks['drpm_lio']['registration_status']=dict(Counter(r['status'] for r in registrations));checks['drpm_lio']['converged_scans']=sum(int(r['converged']) for r in registrations if r['status']=='ok')
   verified[label]=dict(methods=checks,only_enabled_flag_and_log_directory_changed=True,input_hash_matches_adopted_run=True,read_topics=topics,fast_visual_calls=0)
   metrics['fast_without_lidar']={'status':'unsupported_standalone_vio','position_rmse_m':None,'rotation_rmse_deg':None}
   results[label]=dict(window=w,metrics=metrics)
   for group,methods in groups.items():
    fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
    for j,ax in enumerate(axes):
     for method in methods:ax.plot((grid-start)*1e-9,data[method][:,j],label=METHODS[method][0],color=METHODS[method][1],lw=1.2)
     ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
    axes[0].legend(fontsize=9);axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | '+group+(' (FAST VIO unsupported)' if group=='rovio_only' else ''))
    fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}_{group}.png',dpi=145);pdf.savefig(fig);plt.close(fig)
 (OUT/'evaluation.json').write_text(json.dumps(results,indent=2));(OUT/'verification.json').write_text(json.dumps(verified,indent=2))
 unsupported='FAST-LIVO2のLiDARなしは現行公式実装では単体VIOとして未対応です。画像照合にLiDAR由来の三次元点を使い、画像とIMUだけから地図を初期化・更新する経路がありません。単純にlidar_en=0としてもVIOモードにはなりません。無効な設定での停止やIMUだけの予測をVIO精度として数値化せず、未対応と記録しています。ROVIO単体の保存済み結果は掲載しています。'
 conditions='DRPMは採用した公式更新＋距離方向ノイズ（3cm、最大10反復）を維持し、ROVIO入力だけ無効化。実際に読み込んだトピックはLiDARとIMUだけです。比較対象FAST-LIVO2は測定済みfast_lio（最初から画像なし）を再利用。ROVIOも既存の単眼link3結果を再利用。離陸前から推定し、追従区間だけ共通10Hzで評価。開始時に一度だけ位置・姿勢を整合し、以降の補正や尺度調整はしません。縦軸は省略なし。大きな誤差でDRPMの曲線が読みにくいため、DRPM単独の全範囲図も併記しています。'
 caution='FAST-LIVO2画像なしのkm単位の値は発散した推定結果です。既存の診断では短い飛行は追従開始前に発散しています。離陸等を評価区間から除いても、それまでに生じた速度・姿勢・地図の誤りはリセットしません。「画像ありで初期化して追従中だけ画像を止める」既存条件とは別です。'
 links='<a href="comparison.pdf">PDF</a> / <a href="../evaluation.json">評価値</a> / <a href="../verification.json">検証記録</a> / <a href="../fast_without_lidar_audit.json">FAST-LIVO2のLiDARなし実装確認</a>'
 (plots/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>センサ除外比較</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style><h1>DRPM VisionなしとFAST-LIVO2 Visionなし</h1><p>'+conditions+'</p><p>'+caution+'</p><p>'+links+'</p><table><tr><th>飛行</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'+htmlrows+'</table><h2>ROVIO単体とFAST-LIVO2 LiDARなし</h2><p>'+unsupported+'</p>'+''.join(f'<h2>{l} | {g}</h2><img src="{l}_{g}.png">' for l in LABELS for g in groups))
 (OUT/'README.md').write_text('# センサ除外比較\n\n'+mdrows+'\n'+conditions+'\n\n'+caution+'\n\n## FAST-LIVO2 LiDARなし\n\n'+unsupported+'\n\n固定revision: 0d2c0346107b75b59934975adec9a6eeeb913c64。根拠はfast_without_lidar_audit.jsonの公式ソース位置参照。画像更新だけの独立実験には三次元点の初期化・管理等の新規実装が必要であり、今回の設定切替の比較には含めません。\n')
 print(mdrows)
if __name__=='__main__':main()
