#!/usr/bin/env python3
"""Re-evaluate saved adopted DRPM and image-enabled FAST-LIVO2 on one time grid."""
import csv, hashlib, json, re, warnings
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from plot_livo_comparison import read, interpolate, LABELS
ROOT=Path(__file__).resolve().parent
LOG=ROOT/'logs/zab-offline'; BASE=LOG/'livo_comparison'
OUT=LOG/'adopted_drpm_vs_fastlivo2'
METHODS={
 'adopted_drpm':('MIMOSA + DRPM + ROVIO (official update, radial noise)',LOG/'drpm_official/official_update','#D55E00'),
 'fast_livo2':('FAST-LIVO2 (LiDAR + IMU + image)',LOG/'livo_calibrated/fast_full','#0072B2')}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    plots=OUT/'plots';plots.mkdir(parents=True,exist_ok=True)
    references={
      'adopted_drpm':(json.loads((LOG/'drpm_official/evaluation.json').read_text()),'official_update'),
      'fast_livo2':(json.loads((LOG/'livo_calibrated/evaluation.json').read_text()),'fast_full')}
    selection=json.loads((ROOT/'comparison/adopted_drpm.json').read_text())
    results={};verification={};rows='';markdown='| 飛行 | 方式 | 位置RMSE [m] | 姿勢RMSE [deg] |\n|---|---|---:|---:|\n'
    with PdfPages(plots/'comparison.pdf') as pdf:
      for label in LABELS:
        w=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'];start=w['start_ns'];end=w['end_ns']
        grid=np.arange(((start+99999999)//100000000)*100000000,end+1,100000000,dtype=np.int64)
        truthpath=BASE/'inputs'/label/'truth.csv';truth=read(truthpath);gp,gq=interpolate(truth,grid);g0,q0=interpolate(truth,[start])
        data={};metrics={};audit={};stamps=[]
        for method,(name,directory,color) in METHODS.items():
          d=directory/label;summary=json.loads((d/'summary.json').read_text());assert summary['returncode']==0 and summary['window']==w
          cfg=yaml.safe_load((d/'config.yaml').read_text())
          if method=='adopted_drpm':
            reg=cfg['lidar']['geometric']['scan_to_map']
            for k,v in selection['scan_to_map'].items():assert reg[k]==v,(k,reg[k],v)
          else:
            assert cfg['parameters']['common']['img_en']==1 and not cfg['disable_visual_update']
            bag=Path(summary['command'][1]);assert bag.resolve()==(BASE/'inputs'/f'{label}.bag').resolve()
            match=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',(d/'run.log').read_text())
            assert match and int(match[2])>0 and int(match[3])==0
          a=read(d/'odometry.csv');p0,r0=interpolate(a,[start]);R=q0[0]*r0[0].inv();T=g0[0]-R.apply(p0[0]);p,q=interpolate(a,grid)
          ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
          with warnings.catch_warnings():warnings.simplefilter('ignore');ea=er.as_euler('xyz',degrees=True)
          errors=np.c_[ep,ea];assert np.isfinite(errors).all();native=a[0][(a[0]>=start)&(a[0]<=end)];stamps.append(native)
          expected=np.arange(((start+49999999)//50000000)*50000000,end+1,50000000,dtype=np.int64);assert np.array_equal(native,expected)
          m=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(ep**2,axis=1)))),rotation_rmse_deg=float(np.sqrt(np.mean(np.rad2deg(er.magnitude())**2))),
            axis_rmse=np.sqrt(np.mean(errors**2,axis=0)).tolist(),axis_order=['x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg'],
            position_max_m=float(np.linalg.norm(ep,axis=1).max()),position_end_m=float(np.linalg.norm(ep[-1])),
            rotation_max_deg=float(np.rad2deg(er.magnitude()).max()),native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9),
            common_grid_samples=len(grid),alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist())
          reference,key=references[method]
          for k in ['position_rmse_m','rotation_rmse_deg']:assert np.isclose(m[k],reference[label]['metrics'][key][k],rtol=0,atol=1e-12)
          metrics[method]=m;data[method]=errors
          audit[method]=dict(odometry_path=str(d/'odometry.csv'),odometry_sha256=digest(d/'odometry.csv'),config_path=str(d/'config.yaml'),config_sha256=digest(d/'config.yaml'),summary_path=str(d/'summary.json'),prior_metrics_reproduced=True,native_stamps_complete=True)
          with (plots/f'{label}_{method}_errors.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*errors.T))
          rows+=f'<tr><td>{label}</td><td>{name}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td></tr>'
          markdown+=f'| {label} | {name} | {m["position_rmse_m"]:.6f} | {m["rotation_rmse_deg"]:.6f} |\n'
        assert np.array_equal(*stamps)
        results[label]=dict(window=w,metrics=metrics);verification[label]=dict(methods=audit,truth_sha256=digest(truthpath),same_tracking_stamps=True)
        fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
        for j,ax in enumerate(axes):
          for method,(name,_,color) in METHODS.items():ax.plot((grid-start)*1e-9,data[method][:,j],label=name,color=color,lw=1.3)
          ax.axhline(0,color='gray',lw=.6);ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
        axes[0].legend(fontsize=9);axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | Adopted DRPM vs FAST-LIVO2')
        fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}.png',dpi=145);pdf.savefig(fig);plt.close(fig)
    (OUT/'evaluation.json').write_text(json.dumps(results,indent=2));(OUT/'verification.json').write_text(json.dumps(verification,indent=2))
    description='''同じ2飛行・link3の単眼カメラ・LiDAR・IMU。保存済み推定値を再集計し、推定器の再実行はしていません。MIMOSAはROVIOを融合し、公式DRPM更新＋距離方向ノイズ（標準偏差3cm）、最大10反復を使用。FAST-LIVO2は画像補正ありの既存fast_full条件です。離陸前から推定した結果のうち軌道追従区間のみを共通10Hzで評価し、開始時の位置・姿勢整合は一度だけ。軸省略・途中の再整合・スケール補正なし。元の推定時刻は変更せず、誤差CSVには補間評価時刻を明記しています。'''
    caveat='''両飛行とも位置誤差はFAST-LIVO2、姿勢誤差は採用MIMOSA＋DRPM＋ROVIOの方が小さい結果です。融合方式、地図処理、重み等が異なるため、この差をtight couplingだけの効果とは断定しません。採用MIMOSAのLiDAR制約は追従中の全スキャンで生成されましたが、停止条件を満たしたのは133/495、788/2002スキャンです。未収束のスキャンも10回目の結果を採用した設定として比較しています。'''
    (plots/'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>採用DRPMとFAST-LIVO2</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style><h1>採用DRPMとFAST-LIVO2</h1><p>'''+caveat+'</p><p>'+description+'''</p><p><a href="comparison.pdf">全図PDF</a> / <a href="../evaluation.json">数値</a> / <a href="../verification.json">照合記録</a> / <a href="../README.md">条件</a></p><table><tr><th>飛行</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'''+rows+'</table>'+''.join(f'<h2>{l}</h2><img src="{l}.png">' for l in LABELS))
    (OUT/'README.md').write_text('# 採用DRPMとFAST-LIVO2の比較\n\n'+caveat+'\n\n'+markdown+'\n'+description+'\n\n採用設定: comparison/adopted_drpm.json。ROVIOの融合重みは既存の別飛行校正行列を維持。採用方針は記録し、通常のシミュレーションlaunchは変更していません。再現: python3 /root/uas_native/plot_adopted_drpm_vs_fastlivo.py\n')
    print(markdown)
if __name__=='__main__':main()
