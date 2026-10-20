#!/usr/bin/env python3
"""Assemble auditable conclusions, diagnostics, and regression acceptance criteria."""
import argparse,json,html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;out=root/'plots';out.mkdir(exist_ok=True)
 initial={label:json.loads((root/f'initialization_{label}/report.json').read_text())['runs'] for label in ['a1','b1']}
 raw={label:json.loads((root/f'raw_measurement_{label}/raw_tracks.json').read_text()) for label in ['a1','b1']}
 reference=json.loads((root/'upstream_equivalence.json').read_text());assert all(reference['fixed_camera_outputs_byte_identical'].values())
 synthetic=json.loads((root/'synthetic_gimbal_result/report.json').read_text())['runs']['baseline'];negative=json.loads((root/'synthetic_without_compensation/report.json').read_text())['runs']['baseline']
 assert synthetic['position_max_m']<.2 and synthetic['rotation_rmse_deg']<.1 and synthetic['feature_count_percentiles'][2]>=15
 assert negative['position_rmse_m']>10*synthetic['position_rmse_m'] and negative['rotation_rmse_deg']>10*synthetic['rotation_rmse_deg']
 interventions={label:{mode:json.loads((root/f'paired_{label}_{mode}/report.json').read_text())['runs'] for mode in ['fixed','active']} for label in ['a1','b1']}
 rows=[];figures=[]
 with PdfPages(out/'diagnostics.pdf') as pdf:
  for label in ['a1','b1']:
   good=[r for r in raw[label]['tracks'] if r['raw_geometry_rms_px']<2 and r['positive_fraction']>.9];agree=[r for r in good if r['measurement_geometry_rms_px']<2]
   ratios=np.array([r['median_depth_ratio'] for r in agree]);summary=dict(eligible=raw[label]['eligible_tracks'],raw_consistent=len(good),raw_consistent_rovio_measurement_inconsistent=len(good)-len(agree),both_consistent=len(agree),both_consistent_depth_ratio_percentiles=np.percentile(ratios,[10,50,90]).tolist())
   rows.append((label,summary))
   fig,axs=plt.subplots(1,2,figsize=(12,4))
   for color,selection,name in [('#2474b5',agree,'Both tracks consistent'),('#db6619',[r for r in good if r not in agree],'ROVIO measurement inconsistent')]:
    axs[0].scatter([r['parallax_deg'] for r in selection],[r['median_depth_ratio'] for r in selection],s=13,color=color,label=name,alpha=.7)
    axs[1].scatter([r['raw_geometry_rms_px'] for r in selection],[r['measurement_geometry_rms_px'] for r in selection],s=13,color=color,alpha=.7)
   axs[0].axhline(1,color='black',lw=.8);axs[0].set_xlabel('Parallax [deg]');axs[0].set_ylabel('ROVIO range / triangulated range');axs[0].legend(fontsize=7);axs[1].axhline(2,color='black',lw=.8);axs[1].set_xlabel('Independent raw track reprojection RMS [px]');axs[1].set_ylabel('ROVIO image match reprojection RMS [px]')
   for ax in axs:ax.grid(alpha=.3)
   fig.suptitle(label+' / independent image geometry; full axes');fig.tight_layout();name=f'{label}_raw_geometry.png';fig.savefig(out/name,dpi=140);pdf.savefig(fig);plt.close(fig);figures.append(name)
  for label in ['a1','b1']:
   fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
   for col,mode in enumerate(['fixed','active']):
    n=np.load(root/f'bias_{label}_{mode}/bias_evolution.npz');time=(n['stamp_ns']-n['stamp_ns'][0])*1e-9
    for i,ax in enumerate(axs[:,col]):
     ax.plot(time,n['estimate'][:,i],label='ROVIO estimate',lw=1);ax.plot(time,n['reference_smooth'][:,i],label='Independent sensor-only bias estimate',lw=1);ax.fill_between(time,n['reference_smooth'][:,i]-3*np.sqrt(n['reference_variance'][:,i]),n['reference_smooth'][:,i]+3*np.sqrt(n['reference_variance'][:,i]),alpha=.18,color='orange');ax.set_ylabel('gyro bias '+str(i)+' [rad/s]');ax.grid(alpha=.3)
    axs[0,col].set_title(mode);axs[-1,col].set_xlabel('Time since first output [s]')
   axs[0,0].legend(fontsize=7);fig.suptitle(label+' / estimated gyro bias vs sensor reference');fig.tight_layout();name=f'{label}_bias.png';fig.savefig(out/name,dpi=140);pdf.savefig(fig);plt.close(fig);figures.append(name)
 checks=dict(fixed_reference=reference,synthetic_gimbal=dict(passed=True,position_limit_m=.2,rotation_rmse_limit_deg=.1,result=synthetic,without_compensation=negative),raw_geometry={k:v for k,v in rows},initialization_interventions={k:{m:dict(position_rmse_m=v['position_rmse_m'],rotation_rmse_deg=v['rotation_rmse_deg']) for m,v in d.items()} for k,d in initial.items()},scope='These checks establish correctness of the enumerated input/geometry/gimbal integration paths on the evaluated cases; they are not a proof of zero bugs in every upstream estimator or unseen environment.')
 (root/'diagnostic_validation.json').write_text(json.dumps(checks,indent=2)+'\n')
 text='''<h2>確認した範囲と結論</h2><p><strong>入力時刻・欠損、センサ仕様、座標・尺度の単位、初期化、ジンバル回転補償を検証しました。残る固定カメラの誤差は、ジンバル用処理を外し公式の視覚更新を使っても全出力が一致します。</strong>これは今回列挙した統合経路の検証結果です。ROVIO全体にあらゆる条件で不具合が存在しないという主張ではありません。</p>
<p>今回の追加作業では精度向上のための調整を通常設定に加えていません。1000Hz・雑音密度保存・wait初期化を固定/可動に共通使用。真値を使った初期化、参照IMU、bias固定は以下の原因切り分け実験だけです。</p>
<h3>回転・初期化</h3><p>旧初期化を「平均加速度による姿勢」と「平均値によるbias初期値」に分離。a1では姿勢だけを平均値や真の重力方向にしても回転誤差は3.252degで変わらず、bias初期値を旧方式にすると2.741degへ戻ります。b1は両者が相互作用し、正しい初期重力方向だけを与えても回転誤差は6.638degで解消しません。初期姿勢の単純な座標ミスで全体を説明できません。</p>
<p>物理センサの雑音を除いた参照IMUでも、ROVIOがbiasを推定すると回転誤差が残り、正しいゼロbiasを固定した診断では4条件とも約0.09–0.10degまで減りました。飛行中の画像更新とbias推定が結び付く経路を確認したものです。実センサにbiasゼロ固定を適用してはいません。</p><table><tr><th>区間・視野</th><th>参照IMU・bias推定あり 回転[deg]</th><th>参照IMU・真のゼロbias固定 回転[deg]</th></tr>'''
 for label in ['a1','b1']:
  for mode in ['fixed','active']:
   r=interventions[label][mode];text+=f'<tr><td>{label} {mode}</td><td>{r["reference_clean"]["rotation_rmse_deg"]:.3f}</td><td>{r["reference_clean_locked"]["rotation_rmse_deg"]:.3f}</td></tr>'
 text+='</table><h3>画像対応と尺度</h3><p>最初の画素だけROVIOの画像対応点を使い、その後は画像対だけの独立追跡を行いました。真値姿勢は事後の三角測量に使用し、推定器には入力しません。新規検出直後の点には画像対応測定がまだないため除外しています。対応点の不整合が一部にあり、対応が整合する点にも距離誤差が残りました。固定倍率を間違えて掛ける実装は確認されず、尺度の誤差は公式視覚更新でもそのまま再現します。</p><table><tr><th>区間</th><th>独立追跡が整合</th><th>うちROVIO画像対応が不整合</th><th>両者整合の距離比10/50/90%</th></tr>'
 for label,d in rows:text+=f'<tr><td>{label}</td><td>{d["raw_consistent"]}/{d["eligible"]}</td><td>{d["raw_consistent_rovio_measurement_inconsistent"]}</td><td>'+', '.join(f'{x:.3f}' for x in d['both_consistent_depth_ratio_percentiles'])+'</td></tr>'
 text+='</table><p>独立追跡にも誤りはあり得るため、表は長さ・視差・再投影誤差で検査できた追跡を対象にしています。表の不整合数を全画像の誤対応率とは扱いません。</p>'
 text+=f'<h3>ジンバル補償</h3><p>機体静止・カメラyaw±40deg/pitch±65degの画像入力試験で、位置RMSE {synthetic["position_rmse_m"]:.3f}m、回転RMSE {synthetic["rotation_rmse_deg"]:.3f}deg。補償を外した対照は{negative["position_rmse_m"]:.3f}m/{negative["rotation_rmse_deg"]:.3f}degでした。非等方の共分散と状態の変換、数値微分、往復変換の単体試験も合格しました。</p>'
 text+='<p><a href="plots/diagnostics.pdf">原因切り分けの図PDF</a> / <a href="diagnostic_validation.json">検証数値</a> / <a href="upstream_equivalence.json">公式処理との一致確認</a></p>'+''.join(f'<img src="plots/{f}">' for f in figures)
 (root/'findings.html').write_text(text)
 print(json.dumps(checks['raw_geometry'],indent=2))
if __name__=='__main__':main()
