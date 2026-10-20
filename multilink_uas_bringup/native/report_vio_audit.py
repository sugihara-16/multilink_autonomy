#!/usr/bin/env python3
"""Standalone figures and evidence report for the VIO integration audit."""
import html,json,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from summarize_vio_audit import numeric
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/vio_audit';REPORT=OUT/'report'
LABELS=['run1_tail_a2_prefix','run1_tail_b2','run2a_end']
SHOW={'baseline':('Original','#D55E00'),'gyro_model_matched':('Gyro model matched','#0072B2'),'gyro_matched_depth10':('Gyro matched + initial distance 10m','#009E73'),'features75':('75 features','#CC79A7')}
DESCRIPTIONS={
 'baseline':'現行設定（25特徴）','features75':'特徴上限25→75（他は現行）','stereo':'ステレオ距離初期化ON','depth10':'距離初期値2→10m（逆距離0.5→0.1）','direct':'画像の明るさの残差で更新','initial_mean':'最初の姿勢初期化用加速度を2〜5秒平均に変更',
 'bias_cov':'加速度偏りの初期分散0.0004→0.04','bias_cov_matched':'同分散0.0004→0.01','bias_oracle':'加速度偏りの初期値に静止時の真値姿勢から得た値を設定※','bias_removed':'同偏りを入力から除去※',
 'gyro_rw_static':'ジャイロ偏りの時間変動分散3.8e-7→1e-12のみ','gyro_model_static':'ジャイロ3設定を変更（初期偏り分散1e-8、変動1e-12、観測4e-11）',
 'gyro_model_matched':'ジャイロ3設定を模擬センサに整合（初期偏り分散1e-10、変動1e-16、観測4e-11）',
 'gyro_matched_depth10':'ジャイロ整合＋距離初期値10m',
 'gyro_measurement_only':'ジャイロ観測分散7.6e-7→4e-11のみ','gyro_initial_only':'ジャイロ初期偏り分散3e-4→1e-10のみ','gyro_bias_matched':'ジャイロ初期偏り分散1e-10＋変動分散1e-16（観測分散は現行）'}
def main():
 REPORT.mkdir(exist_ok=True);s=json.loads((OUT/'summary.json').read_text());inputs=json.loads((OUT/'input_audit.json').read_text());sections=[];depth_summary={}
 with PdfPages(REPORT/'all_diagnostics.pdf') as pdf:
  for label in LABELS:
   variants=s[label];base=variants['baseline'];anchor=base['anchor_ns'];fig,axes=plt.subplots(6,1,figsize=(13,15),sharex=True)
   for variant,(title,color) in SHOW.items():
    if variant not in variants:continue
    data=np.load(OUT/'experiments'/label/variant/'errors.npz');t=(data['stamp_ns']-anchor)*1e-9
    for i,ax in enumerate(axes):ax.plot(t,data['errors'][:,i],color=color,label=title,lw=1)
   for ax,title in zip(axes,['X error [m]','Y error [m]','Z error [m]','Roll error [deg]','Pitch error [deg]','Yaw error [deg]']):ax.set_ylabel(title);ax.grid(alpha=.25);ax.axhline(0,color='k',alpha=.2,lw=.5)
   axes[0].legend(fontsize=8,ncol=2);axes[-1].set_xlabel('Time since original tracking alignment [s]');fig.suptitle(label+' | VIO errors, unchanged inputs',fontsize=16);fig.tight_layout(rect=[0,0,1,.975]);fig.savefig(REPORT/(label+'_errors.png'),dpi=140);pdf.savefig(fig);plt.close(fig)
   fig,axes=plt.subplots(4,1,figsize=(13,12),sharex=True)
   for variant,(title,color) in SHOW.items():
    if variant not in variants:continue
    dest=OUT/'experiments'/label/variant;data=np.load(dest/'errors.npz');t=(data['stamp_ns']-anchor)*1e-9;e=data['errors'];axes[0].plot(t,np.linalg.norm(e[:,:3],axis=1),label=title,color=color,lw=1);axes[1].plot(t,e[:,5],color=color,lw=1)
    state=numeric(dest/'result_state.csv');state=state[(state['stamp_ns']>=anchor)&(state['stamp_ns']<=data['stamp_ns'][-1])];ts=(state['stamp_ns']-anchor)*1e-9;axes[2].plot(ts,state['gyb_z'],color=color,lw=1)
    if variant=='baseline':
     for c,color2 in enumerate(['#0072B2','#E69F00','#009E73']):axes[3].plot(ts,state[f'n{c}'],color=color2,lw=.8,label=f'Camera {c} (link {[1,3,4][c]})')
   true_bias=inputs['sensors'][label]['best_gyro_shift']['mean_bias_rad_s'][2];axes[2].axhline(true_bias,color='k',ls='--',lw=1,label='IMU/truth diagnostic bias');axes[2].legend(fontsize=8)
   for ax,title in zip(axes,['Position error norm [m]','Yaw error [deg]','Estimated gyro Z bias [rad/s]','Active features (original)']):ax.set_ylabel(title);ax.grid(alpha=.25)
   axes[0].legend(fontsize=8,ncol=2);axes[3].legend(fontsize=8,ncol=3);axes[-1].set_xlabel('Time since original tracking alignment [s]');fig.suptitle(label+' | Error, bias and camera usage',fontsize=16);fig.tight_layout(rect=[0,0,1,.975]);fig.savefig(REPORT/(label+'_diagnostics.png'),dpi=140);pdf.savefig(fig);plt.close(fig)
   ds={}
   for dest in (OUT/'experiments'/label).iterdir():
    if not (dest/'mesh_depth.csv').exists():continue
    d=numeric(dest/'mesh_depth.csv');f=numeric(dest/'result_features.csv');ids,first=np.unique(f['id'],return_index=True);initial=dict(zip(ids,f['stamp_ns'][first]));age=np.array([(t-initial[idx])*1e-9 for t,idx in zip(d['stamp_ns'],d['id'])]);valid=(age>=2)&(d['status']==4)&np.isfinite(d['mesh_depth_m'])&(d['mesh_depth_m']>.8)&(d['mesh_depth_m']<60);ratio=d['estimated_depth_m'][valid]/d['mesh_depth_m'][valid];ds[dest.name]=dict(count=int(valid.sum()),ratio_p10_p50_p90=np.quantile(ratio,[.1,.5,.9]).tolist(),minimum_feature_age_s=2)
   depth_summary[label]=ds
   sections.append(f'<section><h2>{label}</h2><p>再計算した現行条件と旧VIOの最大位置差: {base["raw_position_difference_from_original_max_m"]:.0f} m。評価点数: {base["count"]:,}。カメラ別有効特徴数の平均: '+', '.join(f'{v:.1f}' for v in base['features_mean_by_camera'])+f'。他カメラでもTRACKEDとなった特徴数の平均: {base["cross_tracked_per_frame_mean"]:.2f}。</p><img src="{label}_errors.png"><img src="{label}_diagnostics.png"></section>')
 rows=[]
 for variant,description in DESCRIPTIONS.items():
  row=f'<tr><th>{description}<br><code>{variant}</code></th>'
  for label in LABELS:
   r=s[label].get(variant);row+=f'<td>{r["max_position_m"]:.3f}</td><td>{r["max_abs_six_axes"][5]:.3f}</td>' if r else '<td colspan="2">未実施</td>'
  rows.append(row+'</tr>')
 count=sum(len(v) for v in s.values());baseline_equal=all(s[k]['baseline']['raw_position_difference_from_original_max_m']==0 for k in LABELS)
 headline_table='<table><tr><th>飛行</th><th>旧設定の最大位置誤差</th><th>ジャイロ整合＋初期距離10m</th></tr>'+''.join(f'<tr><th>{k}</th><td>{s[k]["baseline"]["max_position_m"]:.3f} m</td><td>{s[k]["gyro_matched_depth10"]["max_position_m"]:.3f} m</td></tr>' for k in LABELS)+'</table>'
 content=f'''<!doctype html><html lang="ja"><meta charset="utf-8"><title>VIO設定・入力の追加監査</title><style>body{{max-width:1300px;margin:32px auto;padding:0 20px;font:16px/1.7 sans-serif;color:#20262d}}h1,h2{{line-height:1.4}}table{{border-collapse:collapse;width:100%;font-size:14px}}th,td{{border:1px solid #ccc;padding:7px;text-align:right}}th:first-child{{text-align:left}}thead{{background:#edf1f4}}img{{width:100%;display:block}}code{{font-size:12px;color:#555}}section{{margin-top:45px}}.lead{{padding:20px;background:#edf5fa;border-left:5px solid #0072B2}}a{{color:#0067a8}}</style>
 <h1>VIO設定・入力の追加監査</h1><p>2026-10-03 · 3飛行 / {count}条件 · 元の環境・画像・IMUを固定した追加計算</p>
 <div class="lead"><b>現行のVIO誤差を、環境だけの影響として扱うことはできません。</b><p>センサ入力やカメラ接続の破損を示す証拠はありません。現行のジャイロ誤差モデルがシミュレーションのセンサ特性と合っておらず、推定した偏りと向きがずれることを確認しました。模擬センサに合わせたモデルへの変更で3飛行とも向きの誤差が減少します。移動距離の誤差は残り、特徴点数・距離初期値にも依存します。</p></div>
 {headline_table}<p>この変更は比較実験用の設定として保存しています。本番のlaunch設定・既存結果は変更していません。3飛行での改善であり、残り11飛行での検証は未実施です。</p>
 <p><a href="all_diagnostics.pdf">全図PDF</a> · <a href="summary.json">数値・評価条件</a> · <a href="input_audit.json">校正・時刻・IMU検査</a> · <a href="depth_summary.json">特徴距離とメッシュの比較</a> · <a href="reproduction.txt">再現手順</a> · <a href="candidate_sim_rovio.patch">比較用の設定差分</a> · <a href="experiment_results.zip">45条件の結果・設定・再現コード</a> · <a href="validation.json">検証結果</a></p>
 <h2>確認できたこと</h2><ul>
 <li>全14飛行のカメラ校正は同一。モデル定義から計算したIMUとの相対位置・向き、画角から計算した焦点距離と一致。3飛行の全カメラ取得時刻は一致し、1000 HzのIMU・20 Hzの画像に途中欠落なし。追従中の関節変動はゼロ。</li>
 <li>通信を通さず、元の保存順で全入力を推定器に直接渡す再計算を実施。3飛行の現行条件は、旧結果と共通時刻の位置が完全一致: {baseline_equal}。姿勢も数値誤差水準。通信負荷や高速再生が今回のドリフトの説明にはなりません。</li>
 <li>IMUの回転積分と真値の回転変化の時刻差は0〜1 ms。静止時の偏りと真値姿勢を使った診断では、加速度積分から求めた0.1秒間の速度変化も真値と整合。これは入力の検査であり、通常条件の推定器に真値を渡したものではありません。</li>
 <li>現行25特徴の追跡状態は維持されており、出力が途切れる・繰り返し初期化する故障ではありません。特徴数は3台合計25点で、カメラ別の均等配分は保証されず、途中で一台へ偏ります。画像上で見つかる多数の角点を全部使っているわけではありません。</li>
 <li>3カメラのうち2台の光軸は約179°離れ、他の組は約89〜92°。画角は120°。同時に同じ場所を見られる領域は限定されます。現行コードのステレオ初期化は各カメラの次の番号の1台だけを試し、全組合せの探索ではありません。ONにするだけでの一貫した改善はありませんでした。</li></ul>
 <h2>向きのずれを生む非環境要因</h2>
 <p>ジャイロは機体の回転速度を測るセンサです。元の記録から得た定常的な偏りは約±0.000007〜0.000008 rad/sですが、現行ROVIOは直線飛行中にこれより大きな偏りを推定します。run1_tail_a2_prefixの終端ではZ軸約0.003 rad/sでした。画像の拘束と整合するために、実際にはない偏りを推定している状態です。画像対応そのものの誤りを全件特定した、という意味ではありません。</p>
 <p>現行設定では、ジャイロの初期偏り分散3e-4、偏りの時間変動分散3.8e-7、角速度観測に対応する分散7.6e-7。模擬センサは1000 Hz、各サンプルのノイズ標準偏差2e-4 rad/s、偏りの時間変動は既定0です。比較条件ではそれぞれ1e-10、1e-16、4e-11へ変更しました。最後の値は(2e-4)²×0.001秒で、この実装の時間刻みの扱いに換算した値です。これは真値を注入せず、既知のセンサ生成設定だけを使用した比較です。実機にそのまま適用する設定ではありません。</p>
 <h2>全比較結果</h2><p>各欄は「追従中の最大3D位置誤差m / 最大絶対Yaw誤差deg」。小さい値ほど良好。元と同じ時刻に位置と姿勢を一度だけ整合し、スケール補正・途中再整合はしていません。</p>
 <table><thead><tr><th rowspan="2">変更内容</th>'''+''.join(f'<th colspan="2">{label}</th>' for label in LABELS)+'''</tr><tr>'''+''.join('<th>位置 [m]</th><th>Yaw [deg]</th>' for _ in LABELS)+'''</tr></thead><tbody>'''+''.join(rows)+'''</tbody></table>
 <p>※ bias_oracle / bias_removed は、開始後2〜5秒の静止データと真値姿勢を用いた診断専用の比較です。推定器の一般性能を示す値として採用しません。gyro_model_*は真値を使用しません。ジャイロ3設定の組は複数パラメーターを同時変更しているため、個別の寄与は追加の分離実験と合わせて判断します。</p>
 <h2>距離・初期化・飛行条件</h2><p>距離初期値を2mから10mへ変えると改善する飛行がありますが、全飛行を一つの初期距離で改善する証拠にはなりません。特徴数を75へ増やす比較も実施しました。推定した特徴距離を、真値のカメラ姿勢からメッシュへ伸ばした線の実距離と比較しています。新しく追加した点の未収束を混ぜないため、別集計では追跡開始から2秒以上経った点だけに限定しています。ロボット本体の遮蔽はこのメッシュ照合に含めておらず、誤対応の全件判定ではありません。</p>
 <p>加速度センサの偏りは飛行ごとに符号が異なります。現行ROVIOは0から開始し、初期標準偏差0.02 m/s²に対して実測の大きさは約0.1 m/s²です。ただし、初期分散を広げる、既知の偏りを与える、初期加速度を平均する、という変更では一貫した改善は得られません。これだけを主原因とは断定できません。世界の重力9.8とROVIOの9.81との差は上記のバイアス診断に含まれます。</p>
 <h2>判断と次の扱い</h2><p>今回の旧結果は「現行のセンサ設定・推定器設定・飛行条件をまとめた結果」です。環境だけを変えた比較ではありません。まずシミュレーション用のジャイロ誤差モデルを分離し、残る距離誤差と画像対応を評価するのが妥当です。姿勢が改善しても位置誤差は残るため、修正完了・環境非依存とは扱いません。一定速度の長い直線では奥行きと移動距離を区別する情報が弱くなることが原因候補ですが、同じ環境で加減速を変える比較はこの追実験では行っていません。</p>
 <p>本番設定、旧LIO/VIO結果、原本rosbagは変更していません。LIOは今回再計算していません。再計算の対象は代表3飛行で、残り11飛行の改善を保証する結果ではありません。カメラ画像の実描画時刻に一定の遅れがあるかの独立測定、全画像対応残差の正誤判定は未実施です。</p>'''+''.join(sections)+'</html>'
 (REPORT/'index.html').write_text(content)
 (REPORT/'depth_summary.json').write_text(json.dumps(depth_summary,indent=2))
 for name in ['summary.json','input_audit.json','epipolar.json']:shutil.copy2(OUT/name,REPORT/name)
 (REPORT/'reproduction.txt').write_text('''元データ: logs/zab-full-campaign/dataset_manifest.json の採用bagのみ。
1. ROS2 env.shをsourceし prepare_vio_audit.py <run名...>（有界キャッシュ・MD5確認）。
2. build_vio_audit.py 25 75。既存ROVIOビルドのヘッダ/ライブラリを使い、logs内に別実行ファイルを作る。
3. クリーンROS1環境（env_ros1.sh + slam_ros1_ws/devel/setup.bash）、ROS_MASTER_URIを11375へ設定し専用roscore。
4. run_vio_audit.py <run名> <比較条件...>。全条件名はHTML/スクリプト参照。入力は開始から再生、保存出力は全時刻。
5. check_vio_inputs.py、summarize_vio_audit.py、check_vio_depth.py、report_vio_audit.py。
   メッシュ照合は/tmp/zab_collision.ply（ZaBのworld座標衝突メッシュ）を参照する。
評価は元error_plots/evaluation.jsonと同一tracking anchor。整数stamp_nsを保持。
CSV結果は各実験の result_vio.csv / result_state.csv / result_features.csv。
bias_oracle/bias_removedだけは初期静止時の真値姿勢を利用する診断。その他の推定器には真値を入力しない。
ROSキューを介さないC++診断器は同じ順序・同じ画像/IMUでコールバックを直接呼ぶ。
正常完了時に入力件数・3カメラ同期件数を記録。初期2秒のIMUは元と同じく除外。
距離画像や点群の代用入力、真値による途中補正、スケール補正は行わない。
''')
 print('REPORT',REPORT,'experiments',count,flush=True)
if __name__=='__main__':main()
