#!/usr/bin/env python3
"""Audit complete campaign and publish manifest; execute in ROS2 environment."""
import csv,json,hashlib
from pathlib import Path
import numpy as np,yaml
import run_fast_multicamera_all as c

def main():
 entries=json.loads((c.ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'];manifest={};p=c.OUT/'provenance.json';prov=json.loads(p.read_text())
 for name,digest in prov['files'].items():
  if name.endswith('.so'):assert c.sha(name)==digest,'Estimator library changed'
 for entry in entries:
  label=entry['label'];d=c.OUT/label;s=json.loads((d/'summary.json').read_text());assert s['returncode']==0 or (label=='run2b' and s.get('recovered_complete_inference'));c.finalize(d);m=json.loads((d/'metrics.json').read_text())
  cfg,_=c.config(c.ROOT/'logs'/entry['run']);actual=yaml.safe_load((d/'config.yaml').read_text())
  def same(a,b):
   if isinstance(a,dict):return set(a)==set(b) and all(same(a[k],b[k]) for k in a)
   if isinstance(a,list):return len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
   if isinstance(a,float):return abs(a-b)<=1e-12
   return a==b
  assert same(cfg,actual),label+' configuration differs'
  raw=np.genfromtxt(d/'odometry.csv',delimiter=',',names=True);assert np.all(np.diff(raw['stamp_ns'])==50000000)
  for name in ['odometry','truth']:
   with (d/f'{name}.csv').open() as f:
    rows=list(csv.DictReader(f));ts=[int(r['stamp_ns']) for r in rows];assert all(s['window']['start_ns']<=t<=s['window']['end_ns'] for t in ts);assert all(b>a for a,b in zip(ts,ts[1:]))
  if not s['reused']:
   assert s['binary_sha256']==c.sha(c.WS/'devel/lib/fast_livo/fastlivo_stream')
   for chunk in s['chunks']:
    receipt=json.loads((c.ROOT/'logs'/entry['run']/'cloud_chunks'/(Path(chunk['name']).stem+'.json')).read_text());assert receipt['md5']==chunk['md5'] and receipt['compressed_bytes']==chunk['bytes']
  else:assert s['binary_sha256']==c.sha(c.WS/'devel/lib/fast_livo/fastlivo_offline')
  e=np.genfromtxt(d/'errors.csv',delimiter=',',names=True);norm=np.sqrt(sum(e[k]**2 for k in ['ex_m','ey_m','ez_m']));bad=np.flatnonzero(norm>10)
  m['first_error_over_10m_tracking_s']=float((e['evaluation_stamp_ns'][bad[0]]-s['window']['start_ns'])*1e-9) if len(bad) else None;c.write(d/'metrics.json',m)
  manifest[label]=dict(complete=True,reused=s['reused'],directory=str(d),source=entry,window=s['window'],metrics=m,recovery_note=s.get('recovery_note'));print(label,m['position_rmse_m'],m['rotation_rmse_deg'],m['missing_tracking_stamps_ns'],flush=True)
 assert len(manifest)==14;c.write(c.OUT/'manifest.json',manifest)
 prov['final_scripts_sha256']={str(c.ROOT/n):c.sha(c.ROOT/n) for n in ['run_fast_multicamera_all.py','fast_stream_transport.py','run_fast_stream_child.py','finalize_fast_multicamera_all.py','plot_fast_multicamera_all.py','control_fast_multicamera_all.py']};prov['verification']='All 14 runs: settings/calibration matched within 1e-12, shared estimator libraries unchanged, checksum-verified source chunks, finite unit poses, no interior timestamp gaps, all cameras contributed and covariance chain exact.';prov['parallelism']='Two independent flight workers, isolated ROS masters. Up to8 prefetched chunks per worker; extra3 leading chunks of run1_middle prefetched across flight boundary.';c.write(p,prov)
 rows=['| Run | 位置RMSE [m] | 姿勢RMSE [deg] | 追従出力数 | 取得 |','|---|---:|---:|---:|---|']
 for label,v in manifest.items():
  m=v['metrics'];rows.append(f"| {label} | {m['position_rmse_m']:.6f} | {m['rotation_rmse_deg']:.6f} | {m['native_samples']} | {'既存trial1' if v['reused'] else '新規1回'} |")
 missing={l:v['metrics']['missing_tracking_stamps_ns'] for l,v in manifest.items() if v['metrics']['missing_tracking_stamps_ns']}
 report='\n'.join(['# FAST-LIVO2 3カメラ・全14飛行（2026-10-05）','',
 '全14飛行を取得。12飛行を新規計算し、run2a_end / run1_tail_a2_prefixは前回の同一構成のtrial1を事前指定して再利用。最良試行を選んだものではない。発散値も保存・掲載する。','',*rows,'',
 '## 条件と読み方','',
 '- link1 → link3 → link4の順に画像を共通状態・共分散へ反映する独自拡張。LiDAR/IMUの再適用はしない。固定露出・固定取付変換。推定器本体と設定は前回の3カメラ実験を維持。カメラ校正の再計算に伴う丸め差は最大1e-12以下で照合。',
 '- センサ初期化は元時刻2.003秒から。離陸・向き合わせも初期化に使用するが、評価は既存の軌道追従区間に限定。推定器は分割MCAPの境界で再起動しない。真値は推定へ入力しない。',
 '- odometry.csv / truth.csvは追従区間のみ、_full.csvは初期化を含む監査用。整数nsの元header stampを維持し、時刻の伸縮・再付与なし。点群・画像データは結果に含めない。',
 '- 推定CSVは推定器の座標系、真値CSVはシミュレーションの世界座標系。整合用の回転・平行移動はmetrics.jsonに記録。',
 '- 誤差は共通10Hzで補間評価。開始時に位置・姿勢を一度だけ整合。途中の再整合、スケール補正、軸の省略なし。姿勢RMSEは三次元の相対回転角。六軸図の角度は真値に対する相対回転のroll/pitch/yaw。',
 '- 元のFAST処理を維持しており、状態出力20Hzに対し、この瞬時LiDAR構成で実際に画像補正を試みる頻度は約10Hz。各カメラの処理回数・特徴点があった回数をmetrics.jsonへ記録。',
 '- 今回は各runの1試行。以前の2飛行では同じ入力でも試行間の数値差があったため、この結果を平均性能や成功確率とは扱わない。過去の単眼・露出推定ONとの比較はカメラ数だけを変えた比較ではない。','',
 '## 検証','',
 '- 全MCAPの保存時MD5を照合。入力転送処理は既存run2a_end全データでROS1メッセージのバイト一致を確認し、出力時刻・更新回数も一致。',
 '- 全runで有限位置・単位Quaternion・追従中の20Hz連続性・全カメラの寄与・カメラ間の共分散引継ぎを検証。推定ライブラリのSHAは変更なし。',
 '- 記録末尾の出力不足: '+str(missing)+('。元記録が最後の画像更新を解放する次のLiDAR以前に終了するため。架空のセンサや外挿値は追加せず、誤差評価の終了時刻をmetrics.jsonに明記。' if missing else '（なし）。'),
 '- run2bは全入力処理後に親処理が中断。最終入力・更新・カメラ集計と追従全時刻の連続性から完了を照合して復旧。OS終了コードと入力の逐次SHA256は保存前で失われたため未取得と明記。run1_middleは途中入力で中断したため、その未完了試行を別保存し、最初から同じ設定で再実行。',
 '- 元クラウドbagと以前の比較結果は変更なし。取得用MCAPキャッシュは使用後に削除。',
 '- wall_sにはDriveからの取得待ち、2飛行の並行処理、メモリ退避、ユーザー指示による一時停止待ちが含まれるため、推定器単独の処理速度比較には使わない。','',
 '## 成果物','',
 '出力: `/root/uas_native/logs/zab-offline/fast_multicamera_all/`。`plots/index.html`と`plots/all_flights.pdf`に六軸図。`manifest.json`、`evaluation.json`、`provenance.json`に全数値・出典・照合情報。各run配下に真値/推定/誤差CSV、設定、実行ログ。','',
 '再現: ROS master11376を起動し、ROS2 Humble環境で`python3 /root/uas_native/run_fast_multicamera_all.py`。`--labels`でrunを指定し、`--master-port`で独立masterを指定できる。既存の完了結果は上書きしない。`finalize_fast_multicamera_all.py`で全14飛行を照合・集計する。',''])
 (c.OUT/'README.md').write_text(report)
 (c.ROOT.parents[1]/'for_codex/resources/FASTLIVO2_ALL_RUNS_20261005.md').write_text(report)
 from plot_fast_multicamera_all import main as plot
 plot()
if __name__=='__main__':main()
