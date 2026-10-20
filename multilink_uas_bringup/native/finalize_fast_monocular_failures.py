#!/usr/bin/env python3
"""Audit five monocular checks and document the limits of this ablation."""
import json,hashlib
import numpy as np
from pathlib import Path
import check_fast_monocular_failures as c

def main():
 p=c.OUT/'provenance.json';prov=json.loads(p.read_text());results={}
 for name,digest in prov['libraries'].items():assert c.sha(name)==digest,'Estimator changed'
 binary=Path('/root/multilink_autonomy_ws/comparison_ws/devel/lib/fast_livo/fastlivo_stream');assert c.sha(binary)==prov['binary_sha256']
 for label in c.LABELS:
  d=c.OUT/label;s=json.loads((d/'summary.json').read_text());assert s['returncode']==0 and s['binary_sha256']==prov['binary_sha256'];c.finalize(d);m=json.loads((d/'metrics.json').read_text())
  for chunk in s['chunks']:
   receipt=json.loads((c.ROOT/'logs'/s['source']['run']/'cloud_chunks'/(Path(chunk['name']).stem+'.json')).read_text());assert receipt['md5']==chunk['md5'] and receipt['compressed_bytes']==chunk['bytes']
  old=c.read(c.LOG/'fast_multicamera_all'/label/'odometry_full.csv');gt=c.read(c.LOG/'fast_multicamera_all'/label/'truth_full.csv');start=m['evaluation_start_ns'];end=min(s['window']['end_ns'],int(old[0][-1]),int(gt[0][-1]));grid=np.arange(start,end+1,100000000,dtype=np.int64)
  e,_=c.errors(old,gt,start,grid);over=np.linalg.norm(e[:,:3],axis=1)>100;hit=np.flatnonzero(np.convolve(over.astype(int),np.ones(51,int),'valid')==51)
  m['three_camera_full_window_failure_s']=float((grid[hit[0]]-start)*1e-9) if len(hit) else None
  c.write(d/'metrics.json',m);results[label]=m
 c.write(c.OUT/'manifest.json',dict(status='complete',conditions=prov,runs=results));c.plot()
 def timing(v):
  t=v['first_sustained_100m_s'];return f'{t:.1f}' if t is not None else '確認区間では未達'
 table=['| run | 単眼: 100m超開始 [s] | 3台: 100m超開始 [s] | 単眼 位置RMSE [m] | 3台 位置RMSE [m] | MIMOSA 位置RMSE [m] |','|---|---:|---:|---:|---:|---:|']
 for label,m in results.items():
  v=m['metrics'];table.append(f"| {label} | {timing(v['monocular'])} | {m['three_camera_full_window_failure_s']:.1f} | {v['monocular']['position_rmse_m']:.3f} | {v['three_camera']['position_rmse_m']:.3f} | {v['mimosa']['position_rmse_m']:.3f} |")
 n=sum(m['metrics']['monocular']['first_sustained_100m_s'] is not None for m in results.values())
 report='\n'.join(['# FAST-LIVO2 発散5飛行での単眼確認','',f'対象5飛行中{n}飛行で、link3単眼でも事前定義した発散（位置誤差100m超が5秒連続）を確認。','',*table,'',
 '時間は共通の追従評価開始から、100m超が5秒以上連続する最初の区間の開始まで。判定が確定するのは表の時刻から5秒後。RMSEは各行で3方式が同じ時刻範囲を使用。3台の100m超開始時刻だけは保存済み全区間で調べるため、単眼の終了後の時刻も含む。早期終了を含むRMSEを、前回の全区間RMSEとは比較しない。評価開始/終了nsは各metrics.jsonに記録。','',
 '## 条件','',
 '- 前回の3台構成からmulticamera設定を削除。従来の単眼分岐を使用。選択は以前のカメラ比較で良かったlink3に事前固定。結果を見て選び直していない。',
 '- カメラ校正、露出推定OFF、画像処理、LiDAR/IMU、初期化、推定器ライブラリ、処理スレッド設定は維持。画像はlink3だけを入力。これはカメラ数・融合経路を切り分ける比較で、公式の標準パラメータをそのまま使った実験ではない。',
 '- 地図/共分散をMCAP境界で初期化しない。元時刻を保持。GTは推定器に入力せず、外側の終了判定と評価にだけ使う。',
 '- 事前の終了判定は元の追従開始時刻に1回整合して判定。比較図/表はMIMOSAも使える最初の共通10Hz時刻で1回整合。両方の開始時刻を記録する。途中の再整合・スケール合わせ・縦軸省略は行わない。',
 '- 100m超が5秒連続すれば、その時点のチャンクの入力と残りの1秒バッファを処理して終了。未到達の場合は最後まで実行。判定しきい値は開始前に固定。短い突発誤差や正確に100mでは停止しないことを合成データで検証。',
 '- 各飛行は今回1試行。発散の存在を確認する実験であり、成功率や平均性能、早期終了後に回復しないことを証明する実験ではない。','',
 '## 検証','',
 '- 短い既存run2a_endを単眼ストリームで実行。IMU111338・点群1113・画像2226の入力、元のbag実行との全出力時刻一致、LIO2224/VIO2223更新一致、終了コード0。位置RMSE0.021m。',
 '- 5飛行で設定の差がmulticamera削除のみであること、終了コード0、入力数一致、単眼経路、出力時刻の連続性、有限位置・単位四元数、元チャンクMD5/サイズ、推定器ライブラリ不変を照合。',
 '- 元データ・前回結果は変更なし。未使用の先読みは判定後に中止。停止対象は専用ダウンロードだけで、推定器はバッファを処理して正常終了する。','',
 '## 解釈','',
 '単眼でも崩れる飛行については、複数カメラ化だけを発散の原因とする説明は成立しない。発散時刻や誤差増加速度は条件によって異なり、複数カメラの実装に問題がないと証明したものではない。共通の入力/校正/設定/推定処理と環境のどこが原因かは未特定。','',
 '出力: `/root/uas_native/logs/zab-offline/fast_monocular_failures/`。plots/index.html、plots/comparison.pdf、各runの真値・推定CSV・共通区間の六軸誤差CSV、summary/metrics/input_audit/stop_decision、provenance。',''])
 (c.OUT/'README.md').write_text(report);(c.ROOT.parents[1]/'for_codex/resources/FASTLIVO2_MONOCULAR_FAILURE_CHECK_20261005.md').write_text(report)
 prov['verification_complete']=True;prov['scripts_sha256']={str(c.ROOT/name):c.sha(c.ROOT/name) for name in ['run_fast_monocular_failures.py','check_fast_monocular_failures.py','finalize_fast_monocular_failures.py','cleanup_fast_monocular_prefetch.py','test_fast_monocular_stop.py']};c.write(p,prov);print(report)
if __name__=='__main__':main()
