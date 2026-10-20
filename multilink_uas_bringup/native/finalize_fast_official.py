#!/usr/bin/env python3
"""Finalize the three upstream trials, including data and shutdown audits."""
import csv,json,hashlib,re,sys
from pathlib import Path
import numpy as np,yaml
from evaluate_fast_official import finalize,plot
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_official_raw';LABELS=['run1_prefix','run1_tail_a1','run1_tail_b1']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 audit=json.loads((OUT/'source_audit.json').read_text());allm={};checks={};table=[]
 for label in LABELS:
  d=OUT/label;finalize(d);m=json.loads((d/'metrics.json').read_text());s=json.loads((d/'summary.json').read_text());p=json.loads((d/'publisher_audit.json').read_text())
  assert s['binary_sha256']==audit['binary_sha256']
  assert p['node_returncode']==0 and s['returncode']==0, 'Official process must exit normally after ABI correction'
  assert p['counts']==[s['counts'][i] for i in [0,1,3]]
  assert m['post_visual']['first_sustained_100m_s'] is not None
  q=np.genfromtxt(d/'odometry_full.csv',delimiter=',',names=True);qq=np.stack([q[k] for k in ['qx','qy','qz','qw']],axis=1);assert np.isfinite(qq).all() and np.allclose(np.linalg.norm(qq,axis=1),1,atol=2e-6,rtol=0)
  for src,dest in [('truth_full.csv','truth.csv'),('odometry_post_visual.csv','odometry.csv')]:
   with (d/src).open() as f,(d/dest).open('w') as out:
    reader=csv.DictReader(f);wr=csv.DictWriter(out,fieldnames=reader.fieldnames);wr.writeheader();wr.writerows(row for row in reader if m['evaluation_start_ns']<=int(row['stamp_ns'])<=m['evaluation_end_ns'])
  old=json.loads((ROOT/'logs/zab-offline/fast_monocular_failures'/label/'metrics.json').read_text())['metrics']['monocular']['first_sustained_100m_s']
  table.append(f"| {label} | {old:.1f} | {m['post_visual']['first_sustained_100m_s']:.1f} | 発散 |")
  checks[label]=dict(algorithm_binary_identical=True,source_chunks_md5_verified=len(s['chunks']),published_counts=p['counts'],image_callback_coverage_to_ns=m['image_callback_coverage_verified_to_ns'],pose_max_gap_s=m['max_gap_s'],post_visual_stamp_match=True,shutdown_returncode=p['node_returncode'],stopped_after_divergence=True,odometry_sha256=sha(d/'odometry.csv'),truth_sha256=sha(d/'truth.csv'))
  allm[label]=m
 plot()
 body='''# FAST-LIVO2 公式単眼の再検証（2026-10-06）

## 結果
公式単眼の本体・カメラライブラリを無改変でビルドして試した3飛行は、すべて発散した。画像補正後の状態でも確認した。3本とも追従区間を最後まで安定して完走する結果にはならなかった。発散時刻の変化は下表に示す。複数カメラへの再適用・全14飛行の再計算は行っていない。

表は追従開始から「位置誤差100m超が5秒連続する区間」の開始までの秒数。各条件1試行であり成功率を推定する実験ではない。

| 飛行 | 以前の単眼 [s] | 今回の公式単眼 [s] | 判定 |
|---|---:|---:|---|
'''+ '\n'.join(table)+'''

## 公式と同一の範囲
- FAST-LIVO2: `0d2c0346107b75b59934975adec9a6eeeb913c64`。CMake/mainを含む75 tracked filesをバイト照合。本体、同期処理、前処理、画像更新を変更していない。独自stream/offline adapterで推定器の内部関数を呼ばず、公式fastlivo_mappingノードへROS topicで送信した。
- カメラライブラリrpg_vikit: `6c886c8e5d83997806e00294826d528cea3581dd`。CMakeとソース全tracked filesを原本照合し、独立WSへビルド。既存PCLとのメモリ互換性を揃えるため、構成側でEIGEN_MAX_ALIGN_BYTES=16 / EIGEN_MALLOC_ALREADY_ALIGNED=1を指定（推定コード変更なし）。本体・カメラライブラリとも公式のmarch=native。インストール済みROSとの互換性のため、構成引数でC++17を指定。
- 公式launchが読む`config/avia.yaml`を基準とし、推定重み・反復回数・地図・露出推定等を維持。画像scaleも公式camera_pinhole.yamlの0.5（入力720×540→本体内360×270）。露出推定ON、点間引き1、点群縮小0.1m、距離誤差パラメータ0.02m。後者は公式値を維持し、実際の模擬ノイズ0.03mへ調整していない。
- 必要な適合はtopic名、32線の点群形式（Ouster形式として読み込み）、機体校正、画像時刻差0、pose保存設定。幾何は記録時URDFから再計算し、カメラ画角から内部校正を独立照合した。
- ROS/OpenCV/PCL/Sophus等はこの環境のインストール済み版。Sophusには既存のGCC向けstd::complex setter互換修正があるが、推定式を変更するものではない。ソフトウェア環境全体を論文実験機と同一にした意味ではない。

## 入力と評価
- カメラはlink3の1台。元RGB画素・解像度・encodingをそのまま転送。自前の白黒化、補正、縮小は行わない。
- ROS2→ROS1の形式変換のみ外部で実施。LiDARの有限XYZ/強度/ringを維持し、瞬時スキャンに合わせ全点の相対時刻を0にする。無効XYZを除く以外に外部で間引き・範囲制限・縮小をしない。IMUの加速度/角速度は元の値。センサheader timestampを維持。
- 記録開始後2.003秒から初期化を含め再生。評価は離陸/向き合わせを除いた既定の追従区間。GTは推定器へ一切渡さず、外部の評価・終了判定だけに使用。
- 公式evo出力はLiDAR更新直後（その時刻の画像補正前）。これと、公式mat_outログの画像補正後状態を別々に保存・評価した。後者の全時刻をevoの元時刻と照合し、補正前の位置も一致確認。ログの小数精度は公式出力のままで、後付けで精度を増やしたとは扱わない。
- 一度だけ開始位置・姿勢を整合し、10Hzの共通時刻で誤差を評価。途中の再整合・スケール補正・外挿なし。元の推定値・真値は整数ns時刻のCSVで保持。
- 元bagの取得チェックサム、送信件数、公式画像callbackに届いた全時刻、評価内のodometry連続性（最大0.1秒）を照合。画像更新前後とも発散。終了時点の未処理画像がある場合も評価区間より後であり、途中欠落とは区別。

## 結論の範囲と終了処理
「無改変の公式推定処理を、公開avia設定を基準として今回の機体・センサに接続すると、これらの飛行で発散した」と言える。環境の単調さだけが原因、FAST-LIVO2がどの設定でも必ず失敗する、という結論ではない。独自画像変換や複数カメラ拡張がなければ発散しない、という仮説は支持されなかった。

発散を確認してからチャンク境界で入力を終了した。初期試行では既存PCLとのメモリ互換性が原因の終了時異常があった。点群縮小だけの小さな再現プログラムでも同じ異常を再現し、互換設定で解消したため、本測定はその設定でやり直した。推定コード・推定パラメータの調整ではない。最終3件はいずれも終了コード0。終了コードは各run内に保持した。Eigenのメモリ整列設定とバイナリ互換性については[公式説明](https://libeigen.gitlab.io/eigen/docs-3.4/TopicPreprocessorDirectives.html)を参照。今回の修正の根拠は、この環境で行った独立再現試験（pcl_abi_test.json）である。

旧カメラライブラリのビルドを参照した暫定試行はpreliminary_mixed_dependency_buildに分離し、上の3飛行に混ぜていない。PCL互換性修正前の3件もpreliminary_pcl_abi_mismatchへ分離した。過去の実験データと元のクラウドbagは変更していない。

## 成果物・再現
- 結果: `/root/uas_native/logs/zab-offline/fast_official_raw/plots/index.html`、`comparison.pdf`（3ページ）、各飛行の真値/推定/六軸誤差CSV。
- `source_audit.json`、`linked_libraries.txt`、各config/summary/input_audit/publisher_audit/metrics、`validation.json`に条件と照合を保存。
- ビルド: `official_fast_ros1.sh bash build_fast_official.sh`（clean環境）。測定: ROS1 master11378を起動し、Humble環境で`run_fast_official_raw.py --labels run1_prefix run1_tail_a1 run1_tail_b1`。集計: `finalize_fast_official.py`。
'''
 resources=ROOT.parent.parent/'for_codex/resources'
 # ROOT is native under multilink_uas_bringup; project is two parents above it.
 resources=ROOT.parents[1]/'for_codex/resources'
 (resources/'FASTLIVO2_OFFICIAL_RAW_VALIDATION_20261006.md').write_text(body)
 (OUT/'README.md').write_text(body)
 (OUT/'validation.json').write_text(json.dumps(dict(status='complete',measurements_valid=True,all_three_diverged=True,process_shutdown_clean=all(v['shutdown_returncode']==0 for v in checks.values()),checks=checks,source_audit_sha256=sha(OUT/'source_audit.json')),indent=2))
 (OUT/'manifest.json').write_text(json.dumps(dict(status='complete',runs=LABELS,multicamera_all_runs_repeated=False,reason='All three official monocular trials still diverged; onset changes reported separately'),indent=2))
 print('\n'.join(table))
if __name__=='__main__':main()
