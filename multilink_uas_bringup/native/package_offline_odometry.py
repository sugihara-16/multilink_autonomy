#!/usr/bin/env python3
"""Collect only verified odometry products, preserving raw estimator frames and timestamps."""
import json,shutil,os
from pathlib import Path
ROOT=Path(__file__).resolve().parent
C=ROOT/'logs/zab-offline';dest=C/'results';manifest=json.loads((C/'odometry_manifest.json').read_text());assert manifest['status']=='complete'
dest.mkdir(exist_ok=True)
for entry in manifest['runs']:
 label=entry['label'];source=C/label;out=dest/label;out.mkdir(exist_ok=True)
 for name in ('truth.csv','lio.csv','vio.csv','window.json','result.json'):
  shutil.copy2(source/name,out/name)
 shutil.copytree(source/'odometry',out/'odometry',dirs_exist_ok=True)
 entry['bag']=label+'/odometry'
manifest['source_remote']=(ROOT/'logs/zab-full-campaign/remote.txt').read_text().strip()
manifest['limitations']=['run1_middle: VIO estimates at 1323.15 s and 1323.20 s unavailable at the input recording boundary; not interpolated or fabricated.','run1_tail_a1 crashed in MIMOSA at the 4x rate cap, then passed at 1x. Subsequent runs use a 1x cap. Root cause of the crash remains undetermined.']
(dest/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(dest/'README.txt').write_text('''ZaB 軌道追従区間の独立LIO/VIOと真値

各label/odometryはROS2 MCAP bagです。以下の3トピックだけを保存しています。
/dragon/ground_truth : 元bagのFC IMU基準の真値
/slam/odometry : MIMOSA LiDAR+IMU（外部odometryなし）
/rovio/odometry : ROVIO 3カメラ+IMU

離陸・開始前のホバリング/向き合わせ・軌道終了後の停止/着陸は出力窓から除外しています。
推定器初期化のため、それ以前のセンサー入力は再生しています。
画像・点群は結果に含めていません。元データはDriveのsource_remoteに保持しています。

各window.jsonは元bag識別子と対象のstart_ns/end_nsを含みます。
CSVのstamp_ns/sec/nanosecとOdometry.header.stampは元の取得時刻を保持しています。
bag保存時刻にも同じ整数ナノ秒を使用します。時刻原点の変更・丸め・補間はしていません。
LiDARとLIO、3カメラ画像とVIOはheader.stampで対応づけてください。
異なるbagは時計がリセットされるため、source_runとstamp_nsの組で識別してください。

座標系は真値・LIO・VIOそれぞれの元のframe_id/child_frame_idを保持しています。
真値に合わせた位置・向きの補正は行っていません。空間的な誤差比較には座標系の整合が必要です。
ros2 bag play /保存先/label/odometry --clock で再生できます。

制約:
run1_middleのVIO終端1323.15秒と1323.20秒は後続入力不足により出力されていません。
この2時刻の値は補作せず、result.jsonのunavailable_tail_stampsに明記しています。
4倍上限で開始しましたが、run1_tail_a1でMIMOSAが異常終了したため同区間を等速で取り直しました。
以後は等速上限です。推定器の設定・センサー内容・取得時刻は変更していません。
''')
print(json.dumps(dict(product=str(dest),runs=len(manifest['runs']),bytes=sum(f.stat().st_size for f in dest.rglob('*') if f.is_file())),ensure_ascii=False))
