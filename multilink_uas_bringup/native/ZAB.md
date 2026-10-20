# ZaB on Gazebo Harmonic

ZaBのトンネルモデルを現在のROS 2 Humble / Gazebo Harmonic / WSL環境で使う。

```bash
ros2 launch multilink_uas_bringup zab.launch.py
```

既定はZaBのGazebo画面、RViz、地上のDRAGON（1 LiDAR、3カメラ、IMU）、洞窟比較と同じ独立LIO（MIMOSA）・VIO（ROVIO）。
RVizに点群、真値・LIO・VIOの軌跡、位置3軸・姿勢3軸の誤差プロットを表示する。ROVIOをMIMOSAへ融合しない。
起動位置はworld座標 `(0,0,0.5)` m、world名は `zab`。
自動arming・離陸・軌道追従・rosbag記録は起動しない。
ZaBの入口はrecord:=falseが既定。明示的なrecord:=trueで記録する。洞窟用dragon_compareもrecord:=falseが既定で、明示的なrecord:=trueの場合だけrosbagを記録する。
軽量な評価CSV・誤差プロット・実行ログは保存する。
終了は起動した端末のCtrl+C、または `python3 /root/uas_native/manager.py stop`。
robot:=trueでは既存managerで推定系とシミュレーションをまとめて管理する。

```bash
# Gazebo画面なし、RVizと推定系は起動
ros2 launch multilink_uas_bringup zab.launch.py headless:=true
# RVizも不要な場合は rviz:=false を追加
# 環境だけを表示（推定系とRVizは起動しない）
ros2 launch multilink_uas_bringup zab.launch.py robot:=false
```

`spawn_x`, `spawn_y`, `spawn_z` は変更可能。既存の洞窟用経路はZaBでは使わない。
ZaBでの安全な飛行軌道と離陸・飛行中のLIO/VIO評価は未実施。洞窟用経路を別worldに適用する起動はmanagerが拒否する。
カメラ、IMU、LiDARの仕様は洞窟で使用したものと同一。物理刻み1 ms、目標倍率1。

## 資産と再構築

配布元: https://zenodo.org/records/17407065 （DOI: 10.5281/zenodo.17407065）

大きな配布モデルは `/root/multilink_autonomy_ws/assets/zab/` に配置し、ソースGitには含めない。
再構築スクリプトはダウンロードとチェックサム検証、展開、Harmonic向けworld作成を行う。

```bash
python3 /root/uas_native/setup_zab.py
# 別の保存先を使う場合は両方へ指定
python3 /root/uas_native/setup_zab.py --destination /path/to/zab
ros2 launch multilink_uas_bringup zab.launch.py assets:=/path/to/zab
```

- `ZaB_Model.zip`: 原本を保持。MD5 `a0409a2c449f67bf5a80d1ac91714886`。
- `models/`: 配布モデル・メッシュ・テクスチャをそのまま展開。
- `zab_harmonic.sdf`: このプロジェクト用のworld。
- `provenance.json`: 配布元、ハッシュ値、変更内容。

変更はsystem plugin名のIgnition→gz-sim移行、物理刻み3 ms→1 ms、重複sceneの統合、GUI設定。
付属地上車両vehicle_blueとそのキーボード指令pluginは除外した。
配布ZIPに含まれない `newJerseyElement`（入口のバリケード1個）も参照を外した。
32個の環境includeと39灯を保持。トンネルの形状・配置・テクスチャは変更していない。
元モデルのライセンスを独自に付け替えず、配布元の条件を参照する。

## WSLでの起動対策

Gazeboサーバーのセンサ描画は既存のD3D12/NVIDIA設定で実行する。
GUIだけは `LIBGL_ALWAYS_SOFTWARE=1` で別プロセスとして起動する。
この環境では大きなZaBモデルをGPU GUIへ読み込むとNVIDIA WSLドライバ内でクラッシュしたため。
GUI描画方法の変更はセンサ画像や物理演算の品質設定には影響しない。

モデル読み込み中に既存DRAGON spawnerが5秒のcontroller activation待ちを超えることがある。
`zab_controller_ready.py` がシミュレーション開始後に4制御器の状態を調べ、未ロードの制御器をロードし、未起動の制御器を再有効化する。
armingや飛行指令は送らない。初期spawnerのtimeoutが出ても、最後に
`All four DRAGON controllers active; robot remains disarmed` が出れば回復している。

検証ログは `native/logs/zab-check/` と `zab-final-validation.log`。
カメラ3台の画像、LiDAR点群、IMU、真値、非arming状態を確認する。


## 100 m直線飛行（明示的に実行すると離陸する）

```bash
ros2 launch multilink_uas_bringup zab.launch.py headless:=true record:=true route:=/root/uas_native/routes/zab_straight
```

検証済みworldハッシュと対応する経路のspawn位置を使い、離陸→始点・向き合わせ→100 m直進→着陸。
巡航1 m/s、始終端の加減速各2秒、経路時間102 sim秒。終点で3秒保持後に着陸し、
非arming状態に戻ったらmanagerが全構成を終了、rosbagのメタデータを確定する。
RVizだけ表示し、Gazebo GUIは起動しない。経路未指定時の自動飛行なし・記録OFFは維持。
詳細・メッシュ照合は `routes/zab_straight/README.md` を参照。
