# Multilink autonomy

研究・統合用のROS2 Humbleワークスペースで使うソースリポジトリ。
以下の構成図・コマンドの相対パスは `/root/multilink_autonomy_ws` を基準とする。
リポジトリ自体はその下の `src/multilink_autonomy` に配置する。

2026-09-30に旧 `/root/uas_native` の
実装・構築スクリプト・資料・ログ一式を移設した。

```text
src/multilink_autonomy/
├── for_codex/                  # 研究方針・引き継ぎ
└── multilink_uas_bringup/      # ament_cmakeパッケージ
    ├── launch/unipilot.launch.py
    └── native/                # 旧uas_nativeの全ファイル
```

UAS本体は `/root/unified_autonomy_stack`、ROS1基盤は `/root/ros1_noetic`、
Python環境は `/root/.venvs/uas` で引き続き管理する。
`/root/uas_native` は移設先への互換symlinkであり、ファイルの複製ではない。
`native/handoff_reference` は履歴用スナップショットとして保持し、colcon探索から除外する。
今後の実装変更はこのWSのソースに行う。

## UniPilot：3カメラROVIO＋MIMOSA

```bash
ros2 launch multilink_uas_bringup unipilot.launch.py mode:=vio_slam headless:=false rviz:=true
```

UASの既存洞窟・UniPilot機体・LiDAR・IMUを使い、前方／左／右の3カメラを追加する。
3台すべてを **1つのROVIO（`ROVIO_NCAM=3`）** へ渡し、`/rovio/odometry` をMIMOSAの
外部odometry制約として融合する。ROVIOとMIMOSAの初期化、および3台由来の特徴点を待って、
真値制御で離陸→2 m四方の移動→着陸する。`Ctrl+C`で全体停止。
DRAGON側のモデル・起動設定には変更を加えていない。

| カメラ | Gazebo topic / ROS1・ROS2画像topic | 投影モデル・水平視野 | 光学中心（base_link基準、m） |
|---|---|---|---|
| 前方 / cam0 | `/rmf_unipilot/cam_front/image` | pinhole・118度 | `(0.18, 0, 0.18)` |
| 左 / cam1 | `/rmf_unipilot/cam_left/image` | equidistant・185度、yaw +90度 | `(0.02, 0.08, 0.18)` |
| 右 / cam2 | `/rmf_unipilot/cam_right/image` | equidistant・185度、yaw −90度 | `(0.02, −0.08, 0.18)` |

各20 Hz、720×540。IMX296の1440×1080に対する1/2解像度で検証する。
Gazeboのwideangle sensorはRGB形式で描画する。ROVIO上流の投影モデルは180度超の画角を扱えないため、
`vio_images.py` が校正値に基づき3画像を480×360のpinholeへ変換し、mono8で渡す。
ROVIOが使う画角は前方118度・左右120度。185度の左右生画像は元topicに保持する。
3画像とも推定に使用するが、左右の生画像の全画角をそのまま推定に使うわけではない。
カメラ・IMUの内部／外部校正は、モデル生成と同じ値から生成する。
3台の画像stampが一致した組のみを更新に使い、不完全な組は待機数の上限で破棄する。
Gazeboでは同期撮影、実機では同期済み画像topicを入力する必要がある。

**この取付配置・校正はシミュレーション用であり、実機UniPilotの校正値ではない。**
UAS公開設定には実機の全3台分の校正が揃っていないため、3台という構成とセンサ仕様を参考にした
仮定配置である。ARL論文が全3台をROVIOへ入力した構成を再現した、という意味でもない。
実機には個体ごとのintrinsics・歪み・IMUとのextrinsics・時刻同期設定が別途必要。

RVizには融合地図・軌跡と3画像を表示する。`/rovio/pcl` の `camId` で、
実際に追跡した特徴点の出自を確認できる。真値は飛行制御と評価にのみ使い、
ROVIOのpose/odometry/twist補正入力とMIMOSAの外部入力へは渡さない。
ROVIOはIMU＋3画像、MIMOSAはLiDAR＋IMU＋ROVIO推定を受ける。

ログには従来の地図・軌跡に加え、`camera_manifest.json`、`raw_cam0.yaml`～`raw_cam2.yaml`、`cam0.yaml`～`cam2.yaml`、
`rovio.info`、`vio_trajectory.npy`、`vio_aligned.csv` を保存する。
`flight_slam_result.json` にVIO単体・MIMOSA融合後の初期整列による誤差と、全カメラの特徴点確認結果を記録する。

ROVIO再ビルド：

```bash
bash "$UAS_NATIVE_ROOT/build_rovio.sh"
```

ソースrevisionは `native/rovio-source-commits.json`、変更差分は
`native/rovio-three-camera.patch` と `native/rovio-lwf-cxx17.patch`。
上流のROS画像subscriberが2台分に固定されていたため、全カメラ対応とstamp一致のバッファを追加した。
Ubuntu22.04向けにC++17・OpenCV4・system yaml-cppへ対応。
ROVIOの起動時内部微分テストは実機校正用の通常起動から分離し、`run_self_test:=true` のROS parameterで明示的に実行できる。
ROVIOノードのみを構築しており、上流のrosbag専用実行ファイルの3カメラ対応は検証していない。

実機データやbagを入力する場合は、ROS1の `native/rovio_unipilot.launch` に
`calibration:=<実機校正ディレクトリ>` と `imu_topic`・`cam0_topic`・`cam1_topic`・`cam2_topic` を指定する。
校正ディレクトリには `rovio.info` と `cam0.yaml`～`cam2.yaml` が必要。入力画像はこの校正と一致する投影モデル・解像度に整流しておく。今回の検証対象はシミュレーションで、実機ドライバの動作は未検証。

## DRAGON：単一LiDAR＋FC IMUによるMIMOSA地図生成

現在の作業対象。UniPilotデモは起動しない。

```bash
ros2 launch multilink_uas_bringup dragon_slam.launch.py headless:=false rviz:=true
```

MIMOSA初期化後に自動で離陸し、高度1.5 mでその場にホバリングする。
Gazeboのモデル名は `dragon`、UniPilotと同じ `(40, 5, 0.5)` にspawnする。
`headless:=true`（既定）でGazebo GUIなし。`rviz:=true` はMIMOSAの地図・軌跡を表示する。
30秒の安定ホバリングを記録後も姿勢を保持する。検証後に着陸する場合は
`land_after_check:=true` を追加する。`Ctrl+C` はシミュレーション全体を終了する。

- 機体・飛行制御は `/root/ros2/aerial_robot_base_ws` の現行ROS2 DRAGONを使用。
  `estimation.mode=2`、真値odometryのノイズ・ドリフトはゼロ。
- ベースリンクを含むlink2にLiDARを1台だけ配置。既存Livoxの形状・質量を使い、
  GPUスキャンは360 × 32点、10 Hz、上下視野−7～52度。
- IMUは既存FC IMUの加速度・角速度（1 kHz）。MIMOSA入力からorientation真値を除去。
  LiDAR外部パラメータはFC基準 `[-0.1925, 0.001, 0.049] m`、回転なし。
- MIMOSA入力はLiDAR＋IMUのみ。機体の真値odometryは飛行制御と誤差評価に使用し、
  MIMOSAには入力しない。地図は `/slam/map`、推定は `/slam/odometry` と `/slam/path`。
- 分散LiDAR、関節変形、水平移動、RLは起動しない。Docker・venvの手動操作は不要。
  ROS1 MIMOSAとbridgeはlaunch内部で環境を分離して起動する。
- ワールドはUniPilotと同じUASの `subt_cave_sim/worlds/darpa_cave_01.sdf` を直接使用。
  洞窟・地形・配置は変更せず、機体をDRAGONへ置き換える。
  GPU LiDARは瞬時スキャンで、Livox実機の非反復走査の再現ではない。

起動ごとの `native/logs/<日時-PID>/` に生成URDF、真値設定、ワールド出典・ハッシュ、各ノードのログ、
`flight_slam_result.json`、`trajectories.npz`、`aligned_trajectory.csv`、
`slam_map.ply` / `slam_map_xyz.npy` を保存する。
真値との比較はFCの同じ原点で、初期SE(3)整列と時刻補間を用いる。
SLAM推定による飛行制御・ループ閉じ込みは今回の範囲に含めない。

DRAGON本体への変更はbringupへの `robot_description_file`・`world_sdf_file`・
`sim_param_path`・`spawn_x`・`spawn_y` の指定口だけ。センサ生成・世界・MIMOSA設定・起動・評価はこのWSに置く。
`native/env_dragon.sh` が既存のROS2環境にaerial_robotのoverlayを追加する。
実装記録は `src/multilink_autonomy/for_codex/WORKLOG.md` に追記する。

洞窟での確認ログは `native/logs/20260930-171114-119726/`。
高度1.49933～1.50006 mで約30秒の安定ホバリング、地図19,245点、
MIMOSA odometry 10 Hzを確認。初期整列後の位置RMSEは0.00565 m。
これは洞窟内の単一点での動作確認結果。

## UniPilot起動（既存機能）

新しい対話Bashでは `.bashrc` がROS2、UAS、このWSを自動読み込みする。
Python仮想環境は有効化しない。`mode:=stack` のRLプロセスだけが専用venvを使用する。
現在開いている端末では初回のみ `source ~/.bashrc` を実行する。

```bash
# UniPilot + SubT洞窟 + Gazebo GUI + RViz
ros2 launch multilink_uas_bringup unipilot.launch.py headless:=false rviz:=true

# GUIなし（GPUセンサ描画にはWSLgを使用）
ros2 launch multilink_uas_bringup unipilot.launch.py

# ROS1 bridge・GBPlanner/Voxblox・RL・CBFも起動
ros2 launch multilink_uas_bringup unipilot.launch.py mode:=stack

# MIMOSA初期化後、自動離陸・2 m四方の飛行・着陸（RLは使用しない）
ros2 launch multilink_uas_bringup unipilot.launch.py mode:=flight_slam headless:=false rviz:=true
```

起動端末の `Ctrl+C` で、その起動に属するプロセス群を停止する。
同時に複数構成は起動できない。ログの場所は起動時に表示し、
`native/logs/latest/` から参照できる。GUIの有無とRVizはどちらのmodeでも指定可能。

`sim` と `stack` は観測・接続検証構成。CBF出力は `/validation/cbf_safe_cmd` に分離され、
機体へ制御指令を送らない。odometryはGazebo真値で、MIMOSAは未接続。
標準モデルのIMU・カメラは無効。自律飛行・SLAM精度評価は別途行う。

`flight_slam` はIMUを有効化した派生モデルを生成し、LiDAR＋IMUをROS1 MIMOSAに入力する。
真値姿勢を含むIMU orientationは入力前に除去し、真値odometryは推定器に渡さない。
位置制御にはGazebo真値を使う。SLAM推定値による閉ループ飛行・自律探索・ループ閉じ込みは未実装。
MIMOSA初期化を待って約1.5 m上昇、2 m四方の経路を飛行し、着陸する。
飛行終了後も地図表示のため構成を維持する。停止は `Ctrl+C`。
`/slam/odometry`、`/slam/path`、`/slam/map` をROS2へ配信する。
ROS1のビルド領域は `slam_ros1_ws/` に分離し、再ビルドは `bash "$UAS_NATIVE_ROOT/build_slam.sh"`。
MIMOSAと依存ソースはUASの `repos/ws_mimosa_exact.repos` のrevisionを使用する。
LiDARとIMUの配送順による範囲外参照を防ぐ待機処理をMIMOSAに追加している。

生成モデルではモータを指令する制御器を速度制御の1系統にし、鉛直速度ゲインを10に設定する。
通常のUASモデルは変更していない。Gazebo点群は同時刻スキャンとして `t=0` で入力し、
実機の走査時刻やIMUノイズの完全再現を意味するものではない。
Jammy向けのfmt対応とIMU待機の差分は `native/mimosa-imu-wait.patch` に保存。

### 飛行・SLAM確認結果（2026-09-30）

採用結果は `native/logs/20260930-154020-97154/`。
先行する高度追従不足・監視/保存処理の修正前の試行とは分けて扱う。

- 離陸確認後に水平約2 m × 2 mを飛行し、元の位置へ着陸。
  高度変化1.439 m、着陸位置 `[39.99975, 4.99970, 0.05000] m`。
- MIMOSA出力949件、最大出力間隔0.1秒（センサ時刻）。飛行中の地図更新57回、保存地図33,699点。
- 初期SE(3)整列のみ、真値位置を推定時刻へ補間して評価。
  飛行区間の位置RMSE 0.01573 m、最大誤差0.03047 m。
- 2秒のシミュレーション一時停止・再開を挟んでも飛行を継続。
- 実際のMIMOSA購読先は `/clock`、`/slam/input/imu`、`/slam/input/points` のみ。
  RL/CBFプロセスおよび有効化されたvenvなし。
- GUI・RVizの起動と描画初期化も確認。最終の通し検証はGUIなし。

`flight_slam_result.json` と `validation.json` に結果、`trajectories.npz` に真値・推定軌跡、
`aligned_trajectory.csv` に整列済み軌跡、`slam_map.ply` / `slam_map_xyz.npy` にSLAM地図を保存。
`flight_slam.png` は軌跡と誤差の図。再作図は以下。

```bash
python3 "$UAS_NATIVE_ROOT/plot_flight_slam.py" "$UAS_NATIVE_ROOT/logs/20260930-154020-97154"
```

この結果は単一の洞窟内短距離デモの接続・動作確認であり、多環境での精度評価ではない。

## 動作確認・管理

```bash
python3 "$UAS_NATIVE_ROOT/manager.py" status
python3 "$UAS_NATIVE_ROOT/check_topics.py" --duration 30 --best-effort

# stack起動中のみ。ROS1はROS2と環境を分離する
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin \
  bash --noprofile --norc -c 'source /root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/env_ros1.sh && python3 /root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/check_planner.py'

# 別端末から停止する場合
python3 "$UAS_NATIVE_ROOT/manager.py" stop
```

`native/run.sh` は従来の検証コマンドとの互換用に保持する。
`check-inference` は切断を確認してからRLへ検査用目標を送るため、検査後に構成を停止する。
launch内部のROS1プロセスはクリーンな環境で起動する。
ROS2を読み込んだ端末で `env_ros1.sh` を直接sourceすると混在防止のため拒否する。
自動読み込みを省くには `MULTILINK_AUTO_ENV=0 bash` をクリーンな親環境から起動する。
この指定は既に読み込んだROS環境を消去するものではない。

## ビルド

```bash
cd /root/multilink_autonomy_ws
colcon build --symlink-install --packages-select multilink_uas_bringup
source install/local_setup.bash
```

非対話スクリプト・IDEタスクは `.bashrc` が自動実行されないことがあるため、
`src/multilink_autonomy/multilink_uas_bringup/native/env.sh` を明示的にsourceする。
初回ビルド前も同ファイルから依存環境を読み込める。
ROS1パッケージはこのcolcon WSへ混ぜない。

移設前のファイルハッシュは `native/workspace-migration-before.json`、
変更前の `.bashrc` は `native/bashrc.before-workspace-migration` に保存している。

## 移設後の確認（2026-09-30）

- colconビルド成功、新規対話Bashで環境とパッケージを自動認識。
- シミュレーション単体はGazebo GUI・RVizを起動し、描画初期化ログを確認。
  30秒観測でLiDAR 24,000点・10 Hz、odom 100 Hz、RTF 0.999、定常変位0 m。
  GUIの画面内容は目視確認していない。
- 全体構成は定常30秒でLiDAR 10 Hz、odom 100 Hz、RTF 0.991、変位0 m。
  起動直後の別計測にある約0.451 mの変位は、初期高さ0.5 mから接地までの沈下を含む。
- ROS1側30秒で点群297件・odom 2,976件・メッシュ31件、world→LiDAR TF有効。
- RL/CBFの15秒推論は147件/751件、全出力有限、機体変位0 m。
- Ctrl+Cによる単体・全体構成の停止後に、管理対象プロセス群の残存なし。
  二重起動拒否、ROS環境混在拒否、環境再読み込み時のPATH不変も確認。
- ROS2を自動読込した端末からROS1法線フィルタ検査を実行し、6条件すべて差分0。
- 移設前の266ファイルは欠落なし、`handoff_reference` 内のファイルハッシュは変更なし。

生の検証結果は `native/logs/*migration*.json` に保存。検証後の構成は停止済み。


## Gitで管理する範囲

現行のソース、設定、依存ソースのrevision、外部ソースへの修正patch、作業記録を管理する。
ビルド結果、実行ログ・計測データ、キャッシュ、実行中の状態、移行時バックアップ、
旧環境の参照コピー `native/handoff_reference/` は `.gitignore` で除外する。
これらはローカルには残るが、cloneには含まれない。
外部のUAS・ROS1・ROVIO・DRAGONのリポジトリ本体も、このリポジトリには含めない。
