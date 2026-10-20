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

## DRAGON：分散3カメラROVIO＋MIMOSA

```bash
ros2 launch multilink_uas_bringup dragon_vio.launch.py headless:=false rviz:=true
```

LiDARはlink2の1台を維持し、**link1・link3・link4にカメラを1台ずつ**配置する。
各リンクからの取り付けは共通で、光学中心の位置 `(0.10, -0.045, 0)` m、
姿勢 `(roll, pitch, yaw) = (0, 0, -127.5度)`。リンクと同じ高さで側面に短い取り付け部を設け、
中央のローターを避けて−Y方向から−X側へ37.5度振る。
四角形の離陸姿勢では3台とも機体の外側を撮影する。
カメラは水平視野120度、720×540、20 Hzの同一仕様。ROVIO入力は480×360の白黒画像。
カメラ筐体は奥行25 mm・幅30 mm・高さ30 mmの箱形表示とし、実機のカメラ重量・取り付け具は未確定のため追加質量は模擬しない。

3台を1つのROVIOへ入力し、その推定をLiDAR・IMUとともにMIMOSAへ融合する。
IMUはFCのものを共有。カメラとFCの位置関係は、生成した機体モデルと四角形の関節角度から計算する。
真値は飛行制御と評価専用で、ROVIOとMIMOSAへ位置・姿勢の補正として入力しない。

全3台の特徴点、ROVIO、MIMOSA、四角形の関節角度を確認してから、自動で離陸する。
高度1.5 mで安定したホバリングを約30秒検査し、その後も位置を保持する。
`land_after_check:=true` を付けると検査後に着陸。終了は `Ctrl+C`。
RVizには地図・推定軌跡と3台の画像を表示する。

**今回は四角形を保つ飛行専用。変形中のカメラ・IMU間の位置関係の変化には未対応。**
ROVIOは固定の取り付け校正を使用し、離陸・ホバリング検査中に機体関節が校正角度から
0.03 radを超えて外れた場合は検査失敗として着陸する。
センサーは各リンクに取り付いているため、Gazebo上ではリンクとともに動く。

画像topicは `/dragon/cam_link1/image`、`/dragon/cam_link3/image`、`/dragon/cam_link4/image`。
順にROVIOのcam0・cam1・cam2へ入り、`/rovio/odometry` をMIMOSAへ渡す。
`camera_manifest.json` に配置・校正、`body_joints.npy` に関節角度、
`flight_slam_result.json` に離陸・ホバリングと推定誤差を保存する。

独立したセンサー確認（実行中のホバリングを観測するだけ）：

```bash
source "$UAS_NATIVE_ROOT/env_dragon.sh"
python3 "$UAS_NATIVE_ROOT/verify_dragon_vio.py" --output "$UAS_NATIVE_ROOT/logs/latest"
```

3画像の同期、実際のカメラ座標と校正値の一致、HOVER状態を確認し、
`dragon_vio_sensors.json` と画像を保存する。
取り付け位置の干渉検査は `verify_camera_clearance.py --output <実行ログのディレクトリ>` で実施できる。
機体関節・ロータージンバルを個別に変えた344姿勢で、カメラ筐体と表示メッシュの交差を検査する。
これは全関節の同時変形や連続経路の無干渉を保証するものではない。

DRAGON依存側の固定座標配信の修正は `native/dragon-rotor-tf.patch` に保存。
`aerial_robot_model` の `rotor_tf_publisher` を再ビルドして適用する。
従来のカメラなし起動は `dragon_slam.launch.py` を使用する。

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


## DRAGON：独立LIO/VIOで洞窟軌道を追従・記録

```bash
ros2 launch multilink_uas_bringup dragon_compare.launch.py headless:=true rviz:=true
```

**この起動は自動で離陸し、596 mの軌道を反復飛行する。**
離陸前に両推定器を待ち（record:=trueの場合は記録開始も待つ）、安定ホバリング後に世界座標FC位置(40,5,2)へ移動する。
飛行制御は真値を使い、FC基準の経路から重心位置・速度・yaw指令へ変換する。
MIMOSAはLiDAR＋IMUのみ、ROVIOは3カメラ＋IMU。ROVIOをMIMOSAへ入力しない。
Gazebo GUIは開かず、RVizだけを表示する。センサ品質設定・物理刻み1 msは維持。

RVizは真値を緑、LIOを橙、VIOを青で表示し、左の画像パネルに位置X/Y/Z[m]と
roll/pitch/yaw[deg]の真値誤差を表示する。真値を推定値と同じ時刻へ補間し、
開始時の剛体座標合わせを各推定器に一度だけ適用する。
位置誤差は世界座標の推定値−真値、姿勢誤差は `R_truth^-1 R_estimate_aligned` のxyz Euler角。
真値による連続補正や区間ごとの再整列は行わない。グラフは直近180 sim秒、CSVは全期間。

記録先は `native/logs/<日時-PID>/`（起動中は `native/logs/latest/`）。

- `rosbag/`：既定では作らない。明示的な `record:=true` の場合のみ、生画像3台・LiDAR・IMU・真値・重心odometry・関節状態・飛行状態・指令・両推定値・誤差・TF。
  無圧縮、1 GiBごとに分割。画像を間引かず保存するため、1周で約120 GBが目安。
- `lio_raw.csv`, `vio_raw.csv`, `truth_raw.csv`：時刻・位置・quaternion。
- `lio_errors.csv`, `vio_errors.csv`：同時刻の真値、整列後位置、6軸誤差。
- `alignment.json`：評価用に固定した初期座標変換。
- `route_tracking.csv`, `route_status.json`：追従指令・真値・追従誤差・周回進捗。
- `error_plot.png`：RViz表示と同じ最新グラフ。
- `implementation/`, `route/` と生成URDF・カメラ校正：実行時の構成スナップショット。

1周後は始点で向きを戻し、次の周回へ進む。推定器の誤差が増えても真値制御で継続する。
追従誤差0.4 m以上では軌道時刻を進めず、0.8 m超、真値欠落、関節固定解除、飛行状態異常、
空き容量20 GB未満ではGazeboを一時停止して記録を保全する。任意地点への自動着陸は行わない。
停止は起動端末のCtrl+C、または `python3 /root/uas_native/manager.py stop`。
記録を有効にした場合、停止後にbagの最終メタデータが確定する。

比較モードのRVizは専用watchdogで再起動するため、RVizを閉じても飛行・記録は止まらない。全体終了にはlaunchのCtrl+Cまたはmanager stopを使う。


## ZaBトンネル環境

```bash
ros2 launch multilink_uas_bringup zab.launch.py
```

Gazebo HarmonicでZaB、地上のDRAGON（LiDAR・3カメラ・IMU）、独立LIO/VIO、RVizの点群・軌跡・6軸誤差表示を起動する。
自動飛行・rosbag記録は開始しない。評価CSVは保存する。
Gazebo画面なしは `headless:=true`（RVizは起動）、RVizなしは `rviz:=false`、環境のみは `robot:=false`。
モデルはWS直下 `assets/zab/`、再構築は `python3 /root/uas_native/setup_zab.py`。
配布物との差分・WSL描画対策・操作方法は [ZaB手順](multilink_uas_bringup/native/ZAB.md) を参照。

2026-10-02、ユーザー指示により10月1日の比較試験2回分のrosbagを削除済み。
CSV、6軸誤差、グラフ、設定・評価結果は各ログディレクトリに保持している。


## ROS2環境とキーボード操作（2026-10-02）

新しい対話Bashでは `.bashrc` から `native/env.sh` を読み込む。
ROS2 Humble、UAS（sim/CBF/RLのROS2パッケージ）、aerial_robot_base、ROS2側ros1_bridge、統合WSが利用可能。
ROS1環境・Python仮想環境は読み込まない。既存端末への反映は `source ~/.bashrc`。

```bash
ros2 run aerial_robot_core keyboard_command.py --ros-args -p robot_ns:=/dragon
```

従来どおり端末内で操作する。GUIは起動しない。
- W/S：base_linkの前後、A/D：左右、[/]：上下。XY各軸0.4 m/s（旧0.2）、Zは0.2 m/s。キーは小文字。
- Q/E：左右旋回。既存のR/T/L/F/H（arming/離陸/着陸/強制着陸/モータ停止）、X、U/J/I/Kも維持。
- 直近0.25秒に受け取った移動キーを合成する。W+Aなら前方0.4＋左方0.4 m/s。反対キー同士は相殺する。
- 端末にはキー解放イベントがないため、厳密な同時押しではない。キーごとに入力が0.25秒途切れると解除する。長押し時に最後の1キーだけ繰り返される端末では合成は維持されない。
- Space・Ctrl+Cで移動指令を止める。終了時は端末設定を復元する。
- base_linkの姿勢は `/dragon/uav/baselink/odom` を使用。傾きも含めた3次元方向であり、世界水平面の移動ではない。
- 姿勢が未取得、無効、または0.5実秒以上途切れた場合は移動指令を止める。
- ROS側のLOCAL_FRAMEは重心yawなので使わず、base_link速度を世界座標へ変換してFlightNavで送る。制御対象は従来どおり重心。
- ROSパラメータ `xy_vel`, `z_vel`, `yaw_vel`, `rp_vel`, `odom_timeout`, `key_timeout` で調整できる。

変更元は `/root/ros2/aerial_robot_base_ws/src/aerial_robot_base/aerial_robot_core/`。
再現用差分は `native/aerial-keyboard.patch`。外部リポジトリの既存変更は含めていない。
テストは同パッケージ `test/test_keyboard_command.py` をROS2環境で実行する。

## 単眼ジンバル：DRPM＋MIMOSA＋ROVIO（2026-10-13）

固定四角形のDRAGONのlink3に、光学中心で交差するyaw/pitchの理想ジンバルを配置する。
LiDARはlink2の1台、カメラはlink3の1台。観測済みの視覚特徴点から候補視野の情報行列を計算し、
現在のLiDARの弱い方向の不確かさを減らす向きを選ぶ。進行方向への偏り・角度変更量の罰則を加え、
実際の関節速度の合成値を20度/秒以下に制限する。

初回ビルド（既存ROS1推定器のビルド済みWSを利用。Python仮想環境の有効化は不要）：

```bash
python3 /root/uas_native/active_perception/build.py
cd /root/multilink_autonomy_ws
colcon build --packages-select multilink_uas_bringup --symlink-install
source install/setup.bash
```

ZaBで離陸してホバリングし、カメラを能動制御する：

```bash
ros2 launch multilink_uas_bringup active_gimbal.launch.py \
  world_file:=/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf \
  spawn_x:=0 spawn_y:=0 spawn_z:=0.5 takeoff:=true
```

同じハード・推定設定でカメラを固定する対照は `mode:=fixed` を追加する。
`takeoff`の既定値はfalse。world指定なしは従来のUAS洞窟とspawn=(40,5,0.5)。
GUI・rosbag自動記録は起動しない。終了は起動端末のCtrl+C。所有するROS1 master・bridge・推定器・Gazeboをまとめて停止する。
既存managerと同じロックを使って二重起動を拒否する。今回のlaunchは`manager.py stop`の管理対象には含まれない。

実装は `multilink_uas_bringup/native/active_perception/`。

- 視覚地図：ROVIOの3次元特徴点・不確かさ・観測方向・時刻を蓄積。IDごとの更新と20cmセルの代表点選択で重複を抑え、60秒で失効する。未初期化の深度を除外する。
- 公式コード： [FIF](https://github.com/uzh-rpg/rpg_information_field) のrevision `76d7b834128bd3488d1aae3888a79f3d5216a4ab` に含まれる特徴点ベースの視覚微分計算を無改変で利用。必要なminkindrもrevisionを固定する。各点の6×6情報を一度計算し、その時刻の候補間で再利用する。FIFのボクセル近似やROS1サンプル一式の移植ではない。
- 選択：DRPMの確率・固有方向・有効情報を読み取り、機体座標の回転→並進順に揃える。85候補＋現在方向を評価し、弱い方向の予測不確かさの相対減少、進行方向、角度変更量で選ぶ。目標視野の評価値と、その時点で実行できる角度指令を分けて保存する。
- 可動ROVIO：画像時刻の実測関節角を補間する。画像更新直前に特徴点の方向・画像パッチの変形・共分散を変換し、既知のカメラ対IMU回転を更新する。光学中心の並進は拒否する。機体関節が四角形から0.03rad以上動いた場合も画像入力を止める。
- 推定器：採用済みDRPM更新＋距離方向ノイズモデル、修正済み単眼ROVIO設定、既存の固定融合共分散を使用。情報の読み出しだけを加えたMIMOSA共有ライブラリと可動ROVIOを`build/active_perception`に独立ビルドする。
- 出力：`native/logs/active-gimbal-日時/` に`decisions.jsonl`、`visual_map.npz`、実行設定・ソース写し・バイナリハッシュを保存。判断ログには対象センサの時刻、候補別評価、目標角、実測角、地図点数、計算時間を含む。

情報評価は候補選択用の近似であり、融合推定器の正確な将来共分散ではない。
地図と現在姿勢の相関・地図ドリフト・未観測の遮蔽・将来の画像追跡成功率を厳密には扱わない。
現在のLiDARで判明する遮蔽と観測方向の大きな違いは除外する。未観測の特徴を捏造せず、地図や退化情報が不足・古い場合は保持する。
地図はそのROVIO起動の座標系に属し、別起動で再利用するには位置合わせが必要。
カメラ並進を伴うジンバルやDRAGONの変形への対応、探索スキャンの方策、長距離での改善効果は今回の実装・検証の範囲外。

検証コマンド：

```bash
OPENBLAS_NUM_THREADS=1 python3 /root/uas_native/active_perception/test_core.py
python3 /root/uas_native/active_perception/build.py test replay
# test_gimbalの実行はROS1の共有ライブラリが見える環境で行う。
# 元bagを用いた固定時の対照再生はtest_static_replay.pyのprepare/replay。
```

ZaBで固定・可動それぞれの離陸と30秒ホバリングを検証済み。固定時の記録データ再生では、
従来ROVIOと位置・姿勢・共分散・バイアス・特徴点の数値が完全一致した。
検証用の真値は飛行制御と独立した評価にのみ使用し、視野選択や推定器には入力しない。

## UniPilot 固定/偏向比較の高速化試作（2026-10-14）

**欠落監査を通ったa1全区間の測定は394.04→306.96秒（1.284倍、22.1%短縮）。全3区間での5倍高速化と推定精度の同等性は未達・未検証です。**
地図内の点を検索する索引を軽量化し、画像・点群の受け渡しとCSV記録をC++へ移しました。
テストした検索結果・推定用行列・センサ値は元実装と一致しています。
物理刻み1ms、LiDAR10Hz/500×48点、IMU200Hz、カメラ480×270/20Hz、各ノイズ設定を維持しています。

全3区間試行 `native/logs/speed6-full/` ではa1/b1は完走しましたが、prefixは328.849/1070秒で追従誤差上限に達して停止しました。
この途中終了の時間を、全区間の高速化倍率へ換算しません。b1ではIMU受信に1件分の周期欠落もあります。
UniPilot単眼推定の大きなドリフトは残るため、通常起動への採用前の実験用です。旧DRAGONの精度結果は変更していません。
最終の軽量化試験は `native/logs/speed6-final-prefetch/`、集計は `native/logs/speed6-final-summary.json`、詳細は `for_codex/WORKLOG.md` です。
先読みなしの再測定は294.64秒でしたが、IMU1件分の周期欠落があり、欠落なしの速度結果には採用していません。
先読みの有無が欠落の原因だと確認したものではありません。

隔離ビルド（システムのGazebo/MIMOSAライブラリは置換しません）:

```bash
python3 /root/uas_native/active_perception/build_performance.py
python3 /root/uas_native/active_perception/build_mimosa_performance.py
python3 /root/uas_native/active_perception/build_serial_fusion.py
python3 /root/uas_native/active_perception/build_compact_mimosa.py --prefetch
python3 /root/uas_native/active_perception/build_native_adapters.py
python3 /root/uas_native/active_perception/verify_compact_mimosa.py --prefetch
python3 /root/uas_native/active_perception/verify_compact_search.py --prefetch
```

測定済みa1全区間の追試例（新規出力先を指定）:

```bash
python3 /root/uas_native/active_perception/unipilot_campaign.py \
  --output /root/uas_native/logs/unipilot-speed-new --labels run1_tail_a1 \
  --paired --fast --airborne-init --gpu-keepalive --jobs 1 --factor 5 \
  --paced --fast-icp --compact-map --map-variant prefetch \
  --native-adapters --native-monitor --clock-hz 1000 \
  --serialized-fusion --tbb-threads 1 --wall-limit 600 --domain-base 84
python3 /root/uas_native/active_perception/summarize_performance.py \
  /root/uas_native/logs/unipilot-speed-new
```

GUI/bagなし。固定/偏向は同じ物理飛行を観測し、独立した単眼推定器で処理します。
推定が遅れるとシミュレーションを待機させ、終了時は追従区間末尾の両VIO/両LiDARの処理を待ちます。
自分で起動した子プロセスは終了時に停止します。`--factor 5`は要求値であり実測倍率ではありません。
監査は完走・センサ時刻の欠落・始端/終端を判定し、精度同等性とは分けて出力します。
a1だけの試験では、全3区間を要求する `all_routes_complete` は意図どおりfalseになります。

`--corner-speed-limit`は曲がり角の前で減速する追加候補です。切り出した角で最大追従誤差17.6→7.0cmを確認しましたが、
全長・推定器同時負荷では未検証です。経路形状と停止上限0.18mは維持し、角の近傍だけ速度を落とします。
通常の手動ジンバルGUI起動は変更していません。


## UniPilot単眼のIMU・初期化修正（2026-10-14）

`active_perception/unipilot_run.py` / `unipilot_campaign.py` は、UniPilotのSDFに指定された時間変化するIMUバイアス（測定の偏り）と、worldの重力をROVIOへ反映する。`--imu-model sensor`が既定。バイアス平均、分散、状態の微分を同じ減衰モデルに揃え、真値からの校正は行わない。初期姿勢には3秒間の平均加速度を使う（`--stationary-mode mean`）。推定のノイズ密度やセンサ自体のノイズを恣意的に減らさない。

`--airborne-init`ではMIMOSAもROVIOと同じホバリング開始合図を待って初期化する。raw入力の欠落監査は待機中も継続する。以前の実験条件を再現する場合は、既存コマンドへ `--imu-model legacy --stationary-mode legacy --legacy-slam-start` を追加する。修正済みと旧条件の結果を混ぜないこと。DRAGON用runと手動GUIの起動設定はこの既定変更の対象外。

同一の画像・IMU・エンコーダ入力を再生して比較する任意診断には `--record-vio-input` を使う。大きな診断ファイルを生成するため既定OFF。自動rosbagは引き続きOFF。`--camera-far`は描画範囲の対照実験用であり、既定20mを維持する。

修正による改善と残るドリフトは `for_codex/WORKLOG.md` の2026-10-14精度修正項目を参照。カメラ配置だけが原因という結論には至っていない。


固定したジンバル角度でUniPilotの軌道を評価する場合は、`unipilot_campaign.py` の既存コマンドに `--modes fixed --fixed-angles-deg -50 0` を指定する（角度はyaw/pitchの順、単位deg、`--paired`とは併用しない）。カメラは推定結果を待たず指定角へ移動し、その後保持する。既定の角度未指定時の動作は変更しない。実測角度と初期化時刻を含む評価は `active_perception/report_fixed_view.py` で保存する。

### UniPilotのセンサ仕様比較

`native/active_perception/unipilot_campaign.py` と `unipilot_run.py` は
`--sensor-profile dragon` で、保存済みDRAGON仕様のカメラ・IMU・LiDARを使える。
`unipilot_gimbal_sim.launch.py` を直接起動する場合は `sensor_profile:=dragon`。
省略時は従来のUniPilot仕様（`native`）。機体・制御・取付位置/姿勢は変更しない。
カメラは120度、画像ノイズなし、720×540を480×360に変換。IMUは1,000 Hzで
DRAGONのノイズ/バイアス仕様。LiDARは360×32・最大30 mに変更されるため、
「ノイズだけ」の比較ではない。ジンバル0度固定は `--modes fixed --fixed-angles-deg 0 0`。
ROVIOの誤差設定までDRAGON参考試行に揃えるには
`--rovio-noise-profile dragon` を併用する。`--stationary-duration 0` は
ホバリング中の角速度を静止時バイアスと仮定して差し引く初期化を無効化する。
標準は従来どおり3秒。物理センサ仕様と推定器が仮定する誤差は別々に記録される。


## UniPilotジンバルの採用構成と比較結果（2026-10-19）

採用設定は `native/active_perception/tracking_aware_v57.yaml`。設定固定後の3経路×2乱数条件で、ROVIO＋MIMOSAの融合位置誤差は6組すべて改善し、6組の誤差比の平均から求めた低減率は17.74%だった。経路別平均はa1が21.30%、b1が20.48%、prefixが11.44%。事前に置いた平均20%以上という目標には届いておらず、元の合否判定はfalseのまま保持する。対象3経路以外や実機での改善を示す結果ではない。

既観測の画像特徴とLiDAR形状からカメラの向きを選び、追跡中の特徴を視野に保つ制約と、実際のジンバル速度制限を反映する。yawは±50°、pitchは±90°、合成角速度は最大20°/s。ROVIOの出力から速度と重力方向の傾きを1HzでMIMOSAへ入れる。DRPMによる融合補正はOFF。同時刻の観測を反映した後に地図と推定値を更新し、重力とIMUバイアスのモデルを模擬センサに揃えた。ROVIO単体の位置精度は6組中4組で悪化しており、融合位置の改善と混同しない。長区間には数メートルの誤差が残る。

固定・可動は同じ物理飛行、IMU、LiDARを共有し、カメラが動く前のROVIOと融合の内部出力が一致することを検査する。準備運動を除いた軌道追従期間だけを評価し、始点の位置・姿勢を合わせ、尺度は事後補正しない。prefixの対象は1070秒。画像480×270/20Hz、IMU1000Hz、LiDAR10Hz、画像の描画範囲1000mで比較した。完全な設定・ソース・実行ファイルのハッシュは凍結記録に残す。

追加のv59は、視認性予測にLiDARで支持された特徴距離を使う変更。開発6組では20.71%低減したが、独立検証の最初の3組で2組悪化したため不採用。実行中の4組目を完走させ、未開始の長区間2組は実行していない。実施した全結果と実装は保存している。

結果・六軸プロットへのリンクは `/root/uas_native/logs/gimbal-improvement-20261017/adopted_gimbal.html`。同ディレクトリの `validation_summary_v57.json`、`frozen_candidate_v57.json`、`frozen_v57_sources.zip` が元の結果と検証時の実装。採用時は81個のソース・実行ファイルが検証時と一致することを確認した。

再実行は、新しい保存先を指定してユーザーの端末から起動する。

```bash
python3 /root/uas_native/active_perception/run_selected_gimbal.py \
  --output /root/uas_native/logs/gimbal-repeat-new
```

既定は3経路の固定・可動比較。`--labels run1_tail_a1`で1経路、`--seed 20261026`で乱数条件を指定できる。`--dry-run`ではソースのハッシュと起動コマンドだけを確認する。既存の出力先への上書きは拒否する。終了はCtrl+C。GUIとrosbagは起動・収録せず、Windowsの空きが8GBを下回ったら停止する。

`native/active_perception/selected_gimbal.json` は採用設定の起動引数と必須ファイルのハッシュを保持する。再実行に必要な校正ファイルは `/root/uas_native/logs/gimbal-improvement-20261017/native_fusion_velocity_gravity_calibration.json`。結果フォルダを整理する際も、これらと元の数値・凍結記録は残す。数値と時刻は可逆圧縮し、圧縮前後の再集計結果が一致することを検査済み。
