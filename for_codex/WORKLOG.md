# WORKLOG.md


## 2026-09-30: DRAGONの単一LiDAR・FC IMUをMIMOSAへ接続

### 依頼と構成

UniPilotデモは一旦停止し、現行ROS2 DRAGONを使って、単一LiDAR＋IMUによる地図生成とその場でのホバリングを実装。飛行制御は真値odometry、MIMOSAはLiDAR＋IMUのみを入力する。分散センサ、関節変形、水平飛行、RLは起動しない。

記録先はユーザーの訂正に従い、この `for_codex/WORKLOG.md`。`AGENTS.md`にも同じ規則を追記した。旧 `handoff_reference/legacy_docs/WORKLOG.md` は変更していない。

### 実装

- `multilink_uas_bringup/launch/dragon_slam.launch.py` を追加。既存supervisorに `dragon_slam` を追加し、ROS2 DRAGON、ROS1 MIMOSA、bridge、sensor adapter、自動離陸・ホバリングをプロセス所有権付きで起動・停止する。
- `native/env_dragon.sh` で既存環境に `/root/ros2/aerial_robot_base_ws/install/local_setup.bash` を追加。Docker/venvへの切り替えなし。WSLg環境変数もsupervisorから継承する。
- `native/dragon_sim.launch.py` は現行DRAGON xacroから生成し、link2の既存Livox取付位置にGPU LiDARを1台追加。既存LiDARの質量・形状は保持。360×32点、10 Hz、視野360度×59度（仰角−7～52度）、範囲0.1～30 m、距離ノイズ0.03 m。
- IMUは既存FCの `spinal_imu`（1 kHz）。`/dragon/imu/raw` と `/dragon/lidar/points` をROS2へ公開。初回試行でセンサ下とworldのImu system重複による同一時刻の二重配信を発見し、最終の洞窟構成ではセンサ下のImu system pluginを除去し、洞窟worldのものを1回だけロードする。洞窟worldにないMagnetometerは機体側に残す。
- ユーザーの追加指示に従い、ワールドはUniPilotと同じUASの `subt_cave_sim/worlds/darpa_cave_01.sdf` を直接読み込む。洞窟モデル・地形・配置に変更なし。spawnも `(40, 5, 0.5)` に一致。初期検証で用意した室内worldは最終実装から削除した。描画は洞窟のOgre2を使用。
- 現行DRAGON bringupの `estimation.mode=2` を使用。生成Simulation.yamlで真値の全noise/driftを0にする。元のrobot設定を変更しない。
- `native/mimosa_dragon.yaml`：MIMOSA bodyはFC IMU、LiDARの `T_B_S=[-0.1925, 0.001, 0.049, 0, 0, 0, 1]`。FCとLiDAR双方のlink2原点から算出。odometry/radar/photometric入力は無効。
- `native/slam_inputs.py` の購読先をパラメータ化。生IMUの真値orientationを除去、GPUスキャン全点の時刻差を0としてMIMOSAへ入力する。UniPilotの既定topicは維持。
- `native/dragon_hover.py` はMIMOSAの初期化・地図を待ってarm→takeoff→HOVER。5秒の安定待ち後に約30秒のホバリングを評価・保存。既定ではホバリング継続、`land_after_check:=true` では検証後に着陸。真値とMIMOSAの時刻補間・初期SE(3)整列による評価処理は既存 `flight_slam.py` を共有。
- `native/verify_dragon_slam.py` は観測だけを行い、センサ数・点数・時刻の単調性・実測レートを保存する。飛行指令を送らない。
- aerial_robot側の変更は `robots/dragon/launch/bringup_launch.py` への `robot_description_file`・`world_sdf_file`・`sim_param_path`・`spawn_x`・`spawn_y` の指定口追加のみ。差分は `native/dragon-bringup-options.patch` にも保存。本体ソースを統合WSへ複製していない。
- WSのREADMEに起動・停止・入出力・ログ・制約を追記。

### 起動

```bash
ros2 launch multilink_uas_bringup dragon_slam.launch.py headless:=false rviz:=true
# 検証後に着陸する場合
ros2 launch multilink_uas_bringup dragon_slam.launch.py land_after_check:=true
```

モデル名 `dragon`、洞窟のUniPilotと同じ `(40, 5, 0.5)` にspawn。既定高度はCoG 1.5 m。`Ctrl+C`でそのlaunchの構成を終了する。

### 検証

- `colcon build --packages-select multilink_uas_bringup --symlink-install` 成功。
- Python構文・XML/YAML読込、aerial_robotの `git diff --check` 成功。
- `native/logs/20260930-170511-116050` はIMU重複修正前の試行。ホバリング・地図生成は動作したが採用結果と区別する。
- `native/logs/20260930-170657-117215` でGUI/RVizありの離陸・ホバリング・着陸を確認。GUIログのrender rate約58 fps、RVizの描画初期化を確認。
- 同試行の観測 `sensor_validation.json`：GPU LiDAR 1台、IMU 1台、11,520点/scan、IMU 1,000 Hz、LiDAR 10 Hz、MIMOSA odometry 10 Hz、時刻重複0。MIMOSA実購読は `/clock`・`/slam/input/imu`・`/slam/input/points` のみ（`mimosa_connections.txt`）。`estimation.mode=2` と真値noise/drift=0を実際のparameterからも確認した。

### 制約

GPUスキャンはLivoxの非反復パターンや実機の走査時間を再現していない。結果は洞窟内の単一点での短時間シミュレーション動作確認であり、実機精度・多環境での頑健性を示すものではない。MIMOSA推定値による閉ループ飛行・ループ閉じ込みは範囲外。地図はキーフレーム更新時に配信するため、静止ホバリング中に毎スキャン増えるものではない。RVizはlaunch開始時から起動すると確実に地図を受信する。

### 洞窟での最終確認

環境をUASの洞窟に揃えるよう追加指示があり、室内の結果は参考扱いに変更した。`20260930-171012-118833` の室内での通常ホバリング確認は指示受領時に停止した。

採用ログ：`native/logs/20260930-171114-119726/`。

- UASの同じ `darpa_cave_01.sdf` を直接使用。Gazeboのworld名は `map`、モデル一覧に洞窟タイル・救助対象と `dragon` があり、`rmf_unipilot` は存在しない（`gazebo_models.txt`）。
- 自動離陸→HOVER（flight_state=5）を確認。約30秒の安定区間のCoG高度は1.49933～1.50006 m。検証完了後もホバリングを継続する既定動作を確認した。
- 地図19,245点、飛行中の地図更新8回、MIMOSA odometry 514件。最大出力間隔0.1秒。
- 初期SE(3)整列・真値時刻補間による離陸以降の位置RMSE 0.00565 m、最大誤差0.01155 m。飛行制御は真値であり、この値をSLAM閉ループ飛行の結果とは扱わない。
- `sensor_validation.json`：IMU 1 kHz、LiDAR/MIMOSA odometry 10 Hz、重複・逆順時刻0、LiDAR/IMU各1台、11,520点/scan、検査中のflight_state=5、passed=true。
- MIMOSA購読先に真値odometryなし。`sim_parameters_actual.yaml` に真値noise/drift=0を記録。
- `slam_map.ply`・`slam_map_xyz.npy`・`trajectories.npz`・`aligned_trajectory.csv`・`flight_slam_result.json` を保存。`dragon_cave_map.png` で洞窟形状の点群を目視確認、`flight_slam.png` に軌跡と誤差を描画。
- ワールド出典とSHA-256を起動時に `world.json` へ記録する処理も追加。この検証実行には起動後に同じ出典情報を記録した。
- 洞窟実行にGazebo GUIを接続して描画初期化と `dragon` へのカメラ移動サービス成功を確認。最終GUI検査では通常launchと同じ `GZ_GUI_PLUGIN_PATH` を指定し、60 fps制限pluginもロードされた（`gui_cave_check_final.log`）。初回の単独GUI検査で同変数を省略した際のplugin未検出は検査手順側の問題で、通常launchには設定済み。
- 最終ビルド、launch引数表示、Python構文、aerial_robot差分検査に成功。保存結果の離陸・HOVER・地図更新・約30秒保持・センサ検査成功をassertで再確認。検証プロセスは停止し、supervisorのstatusがnullであることを確認した。

## 2026-09-30: UniPilotの3カメラROVIO＋MIMOSA

ユーザー指示：DRAGONは変更せず、UniPilotに3台のカメラを用意し、すべてをROVIOへ入力してMIMOSAへ外部odometryとして融合。UASの洞窟でシミュレーション検証する。

論文のUniPilot搭載カメラ数（3台）と、ROVIO融合評価で全3台を使用したかは別であることを説明済み。今回の3台全使用はユーザー指定の統合構成であり、ARLの論文構成の厳密な再現とは主張しない。

- `slam_ros1_ws/src/rovio` にETH ROVIOとsubmodule lightweight_filtering、`src/kindr` に依存を導入。revisionは `native/rovio-source-commits.json`。
- ROVIOのフィルタは `ROVIO_NCAM=3` で構築。ROS subscriberは元コードがcam0/cam1のみだったため、全カメラへ一般化した。同一stampの全画像を小さなバッファで揃えてから更新し、不完全な組・古い組を捨てる。stampの書換えはしない。
- Ubuntu22.04のlog4cxx対応でC++17、OpenCV4対応で色変換定数更新、system yaml-cppを使用。ビルドは `native/build_rovio.sh`。端末のvenv自動有効化なし。
- `native/prepare_vio_model.py`：前方・左・右の3カメラを派生UniPilotモデルへ追加。20 Hz、720×540（IMX296の1440×1080に対する1/2解像度）。前方118度のpinhole、左右185度のequidistant。Gazebo wideangleはL8非対応のためRGB描画し、ROVIO入力時に全カメラをmono8へ変換。
- 実機の完全な3カメラ校正は公開設定に揃っていないため、シミュレーションの仮定配置から内部・外部校正を一貫して生成。`camera_manifest.json` に出典上の制約と配置を記録。生成calibrationを実機用と扱わない。
- `native/rovio_unipilot.launch` は3画像・IMUのtopicを引数化。ROVIOの補正用pose/odometry/twist入力は未接続名へremapし、真値を入力しない。
- `mode:=vio_slam` を既存UniPilot launch/supervisorへ追加。Gazebo→ROS2→ROS1の画像bridge、ROVIO、MIMOSA外部odometry入力、自動飛行・評価を起動する。
- `native/mimosa_vio.yaml`：外部odometry enabled、固定sigmaは並進0.1 m・回転2度、D-opt閾値1。MIMOSAのLiDAR/IMU構成も維持。
- `native/vio_flight.py`：全3カメラ由来のROVIO特徴点と初期化を待ち、真値制御で離陸・2 m四方の移動・着陸。VIO単体とMIMOSA融合推定の両方を真値に対して評価。
- `native/verify_vio.py`：観測専用のROS1プローブ。3画像の画素分散・解像度と、ROVIO特徴点のcamIdを記録し画像も保存。

初期試行でGazeboのcutoff角上限、wideangleのL8非対応、画像topic指定を修正。`20260930-182139-139047` で全3台の画像とROVIO特徴点camId 0/1/2、外部odometry受信を確認。側方画像に機体フレームが大きく写り込むため、最終配置は光学中心の高さを0.18 mへ変更して再検証する。


### 最終構成と検証結果

- カメラの光学中心を高さ0.18 mに変更。ROVIO上流は180度超の投影を扱えず、185度画像の直接入力で座標変換エラーが出たため、`native/vio_images.py` で校正に基づく透視投影への整流を追加。生画像720×540は保持し、ROVIOには全3台のmono8・480×360画像を入力する（前方118度、左右120度）。画像stampは保持。全185度の側方視野をROVIOが使用するわけではない。
- ROVIO起動時の内部微分テストは `run_self_test` parameterで明示的に有効化する方式へ変更。通常起動で実行しない。上流差分は `native/rovio-three-camera.patch` と `native/rovio-lwf-cxx17.patch` に保存。
- `20260930-182621-141732` は整流追加前に停止した参考試行。採用ログは **`native/logs/20260930-182909-143250/`**。
- UASの洞窟で自動離陸→2 m四方を移動→着陸まで完了、`failure=null`。飛行制御はGazebo真値であり、VIOによる閉ループ制御ではない。
- 全3台の画像受信数は1980/1979/1979。ROVIOの有効特徴点camId 0/1/2を確認。ROVIO出力1937件、最大間隔0.05秒。独立プローブで整流後3画像の同一stampを69組確認、中央値間隔0.05秒（`rectified_sync.json`）。
- 初期SE(3)整列と真値時刻補間による位置RMSE：ROVIO **0.18099 m**（最大0.24121 m）、MIMOSA融合 **0.01846 m**（最大0.03449 m）。MIMOSA最大出力間隔0.1秒、地図33,505点、飛行中の地図更新57回。
- `rovio_connections.txt` でIMU＋整流後の3画像の実接続を確認。ROVIOの補正用pose/odometry/twistに接続なし。`mimosa_connections.txt` で `/rovio/odometry` の入力接続を確認。MIMOSAのD-opt診断も記録。
- 最終ROVIOログに座標変換エラーなし。IMU初期化前の画像1組を破棄する起動時警告のみ。RViz起動・3画像の表示設定を確認。
- 保存物：`probe.json`、生画像3枚、整流画像3枚、`three_cameras.png`、`vio_validation.png`、`vio_aligned.csv`、`vio_trajectory.npy`、既存の地図・MIMOSA軌跡・`flight_slam_result.json`。
- ROS2 bringup再ビルド成功。保存結果の離陸・3台の特徴点・地図生成・出力間隔・推定誤差をassertで検査して成功。

起動：`ros2 launch multilink_uas_bringup unipilot.launch.py mode:=vio_slam rviz:=true`。Gazebo GUIも開く場合は `headless:=false` を追加。

制約：実機の3カメラ校正・同期・露光特性を再現した検証ではなく、仮定配置によるシミュレーション検証。実機使用には校正と同期確認が必要。DRAGONの実装は変更していない。

- 最終Python構文検査とROVIO／lightweight_filteringの `git diff --check` に成功。整流後3画像で洞窟が見えることを目視確認。検証プロセスを停止し、supervisorのstatusがnullであることを確認した。


## 2026-10-01: Git管理の開始

- ユーザーが初期化した `src/multilink_autonomy` のリポジトリに、ここまでの実装を役割別にコミットする。対象はこのリポジトリ内。外部リポジトリへのコミット・pushは行わない。
- `.gitignore` にROS/CMakeのビルド結果、Pythonキャッシュ、ログ、supervisorの状態・lock、コンパイル済み `check_normals`、移行記録・バックアップ、旧環境の参照コピー、WindowsのZone.Identifierを追加。ローカルの実体は削除しない。
- ワークスペース直下にあった現行READMEをリポジトリへ取り込み、コマンドの基準ディレクトリとGit管理範囲を明記。
- 分割：除外設定／UAS環境構築・既存ツール／UniPilot LiDAR・IMU地図生成／DRAGONホバリング・地図生成／3カメラROVIO／共通起動処理・文書。
- コミット前に対象ファイル一覧と除外対象を確認する。機能コードは変更しないため、シミュレーションは再実行せず、Gitの差分・追跡対象・最終状態を検査する。

- 検査結果：追跡対象69ファイルにバイナリ・ログ・キャッシュ・旧参照コピーなし。代表11パスの除外を `git check-ignore` で確認。外部patchは文脈行の空白を保持するため `.gitattributes` で空白検査の対象外とした。既存launchとrosinstallの不要な末尾空白だけ整理し、機能は変更していない。全コミット対象の `git diff --cached --check` が成功。
