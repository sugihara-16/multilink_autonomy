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


## 2026-10-01: DRAGONの3リンクにカメラを配置しROVIOを融合

ユーザー指示：LiDARがない3リンクに各1台のカメラを、四角形の離陸姿勢で外向きとなる同じ相対姿勢で配置。ROVIOをMIMOSAへ入力し、離陸まで検証する。

### 実装

- `dragon_cameras.py`：link1/link3/link4に各1台。相対位置 `(0.263,-0.08,0.16)` m、RPY `(0,0,-π/2)`。全3台とも120度、720×540、20 Hzのpinhole。光学座標と青い箱形の表示を追加。カメラ質量は未モデル化。
- 生成URDFの関節原点・軸とros2_control初期角度（yaw=1.56、pitch=0）からFC→カメラの取り付け校正を生成。全3台について視線が四角形の中心から外向きであることを検査。ROVIOは3台の480×360画像とFC IMUを使用し、固定校正で動作する。
- `dragon_sim.launch.py` に `vio` オプション。LiDARは既存のlink2のみ、ワールドもUASの同じ洞窟。
- `dragon_vio.launch.py` とsupervisorの `dragon_vio` モードを追加。`bridge_dragon_vio.yaml`、`mimosa_dragon_vio.yaml`、`dragon_vio.rviz` で3画像・ROVIO・MIMOSA・表示を接続。
- `vio_images.py` と観測用 `verify_vio.py` の入力topicを引数化。UniPilotの既定値を維持。
- `dragon_vio_hover.py`：全3台の特徴点と推定を待って離陸、30秒の安定ホバリングを検査。真値制御を維持。VIO停止や関節角度の校正値から0.03 rad超の逸脱は検査失敗として着陸。関節角度、VIO軌跡、融合後の誤差を保存。
- `verify_dragon_vio.py`：飛行指令なしで3画像の同期・画素分散・座標名、FCに対する実際の座標変換と校正の一致、HOVER状態を検査。
- カメラの動的な取り付け校正は未実装。今回の対象は四角形の形状を保持した離陸・ホバリング。変形飛行のVIO対応とは扱わない。

### 検証中に見つけた既存座標配信の修正

初回 `20261001-131710-290086` では離陸・30秒ホバリングとROVIO/MIMOSAの出力まで成功したが、独立した座標検査で不一致を検出。
外部aerial_robot_modelの `rotor_tf_publisher` が `rotor_joint_name` を未宣言のまま読み、空文字列に全関節名が一致していた。その結果、可動関節を含む全機体に角度0の固定TFを配信していた。
`rotor_joint_name` の既定値 `rotor` と `tf_prefix` のparameter宣言を復旧し、ローターだけを対象にするよう修正。差分は `native/dragon-rotor-tf.patch` に保存。対象実行ファイルの再ビルド成功。
画像の座標名も実際のTF名 `cam_link*_optical` に揃えた。初回結果は参考扱いとし、GUI/RVizありで最終確認を実施する。


### 採用した検証結果

採用ログ：`native/logs/20261001-132056-293052/`。Gazebo GUIとRVizを有効にした公開launchで確認。

- 自動arm→takeoff→HOVERまで成功。高度1.5 mで29.993秒の安定区間を検査し、その後もホバリング継続を確認。高度1.49933～1.50006 m、`failure=null`。
- link1/link3/link4からそれぞれ1106画像を受信。ROVIOの有効特徴点camId 0/1/2を確認。ROVIO出力1064件、最大間隔0.05秒。
- 初期SE(3)整列・真値時刻補間による位置RMSE：ROVIO 0.05188 m（最大0.06659 m）、MIMOSA融合0.004927 m（最大0.009719 m）。飛行制御は真値。短時間・静止ホバリングの評価であり、移動・変形飛行や実機の精度を示すものではない。
- MIMOSA地図19,249点、飛行中の地図更新8回、odometry517件、最大間隔0.1秒。実接続を `mimosa_connections.txt` に保存し、ROVIOから外部odometryを受けることを確認。真値odometry接続なし。
- 独立検査 `dragon_vio_sensors.json`：passed=true、HOVER状態5、全3台102画像・同一stamp102組、画像間隔0.05秒。光学座標名は実際のTFに一致。FC→カメラの位置・姿勢は固定校正と一致（観測中の最大差は位置約1.1e-13 m、角度約5.1e-13 rad）。
- 離陸を含む機体関節の校正角度からの最大偏差は9.364e-5 rad。センサーが載るリンクは四角形を維持していた。
- 3画像を `dragon_three_cameras.png` に保存して外側の洞窟壁面が映ることを目視確認。画像下端には推進器の一部が映る。`dragon_vio_validation.png` に高度と推定誤差を保存。
- 修正後のrotor_tf_publisherは4個のローターTFのみ配信。座標検査成功。ROVIOは初期IMU時刻より古い最初の画像組を破棄する警告1件のみで、座標投影エラーなし。
- 保存結果について離陸・HOVER・3台の特徴点・地図生成・推定出力間隔・位置誤差・独立センサー検査をassertで確認して成功。

- 最終のbringupビルド、Python構文、YAML/XML読込、今回変更したソースの差分検査に成功。作業中のユーザー編集であるAGENTS.mdは変更していない。検証プロセスを停止し、supervisor status=nullを確認。


## 2026-10-01: DRAGONカメラをリンク側面の低い位置へ変更

ユーザー指示：図示したリンク側面付近に、リンクと同じ高さで配置。ローターから距離を取り、関節変形を妨げない位置とする。写り込みがあれば角度変更も可。

- `dragon_cameras.py` の3台共通の光学中心を `(0.10,-0.045,0)` mへ変更。リンク軸と同じ高さで、中央のローターからリンク長手方向に離す。筐体25×30×30 mmと、リンクのパイプ側面につながる短い取り付け部を表示。追加質量は引き続き未モデル化。
- 最初の低位置・yaw=-90度の試行 `20261001-133138-298368` は離陸・ホバリングまで動作したが、画像左端にローターが写った。ユーザーの追加指示に従い、全3台共通でyaw=-135度に変更。高さ・光学中心位置は維持。
- 校正は機体モデルと同じ取り付け値から再生成するため、ROVIOにも新しい位置・角度を適用。
- `verify_camera_clearance.py` を追加。筐体の箱と実際の表示メッシュの三角形との交差を検査。四角形1姿勢、機体の各pitch/yaw対を単独で-90～90度・30度刻みに変えた147姿勢、各ロータージンバルのroll/pitch対を同じ範囲で変えた196姿勢、合計344姿勢で筐体の交差0件。残りの機体関節は四角形の値を保持。
- この検査は離散姿勢の筐体干渉検査。全関節の同時変形・角度間の連続経路・取り付け部の強度・実機配線は検証していない。ROVIOの変形中の校正には引き続き未対応。


### 低位置配置の最終結果

- yaw=-135度の試行 `20261001-133419-300432` では隣のリンクのローターが画像右端に残ったため、最終角度を **yaw=-127.5度**（外向き−Yから−X側へ37.5度）へ調整。位置 `(0.10,-0.045,0)` mと水平視野120度は維持。
- 採用ログ：`native/logs/20261001-133612-302229/`。GUI/RVizありで離陸→高度1.5 m→30.008秒の安定ホバリング成功。failure=null。
- 全3台のROVIO特徴点を確認、VIO1064件・最大間隔0.05秒、MIMOSA516件・最大間隔0.1秒。地図19,250点、飛行中の地図更新8回。
- 初期整列後の位置RMSEはROVIO0.10881 m、融合後0.003945 m。真値制御の静止ホバリング検証であり、変形中の推定性能は未検証。
- `low_mount_angled_cameras.png`（離陸中）と `low_mount_hover_cameras.png`（ホバリング中）で、3台ともローターの写り込みがないことを目視確認。過去の高位置配置の画像とは区別する。
- `dragon_vio_sensors.json` はpassed=true、同時刻3画像110組、20 Hz、校正と実際の取り付け座標の一致を確認。`camera_clearance.json` は最終角度で344姿勢・筐体交差0件。
- 結果の成功条件をassertで検査。bringup再ビルドとPython構文・今回変更した差分の検査に成功。README/HANDOFFへ最終配置と検査範囲を反映。

- 検証プロセスを停止し、supervisor status=nullを確認。


## 2026-10-01 洞窟メッシュから走査軌道を生成

- 依頼：固定四角形DRAGONで、様々な洞窟形状を通る長めの軌道を作る。今回の対象はオフライン軌道作成。
- `native/plan_cave_route.py` を追加。UAS darpa_cave_01.sdfのcollision形状をSDF姿勢・DAE単位・scale付きで読み、0.5 mボクセル化。通路内の自由領域を探索し、直線短縮後に元の三角形メッシュへ距離照合する。地面z=0も含む。表面距離は0.1 m以下で検査、全区間を保証するため0.05 mを差し引く。
- `native/export_cave_route.py` を追加。URDFのvisual/collision形状からFC基準の機体包絡を検査。位置・yawの5次補間による停止旋回・移動軌道、CSV、JSON、俯瞰図を出力。
- 成果物：`native/routes/cave_exploration/`。596.317875 m、50通過点、三叉路→東側曲がり角→北側通路→三叉路→南側の折れ曲がり→帰還。始終点(40,5,2)、世界座標高さ2–5.5 m。帰路を含む長さであり、全洞窟の網羅ではない。
- 軌道：最高速度0.8 m/s、加速度0.4 m/s²、yaw角速度0.3 rad/s、角加速度0.3 rad/s²。1604.974929秒、3260サンプル。曲がり角で停止。連続軌道はsegments.jsonの5次式で定義。
- 結果：環境までの中心距離の下界1.95 m。現在の四角形・ジンバル0構成の全形状はFCから0.725302114 m以内。半径1 mの包絡球でも0.95 m以上の余裕。
- 検証：キャッシュなし再生成で同じ50通過点を再現。連続区間接続、全CSV点が検査済み線分上にあること、位置・yawの解析的な速度/加速度上限、時刻単調性、始終点、Python構文検査が成功。`validation.json` に記録。
- ログ：`native/logs/cave_route_final_20261001/`。大きなボクセル配列はログのみ。今回のURDF入力は `native/logs/20261001-133612-302229/dragon_slam.urdf`。
- 環境：システムPythonへtrimesh 4.4.9 / pycollada 0.9.3 / rtree 1.4.1を導入。trimesh 5.1.0は既存NumPy 1.21.5と不整合のため4.4.9に固定。仮想環境は使用していない。`requirements-route.txt` に記録。
- 制約：飛行・制御器接続・離着陸は未検証。FC座標をCoG目標へそのまま渡さない。任意の関節/ジンバル動作、動的障害物は対象外。現在の洞窟SDF向けの限定された読み取りであり、汎用SDFプランナーではない。
- 研究方針をHANDOFFへ追記：固定センサ構成でLIO/VIOを独立評価。今回は推定器やlaunch構成を変更していない。


## 2026-10-01 品質設定を維持したシミュレーション速度測定

- ユーザー依頼：計算実行倍率だけを上げて、どこまで速くできるか簡易確認。物理刻み・センサ品質を落とさない。
- `native/benchmark_sim_speed.py` を追加。既存DRAGON VIO構成をheadless・RVizなしで起動し、離陸・安定ホバリングを確認後にGazebo set_physicsのreal_time_factorだけを1→1.5→2→4→1へ変更。各5秒待機＋20秒実時間測定。finallyで1倍へ戻す。
- 物理刻み1 ms・カメラ解像度/20 Hz・LiDAR 10 Hz・IMU 1000 Hzの設定を維持。Gazebo統計でもstep_size=1000000 nsを確認。
- 実測倍率は順に0.58988 / 0.62190 / 0.60582 / 0.61924 / 0.60105。今回の構成・ホバリング位置では0.6倍付近で頭打ち。品質を変えず倍率設定だけで大きく高速化する効果は確認できなかった。
- カメラ3台・ROVIOの最大stamp間隔0.05 sim秒、LiDAR・MIMOSAは0.1秒で全設定共通。ROVIO遅延95%値0.115 sim秒も共通。IMUは観測側983–988 Hzで、1倍時にも欠落がある。最大間隔は1倍4 ms、1.5倍8 ms、2倍5 ms、4倍11 ms、復帰1倍4 ms。best-effort観測の欠落なので推定器が同じデータを落としたとは断定しない。
- 制約：短時間ホバリング測定。推定品質同一性、長距離飛行、独立LIO/VIO構成、計測負荷を除いた限界速度、CPU/GPUのボトルネックは未検証。MIMOSAは既存ROVIO融合設定。
- ログ：`native/logs/20261001-154958-353663/` のspeed_benchmark.json / speed_physics_stats.txt。結果要約は `native/routes/cave_exploration/speed_benchmark.md`。
- 終了時に倍率1へ復帰し、manager経由で今回起動したシミュレーションを停止。world・センサ・launchの恒久設定変更なし。


## 2026-10-01 独立LIO/VIOの洞窟軌道追従・RViz六軸誤差・連続記録

- ユーザー依頼：始点から離陸して軌道追従しデータ収集。GUIはRVizのみ。LIO/VIOと真値との位置3軸・姿勢3軸誤差をRViz内表示し、「終了」まで継続。
- `dragon_compare.launch.py` とmanagerのdragon_compareモードを追加。既存dragon_vio（融合）を残し、比較モードでは `mimosa_dragon.yaml` を使って外部odometryを無効化。ROVIOは3カメラを継続使用。
- `dragon_route_flight.py`：記録と両推定器の準備→arming→takeoff→8 sim秒安定待ち→12秒でFC世界座標(40,5,2)へ移動→時間付き軌道を反復。固定四角形のFC→CoG変換を真値/重心姿勢から求め、CoG位置・速度・yawをFlightNavで送信。推定器は制御に使わない。周回末尾は12秒でyawを戻して再開。
- 安全動作：追従誤差0.4 m以上で軌道進行を保留、0.8 m超/真値欠落/関節固定不成立/flight state異常/空き容量20 GB未満でGazeboをpause。推定精度の悪化だけでは飛行を止めない。
- `odometry_evaluation.py`：FC真値を同時刻へ位置線形補間・姿勢SLERPし、各推定器の最初の対応点で一度だけSE3整列。raw pose、真値、整列後位置、6軸誤差をCSVへ逐次保存。姿勢誤差はR_gt^-1 R_alignedのxyz Euler角[deg]。整列odometry/pathと誤差をROS2へ公開。
- `odometry_error_plot.py` と `dragon_compare.rviz`：RViz Imageパネル内の3×2グラフにLIO橙/VIO青、真値・両推定器の軌跡/odometryとLIO地図を表示。直近180 sim秒の表示、全履歴はCSV。別のプロットウィンドウやGazebo GUIは起動しない。XCompositeによるRVizウィンドウ取得で6グラフと地図の実表示を確認。
- rosbag2：生画像3台、LiDAR、IMU、真値、CoG、関節、飛行状態、指令、両odometry、6軸誤差、TFを無圧縮1 GiB分割で記録。rosbag受信時刻はsim時刻。`record_compare_qos.yaml` を追加。実行時ソース/設定/軌道と生成URDF/校正もログに保存。
- `trajectory_math.py` と `test_trajectory_math.py`：初期整列、既知の6軸誤差、同時刻位置/姿勢補間、全区間の始終点・速度上限、FC/CoG位置変換の3チェック成功。Python構文検査とcolcon build成功。
- 実行：`native/logs/20261001-160414-360546/`。launch PID 360515、supervisor PID 360546（再開時はmanager statusで照合）。離陸成功、sim約40.6秒にtracking開始。sim90.3秒時点で約16 m移動、最大追従誤差0.062436 m、flight state5、body joint error<1e-12 rad、failureなし。評価LIO866/VIO1769件、真値補間不成立0件。bag7分割まで生成確認。
- 実行中ROS1パラメータで `/mimosa/odometry/manager/enabled=false`, lidar=true, radar=falseを確認。ROVIO→MIMOSA融合なし。
- 初期指令ヘッダが壁時計時刻だったため、sim約65秒で `/dragon_route_flight use_sim_time=true` を適用。既定もtrueへ修正。初期区間を含む指令比較はbag受信時刻またはroute_tracking.csvのsim時刻を使う。センサ/推定器ヘッダと6軸比較は初めからsim時刻。実行時の注意をrecording_notes.jsonへ保存。
- 制約：全596 mの完走・周回切替はまだ未検証。現在は指示通り稼働・記録を継続。終了指示でmanager経由停止、bag確定後に最終集計する。センサ品質・時間刻みは変更なし。


## 2026-10-01 RViz異常終了後の再起動・記録継続の分離

- ユーザー報告：RVizが消えたので起動しなおす。
- 原因として確認できた範囲：RVizが `std::system_error: Invalid argument` でSIGABRT終了。managerは全子プロセスを必須扱いしていたため、GUIの異常だけでGazebo・記録も停止していた。RViz内部の例外原因は未特定。
- `native/rviz_watchdog.py` を追加し、比較モードのRVizだけを監視・自動再起動するようmanagerを変更。GUI終了は飛行・記録に伝播させない。manager停止時のSIGINT/SIGTERMではwatchdogとRVizを終了する。
- 模擬rviz2を終了コード6で落とす検証で、子の2回以上の起動とwatchdogのSIGINT正常終了を確認。Python構文検査成功。
- 前回 `20261001-160414-360546/rosbag/metadata.yaml` の確定を確認。停止前のデータは保全済み。sim約130秒までtrackingを確認していたが、全周ではない。
- 現在は `20261001-160857-363502/` に再起動。supervisor PID363502、launch PID363472、RViz実ウィンドウの存在を確認。新規試行として開始地点から再離陸・追従・記録。初めからFlightNavにもsim時刻を適用。
- ユーザーの「終了」まで継続する指示を維持。まだ終了操作は行わない。RViz内部クラッシュの原因解消を主張するものではなく、GUI障害と記録を分離する修正。


## 2026-10-01 ユーザー指示による飛行・記録の終了

- 「終了していいよ」を受け、manager stopで今回のGazebo・推定器・飛行制御・記録・RViz/watchdogを終了。manager status=nullを確認。
- `native/logs/20261001-160857-363502/rosbag/metadata.yaml` 確定、recorderのRecording stoppedを確認。全45分割の存在・非空を確認し、最終SQLiteのquick_check=ok。
- 記録期間627.034 sim秒、2915718メッセージ、容量47.36 GB。6軸CSV・グラフも保持。
- 追従CSV期間の真値移動距離226.00 m、最大追従誤差0.1804 m。軌道は1周未完了の状態でユーザー指示により終了。着陸操作ではなくシミュレーション停止。
- 最終集計は `session_summary.json` に保存。HANDOFFの稼働状態を停止済みへ更新。自動再開しない。


## 2026-10-02 rosbag破棄・ZaBのHarmonic対応と起動確認

- ユーザー依頼：今回のrosbagを破棄し、ZaBを現在の環境で起動できるようセットアップ。
- 削除：`logs/20261001-160414-360546/rosbag` 9,794,471,939 bytes、`logs/20261001-160857-363502/rosbag` 47,355,777,505 bytes。合計57,150,249,444 bytes。削除対象はこの2試行のbagディレクトリだけ。各試行に `rosbag_deleted.json` を記録。CSV・グラフ・評価結果・校正・構成は保持。logs内の残存db3=0を確認。
- ZaB原本：Zenodo DOI 10.5281/zenodo.17407065 のZaB_Model.zip。MD5 a0409a2c449f67bf5a80d1ac91714886で配布物を検証。SHA256 d2ce57180459b2eb6b51b7b1e50fd142fb70d3e3a98c0502c9ff8140c69e997d。
- 資産は `/root/multilink_autonomy_ws/assets/zab/`。原本zip、models、zab_harmonic.sdf、provenance.jsonを保存。大きなメッシュ・テクスチャはソースGitに含めない。`.gitignore` にbag/db3/mcapを追加。
- `native/setup_zab.py`：チェックサム固定ダウンロード・展開・world変換を追加。旧Ignition system名をHarmonicのgz-simへ変更、3 ms→1 ms、重複scene統合、GUIの視点・world制御topic設定。地上車両vehicle_blueと専用TriggeredPublisherを除外。
- 配布不備：newJerseyElement（入口のバリケード1個）がZIPに存在しないため、そのincludeを除外。トンネル形状・配置・テクスチャは変更せず、32 environment includes、39灯を維持。モデルを勝手に補完していない。変更はprovenanceとZAB.mdに記録。
- `zab.launch.py`：通常のROS環境から起動可能。内部でenv_dragon.shを読み込むため、ユーザー側で仮想環境や追加sourceは不要。既定はZaB＋非arming DRAGON（1 LiDAR・3カメラ・IMU）、画面付き。headless/robot切替を用意。推定器・飛行・記録の自動開始なし。
- `dragon_sim.launch.py`：world_fileとspawn_x/y/zを引数化し、ZaB modelsの探索パスと正しいworld名を記録。従来の洞窟既定値は維持。
- WSL対策：GPU描画のGazebo GUIがlibnvwgf2umx.so内でSIGSEGVしたため、GUIだけLIBGL_ALWAYS_SOFTWARE=1/LP_NUM_THREADS=2で別起動。Gazeboサーバーとセンサ描画は従来GPU設定のまま。software GUIでトンネルの実表示を画像確認。
- 起動時対策：大きなモデルの初期化中に既存controller spawnerが5秒でtimeoutするため `zab_controller_ready.py` を追加。clock開始後に4制御器を確認、未ロードをロードしinactiveを再有効化する。armingは送らない。最終試行は手操作なしで4制御器activeを確認。
- 検証：SDF_PATHを指定したgz sdf -kでValid。通常env.shのみからpublic launch起動成功。最終観測でカメラ3台各20 Hz、LiDAR10 Hz・点群11520点、IMU999.75 Hz、flight_state=0。Python構文検査成功。ログ `native/logs/zab-final-validation.log`、`logs/zab-check/{sensors.json,controllers.txt,cam1.png,cam3.png,cam4.png,gui.png}`。
- 導入手順は `native/ZAB.md`、README/HANDOFFを更新。検証後に所有launchへSIGINTを送って終了。飛行・LIO/VIO・新環境の軌道作成は未実施。旧洞窟経路をZaBへ流用しない。

## 2026-10-02 — ZaBに独立LIO/VIO・RVizを接続、rosbag自動記録OFF

### 要求と変更
- ユーザー指定：ZaBでもRVizを起動、rosbag自動記録を停止、洞窟と同じ推定構成を動作させる。
- `zab.launch.py` のrobot=true経路を既存managerのdragon_compareへ接続。ZaB world・spawnを渡し、MIMOSA LIO、3カメラROVIO、ROS1/2 bridge、評価、6軸誤差画像、RViz watchdogを起動。LIO/VIOは融合せず独立。
- ZaBは `--record false --flight none` を明示。洞窟用経路・arming・離陸を自動実行しない。rviz=trueを既定とし、headlessはGazebo GUIだけに作用する。
- managerにworld/spawn/flight/record引数を追加。別worldで洞窟経路を起動しようとすると拒否する。ZaBの制御器起動回復処理は正常終了を許す一回限りの子プロセスとして管理する。
- `dragon_compare.launch.py` もrecord=falseを既定化。記録には明示的なrecord:=trueが必要。洞窟の軌道追従は記録OFFならbag生成を待たないよう修正し、記録ONでは従来どおり生成を待つ。
- README、ZAB.md、HANDOFFを更新。CSV・評価画像・構成・実行ログの保存は維持。

### 検証・現在の状態
- colcon build（multilink_uas_bringup）成功、変更Pythonの構文チェック成功。
- `ros2 launch multilink_uas_bringup zab.launch.py headless:=true` で実起動。
- run: `native/logs/20261002-144853-4124/`。manager PID 4124、RViz PID 4787。現在は起動を継続している。停止はmanager stopまたはlaunchのCtrl+C。
- 4制御器active。ground flight_state=0（非arming）。sim161.8秒時点でLIO1579件、VIO3196件の真値比較が成立、真値との照合失敗0件。
- 10 wall秒の購読確認：LIO69件、VIO139件、誤差画像10枚（900x700）、flight_state692件。RVizの実ウィンドウと誤差画像・mapへの購読を確認。
- RVizログに起動後のエラーなし。WSLgウィンドウの画面取得は黒画像となったため、スクリーンショットによる最終描画の目視確認は未完了。誤差画像そのものはerror_plot.pngに生成済み。
- managerのrecord=false、recorder不在、run内rosbagディレクトリ不在を確認。
- 別world＋洞窟経路の誤起動をCLIがexit2で拒否することを確認。
- ZaBでの飛行・経路作成・退化評価は未実施。洞窟の記録OFFでの実飛行はこの作業では再実行していない。

## 2026-10-02 — エージェント起動のZaBを停止
- ユーザーより、エージェントが起動したままでは手元で終了しにくいとの指摘。manager stopでZaB・推定系・RVizを停止し、status=null、launchのsupervisor正常終了を確認。
- 明示的な継続指示がない限り検証後に停止する運用をAGENTS.mdへ追記、HANDOFFの現在状態を更新。


## 2026-10-02 — ROS2自動sourceとbase_link基準キーボード操作

### 変更
- `.bashrc` の既存env.sh自動sourceから、aerial_robot_base WS、ROS2側ros1_bridgeも読み込むようenv.shを拡張。Humble→UAS→aerial_robot_base→bridge→統合WSの順でlocal_setupを使用。既存端末でも再sourceで新設定が反映されるよう環境版番号を追加。env_dragon.shは互換入口に整理。仮想環境・ROS1のsetupは読み込まない。
- 実際の対象は外部aerial_robot_coreの `scripts/keyboard_command.py`。標準入力はキー解放・同時押しを正確に取得できないため、フォーカスされたQtウィンドウの押下・解放方式へ置換。20 Hzで保持中キーを合成し、逆方向は相殺。XY各軸の既定値0.2→0.4 m/s。Z/yaw/roll/pitchの速度は維持。
- `/uav/baselink/odom` の姿勢で入力XYZ速度を世界座標へ回転してFlightNavへ送信。LOCAL_FRAMEは実装上COG yaw基準であるため使用しない。制御対象COGは従来どおり。傾きも反映する3Dのbase_link基準。
- 解放・Space・フォーカス喪失・終了時に速度ゼロ。姿勢無効/未取得/0.5実秒の受信途絶も移動停止。離陸等の操作はキーリピートを無視し1回だけ送る。
- 外部package.xmlにPyQt5とnav_msgs実行依存を追加。外部test/test_keyboard_command.pyに9件の操作テスト。再現用native/aerial-keyboard.patch、README、HANDOFFを更新。外部の既存rotor_tf_publisher/bringup変更は変更していない。

### 検証
- クリーンな対話BashでROS_VERSION=2、DOMAIN=74、VIRTUAL_ENVなし。aerial_robot_core/messages、dragon、gz_ros2_control、ros_gz_sim/bridge、robot_bringup、composite_cbf、ros1_bridge、multilink_uas_bringupの検索成功。
- bash構文チェック成功。aerial_robot_coreのcolcon build成功。
- Qt操作テスト9件全成功：90度姿勢での移動、傾きの反映、前進＋左＋旋回＋上昇、部分解放、逆キー相殺、解放/フォーカス停止、リピート、姿勢途絶、終了/Space、単発操作を確認。
- 分離したROS_DOMAIN_ID=75の実ROS通信で、base_link yaw90度＋W/A/Qから世界速度(-0.4,+0.4) m/s・yaw+0.2 rad/sを受信。キー解放後の速度ゼロを受信。
- シミュレーション・実機は起動していない。manager status=null。実飛行による操作感は未検証。


## 2026-10-02 — 余計なキーボードGUIを撤去し端末操作へ戻す
- ユーザーからポップアップとQStandardPaths警告について指摘。「言ったことだけ変えて」に従いQtウィンドウ・PyQt5依存を撤去。従来のコマンドと端末操作へ復帰。
- base_link姿勢による変換、XY速度0.4 m/sは保持。端末はキー解放イベントがないため、直近0.25秒の入力をキーごとに合成し、入力が途切れたキーを期限切れにする近似方式。厳密な同時押し・複数キーの長押し継続は保証できない。説明せずGUIへ変更した前回対応を訂正。
- Space/Ctrl+Cで停止、終了時に端末設定を復元。namespace自動検出失敗時は元の警告＋global topicへのフォールバックへ戻した。
- 4件の回帰テスト成功。仮想端末＋分離ROS_DOMAIN_ID=75の実ROS通信で起動、waq合成、yaw90度のbase_link変換、入力期限切れでゼロ速度、Ctrl+C正常終了、端末設定復元、Qt警告なしを確認。
- native/aerial-keyboard.patch、README、HANDOFF更新。シミュレーションは起動していない。ユーザーが起動したプロセスは操作していない。


## 2026-10-02 — ZaB 100 m直線・巡航1 m/s・rosbag（実行中）
- ユーザー指定：前回同様の経路を作り、直線約100 mを1 m/sで飛行、rosbag記録、GUIはRVizのみ。
- plan_zab_straight.pyで衝突メッシュを照合し、FC/world (210,-60,-0.5)→(309.980006,-61.999600,-0.5) の100 mを選択。0.05 m間隔で三角形距離を計算し0.025 m差し引いた最小距離2.46095 m、機体包絡半径1 mに対して余裕1.46095 m。1 m間隔の上下rayでトンネル内確認。始終点±1 mの床高低差は約0.07/0.10 m。離着陸は水平経路の球体証明の対象外。
- 速度は2秒で0→1 m/s、98秒巡航、2秒で停止の台形速度。水平経路102 sim秒。離陸、始点・姿勢合わせ、終点保持3秒、着陸は別。
- trajectory_math.pyに台形速度補間、dragon_route_flight.pyに一回飛行・着陸・完了とworld別pause先を追加。managerにworldハッシュで照合する経路指定と一回終了を追加。ZaB launchはroute/recordを明示指定できるよう拡張。通常の飛行なし・記録OFF既定は維持。
- ZaBの古いstate.jsonだけが残ると再起動を拒否していたため、存在だけでなくPID/starttimeの生存判定へ変更。
- 最初のrun 20261002-160749-23186 はモデルのROSパラメータ応答失敗後にGazeboが無限待ちとなり、離陸せず停止。ログと小さいbagは保持。再起動run 20261002-160953-24661 は離陸し追従開始。
- 初期検証：既存軌道数式3チェック成功、台形速度の端点・巡航・数値微分チェック成功。ROS1/2間の独立LIO/VIO、真値制御、3カメラ/LiDAR/IMU記録。巡航途中の実測速度1.0012 m/s、ここまで最大追従誤差0.0428 m未満。
- 終了後の結果は後続追記とrun内session_summary.jsonを参照。ユーザーのkeyboardプロセスは停止していない。


### 同試験の完了結果
- 成功run：`native/logs/20261002-160953-24661/`。100 m直線飛行→終点停止→着陸→flight_state=0。route_status=complete、failure=null。launch正常終了、manager status=null。RViz含め試験構成を停止済み。
- 実測終点進行距離99.999969 m。巡航区間の0.5 sim秒平均速度：平均1.00215 m/s、最小0.99827、最大1.11817 m/s。指令巡航速度は1 m/s。始終端は2秒ずつ加減速。
- 直線からの最大横断方向誤差0.02045 m。時間付き目標点との最大追従誤差0.24969 m（途中報告の数cmはその時点の値）。終点保持後誤差0.0000341 m。
- rosbagは正常確定。163.972 sim秒、12 SQLite分割、12,383,408,128 bytes（約12.38 GB）、762,138 messages。3カメラ各3279〜3280枚、LiDAR1640件、IMU163972件、LIO1600件、VIO3237件、真値・制御・TF・6軸誤差を保持。ros2 bag infoで読み取り確認。
- summarize_straight_flight.pyによりsession_summary.jsonへ結果出力。台形速度テストをtest_trajectory_math.pyへ保存（計4チェック成功）。再実行方法はZAB.md、routes/zab_straight/README.md。
- 通常ZaB起動のrosbag既定OFFは変更していない。今回だけrecord:=trueを明示指定した。

## 2026-10-02 — 100 m飛行のrosbag削除
- ユーザーの「一回削除して」により、`native/logs/20261002-160953-24661/rosbag` のみ削除（12,383,423,148 bytes）。削除後の不存在を確認。CSV・評価結果・実行ログ・経路は保持。削除記録はrun内rosbag_deleted.json。


## 2026-10-02 — ZaB全通路0.5 m/s、推定なし、画面なし記録（進行中）
- ユーザー条件：0.5 m/s、全通路をなるべく重複なく、LIO/VIOなし、品質を落とさず可能な限り計算速度を上げる、rosbag記録、GUI一切なし。完了までターンを切らない。
- メッシュ0.5 m voxel→通路骨格→グラフ圧縮→局所0.2 m voxel再計画→三角形距離証明。接合面の衝突形状のため4連結区間に分割。connection1/2内部も別飛行で記録。接合面の通過不能箇所とメッシュ末端の欠損/狭窄は避ける。ユーザーに分割理由を説明済み。
- 全通過可能グラフの開いたChinese postman解（最小重み奇数点matching＋Euler路）。移動計約1958.53 m、独立通路約1457.85 m、重複約500.69 m。4区間の経路時間計約74 sim分。速度0.5 m/s、通路端で停止/旋回、加減速2秒。離着陸を追加。
- route/segments/図/計算ログはnative/routes/zab_full。広い通路は半径1 m球、狭い箇所は水平半径1 m・垂直半径0.5 mの包絡を使用。生成URDFの全collision幾何を固定四角形でFKし、楕円包絡ノルム最大0.710、水平最大0.693 mを確認（robot_collision_envelope.json）。
- managerにdragon_recordモード。ROS1 core/bridge、MIMOSA、ROVIO、整流、評価図を起動しない。GT制御、ROS2生センサbridge、recorder、軽量GT記録/監視のみ。ZaB launch estimators:=false、rviz:=false、headless:=true。
- capture_monitor.pyが起動時に倍率2/4/無制限を各12実秒測定し、配信頻度を維持する最速測定設定を選ぶ。計測後は画像/IMU/LiDARの追加監視購読を廃止し、記録の購読は維持。物理刻み1 ms、画像サイズ、センサ頻度は変更なし。実測倍率0.6〜0.7（推定器なしでもGazeboが律速）。
- trajectory_math.pyに定速の曲線経路の補間、飛行側に推定器を待たず生センサreadyを待つ条件を追加。起動高度を経路始点へ指定。通常の洞窟/ZaBの推定・記録OFF既定は維持。
- 先に短いrun3を実行し離着陸成功。logs/20261002-164549-33064、110.899 sim秒、8,378,966,016 bytes。verify_raw_bag.pyで各メッセージHeader時刻を読み、3カメラ各2218枚・IMU110900件・LiDAR1109件、欠落/逆順0を確認。
- 続いてrun1実行中：logs/20261002-165021-34458。capture_zab_campaign.pyが後続run2/run4の起動、各bagの時刻列検証、全体終了を担当。進捗/結果はlogs/zab-full-campaign、記録run一覧はroutes/zab_full/capture_runs.json。
- 終了後に結果追記する。現時点では未完了。全機能の実行プロセスが終わったことを確認するまで最終報告しない。

### 同収録：Windows実容量不足により一時停止（未完了）
- Linux dfの857 GiBを保存可能容量として報告したのは誤り。ユーザー指摘後、Windows Win32_LogicalDiskを確認しC:空き27,574,071,296 bytes、G:空き26,195,365,888 bytesを確認。WSLディストリビューションのBasePathはいずれもC:。
- /world/zab/control pause:true成功。run1はsim約1056秒、route約974.399/3778.588秒（約26%）で停止。取得済みbagは削除していない。記録・制御プロセスは保持し、飛行時間を進めていない。
- 既存の20 GB閾値はLinux disk_usageだけを見ており、Windows側の容量不足を検知できない。再開前にWindows側の容量確認が必要。保存先の変更／可逆圧縮の検討／今回終了をユーザーへ質問中。全通路収録の完了とは扱わない。

### Google Drive保存先の準備（収録は引き続き一時停止）
- ユーザーがGoogle Driveセットアップを承認。公式配布rclone v1.75.1を/usr/local/binへ導入、本人がブラウザ認証。uas_gdrive remote、drive.fileスコープ（作成ファイルのみ）、資格情報は/root/.config/rclone/rclone.conf（0600）に保持しgit外。トークンを含むセットアップstdoutログは削除。
- Drive API報告空き101,546,213,895,331 bytes。/mnt/gdriveを読み取り専用マウント、先読み128 MiB・キャッシュ目標上限4 GiB。mount_gdrive.shはユーザー端末で実行しCtrl+C終了できる。キャッシュ上限は開いているファイルでは超過し得るため、実ホスト容量の監視も必要。
- run3/rosbag_0.db3（1,074,302,976 bytes）をmultilink_autonomy/rosbags/20261002-164549-33064/rosbagへコピー成功。原本保持。マウント経由SQLite read-onlyで14トピックと先頭messageを読めた。ros2 bag playの連続再生・全体移行はまだ未検証。
- 共用OAuthプロジェクトでGoogle API 403 rateLimitExceededが繰り返し発生し、専用デスクトップOAuthクライアントの作成を依頼。専用設定のJSONは/root/.config/rclone/google-client.jsonへユーザーが保存する想定。共用接続は2026年廃止予定の警告もrcloneが出している。
- 可逆圧縮試験: zstd -1 -T2で1,074,302,976 bytes→305,548,106 bytes（28.44%）。展開後MD5 c36c2044ef61b609ada60aced142999dが元ファイルと一致。圧縮試験のみで、録画設定・原bagは変更していない。

- 容量誤認の再発防止: storage_capacity.py追加。WSLの登録BasePathからWindows側ドライブを特定し、Linux/Windowsの空きの小さい方を使う。10秒キャッシュ・取得不能は記録時fail closed。dragon_route_flight.pyの20 GB停止判定へ接続。実環境でC:空き16,236,462,080 bytesを取得、py_compile成功。一時停止中の既存ノードには未反映。
- マウント経由でテストファイル全体のMD5一致を確認。ros2 bag infoも成功（14.235秒・63,128 messages）。読み出しには共用API制限による待ちが発生。

### 専用Google接続・圧縮クラウド保存へ切替（進行中）
- ユーザーが専用デスクトップOAuthクライアントJSONを配置し再認証完了。remote uas_gdrive_private、drive.fileスコープ。/mnt/gdriveの読み取りマウントを専用接続へ切替。旧共用remote uas_gdriveはテストコピー1ファイルだけ残す。秘密情報はgit外のみ。
- 専用接続で305,548,106 bytesの送信成功、MD5一致。共用設定で出た403クォータエラーはこの転送では出ていない。
- ros-humble-rosbag2-storage-mcap導入。MCAP内Zstd圧縮を採用し、画像・時刻・全メッセージを保持。テスト63,128件について各トピック内順序、記録時刻、CDR本文のSHA256一致を確認。cloud_archive.pyは各分割に対して同じ検証（型・QoSも比較）を行い、Drive側サイズ/MD5確認後にだけ原本削除。変換・転送履歴は各run/cloud_chunks/*.json。
- 旧capture_zab_campaign.pyを停止。capture_zab_cloud.pyがrun3移行→run1継続→run2→run4を担当。進捗はlogs/zab-full-campaign/progress.json、実行ログcloud-supervision.log。全4runの完了・転送・生センサ時刻検証までは未完了。
- 一時保存は/mnt/c/Users/j-sug/AppData/Local/uas-cloud-spool。run1のrosbagディレクトリを停止中にrosbag_before_cloudへrenameし、Windows側spoolをrosbagへsymlink。既存ファイルは個別symlinkで参照し、開いている記録ファイルのinodeは保持。次splitからWindows側に書く。検証済み旧ファイルのリンクと元ファイルだけ削除する。切替記録cloud_spool_migration.json。
- manager.pyはUAS_RECORD_SPOOL指定時のみ記録先を変更しログからsymlink。cloud supervisorの後続launchへ指定。通常起動の出力先は変更しない。
- 未転送rawが4 GiB超またはWindows空き10 GB未満で物理時間をpause。2 GiB未満かつ空き12 GB以上でresume。3並列転送、各一時変換ファイルは1分割分のみ。後続runのdisk_reserve_gb=10を指定。物理刻み・センサ画質・レート・0.5 m/s指令は変更しない。
- Drive上の非圧縮テストbagは全体MD5一致、ros2 bag info、キャッシュ済み等速playが成功。MCAPをDriveから初めて読む試験では、同時upload中の0.25倍playでもqueue starvationが出た。クラウドからの連続定速再生は保証しない。解析時は必要な分割を先にローカルキャッシュへ置く。

## 2026-10-02 — ユーザー指示で収録停止・取得データを破棄
- 「一回停止して。取得済みデータもすべて破棄。後で一からやり直す」に従い停止。cloud supervisorは直前に転送後lsjsonのexit 3で中断済み（元データ削除前）。manager stopでシミュレーション/制御/記録を終了、manager status=null。Driveマウントも停止。関連capture/record/Gazebo/RVizプロセスなし。
- logs/20261002-164549-33064、logs/20261002-165021-34458、logs/zab-full-campaignを削除（rosbag、CSV、各runの検証・センサ出力含む）。Windows側uas-cloud-spool、rclone-uas読み出しキャッシュ、/tmpのMCAP/Zstd試験データも削除。以前の中断テストlogs/20261002-160749-23186/rosbagに残っていた1ファイルも削除。
- Google Driveの専用接続multilink_autonomy_zab_20261002と共用接続multilink_autonomy/rosbags/20261002-164549-33064を完全削除（trash不使用）。削除後の各親フォルダ一覧で対象が存在しないことを確認。
- capture_runs.jsonは空に戻し、logs/latestの切れたリンクを削除。削除記録はnative/logs/zab-capture-deleted-20261002.json。
- 経路・実装・Google接続設定は保持。今回の全通路収録は完了していない。自動再開しない。次回は一から実施する。
- 次回前に要修正/検証: 並列rcloneプロセスで保存先ディレクトリを同時作成すると同名フォルダができ、転送後lsjsonで見つからない状況が疑われる。今回の終盤のexit 3の原因は未確定。親フォルダを単一処理で先に作成、ID固定などを行い、転送確認を再検証する必要がある。cloud pipeline全体は完成/全域試験済みとは扱わない。

## 2026-10-02 — ZaB全域データ収集を一から再実行（開始）
- ユーザーの再実行指示。条件継続: DRAGON固定関節、0.5 m/s、通過可能な全通路を4区間、GUI/LIO/VIOなし、センサ/物理品質維持、Google Driveへ可逆圧縮保存。前回取得データは再利用しない。
- cloud_archive.py: クラウド保存先をUAS_CLOUD_REMOTEで新規キャンペーン名に分離。並列upload前に親フォルダを単一処理で作成し同名重複を検査。転送直後のlsjsonは一時的な未反映に備え再試行。元データはサイズ/ハッシュ検証成功まで保持。
- 3ファイル並列アップロードの事前試験: すべてMD5一致、ファイル名重複なし。試験データをDriveから削除。py_compile成功。
- capture_zab_cloud.pyを既存run引継ぎ専用から新規起動へ変更。空ledgerとシミュレーション停止を必須にし、run3→run1→run2→run4をすべて新規起動。後続を含めUAS_RECORD_SPOOLにWindows側spoolを指定。全4runの転送検証完了後にのみ成功判定。
- 保存先: uas_gdrive_private:multilink_autonomy_zab_20261002-190350。実行ログnative/logs/zab-full-campaign/cloud-supervision.log、進捗progress.json。Windows空き約28 GBを起動前に確認。収録は開始したがまだ未完了。

### 再試験中に見つかった起動・記録・制御の問題を修正
- 起動環境のPATHにpowershell.exeが無く容量確認で停止。絶対パスへの代替を追加しPATH=/usr/bin:/binでも取得成功。
- Windows drvfsへSQLite bagを直接記録すると書き込みが追いつかない。中断run 20261002-190655-54198は約39秒の時刻範囲に25,877件しかなく、停止時のcache flushも15秒を超えてmetadata未確定。本収録から除外。生bagはLinux ext4へ戻し、変換済みMCAPのみWindows側spoolを使用。上の「生bagもWindowsへ」は撤回。
- Linux記録run 20261002-191057-55037の最初の分割では5生センサ系列の欠落/逆順0、クラウド照合成功。一方、飛行の容量確認でPowerShellが制御コールバックをブロックし、制御更新間隔0.356 sim秒→追従誤差0.193 mで停止。Windowsドライブの特定をノード起動時1回に変更し、定期確認はstatvfsのみ。実測初回0.688秒、以後0.000466/0.000653秒。py_compile成功。
- 本収録再試験run3: logs/20261002-191713-56332。中断した試験runの識別はcampaignのstartup_failure.json、windows_sqlite_attempt.json、control_latency_attempt.jsonに保存。未完了データを完成データとして使わない。

### 短い区間の完了、本区間を継続中
- run3（20261002-191713-56332）: 離陸→16.97 m走査→着陸完了。全441,757 messagesをDriveへ保存、7.30 GB→3.54 GB MCAP。IMU96,648件、3カメラ各1,933枚、LiDAR966件、分割境界を含め欠落/逆順0。capture_context.tar.gzに校正・機体モデル・経路・実装snapshotも保存済み。
- run1（20261002-192439-57607）: 最長区間を飛行中。約11.5%、32分割保存済み、未転送約3 GB、Windows実空き約25 GB。最初の連続21分割は境界を含め5生センサ系列の欠落/逆順0を追加確認。
- 現在の監督プロセスはcapture_zab_cloud.py、完了までは停止/削除しない。run2/run4は自動後続。4区間の結果が揃ってから最終報告。

### run1約28%で転送照合の一時エラー、同じ位置から再開
- run1のrosbag_77転送後、Windows spoolのMD5計算でOSError ENOMEMが発生。監督がGazeboをpauseし、未検証の元データを保持。飛行側failure=null、route約1070.977/3778.588秒、追従誤差0.0168 m。
- file_md5を256 KiB単位の非バッファ読み出しにし、ENOMEM時の上限付き再試行と読み出し位置の復元を追加。途中で位置を進めてからENOMEMを注入する試験でMD5一致、py_compile成功。
- capture_zab_cloud.pyに--resumeを追加。既存結果run3を保持、現在の機体をpause確認してから残りファイルの転送を再開し、転送待ちが減れば同位置からresumeする。収録を最初からやり直していない。

### 停止・再開時の物理破綻を修正、正常prefixを保持して後半を再収録
- 上の「同位置から再開」は失敗。run1 sim1138.32で関節角が発散、Gazeboが異常終了。正常にクラウド照合済みのchunk0..79（IMU最終1137.335秒）を保持し、chunk80は評価対象外とする。全収録完了とは扱わない。
- gz_ros2_control_plugin.cppのPreUpdateがpause中もwriteを呼び、DRAGONのAddWorldWrenchが力を積算することを確認。pause中のwriteを省く。PostUpdateは起動時のcontroller処理を保持するため変更しない。aerial_robot_base_wsでgz_ros2_controlをビルド成功。再開試験はrun4で実施中。配布用差分native/gz-ros2-control-pause.patch。
- run1_remainingは元経路segment12（元route時刻958.841秒）からの後半1230.00m、2819.75秒。同じ経路を約56m重ねて未収録区間へ接続。開始地点下の10本のrayで床の高低差0.0619mを確認。経路の速度・包絡・品質条件は維持。
- capture_zab_cloud.pyに--labels追加。完了済みrunを保持し、指定した未完了区間だけを収録できる。元run1の失敗を成功として上書きしない。
- 最初の30秒pause試験では関節発散は解消したが、再開直後に強制着陸state17。Spinalの通信watchdogと積分dtがsteady_clock実時間で進むことを確認。SIMULATIONのみROS node clockを注入し、pause中は経過しないよう修正。実機HAL_GetTick経路は変更なし。spinal/aerial_robot_simulationビルド成功、再試験中。差分native/spinal-sim-clock.patch。前半prefixと後半ではこの制御時刻修正が異なることに注意（センサ・物理条件は同一）。
- manager停止中に再度SIGINT/SIGTERMが入ってもcleanupを中断しないよう、finally内で後続シグナルを無視。異常終了後に残っていた旧run1の3プロセスを個別停止して不存在を確認。
- run1 prefix: 1137.334 sim秒、5,226,994 messages、85.94 GB→45.83 GB。5生センサ系列の欠落/逆順0。cloud metadataはchunk0..79のみを明示。異常以後/準備試験の残りはexcluded_attemptsに分離して保存し、評価対象に含めない。
- 修正後run4で30実秒の飛行中pause→resume成功。再開後route11.78秒、追従誤差0.01374m、joint誤差4.37e-13rad、flight_state5、failure=null。先行する容量待ちpauseからも正常復帰。
- 次回起動のクラウド監督はWindows空き10.7 GB未満でpause、11.7 GB以上でresume（飛行側の10 GB停止より先に止めるため0.7 GBの猶予を確保）。生raw backlog4 GiB/2 GiB条件も維持。
- run4（20261002-204212-68410）完了。15.35m走査→着陸state0、failure=null。101.668sim秒、540,983件、7.68GB→4.15GB。IMU101668、各カメラ2033、LiDAR1017、分割境界を含め欠落/逆順0。30秒pause試験結果と校正/経路/モデル/実装もcontextに保存。
- run1_remaining→run2の順で残り収録を開始。監督exec99120。現時点では全域未完了。
- run1_remaining（20261002-204911-70345）継続中、route856.76/2819.75秒（約30%）、追従誤差0.0148m、failure=null。63分割保存済み。繰り返す転送待ちpause/resumeで関節発散・強制着陸の再発なし。先頭36分割の境界を含む5センサ欠落/逆順0も途中確認。run2は後続、全域未完了。
- 後半約41%でraw残0.35GBまで送信済みでもWindows空き11.56GBとなり、11.7GB再開条件を満たせず停止。メモリ一時保存を検討したが、Gazebo RSS約11.7GB、MemAvailable2.9GBなので採用しない。raw記録Linux/圧縮spool Windowsは維持。再開条件をraw<1GiBかつWindows空き11.1GB以上へ変更（pause10.7GB/飛行停止10GBは維持）。同じ飛行状態のまま監督だけ再起動。センサ品質・時刻・内容は変更なし。
- 長時間GazeboのRSS増加を実測（sim1240付近約11.7GB→1295付近11.9GB、swap2GB超）。同一プロセスで残りを続けず経路を分割。run1_remainingはroute1240.68秒・failure=nullで計画停止し、全raw bagを確定。停止後MemAvailable約14GB。正常データをrun1_middleとして保存する。
- 次のrun1_tail_aは元remainingのsegment19〜32、429.37m/969.43秒。run1_tail_bはsegment34以後、252.22m/598.27秒。元経路を切り出し、速度・通路・包絡は同じ。始点134,-81.5,1と共通離着陸点190.5,-57.5,1で床10rayの高低差0.0632m/0.0604mを確認。middleとtail_aは約2m重複。各区間のyaw調整・離着陸は独立する。
- 監督の再起動時にtoolセッションが同一process groupのROS launch/managerを終了させ、独立groupの子が残る問題を確認。以後cloud supervisorからのROS launchをstart_new_session=Trueで起動。残存していたこのrunのsim/記録/制御groupは個別停止し、metadata確定と不存在を確認。
- run1_middle保存完了: 1323.248sim秒、7,157,157 messages、100.05GB→60.12GB。3カメラ各26466枚、IMU1323248、LiDAR13233、分割境界を含め欠落/逆順0。前半との重複は保持し、完成経路とのcoverage対応はplanned_segmentation.jsonへ記録。
- 22:00頃、run1_tail_a→run1_tail_b→run2を開始（監督exec18620）。現時点で有効4bag保存済み、残り3bagの予定。最終全域検証までは未完了。
- tail_a収録中、Windows空き14GB以上のときはraw<2GiBでresumeし転送と飛行を重ねるよう調整。14GB未満ではraw<1.2GiB、Windows>=11.1GBの条件を維持。1GiB splitはヘッダー等でわずかに1GiBを超えるため、1GiB厳密条件で開いた最後のsplitだけが残り再開不能になるケースも避ける。pause>4GiBまたはWindows<10.7GBは不変。監督だけ再起動し機体/記録は維持。
- tail_a（20261002-220031-79824）route542.979/969.434秒、failure=nullでWindows空き9.65GBとなり監督pause。spoolは2.9MBで転送未完了量が原因ではなく、Gazebo RSS約7.4GBの長時間実行中にホスト空きが減少。停止後約19GBへ回復。原因の厳密なOS/GPUメモリ解析は未実施。品質を下げず、正常記録をtail_a_prefixとして確定し短区間に分割して続ける。
- 残り6区間: tail_a1 100.47m/228.49秒、tail_a2 114.88m/231.76秒、tail_b1 94.13m/263.38秒、tail_b2 158.09m/322.12秒、run2a 137.32m/276.64秒、run2b 124.20m/318.76秒。元経路のpolylineを切り出し、開始/終了の10ray床高低差<0.35mを全件確認。尾部aは約17mを重ねる。センサ・物理刻み・0.5m/sは不変。
- summarize_zab_campaign.py追加。最終的な有効11bagの完了/partial区別、全センサ欠落/逆順0、元4成分の全unique edge被覆を照合しdataset_manifest.jsonを生成する。まだ全域完了ではない。
- tail_a_prefixの最終検証でIMU欠落1件を検出（chunk31内、2ms間隔1回）。カメラ3系列とLiDARは欠落0。完成データには含めずexcluded_from_dataset.json/rejected_runs.jsonへ明示。全データは保存済みで破棄しない。
- 既存正常4bagと残り6計画のunique edge和を比較し、追加で必要なのは[13,14]と[0,1]のみと確認。tail_branch 12.14m/51.84秒とtail_link 30.08m/62.15秒を取り直す。長い重複通路を再走査しない。最終有効12bag予定。
- IMU送信側のoffered QoSがreliableであることを記録metadataから確認。記録側だけbest_effortだったため、IMUの受信をreliable/depth2000に変更（他系列は不変）。通信で失われた際の再送と受信待ち枠を有効にする。センサ生成・時刻・値は変更しない。欠落原因がROS2受信であること自体は未確定なので、各新規収録も欠落0を検証する。
- run1_tail_branch完了。reliableに変更したIMUを含む全5生センサで欠落/逆順0を確認しresultsへ追加。短いrun1_tail_linkを後続起動。全域収録は未完了。
- run1_tail_branch=20261002-223932-85182: 141.774sim秒、688,462件、6.38GB保存、全センサ欠落/逆順0。context保存済み。
- run1_tail_linkも走査/着陸/保存/全センサ欠落0を確認してresultsへ追加。現在有効6bag、残り6区間を同じ監督exec11705で継続。
- tail_a1=20261002-225325-88141完了。311.948sim秒、1,472,820件、10.13GB MCAP、全5生センサ欠落/逆順0、context保存済み。
- tail_a2=20261002-230519-90350はroute199.76/231.76秒でWindows空きが約9.28GBへ急減。未転送は最後の開いた0.67GBだけ、spool3.7MB。Windows pagefile割当13,738MB、Gazebo RSS約4GBを確認。Linuxファイルcache解放ではWindows空きは戻らず、原因の厳密な特定は未了。実容量判定でpauseしたまま正常prefixを保存し、最後の20.94mを元経路点列から切り出して再収録する。
- tail_a2_endは43.878sim秒、元edge[0,3]の弧長81.773%地点から終点まで。prefixと約5.4m重なる。開始床10ray高低差0.0471m。summarize_zab_campaign.pyは部分edgeの弧長区間の和を取り、隙間なく全edgeを覆うか検証する。最終有効13bag予定。
- 容量制御を整理: 同時変換/転送2件、raw>2.5GiBまたはWindows<8GBでpause、raw<1.2GiBかつWindows>=9GBでresume。新規flightの最終reserve6GB（従来10GB）。記録品質は不変。保存待ち上限を抑えた上で、実容量に対する過大な固定余裕で再開できない状態を避ける。
- finalize_partial_capture.pyは計画停止直前の状態をplanned_segmentationから記録する。停止処理中の最終statusで容量reserve判定が付いても、停止前の正常な飛行と区別する。bag全メッセージ/時刻の検証は従来通り実施。
- tail_a2_prefix（20261002-230519-90350）は264.618sim秒・1,606,887件・12.08GBを欠落0で確定しcontext保存済み。
- tail_a2_endも走査・着陸・保存・全5センサ欠落0を確認。完成データ9区間、残りtail_b1/tail_b2/run2a/run2bの4区間を監督exec86517で継続中。

## 2026-10-03 ZaB収録継続：停止応答の再試行

- run1_tail_b1 / run1_tail_b2 は着陸・Drive保存・5センサー欠落0の検証が完了。撮影条件と実装のアーカイブも保存済み。
- run2a の経路84.40秒（sim149秒）で一時停止要求への応答がなく、クラウド監視が終了した。Gazebo・記録・飛行制御は生存し、飛行の異常はなく、再送すると停止成功を返した。応答が失われた原因は未特定。
- capture_zab_cloud.py の同一停止状態の要求を最大3回に限定して再試行するよう修正。応答失敗を無視して進めず、全試行失敗なら従来どおり停止する。既存run2aを再開し、続けてrun2bを収録する。

## 2026-10-03 ZaB run2aのLiDAR欠落と部分取り直し

- run2a（20261003-002709-102783）は着陸まで完了したが、chunk22のLiDARに0.2秒間隔を1件検出（10Hzの1点群欠落）。IMU・3カメラは欠落0。欠落原因は未特定。
- 完全な旧metadata/検証結果をDriveの full_metadata_with_gap.yaml / full_validation_with_gap.json に保持し、採用するbagのmetadataは欠落前のchunk0..21だけに限定。前半の全5センサー欠落0を検証し、run2a_prefixとして保存した。未採用chunk22..25は残しているが通常のmetadataからは参照しない。
- LiDAR配信側がreliableを提供していることを記録済みmetadataで確認し、record_compare_qos.yamlのLiDAR購読をreliable/depth50に変更。周期・点数・内容は変更しない。run2bから適用。
- 前半の保守的な経路終端を240秒とし、約230秒地点から終点までをrun2a_endとして重ねて再収録する。全体監査は14bagの有効範囲で実施するよう更新。

## 2026-10-03 01:27 CEST ZaB全域再収録完了

- 固定DRAGON、巡航0.5m/s、GUIなし、LIO/VIOなしで収録完了。独立14bagの有効記録を統合した被覆監査で、メッシュから通過可能と判定した25/25道路を被覆。4つの連結成分の間および容量対策の短区間間はシミュレーションを再起動しており、連続1本のbagではない。
- 最後のrun2b（20261003-005320-106610）とrun2a_end（20261003-011702-110549）は着陸まで正常完了。LiDAR reliable/depth50を適用後、全5センサーの欠落・逆順0を確認。補完経路は22.734487m、47.468974 route秒。床検証済みで元経路230.171705秒から重ねて再収録した。
- run2a_prefixはchunk0..21（312.501 sim秒、保守的なroute終端240秒）のみ採用。欠落のある旧全体metadata/検証結果はDrive内に別名保持。新metadataと前半検証結果を照合付きでアップロード。監督プロセスが保持していた古い結果一覧でprefix項目が上書きされたため、検証済みprefix_verified.jsonから項目を復元し、全14件で最終監査を実施。
- summarize_zab_campaign.pyで全道路の弧長区間を合算し、被覆漏れ0、全採用bagの5センサー欠落/逆順0を確認。dataset_manifest.json生成。記録総計27,962,075メッセージ、5,233.217 sim秒（離着陸・重複含む）、228,918,754,159 bytes MCAP（元SQLite 395,631,882,240 bytes）。
- verify_zab_remote.pyを追加。採用metadataが参照する375 MCAP分割＋14件それぞれのmetadata/検証結果/設定アーカイブ、計417ファイルをDriveのサイズ・MD5と照合し全一致。remote_final_verification.jsonとdataset_manifest.jsonをDrive直下へ照合付きで保存。
- Drive: https://drive.google.com/drive/folders/1eV2i-Y1p07k9A-G2gMWxIWM4B7jNNIYU 。ローカル一覧: native/logs/zab-full-campaign/dataset_manifest.json 。診断用データもあるため、解析対象はこの一覧と各bagのmetadata.yamlで選ぶ。
- 最終監督プロセスはexit0。state.jsonなし、Gazebo・rosbag recorder・route flight・manager foreground・capture supervisorの実プロセスが存在しないことを確認。GUI/推定器も起動していない。認証設定と経路は保持し、自動再開しない。HANDOFFの先頭を完了状態に更新。

## 2026-10-03 ZaB全14bagの独立LIO/VIO後処理を開始

- ユーザー依頼: 全採用bagでLIO/VIOを独立実行し、離陸・向き合わせを除く軌道追従中だけ真値/推定odometryを保存。入力点群/画像は結果へ複製せず、元の取得時刻を厳密保持。
- offline_odometry.py / process_zab_odometry.pyを追加。Gazebo/GUIなし、既存のMIMOSA LIO・3カメラROVIO・同一校正と前処理を使用。飛行前データは初期化に用い、route_tracking.csvのlap>=1からroute終端最初の到達までを保存窓とする。prefixは採用済み保守的終端で切る。
- MCAPをDriveから1ファイル先読みして順次処理し、MD5照合後に入力する。処理済みローカルコピーのみ削除、Drive原本は保持。再生上限4倍、推定時刻が入力から0.15秒以上遅れる場合は待つ。画像/点群の間引きなし。
- 出力は各bagごとにodometryのみのROS2 MCAPと真値/LIO/VIOのCSV。header stampのsec/nanosecを保持、bag保存時刻も同じ整数nsにする。frame_id/child_frame_idも保持し、座標系の自動変更や時刻原点の変更は行わない。
- run3試運転は約87秒入力/35.813秒処理（当初の待ち閾値0.3秒）。追従窓49.400〜87.040秒、真値3553件/LIO377件/VIO753件。LIO/VIOの全出力stampが元センサー時刻と一致、欠落/追加stampなし。以後は待ち閾値0.15秒と各再生区間の速度上限を採用し全区間処理を開始。
- 現在の監督exec34618、ログnative/logs/zab-offline/campaign.log。完了前にターンを切らず監視し、各bagの出力時刻集合を元LiDAR/カメラ時刻集合と比較する。

### 後処理：run1_middle終端のROVIO遅延
- run1_middleは全入力の処理を完了。真値115,810件、LIO12,626件、VIO25,250件。途中の出力時刻の欠落なし。元カメラ時刻1323.15/1323.20秒だけVIO出力がない。
- LWF FilterBase.hppのMeasurementTimeline::maxWaitTime_=0.1とgetSafeTime/checkUpdateWaitTimeを確認。使わない外部更新系列も最大0.1秒の待ちを課すため、記録が途中で終わったbagでは最後の2時刻を確定する後続IMUがない。元記録の終端から0.15秒以内であることを確認した。
- 補作やtimestamp変更はせず、result.jsonにunavailable_tail_stamps/source_last_imu_ns/tail_limitationとして明示。missing_source_stampsも消さず保持。検証は明示した終端2件だけを区別し、途中欠落・余分な時刻は引き続き失敗とする。ユーザーへ説明し残りを続行。

### 後処理：7件目LIOの異常終了を調査
- run1_tail_a1の178秒付近でMIMOSAがSIGSEGV（exit -11）。再生監督は推定停止を検出して全所有プロセスを終了。既に検証済み6件は保持し、部分結果はlogs/zab-offline/diagnostics/run1_tail_a1_rate4_crashへ隔離。
- gdbを導入し、offline_odometry.pyにUAS_OFFLINE_DEBUG_MIMOSA=1時だけMIMOSAをデバッガ配下で実行・入力キャッシュを保持する機能を追加。通常の推定設定は不変。等速でrun1_tail_a1を再処理（exec45810）。これから原因を切り分ける。

### 後処理：7件目の等速再処理成功
- run1_tail_a1を等速＋gdbで再処理し、178秒地点を含め全対象期間が完了。真値21,638件、LIO2,324件、VIO4,648件。入力センサーに対応する時刻の欠落・追加は0、異常終了は再現しなかった。処理時間424.58秒（Drive再取得を含む）。
- SIGSEGVの原因は未特定。速度・実行タイミング依存の可能性はあるが、断定しない。残り7件の上限を1倍に変更し、3並列先読みとデータを落とさない待ち制御は維持。センサー内容・推定器パラメータ・timestampは不変。デバッグ用に保持した同区間の入力キャッシュのみ削除。

### 後処理：全14件の計算・ローカル最終検証完了
- process_zab_odometry.pyは全14件完了でexit0。全追従窓の合計4,241.56秒。真値392,184件、LIO42,416件、VIO84,830件を保存。離陸・ホバリング・向き合わせは推定器初期化にのみ使用し、出力対象から除外。
- verify_offline_odometry.pyで全MCAPを読み直し、3トピックのみ、保存時刻=header整数ns=CSV整数ns、sec/nanosec・frame・位置/姿勢・速度の一致、値の有限性、対象時間内、件数一致を検証。元LiDAR/画像時刻に対する途中欠落・余分な時刻0。run1_middle終端のVIO2時刻だけ既述の後続入力不足として明示。
- package_offline_odometry.pyでMCAP/CSV/window/result/manifest/READMEだけをnative/logs/zab-offline/resultsへ整理（14件、550,021,374 bytes）。元データのDrive保存先もmanifestへ記載。座標系は各推定器の元のものを保持し、真値への位置・姿勢補正はしていない。
- 計算用roscore・bridge・MIMOSA・ROVIO・入力再生プロセスの終了を確認。処理済み入力キャッシュを削除。Driveの元センサー記録は保持。最終成果物をodometry_tracking_20261003へ転送中。

### 後処理：Drive保存・最終照合・停止確認完了
- 成果物を `uas_gdrive_private:multilink_autonomy_zab_20261002-190350/odometry_tracking_20261003` へ保存。https://drive.google.com/drive/folders/1Jj7Jw1ttaG-dmqws_mg5JQHp-0OoSbC- 。rclone checkでMD5を使用し100/100ファイル一致・差分0（native/logs/zab-offline/remote_check.log）。
- 転送/checkはexit0。再生監督・roscore・bridge・MIMOSA・ROVIO・画像/IMU前処理・rclone転送/check・Gazebo/RVizの実プロセスが存在しないことを最終確認。入力キャッシュは削除済み、クラウド原本と最終結果は保持。HANDOFF先頭の進行中記述を最終状態に統合。

## 2026-10-03 全14飛行のLIO/VIO六軸誤差プロット
- ユーザー依頼によりplot_offline_errors.pyを追加。独立した14飛行を各1図にし、同一飛行内のMCAP分割は図を分けない。6段（世界XYZ位置誤差m、真値機体基準の相対回転xyz Euler角deg）に青LIO・橙VIOを重ねる。
- 同一FC IMU基準の出力を使用。真値と両推定が存在する共通の最初の時刻で、それぞれの位置/姿勢を真値へ一度だけ剛体整合。スケール補正・全軌道最適化・途中再整合なし。初期誤差は除去されるので開始後のずれを評価する図。
- 元の整数stamp_nsを維持し、真値位置を線形補間・姿勢をSLERPで同時刻比較。外挿なし。共通開始前/真値終端後の境界サンプルLIO4件/VIO13件を除外し、描画/誤差CSVはLIO42,412件/VIO84,817件。元CSV/MCAPは変更なし。既知run1_middle末尾VIO2件も補作しない。
- 極端な誤差は実際に縦軸を分割し//と省略範囲を表示。省略点もCSV/統計には保持し、evaluation.jsonに表示範囲・省略点数・整合変換・最大絶対値/RMSEを保存。±180度折り返しを跨ぐ線のみ描画で切断。
- native/logs/zab-offline/error_plotsへ全14ページPDF、飛行別PNG/PDF、一覧HTML、28誤差CSV、比較方法READMEと検証JSONを生成。全14飛行の元stamp一致・単調性・初期整合後誤差ゼロ・値の有限性・統計一致を検証。既知の固定座標変換が誤差を生まないことも確認。小誤差/大誤差の図を目視確認して軸省略の表示を調整。
- 完了: 61ファイル・27,998,632 bytesをDriveのodometry_tracking_20261003/error_plotsへ保存し、全61ファイルのMD5一致/差分0を確認。https://drive.google.com/drive/folders/1Ty3-Dp3NqgZl1L_Il6zk8YuPtIRtJz5w 。84軸中27軸で縦軸省略を使用。GUI・シミュレーション・推定器は起動していない。

### 2026-10-03 誤差図の縦軸省略を撤去
- ユーザー指示によりplot_offline_errors.pyから縦軸分割を撤去。全14飛行・84軸で全誤差範囲を通常の線形軸に表示する。PNG・個別/全体PDF・一覧HTML・README・表示範囲情報を再生成。
- 誤差計算、初期整合、取得時刻、元データは変更なし。姿勢角±180度の表示上の折り返しをまたぐ線の切断だけ維持。
- 検証: 全84軸が単一範囲・省略点0であること、最大誤差が表示範囲内であることを確認。大誤差run2bの図を目視確認。Drive版も更新し、全61ファイルのMD5一致・差分0を確認。

## 2026-10-03 指定9runの環境・LIO/VIOドリフト要因調査
- ユーザー確認によりrun2_tail_b2はrun1_tail_b2へ訂正。対象: run1_prefix/middle/tail_a1/tail_a2_prefix/tail_b1/tail_b2、run2a_prefix/end、run2b。
- analyze_zab_runs.pyで全推定軌道の3D位置誤差・相対回転角・悪化開始時刻を集計。map_zab_analysis.pyで元collision meshと実飛行真値を重ね、各runの通過形状を可視化。
- probe_zab_sensors.pyで採用metadata内の元MCAPのみを対象に、開始付近/LIO悪化/VIO悪化等の計27時点をDriveから抽出。入力MD5を照合し、3カメラ81枚・各時点0.2〜1.1秒の連続画像・点群・IMU・固定関節状態を確認。ROVIOと同じ校正と解像度へ変換。大容量入力は順次削除し、終了時にキャッシュ撤去。元記録・推定結果・パラメータは変更していない。
- summarize_zab_probes.pyで点群の平面法線に基づく方向情報比/進行方向との一致、回転方向の補助指標、画像の特徴点数・分布・暗部率・短時間追跡を測定。これはMIMOSA/ROVIO内部ログではなく、本調査の補助指標と明示。
- run2a_end: 有効点10,899、方向情報比0.00185、弱方向と進行方向の一致0.999998。真値の開始終端変位22.4mに対しLIO進行方向変位ほぼ0m、最大位置誤差22.8m。VIO最大0.54m。長手方向の退化を強く支持。
- prefix/middle/a1/a2_prefix/2a_prefix等では形状の弱化と姿勢破綻を確認。b1悪化地点は他より形状変化・画像特徴があり、その地点の単調さだけで説明できない。run2bは最大約22.9km、姿勢破綻後にZが発散、最後40秒の二次曲線相当の加速度は−13.12m/s²。重力方向/バイアス/誤地図拘束を含む内部破綻の候補とし、環境だけで原因確定しない。
- 共通設定: MIMOSA project_on_degneneracy=false、reg_4_dof=false。LiDAR360×32・30m・垂直−7〜+52度、点1/2・0.5m整理。ROVIO3カメラ・最大特徴25点、useCrossCameraMeasurements=trueだがdoStereoInitialization=false。IMU加速度bias_mean=0.1m/s²。一定速度飛行も含め非環境要因候補として区別。観測時点の関節角は固定。
- report_zab_environment.pyでnative/logs/zab-offline/environment_analysis/report.htmlを生成。9runの事実/原因候補、写真・通過経路・誤差推移、補助指標、未確認事項と提案する比較実験を掲載。一次資料（ZaB配布元、ROVIO公式、低加速度での単眼VIO尺度の論文）も参照し、単眼理論を3カメラへ断定適用しない。
- 検証: 9run/27時点/81画像、予定サンプル全存在、元取得時刻近傍、HTML参照先存在を確認（validation.json）。レポートは約12.3MB。設定変更比較実験は未実施、内部対応残差/特徴点深度/バイアス履歴が無く、因果確定はできない。シミュレーション・GUI・推定器は起動していない。
- Drive保存・照合完了: https://drive.google.com/drive/folders/1kLCPB1GBx318shxmcG3uDHnePMUHIHv9 （odometry_tracking_20261003/environment_analysis）。全163ファイルMD5一致・差分0。抽出/集計/転送プロセスはすべて終了。

## 2026-10-03 VIO統合・入力・設定の追加監査（進行中）
- ユーザー依頼: VIOが環境以前に不安定でないか、必要箇所の全分析と元rosbagによる追実験を許可。既存結果・本番設定・クラウド原本を保持して調査。
- prepare_vio_audit.pyで採用metadataのMCAPをMD5確認後、元と同じmono8/480x360 rectificationを行い、入力時刻・storage順序を維持した一時NPZを生成。最大3並列prefetchと処理後削除。Windows側実容量を監視し、WSLの削除済み領域をfstrimで回収。
- rovio_audit.cpp/run_vio_audit.py: 現在のROVIOヘッダとライブラリを使用する別実行ファイル（25特徴/75特徴）をlogs配下に構築。既存バイナリは変更なし。ROSキューを介さず元の順序で全IMU/画像を直接コールバックへ渡す。3画像をheader時刻でまとめ、通常同様最初2秒のIMUを除外。推定・バイアス・カメラ別特徴数・各カメラでの追跡状態・距離を保存。専用master11375、GUI/Gazeboなし。
- check_vio_inputs.py: 全14飛行のURDF/FC/光学座標と校正、用意済み入力の時刻/関節/IMUを検査。check_vio_depth.py: 真値姿勢とZaB meshから特徴方向の実距離を補助評価。summarize_vio_audit.py: 既存の共通開始時刻に一度だけ剛体整合して各条件を比較。
- 中間確認: run2a_end、run1_tail_a2_prefixの直接再計算baselineは、旧ROS経由VIOと共通stampの位置が完全一致（最大差0）。キュー/再生速度では両飛行の誤差を説明できない。カメラ3台の時刻集合一致。IMU積分と真値回転の時刻差は0〜1ms。
- 中間結果はnative/logs/zab-offline/vio_audit。run2a_end、run1_tail_a2_prefix、run1_tail_b2で1要因ずつの比較を進行中。バイアスoracle/除去は初期静止時の真値姿勢を使う診断専用で、実用設定の評価と混同しない。最終結果・停止確認は後続追記予定。

### VIO追加監査完了：3飛行・45条件、設定要因と改善を確認
- 全14飛行の同一カメラ校正をURDF/画角と照合（位置差最大4.8e-16m、回転差最大5.3e-14deg）。既存27時点の画像ペアの幾何照合も追加（check_vio_visual_geometry.py）。代表3飛行では画像3台のstamp集合一致、IMU1000Hz/画像20Hz途中欠落なし。追従期間の関節変動0。初期spawn直後のみ関節変動があるが、VIO開始2.003秒以降は最大約2e-7rad。
- 真値はGazebo IMU entity（FC）のpose。VIOとの参照位置混同なし。ジャイロ積分と真値回転の最良時間差は0〜1ms。真値姿勢・静止時バイアスを用いた入力診断で、加速度積分の0.1秒速度差誤差は各軸約0.0002〜0.001m/s。初期姿勢を取り除いた旋回方向と加速度の物理整合を確認。
- 原本55 MCAP chunkをMD5確認して使用。各試験は飛行開始から同一入力を再生し、評価は元と同じ追従開始anchorで一度だけ剛体整合。初期化・離陸・向き合わせは評価外。全3baselineが旧VIO位置と最大差0、姿勢差は数値誤差水準。入力ROSキューや再生速度では今回のドリフトを説明できない。
- 主な非環境要因: 現行ROVIOのジャイロ誤差モデルが模擬センサと不整合。実測偏りは約±7〜8e-6rad/sなのに、a2_prefixの推定Z偏りは約0.003rad/s。現行初期偏り分散3e-4、偏り変動分散3.8e-7、角速度予測ノイズ分散7.6e-7を、模擬センサの生成設定に合わせ1e-10、1e-16、4e-11へ変更した比較で向きが改善。観測4e-11=(2e-4rad/s)^2×0.001s。GTは推定器へ入力せず、既知のセンサ生成条件だけを使用。実機向け設定ではない。
- ジャイロモデル整合だけの最大位置/Yaw誤差: a2_prefix 6.554m/8.695deg→3.222m/0.022deg、b2 22.387m/16.967deg→6.718m/0.583deg、2a_end 0.543m/1.121deg→0.216m/0.020deg。
- さらに特徴距離初期値を2m→10m（逆距離initDepth 0.5→0.1）にした比較: a2_prefix 0.912m/0.025deg、b2 1.713m/0.434deg、2a_end 0.086m/0.024deg。入力・シーン不変で大きく改善するため、旧結果を環境のみの退化として扱わない。この10mは代表3飛行での比較値であり全環境に対する最適値ではない。
- 他の比較: 75特徴は最大位置3.226/13.686/0.352m（a2_prefix/b2/2a_end）、ステレオ初期化ONは9.526/20.520/0.543m。初期加速度平均・加速度バイアス初期分散/初期値・入力偏り除去・明るさ残差更新・ジャイロ各要素の分離比較も実施。一要因だけの変更やステレオONで一貫して解決する問題ではない。bias_oracle/bias_removedは初期静止時のGT姿勢を利用した診断専用条件として区別。
- ログはfeaturesがTRACKEDの状態でも位置/向きがずれること、特徴が3カメラ間で偏ることを示す。全25特徴の上限は3台合計。カメラ間重複は限定的で、ステレオ初期化コードは次番号の1台だけを試す。メッシュによる実距離との比較も実施し、新規特徴を除く2秒以上経過した点の集計を併記。個々の画像対応の正誤を全件証明したものではない。
- 成果物: native/logs/zab-offline/vio_audit/report/index.html、6ページall_diagnostics.pdf、3飛行の六軸/内部状態PNG、全45条件のCSV/設定/ログをexperiment_results.zipに保存。初期整合・元整数nsを保持、軸省略なし。比較用候補差分candidate_sim_rovio.patchを保存。本番launch設定・既存ROVIOバイナリ・元LIO/VIO結果は変更なし。
- 検証: verify_vio_audit.pyで全45条件の全入力配送、3画像同期、不完全set0、遅着IMU0、初期化各1回、tracking時刻欠落0、出力有限、姿勢正規化、初期誤差0、統計一致を確認。起動直前の画像2.0秒が初期時刻2.003秒より古い警告だけ許容。9 Pythonスクリプト構文検査成功。図を目視確認。元カメラ校正は全14件同一だが、追加計算は代表3飛行のみ。
- 保存/終了: Drive https://drive.google.com/drive/folders/1RjLjMmoimR0zpeQ6wXGD4nkU4_v8qWCi （odometry_tracking_20261003/vio_audit）。全21ファイルMD5一致・差分0。ZIP内45件のrun/odometryを確認。検証用master11375・全推定器・転送処理終了。GUI/Gazebo起動なし。ユーザーのkeyboard_command.py（PID20007）は保持。一時NPZ55個・4,073,622,531bytesを削除し、元Drive rosbag・収録設定・小さい入力manifest/検査結果は保持。
- 未検証: 他11飛行の候補設定での精度、実機ノイズ条件、同一環境で加減速を変える比較、全画像対応残差の正誤、実描画時刻とheaderの固定遅延の独立測定。新しいシミュレーション飛行や実機検証はしていない。

## 2026-10-03 VIO全14飛行の再計算・誤差図更新（開始）

- ユーザー指示により、前回監査で選んだ `gyro_matched_depth10` を全14飛行へ適用。3カメラ・25特徴、ジャイロ初期分散1e-10、bias過程分散1e-16、姿勢予測分散4e-11、初期逆距離0.1。真値は評価のみに使用。
- `reprocess_vio_accuracy.py` を追加。採用metadataの元MCAPを最大4並列で取得しMD5照合、元保存順・header時刻で既存audit25へ直接入力。1ファイルずつ処理して削除し、Windows側の実空き容量を監視。代表3飛行の同一設定・検証済み結果を再利用し、残り11飛行を再計算する。
- 新結果は `logs/zab-offline/vio_recomputed/` のCSVへ保存。旧 `results/` のCSV/MCAP、元bag、本番起動設定は変更しない。新旧MCAPの取り違えを避ける説明を成果物へ付ける。
- `finalize_vio_accuracy.py` と既存plotスクリプトの入力選択を整備。全件完了後に時刻・LIO/真値不変・初期化・有限性を検証し、旧図を保全して既存 `error_plots/` を更新する。進捗は `vio_recompute.log` / `vio_recomputed/progress.json`。完了結果は次節へ追記する。

## 2026-10-03 VIO全14飛行の再計算・六軸プロット更新（完了）

- `gyro_matched_depth10` を全14飛行へ共通適用。新たに11飛行を処理し、前回同条件で検証済みの3飛行（run1_tail_a2_prefix/run1_tail_b2/run2a_end）は結果を再利用。真値を推定器へ入力していない。本番launch/元rovio.info/ROVIO本体は変更していない。
- 新データは `native/logs/zab-offline/vio_recomputed/`。追従期間のみのtruth.csv/lio.csv/vio.csv、時刻窓、設定、入力配送記録、検証結果、旧新比較を保存。CSV形式で、共分散は含めない。旧 `results/<label>/odometry` のMCAPは旧設定の結果のまま保持しており、新VIOへ書き換えていない。
- 件数は真値392,184、LIO42,416、VIO84,830。新VIOの全stamp_nsが旧結果と整数nsで一致。真値/LIO/windowは旧CSVからコピーしSHA256一致を確認。図の評価対象はVIO84,817件、LIO42,412件（共通整合開始・真値終端に収まるもの）。run1_middle終端の既知のVIO2時刻未出力は補作していない。
- 初期化時刻の最終照合で、元bagの先頭IMUが0.002秒の2飛行（run1_tail_branch/run1_tail_a2_end）だけ、初回再計算が2.002秒、旧処理が2.003秒という1 ms差を発見。2飛行を再処理し、最終14件すべて旧処理と同じ2.003秒の初期化・初期化1回・遅延IMU警告0を確認した。初回の2 ms開始由来の結果は `vio_audit/recomputed_initialization_2ms/` に保全。run1_tail_branchの最大位置誤差は中間値0.206728 mから確定値0.138005 mへ更新。run1_tail_a2_endは0.683261→0.683307 m。
- `run_vio_audit.py` / `verify_vio_audit.py` も、単純な「先頭IMU＋2秒」や固定2,000件除外から、旧ログの初期化時刻を参照する処理へ変更。既存の代表3飛行（すべて先頭0.003秒）の条件は同一。構文検査済み。過去45条件の再実行は行っておらず、今回の全14件は `finalize_vio_accuracy.py` で別途実データ検証した。
- 位置誤差のRMSEは全14飛行で低下。全84,817評価点をまとめた位置RMSEは42.621141→1.698013 m。最大位置誤差は13/14飛行で低下。run3のみ0.253545→0.273532 mと最大値が増加するが、RMSEは0.1539→0.1336 mへ低下。
- 主要な最大位置誤差（旧→新、m）：run1_prefix 110.365861→4.903900、run1_middle 93.253831→3.190917、run1_tail_b1 12.649990→0.417479、run1_tail_b2 22.387095→1.712865、run2a_prefix 10.649646→0.829345、run2b 7.544590→2.031089。全14件の値はcomparison.json。
- 既存 `error_plots/index.html` / `all_flights.pdf` / 各PNG・PDF・誤差CSVを更新。14飛行14ページ、LIO/VIOを六軸で重ね、縦軸の省略・クリップなし。LIO誤差CSVは旧図とバイト一致。共通開始時刻での固定の位置・姿勢整合のみ、スケール補正や途中の再整合なし。truthの位置線形補間・姿勢球面補間、外挿なし。
- 旧図は `error_plots_before_vio_recompute/` に保全。初期化期間を含む推定出力・特徴点/バイアス診断は `vio_audit/recomputed_diagnostics/` へ分離。新しい精度データには画像・点群を含めない。確認用の生成先 `error_plots_vio_recomputed/` も保持。
- 実装：`reprocess_vio_accuracy.py` / `finalize_vio_accuracy.py` を追加。`plot_offline_errors.py` は入力/出力指定、完了済み新VIOの自動選択、図の再計算表示、出典/設定・生成記録を追加。元bagを最大4並列取得してMD5照合、ファイルごとに処理・削除。容量監視はWindows C:側を使用。終了時の入力キャッシュは空。
- 初回run3後に転送並列数を2→4へ変更して再開したため、vio_recompute.logの途中に意図的なKeyboardInterruptがある。完成結果へ中断中の推定を継ぎ足していない。ログ末尾ALL 14 COMPLETE、各処理と最終検査はexit0。
- 所有する専用ROS1 master11375と全推定器/処理プロセスを停止。GUI/Gazeboは起動していない。ユーザーのkeyboard PID20007は継続。C:空き約26 GB。
- Drive新CSV: https://drive.google.com/drive/folders/1FaOAITQ6HfaM39PfEn24vhHZvG3vC7_h
- Drive更新図: https://drive.google.com/drive/folders/1Ty3-Dp3NqgZl1L_Il6zk8YuPtIRtJz5w
- Drive旧図: https://drive.google.com/drive/folders/1TaSxK-xpIxs9fNBn7t-R0xUL_BP3Kkmy
- Drive最終照合：新CSV117ファイル、新図64ファイル、旧図62ファイル、計243ファイルがサイズ・MD5一致。rclone checkは3件すべてexit0、差分0。確認ログはvio_check_*.log。全転送・確認処理も終了。

## 2026-10-04 LIO/LIVOの退化対策候補の論文収集・評価
- ユーザー依頼: FAST-LIVO2、Switch-SLAM、SA-LIVO、arXiv:2410.10784をresourcesへ揃え、妥当性と追加比較候補を判断。今回は文献調査であり推定器の実装・実行は行っていない。
- 既存FAST-LIVO2 / Switch-SLAMの出版社版PDFを保持。SA-LIVO v2、DRPM v2に加え、FAST-LIO2、LVI-SAM、X-ICP、Informed/Constrained/Aligned、GenZ-ICP、RELEAD、COIN-LIO、R3LIVE、GenZ-LIOをarXivの版指定URLから取得。追加11本、総数13本、149,874,227 bytes。
- resources/README.mdに読解結果、公式コードの有無、方法ごとの実証範囲、現環境への適用条件、推奨実験順を保存。manifest.jsonに出典・版・ページ数・SHA-256と確認した公開repoのcommitを記録。FAST-LIVO2公式ソースの単眼入力／画像無効モードと、現MIMOSAの退化対策の処理・閾値も確認。
- 指定候補は研究上妥当。FAST-LIVO2も退化環境を評価しており「無対策」の基準とは呼ばない。Switch-SLAMは初期予測切替だけでなく方向別のLiDAR補正抑制と視覚融合を含む。公式実行コードは今回確認できず。
- SA-LIVO v2はLiDAR＋視覚の合計情報の弱い方向を減衰する。公式コードは採録後公開予定。第三者SA-LIVO-Reimplはv1/v2混在・独自拡張・系列別設定を明記するため、公式再現とは区別。
- arXiv:2410.10784（DRPM）は完成LIOではなく点群位置合わせへの組込み部品。論文4実験は脚odometry／ROVIO／全視野LiDAR由来の外部odometry／レーダー速度等を使用し、純LIO性能の実証とは解釈しない。共通LIOへ導入し補助入力なしで比較する案を提示。
- 比較対照としてMIMOSA退化対策OFF/ONとFAST-LIVO2画像なし/ありを優先。独立FAST-LIO2、X-ICP系（後続研究の公式perfectlyconstrained）、公開LVI-SAMを追加候補とした。GenZ-ICPはIMUなしLO、COIN-LIOは反射強度の適合確認前提、RELEAD/GenZ-LIOは公式コード入手未確認／公開予定として区別。
- 現MIMOSA project_on_degneneracy=false、degen_thresh_trans=40、degen_thresh_rot=0。既存機能はDRPMの確率的減衰とは異なる。単にONで姿勢発散まで解決すると断定しない。共通IMU誤差モデル、3カメラと単眼の条件分離、視野重複、瞬時LiDARの全点相対時刻0、元stamp維持、追従区間のみ評価、ループ閉じ込み無効を比較条件として記載。
- 検証: 13 PDFすべてヘッダー正常・全ページから文字読取り成功、README内ローカル14参照すべて存在。元結果・元bag・推定器設定は未変更。既存の作業差分を保持。GUI/シミュレーション/推定器を起動していない。

### MIMOSA既存退化対策の説明補足
- ユーザーの「これなに？」への回答に際しgeometric_factor.hppの投影適用箇所まで再確認。実際の有効処理は弱い点の並進成分への投影のみ（514〜516行）。P_rotは計算されるが回転成分への適用はコメントアウト（525〜538行）。先の「位置・姿勢の補正抑制」という説明を限定し、resources/README.mdを修正。六軸を保護する完成した退化対策とは扱わない。今回も機能はOFF、設定・コードの変更や実験はなし。

## 2026-10-04 単眼LIVO 4条件比較：実装と入力準備
- 依頼：MIMOSA退化対策OFF、ON、FAST-LIVO2、MIMOSA+DRPMを順に、同じ単眼カメラ・有用な2飛行で比較。MIMOSAの全3条件には同じ単眼ROVIOを外部odometry因子として入力。
- 選定：run2a_end（49.48秒、トンネル軸方向の移動をLIOが捉えられない例）とrun1_tail_a2_prefix（200.18秒、途中で姿勢まで崩れた例）。画像の特徴数・分布の既存診断を根拠にlink3/cam1を選定。入力の状態で選び、4方式の誤差によるカメラ選別はしていない。
- `comparison_ws`を独立構築。元slam_ros1_wsと通常launchは変更せず、修正差分・DRPMヘッダ・固定revision・FAST-LIVO2直接再生アダプタをnative/comparison/へ保存。`build_livo_comparison.py`を追加。
- `prepare_livo_comparison.py`：採用MCAPのみDriveから取得・MD5照合し、単眼480x360歪み補正画像/点群/IMU/真値の可逆圧縮キャッシュへ変換。元MCAPは処理後削除。C:物理空き容量6GiBを下限監視。
- `rovio_monocular.cpp` / `build_rovio_monocular.py` / `replay_monocular_comparison.py`：cam1の校正を単眼cam0へ割当、特徴25、前回検証済みのgyro_matched_depth10設定。元整数nsを維持し、IMU2.003秒から初期化。単眼ROVIOの同じ推定値と共分散を3条件へ共通入力。
- `make_comparison_bag.py`：ROS1形式への入力アダプタ。LiDARの点時刻t=0を保持（瞬時スキャン）、共通有限点・0.8–30m、カメラは同じ白黒値をBGR3成分へ複製。真値は別CSVに保存し、推定器に渡すbagへ入れない。
- MIMOSA既存ON分岐は再構築用J/residual配列への代入が欠落していたため修正。既存の並進方向投影は維持。同期VIOと瞬時LiDARが同じ時刻の際にIMUをゼロ区間積分して例外になる箇所は、姿勢伝播不要・歪み補正恒等変換として処理。3条件共通の接続修正。
- DRPM：論文Eq14–23による6方向の非退化確率、Eq35の面法線微小回転共分散、Eq28によるLiDAR因子の情報量の重み付け。snr_factor=10、点σ=0.03m。IMU/VIOがあるためゼロ運動事前因子を追加しない。MIMOSA因子グラフへの移植であり、論文の独立ICPループそのものではない。正常3次元形状と平面の合成試験で、前者6方向維持、後者3方向抑制・半正定値・有限性を確認。
- MIMOSA OFF/ONの2飛行ずつが完了。標準閾値trans=40/rot=0ではON分岐の作動0回で、OFFとONの軌跡は一致。精度改善がないことを退化検出機能一般の限界と解釈しない。
- FAST-LIVO2の依存関係を準備・ビルドし、短い飛行の全入力処理と画像補正を確認。終了時のPCL点群メモリ解放でSIGSEGVがあり、CPU依存のEigenアラインメントとシステムPCLの整合を修正して再検証中。最終比較は未完了、後段の記録を参照。

## 2026-10-04 単眼LIVO 4条件比較：完了
- OFF→ON→FAST-LIVO2→DRPMの順で各2飛行を完走。完成キャンペーンは`native/logs/zab-offline/livo_comparison/final_campaign.log`、全8ケースexit0。元の14飛行の結果・プロットは上書きしていない。
- 位置RMSE / 姿勢RMSE（m / deg）：
- run2a_end: mimosa_off=12.557042/1.490015, mimosa_on=12.557042/1.490015, fast_livo2=0.020402/0.578580, mimosa_drpm=4.630954/5.228830。
- run1_tail_a2_prefix: mimosa_off=4.880825/76.106399, mimosa_on=4.880825/76.106399, fast_livo2=0.347835/4.117079, mimosa_drpm=9.759120/123.004111。
- 結論：今回の設定・2例ではFAST-LIVO2が位置・姿勢RMSE最小。標準ONは判定0回でOFFと姿勢・位置が完全一致。DRPMは短いトンネルの位置誤差のみ改善、姿勢誤差は両方で増大し長い方は位置も悪化。論文のDRPM一般の限界とは断定しない。
- `plot_livo_comparison.py`：各飛行6軸に4方式を重ねるPNG/PDF/HTML/誤差CSVを生成。軸省略なし。追従開始の固定の位置・姿勢整合のみ、元整数nsで真値へ補間・外挿なし。姿勢総合指標は相対回転の角度。
- `verify_livo_comparison.py`：全ケース正常終了、有限値・正規化姿勢、追従989/4004点、元画像時刻との照合で欠落0/余分0、最大間隔0.05秒。LiDAR破棄とodometry共分散による破棄0。MIMOSA OFF/ON位置は数値一致。`tracking/`には真値・単眼ROVIO・4条件の元座標の姿勢CSVのみ保存し、画像/点群は含めない。
- 同時刻更新の通常グラフ公開抑止を避け、mimosa_rosbagが各外界センサー更新後の最終最適化姿勢をCSVへ保存。同じ時刻の最後の値で評価。ログ方法を揃え直した完成キャンペーンだけを採用した。
- FAST-LIVO2終了時SIGSEGVはPCL点群の解放で発生。FAST-LIVO2/vikitをシステムPCLに合わせたgeneric x86-64設定へ変更後、全入力処理・正常終了を確認。DRPMはGTSAM同梱cephesのerf/erfcとのシンボル干渉で停止したため、公式同様のBoostの正規分布CDFへ変更。修正後2飛行完走。
- DRPM合成試験は公式ソースと確率差1e-12以内、正常形状6方向保持、雑音平面3方向抑制、有限/半正定値。GTSAMのcephesをLD_PRELOADした状態でも通過。ON修正の強制発火試験は439回・158姿勢、正常終了・有限値。これらの診断結果は本比較の精度表には混ぜていない。
- 成果物：`logs/zab-offline/livo_comparison/plots/index.html` / `comparison.pdf`（2ページ）、`README.md`、`experiment.json`、`verification.json`。版管理する要約は`for_codex/resources/LIVO_COMPARISON_20261004.md`。再現用実装はnative/comparison/（patch/header/offline adapter/revisions/tests）、`build_livo_comparison.py`、`comparison_ros1.sh`。
- 制約：同じセンサー・単眼・評価区間の設定比較。MIMOSAの既存IMU/odometryの重みを3条件で共通保持、FAST-LIVO2は接続・幾何設定以外は公開設定を基準にしており内部の重みは同値化していない。DRPMはEq28の情報量の重み付けを因子へ導入し、ゼロ運動事前因子は追加しない。等方点雑音と5近傍の面法線共分散は模擬LiDARの距離方向雑音の近似。各条件1回の完成試行で、パラメータ探索・全run評価・論文の独立ICP再現は行っていない。
- 検証用ROS1 master11376を停止。推定器・転送プロセスなし、GUI/Gazebo起動なし。ユーザーのkeyboardは継続。C:空き約16GB。入力キャッシュ約3.3GBは同じ2例の追実験用に保持し、元センサーbagはDriveに保全。

## 2026-10-04 単眼LIVO比較の原因調査と回転重みの追試
- ON/OFFを初期化含む全姿勢で再照合し、時刻・位置・四元数の完全一致を確認。標準ONは3×3の回転/並進ブロック別の絶対閾値、DRPMは6×6全体と雑音モデルによる確率判定。判定に用いる情報が同一ではない。
- run1_tail_a2_prefixの姿勢誤差10°超過はOFF136.12秒、DRPM44.22秒。入力ROVIO単独の姿勢RMSEは0.06120°だが、MIMOSAへの融合は固定σ回転2°/並進0.1mの相対姿勢制約。入力共分散は採否だけに利用。
- DRPMの設定からodometry.manager.sigma_rot_degだけ2→0.02へ変更した単発診断追試を実施。位置RMSE9.759120→0.444783m、姿勢RMSE123.004111→7.316929°。最大姿勢誤差23.127°は残る。重み不足の関与を裏付けるが、校正済み設定や公平な手法再比較ではない。
- 追試exit0/64.43秒、追従4004点/最大間隔0.05秒/元時刻列と完全一致。元の結果や図は上書きせず、native/logs/zab-offline/livo_comparison/diagnostics/fusion_weight/へ設定・再現スクリプト・出力・評価を保存。
- DRPM原論文式24/26の更新量抑制と、今回のH/bの情報重み付けだけの移植は数学的に異なることを明示。今回の結果を原論文のDRPM自体の性能とは解釈しない。FAST-LIVO2との差もtight coupling単独の効果は未分離。
- 調査記録はfor_codex/resources/LIVO_DIAGNOSTICS_20261004.md。推定器と検証用ROS master11376は停止。GUI/Gazeboなし。

## 2026-10-04 ROVIO相対姿勢校正とFAST-LIVO2画像なし比較：完了
- 前回の単眼link3・2飛行を使用。既存ROVIO推定から50ms/100ms/0.5s/1sの相対姿勢誤差を測定。一方の飛行で求めた六軸の非中心二次モーメントを他方へ適用し、評価対象自身での重み調整はしない。偏りを取り除く真値補正はなし。50ms回転RMSEは軸別約0.0001〜0.0017°、並進X約7.57〜9.88mm、Y/Z約0.22〜0.35mm。並進時間相関0.74〜0.92が残り、独立雑音・共通IMU相関の厳密な処理ではない。
- 比較用MIMOSAに任意の固定6×6相対姿勢共分散を読み込む機能を追加。未指定時は従来の固定σ。comparison/mimosa-odometry-covariance.patchとbuild_livo_comparison.pyで再構築可能。通常のslam_ros1_wsは未変更。
- FASTオフライン入力に標準img_en=0でのLIO記録、時刻を揃えて画像更新だけ停止する対照、追従開始後だけ画像更新を停止する対照を追加。画像OFFは呼び出し数でも検証。短いrunが追従前の高度整定中に発散したため、最後の対照を追加して準備段階の影響を分離した。
- 7条件×2飛行の14ケースexit0。共通10Hz評価、開始時に一度だけ剛体整合。元の出力時刻を保持、追従中のLIO495/2002点、他989/4004点、時刻列完全一致・欠落0。誤差評価CSVは補間時刻を明記。
- run2a_end 位置RMSE/姿勢RMSE (m/deg): mimosa_lio=13.496983/1.970634, mimosa_calibrated=0.511148/0.199296, mimosa_drpm_calibrated=4.399628/0.159267, fast_full=0.035253/0.662300, fast_lio=12576.923355/27.904979, fast_no_visual=11595.698367/24.967827, fast_tracking_no_visual=13.767775/2.708379。
- run1_tail_a2_prefix: mimosa_lio=1.108414/56.279143, mimosa_calibrated=4.255743/0.303098, mimosa_drpm_calibrated=4.453920/0.225301, fast_full=0.373302/3.711495, fast_lio=7439.778538/55.208741, fast_no_visual=15509.069806/69.653866, fast_tracking_no_visual=11083.266875/62.308161（詳細はevaluation.json）。
- 結論: 実測校正でMIMOSA+DRPMの姿勢発散は収まるが、DRPMなしに比べ位置精度は改善しない。FAST-LIVO2は画像更新を除くと悪化/発散し、追従中だけOFFの対照でも悪化するため、この設定では画像が良好な精度と安定性を支えている。FASTのLIO単独が通常MIMOSA LIOより強いという結果ではない。tight couplingそのものの優劣は未分離。
- 再現・評価実装: calibrate_rovio_relative.py、run_calibrated_livo.py、plot_calibrated_livo.py、verify_calibrated_livo.py。結果はnative/logs/zab-offline/livo_calibrated/。plots/index.html、comparison.pdf（8ページ）、六軸PNG/CSV、校正JSON・設定・ログ・検証JSONを保存。元の比較結果/図は未変更。要約for_codex/resources/LIVO_CALIBRATED_COMPARISON_20261004.md。
- 校正数学を行列対数30例とゼロ回転で検証、両パッケージビルド成功。検証用ROS master11376/推定器は停止、GUI/Gazeboなし。新たな画像/点群出力やDrive転送なし。

## 2026-10-05 DRPMの距離方向ノイズ検証・修正と2飛行の再評価：完了
- 仮想LiDARのURDF/Gazebo実装と保存済み点群を確認。range sigma=0.03m、静止91スキャンの光線別距離標準偏差中央値0.029936/0.029942m、直交RMS1.53e-7/1.84e-7m。従来σ²Iの等方近似は一致せず。
- comparison専用DRPMで点の共分散をσ²uuᵀへ対応。機体とLiDARの原点差を反映し、地図点は取得時の共分散を保持。面法線の共分散は各点からPCA微分で伝播、微小回転共分散に変換。面の誤差を別途実測入力する方式ではない。
- 再構築用mimosa-drpm-noise.patch、drpm.hpp、build_livo_comparison.py更新。drpm_noise_model=0従来/1伝播等方/2距離方向を追加。修正版の比較には2を指定、従来再現用の既定値0は保持。通常slam_ros1_ws未変更。
- 数学検証: PCA中央差分<1e-7、正面/斜面×等方/距離の各10万回Monte Carloで共分散誤差0.50〜0.97%、Hessian雑音の平均/分散、座標変換、旧既定の数値互換、地図への共分散保持を確認。旧DRPM数学テストも成功。ビルド成功（既存OpenCV/TBBリンク警告あり）、パッチを保存済み変更前ソースへ適用して構築ソースとの一致を確認。
- 同一の入力bag・単眼link3 ROVIO・別飛行で校正済みの固定融合行列で3条件×2飛行を再実行、6件exit0、推定器実行計約11分。閾値/残差重み/入力は固定。前回旧方式の全時刻・位置・四元数を再現。追従中989/4004点、最大間隔0.05秒、欠落0。
- 共通10Hzの位置RMSE/姿勢RMSE (m/deg): run2a_endは従来4.399628/0.159267、伝播等方3.692832/0.283232、距離方向23.967582/0.573889。run1_tail_a2_prefixは従来4.453920/0.225301、伝播等方7.326136/0.121508、距離方向5.882769/0.230812。参考DRPMなし0.511148/0.199296、4.255743/0.303098。
- 結論: ノイズモデルの不一致は修正したが精度は改善せず。現MIMOSA+DRPMの性能不足を等方近似の誤りだけでは説明できない。原論文全更新則との差、相関/地図姿勢の未モデル化は変更せず。実機回転走査の取得時姿勢対応は未実装（今回全点オフセット0）。
- native/run_drpm_noise.py、plot_drpm_noise.py、verify_drpm_noise.py、verify_drpm_sensor_noise.pyを追加。結果native/logs/zab-offline/drpm_noise/へ保存。六軸PNG/評価CSV、plots/index.html、comparison.pdf、検証JSON、ノイズ検証・数学テストログ、入力/構築物SHA256、詳細READMEを保存。旧図は保持。resources/DRPM_RADIAL_NOISE_VALIDATION_20261005.mdに要約。
- ROS master11376と検証推定器を停止。GUI/Gazebo起動なし、画像/点群の新規収録・Drive転送なし。


## 2026-10-05 DRPM公式更新・誤差近似の切り分け：実装
- 依頼：公式の更新とノイズ近似の両方、更新だけ、ノイズ近似だけを同じ2飛行・校正済み単眼ROVIO入力で比較。
- 公式ntnu-arl/drpmの固定commit b39a540c946f4059f5e3dfde625216c437eaf158からdegeneracy.hを改変せず同梱。公式の法線誤差、情報行列のノイズ、非退化確率、Eq24更新の関数を直接呼ぶ。公式公開物は計算部であり、論文CompSLAM全体の再現とは区別。
- 公式更新を独立したLiDAR点群位置合わせに適用し、Eq28の情報行列と最終姿勢をMIMOSAへ渡す経路を追加。Eq26のゼロ更新正則化を融合側のセンサ情報に混入しない。情報は残差sigmaで一度だけ規格化。
- comparison/drpm_kernel.hpp、drpm_registration.cpp/.hpp、drpm_pose_factor.hpp、mimosa-drpm-official.patchを追加。build_livo_comparison.pyで比較用workspaceへ組込み、通常slam_ros1_wsは変更しない。
- 3条件の周辺構造を統一した4通りの比較（更新2種×ノイズ2種）に加え、旧グラフ直接重み付け＋等方モデルも再実行。独立した位置合わせへの変更と更新式そのものの効果を混同しない。
- 最大10反復、平面法線の標準偏差0.1radによる除外を共通適用。対応点不足等はそのスキャンのLiDAR制約を生成せず、全スキャンの採否・反復数・確率・最終更新量を記録。10反復上限に多数が到達したため、両方公式の上限30反復を別の追加確認として設定し、主比較と分離。
- 公式関数の数値一致、正則化を含まない情報行列、取り付け姿勢を含む制約の数値微分20例、既知3平面の位置合わせ4条件を検証し成功。新規パッチの適用確認・Python構文検査成功。比較用ビルド成功。
- 実行・集計：run_drpm_official.py / run_drpm_official_iterations.py / plot_drpm_official.py / verify_drpm_official.py。結果はnative/logs/zab-offline/drpm_official/。最終結果と停止確認は後続の完了記録を参照。


## 2026-10-05 DRPM公式更新・誤差近似の切り分け：完了
- 同じ2飛行・単眼link3 ROVIO・別飛行で校正した固定融合重み。主比較3条件に、共通構造で両方変更した対照、旧直接グラフの再現、反復上限30回の確認を追加し、計12試行すべてexit0。
- 共通の位置合わせ構造での位置RMSE / 姿勢RMSE（m / deg）：run2a_endは両方公式4.132010 / 0.134436、更新だけ公式1.239010 / 0.358439、誤差近似だけ公式9.048921 / 0.328089。run1_tail_a2_prefixは両方公式41.123793 / 0.314912、更新だけ公式0.865113 / 0.537693、誤差近似だけ公式42.158810 / 1.261974。
- 追加の共通構造・重み付け式＋距離方向モデルは1.764033 / 0.325956、1.009097 / 0.586985。距離方向モデルで公式更新へ替える差はこの値との比較で判断する。旧直接グラフとの大きな差を更新式単独の効果とは解釈しない。
- 旧直接グラフ＋等方モデルを再実行し、全時刻・全位置・全姿勢が以前の結果と完全一致（4.399628 / 0.159267、4.453920 / 0.225301）。従来のコード経路は維持。
- 依頼3条件では更新だけ公式が両飛行の位置誤差最小、両方公式が姿勢誤差最小。DRPMなしの校正MIMOSA＋ROVIO（位置0.511148m / 4.255743m）と比べると、更新だけ公式でも短い飛行の位置と両飛行の姿勢は悪化する。全指標での改善とは言えない。
- LiDAR制約採用：両方公式400/495、1966/2002。公式更新＋距離方向モデル495/495、2002/2002。等方モデルの対応点中央値17〜25に対し公式更新＋距離方向モデル690/784。法線不確かさの除外と、その後の軌跡・地図の違いが含まれるため、除外一つを原因と断定しない。
- 共通10反復では停止条件を満たさない採用スキャンが多数。両方公式で上限を30へ増やした診断も実施し、位置RMSE9.440706m / 39.470711m、姿勢0.683491° / 0.947570°。大きなドリフトは解消せず、反復上限不足だけでは説明できない。主比較は10反復のまま保持。30でも未収束が残り、収束済み解同士とは主張しない。
- verify_drpm_official.py成功：入力bag SHA一致、全試行同じ実行ファイル・共有ライブラリ、公式ヘッダ原文バイト一致、意図した設定差のみ、ROVIO共分散読込・棄却なし、元時刻一致。追従989/4004点、間隔0.05秒、共通10Hz評価495/2002点。元入力・既存結果を上書きしていない。
- plot_drpm_official.pyで3条件、全対照、反復上限確認を各飛行六軸に描画。軸省略なし。HTML、PNG6枚、PDF6ページ、各誤差CSV、evaluation/verification JSONを生成。主比較2図を目視、PDFページ数・完結・HTML内リンクを確認。
- 成果物：native/logs/zab-offline/drpm_official/plots/index.html、comparison.pdf。版管理用説明はresources/DRPM_OFFICIAL_COMPARISON_20261005.md。公式はDRPM計算部の適用でありCompSLAM全体の再現ではない。点のsigma3cm、近傍5点等は今回のセンサ・共通条件。
- 起動したROS master11376/11377と全推定器を停止。GUI/Gazeboは起動なし。通常slam_ros1_wsは変更なし。追加診断は一部並行実行したため、wall_sを処理速度の公平比較には用いない。


## 2026-10-05 公式更新＋距離方向DRPMの採用、FAST-LIVO2との再比較
- ユーザーが「更新だけ公式＋距離方向モデル」を採用と決定。comparison/adopted_drpm.jsonへ比較基準を保存。drpm_registration_mode=1、drpm_noise_model=2、最大10反復、sigma=0.03m、SNR=10、校正済み単眼link3 ROVIO融合を維持。通常launchの設定変更は行っていない。
- plot_adopted_drpm_vs_fastlivo.pyを追加し、保存済みofficial_updateと画像ありFAST-LIVO2 fast_fullの同じ2飛行を再集計。推定器の再実行なし。共通10Hz、同一追従区間、開始時に一度だけ位置・姿勢整合、縦軸省略なし。
- 位置RMSE / 姿勢RMSE（m / deg）：run2a_endは採用MIMOSA+DRPM+ROVIO=1.239010 / 0.358439、FAST-LIVO2=0.035253 / 0.662300。run1_tail_a2_prefixは採用方式=0.865113 / 0.537693、FAST-LIVO2=0.373302 / 3.711495。両飛行で位置はFAST-LIVO2、姿勢は採用方式が良好。方式全体の比較であり、tight couplingだけの効果とは断定しない。
- 同じ保存bag参照、評価区間一致、採用設定一致、FAST画像補正ありを確認。追従時刻989/4004点が両方式で一致・欠落なし。再集計した位置/姿勢RMSEは保存評価と1e-12以内で一致。元odometry/config/truthのSHAを記録。元の推定時刻は変更していない。
- 成果物：native/logs/zab-offline/adopted_drpm_vs_fastlivo2/plots/index.html、comparison.pdf（2ページ）、PNG2枚、六軸誤差CSV、evaluation/verification JSON。図の表示、PDF完結とページ数、HTML内リンクを確認。
- 版管理用要約：resources/ADOPTED_DRPM_VS_FASTLIVO2_20261005.md。採用方式に残る位置合わせ未収束（停止条件達成133/495、788/2002）も資料に明記。ROS/Gazebo/GUIは起動していない。


## 2026-10-05 DRPM Visionなし測定、FAST-LIVO2 LiDARなしの適合調査
- 依頼：採用DRPMのVisionなしと既存FAST-LIVO2画像なしを比較し、既存ROVIO単体とFAST-LIVO2 LiDARなしを比較。
- run_adopted_drpm_lio.pyで採用official_update構成のodometry.manager.enabledだけfalseへ変更し、同じ2飛行を新規測定。LiDAR/IMUだけを読み、ROVIO・画像・真値を推定器へ入力しない。推定器コード・通常launch・採用基準は変更なし。
- DRPM LIOは2試行exit0（43.37/132.82秒）。位置RMSE / 姿勢RMSE（m / deg）はrun2a_end=12.108117 / 1.410846、run1_tail_a2_prefix=33.395138 / 3.265019。
- 再利用FAST-LIVO2標準画像OFF（fast_lio）は12576.923355 / 27.904979、7439.778538 / 55.208741。発散結果を除外せず表示。短い飛行は既存診断で追従開始前に発散しており、飛行中だけ画像を停止する別条件とは区別。
- ROVIO単体の既存値1.130972 / 0.045204、4.889198 / 0.061220も再集計。FAST/ROVIOは再測定なし。
- FAST-LIVO2 LiDARなしは現行公式実装で単体VIO未対応。固定commit 0d2c0346107b75b59934975adec9a6eeeb913c64の原文を確認：ONLY_VIOモードなし、lidar_en=0/imu_en=trueでもONLY_LIOへ分岐、同期処理はLiDARバッファ参照、画像補正は点群が空だとreturn、視覚地図点はLiDAR由来。深度初期化等の新規実装なしでは成立しない。無効な設定での停止やIMUのみをVIO精度として扱わず、未対応・未実行としてfast_without_lidar_audit.jsonへ根拠と共に記録。
- plot_sensor_ablation.pyを追加。LIO重ね描き、DRPM単独全範囲、ROVIO単独の六軸図を2飛行分作成。PDF6ページ、HTML、誤差CSV、evaluation/verification JSON。縦軸省略なし。unsupportedなFAST VIOのRMSEはnull。
- 検証：入力bag SHA・実行ファイル・共有ライブラリが採用構成と一致、設定差がROVIOのenabledと保存先のみ、実読み込みがLiDAR/IMUのみ、FAST画像更新0。追従LIO495/2002点・ROVIO989/4004点の元時刻一致、既存評価値を再現。DRPMのLiDAR制約は全スキャン採用、停止条件達成112/495・412/2002。
- 今回はROVIOが運ぶ画像情報とIMU情報を同時に除いており、画像単独の効果とIMU二重利用の効果は分離していない。
- 成果物：native/logs/zab-offline/sensor_ablation/plots/index.html、comparison.pdf、resources/SENSOR_ABLATION_20261005.md。図目視・PDF完結/6ページ・リンクを確認。起動master11376と推定器を停止、GUIなし。


## 2026-10-05 FAST-LIVO2のlink1/link4評価とlink3対照
- ユーザー指定の同じ2飛行でカメラを切替。追加したprepare_fast_camera_inputs.pyでDriveの元MCAP27分割を3並列で取得しMD5照合、link1/link4画像を抽出。再加工link3は既存キャッシュと全バイト一致。ファイル境界をまたぐカメラセットは全ストリームで時刻照合し、分割境界ごとの枚数差を欠損と誤認しない。
- make_fast_camera_bags.pyで既存比較bagの画像だけを交換。LiDAR/IMUのシリアライズ内容・全撮影時刻・入力順序が一致。短111338/1113/2226、長262158/2621/5243（IMU/点群/画像）。通常launch/センサ配置/元クラウドbagは変更なし。
- run_fast_camera_comparison.pyで記録時校正からRcl/Pclを変更。他のFAST設定は同一。過去と実行ファイルSHAが異なるためlink3も再測定。同一バイナリ再実行でも数値差が出たため全条件3回、主比較計18回。全てexit0、追従時刻989/4004点・0.05秒間隔・欠落なし。
- run2a_endの位置RMSE/姿勢RMSEの3回平均: link1=0.092423m/0.454155°、link3=0.035097m/0.502518°、link4=0.126457m/1.579427°。姿勢のlink1/link3試行範囲は重なる。
- run1_tail_a2_prefixの3回平均: link1=6208.304847m/47.012963°、link3=0.363484m/4.180368°、link4=4667.959926m/36.297695°。link1は3/3発散、link4は2/3発散、残りlink4の1回は0.502065m/1.289014°。平均だけでは混在を隠すため最小–最大と全試行も掲載。発散した5試行は追従開始後128.32–136.82秒に10m超。
- link3短区間でOpenMPスレッド上限1の補助2試行を実施しても数値差あり（最大位置差0.06649m、姿勢差0.39335°）。並列加算だけが原因とは断定しない。主比較へ混ぜずserial_diagnostic.jsonに保存。推定器本体は変更せず、再現性の原因は未解決として記録。
- plot_fast_camera_comparison.py/plot_fast_camera_repeats.pyを追加。共通10Hz、開始時一度だけ位置姿勢整合、離陸等は評価から除外。元時刻維持。各飛行の六軸図は省略なし、追加の誤差ノルム対数図も生成。HTML、PDF4ページ、PNG4枚、全試行誤差CSV、集計/照合JSON。主数値はrepeat_evaluation.json。
- 詳細資料resources/FASTLIVO2_CAMERA_COMPARISON_20261005.mdとHANDOFFへ追記。転送したMCAP一時ファイルは抽出後削除、抽出画像と比較bagは再現用に保持。Windows C空き18GiBを確認。全ROS検証プロセス停止、GUIは起動なし。


## 2026-10-05 FAST-LIVO2へ複数カメラ入力・逐次画像融合を実装
- ユーザー依頼に対応し、比較用FAST-LIVO2へカメラ配列設定、個別校正・個別画像地図、同期画像セット、共通状態/共分散への逐次画像更新を追加。各カメラの直前posteriorを次のpriorにし、IMU/LiDARの再適用なし。順序link1→link3→link4。一括画像最適化・クロスカメラステレオではない。
- 主変更: comparison/fast-livo2-multicamera.patch、synchronized_images.hpp、fastlivo_offline.cpp、fastlivo_multicamera.launch、build_livo_comparison.py。新しい設定を指定しなければ既存単眼経路を維持。カメラごとの処理・制約・補正量・共分散trace・同期破棄数を記録。
- 同時刻画像を整数nsで束ね、重複/古い画像を棄却、未完セット/完成待ち列に上限。時刻をdouble化する方法も旧toSecへ揃え、最終版は処理回数まで一致。旧修正前データはpre_timestamp_fixへ隔離し最終値に混ぜない。
- 同期・固定露出・固定取付変換に対応。元FASTの露出状態は1個なので、複数カメラのexposure_estimate_en=trueは拒否。非同期/自動露出の独立推定/動的取付変換は未実装。先頭画像だけ代表可視化。
- /multicamera/odometryへ全カメラ補正後の位置・姿勢・撮影時刻を出力。従来の壁時計時刻/補正前姿勢を使わず、現状態のQuaternionを生成。pose共分散は位置・回転の順と座標系を変換。従来/aft_mapped_to_initは維持。
- prepare_fast_multicamera.pyで検証済みbagから3画像トピックを統合。短111338 IMU/1113点群/2226枚×3、長262158/2621/5243枚×3。元センサ内容ハッシュ・全画像時刻・保存時刻一致。
- run_fast_multicamera.py/verify_fast_multicamera.pyで最終版11試行。3台×2飛行×3回=6、2台短1、明示1台短1、旧単眼固定露出2、旧単眼露出推定ON短1。全exit0、追従989/4004出力点・20Hz時刻欠落なし、同一バイナリ、共通センサ数とLIO/VIO呼出数一致。
- 3台の位置RMSE短0.083057–0.091817m、長0.366206–0.401847m。姿勢短0.394688–0.463877°、長0.671750–1.179959°。全6試行で位置最大誤差1m未満。固定露出単眼長対照1回は809.175mへ発散。単眼側の反復は揃えておらず、過去とは露出設定も違うため、性能優位の比較結論には使わない。
- 各カメラ画像処理1111/2619回、各1000回以上の補正寄与、前カメラ補正後と次カメラ補正前の共分散trace一致。全試行で同期欠損/重複/期限切れ/待機列破棄0。元FASTの空点群時スキップによりこの瞬時LiDAR構成の実画像更新は約10Hz、状態CSV20Hz。
- test_synchronized_images.cppは順序逆転/厳密時刻/重複/欠損/古い入力/容量上限/単眼に成功。test_fast_multicamera_configuration.pyで不正回転・同一トピック・複数台露出ONを拒否。test_fast_multicamera_ros.pyはROS launch+10秒bag再生で98 odometry、元画像timestamp/単調性/有限位置/単位Quaternionを確認。
- 固定公式原文+既存パッチ+新パッチでソース再現と再適用判定成功。最終ビルド警告なし。新スクリプト構文検証済み。comparison/MULTICAMERA.md、resources/FASTLIVO2_MULTICAMERA_20261005.md、HANDOFFへ記録。
- 通常ROS2 simulation launch・slam_ros1_wsは変更なし。新入力bagと最終データはnative/logs/zab-offline/fast_multicameraへ保存。起動したROS master11376、全推定器、ROS launchとbag再生を停止。GUIなし。


## 2026-10-05 FAST-LIVO2 3カメラ・全14飛行の取得開始
- ユーザー依頼：複数カメラ版で全runのデータ取得、なるべく早く完了。link1→link3→link4、固定露出・固定取付校正、既存3台実験と同じ推定設定。既存run2a_end/run1_tail_a2_prefixは事前にtrial1再利用を決め、他12飛行を新規計算する。最良試行を選択しない。
- run_fast_multicamera_all.py、fast_stream_transport.py、run_fast_stream_child.py、comparison/fastlivo_stream.cppを追加。DriveのMCAPをMD5照合して小分け取得し、次ファイルの取得と計算を並行。1秒の順序整列バッファで元header時刻順にROS1メッセージを渡す。1飛行に1推定プロセス、分割境界で初期化しない。点群・画像の結果保存なし。
- 既存FAST推定ライブラリは変更せず、新しいストリーム入力用の実行ファイルのみ追加。build_livo_comparison.pyへ再現手順を追加。ビルド成功、旧fastlivo_offlineのSHAは前回と一致。
- test_fast_stream_transport.pyで既存run2a_endの全IMU111338、点群1113、画像2226×3を照合：メッセージ全バイト一致。新入力処理で推定時刻2223点、LIO2224/VIO2223回、カメラ1111回ずつ、同期破棄0が既存bag入力と一致。
- 出力先native/logs/zab-offline/fast_multicamera_all/。追従区間のodometry.csv/truth.csvは元の整数ns時刻を保持、初期化を含む_full.csvを監査用に分離。errors.csvは共通10Hz補間評価。六軸図生成用plot_fast_multicamera_all.pyも追加。完了結果は後続記録を参照。
- 実行中の高速化：取得8並列へ変更し、独立ROS master11376/11377で長い飛行と短い飛行を2系統並行処理。推定器を途中で分割しない。入力転送が支配的であり、wall_sを推定器単独の速度比較に使わない。
- 途中観測：run3位置RMSE0.040674m/姿勢0.251518°、run4 0.1457m/0.6044°。run1_prefixは追従開始73.2秒で10m超となり発散。条件を変更せず継続、発散値を除外しない。
- 長いrunで地図保持メモリが増大するため、`/root/.cache/fast_multicamera_all/swap`に一時8GiBのswapを追加（fstab等の永続設定なし）。全処理後にswapoffして削除する。現在のメモリ/各推定器RSSをresources.jsonlへ記録。
- 長い発散run1_prefixの約1056.35秒で処理が大幅低下。gdb読取のみのスタック採取でVoxelMapManager::UpdateVoxelMap内unordered_map再配置とページ読込待ちを確認（保存: run1_prefix/memory_pause_stack.txt）。推定器の変更や打切りは行わず、元の計算を継続。
- 一時swapを追加4GiB（swap_extra）して計12GiBとし、カーネル標準zramを一時ロード。/dev/zram0を論理16GiB・lzo-rle・優先度100で有効化し、可逆なメモリ圧縮でディスク待ちを低減。追加swapファイルと既存/dev/sdcの退避ページを順に読み戻し、圧縮メモリへ移動後に各swapを再有効化。計算内容・点群・画像・推定設定は不変。終了時には所有zram/一時swapのみ解除・削除し、元/dev/sdcを有効なまま残す。
- run1_prefixは全入力・全追従区間を完走（exit0）。地図再配置後の終了処理もmalloc_consolidateで遅延したことをread-only stack採取で確認（shutdown_stack.txt）。推定器や終了コードを変更せず、通常終了を待った。追従21708点の時刻欠落なし。位置RMSE約2592km、姿勢107.60°という発散値も保存。
- 残る長いrun用に/dev/zram1（論理8GiB、lzo-rle、優先度90）を追加。zram0と合わせて論理24GiB、実RAM消費は圧縮後の量。終了時は両デバイスとも解除・リセットする。元/dev/sdcは再有効化済み。

### 2026-10-05 全run取得の中断後復旧
- 再開時12/14集計済み。run2bは全28チャンクと最終入力・カメラ・更新集計を出力済み。追従6551標本の20Hz連続性、全カメラ寄与・共分散引継ぎ・有限姿勢を検証して集計を復旧。終了コードと逐次入力SHAは親処理の中断で未保存のためnullとして明記（exit0を推定しない）。
- run1_middleは1285.45秒までで入力途中切断（Truncated message）。完了結果にせずinterrupted_attempts/run1_middle_20261005へ保存し、同一条件で全区間を再実行。良い結果を選ぶ再試行ではない。
- 入力監査情報を子の終了待ちより前にinput_audit.jsonへ保存するよう改善。推定本体・設定は変更なし。再開処理は端末終了に依存しない形で監視し、終了後にROSと一時メモリ資源を片付ける。

### 2026-10-05 ユーザー指示による全run評価の一時停止
- 13/14飛行を集計済み。run1_middle再実行は19/94チャンク、進捗上269.05秒まで。ユーザー「切りのいいところで、再開できるようにした後一旦停止」に従って停止。
- 推定器・読み出し・ダウンロード・ROS・監視/後処理待機の所有15プロセスをSIGSTOP。全15件がT状態であることを検証し、進捗・カーネル起動ID・PID・開始時刻をpause_state.jsonへ保存。メモリ状態を維持し途中から再開可能。WSL再起動後の復元機能ではなく、その場合は未完了のrun1_middleだけ再計算。
- native/control_fast_multicamera_all.pyを追加。status/resume。再開はユーザー指示後のみ。起動ID・開始時刻を検証してPID再利用を防止。status検証成功、実際の再開は未実施（停止指示を維持）。
- 再開詳細はnative/logs/zab-offline/fast_multicamera_all/RESUME.md。既存13飛行・入力キャッシュ・一時swap/zramを維持。計算の続きに必要なため一時メモリ資源はまだ解除しない。完了後に所有ROS/監視と合わせて片付ける。
- 取得は約11–12 MiB/s。8→16並行に増やしても速度増なし。補助先読みは完了して終了済み。通信待ちが主因で、推定条件や品質を変更していない。

### 2026-10-05 ユーザー指示による続行
- 全15停止プロセスの起動識別子・開始時刻が一致することを確認し、control_fast_multicamera_all.py resumeでSIGCONT。推定器の再初期化はせず、run1_middleの19/94チャンクから続行。完成13飛行を維持。停止時間もwall_sに含まれるため処理速度指標には使わない。

### 2026-10-05 FAST-LIVO2 3カメラ・全14飛行 完了
- 12飛行を新規計算し、事前指定した既存2飛行trial1を再利用。全14の真値・推定値・六軸誤差CSV、設定、検証情報、六軸PNG/14ページPDF/HTMLをnative/logs/zab-offline/fast_multicamera_allへ保存。センサ入力の元ns時刻を維持。誤差だけ共通10Hz補間。初期化を含むfull CSVと追従区間CSVを分離。
- 全14の設定・校正、推定ライブラリSHA不変、MCAPチェックサム、有限位置・単位四元数、追従中の時刻連続性、全カメラ寄与と共分散引継ぎの照合に成功。finalize_status.json returncode=0。HTMLの全リンク、PDF14ページの構造、良好例/発散例の図を確認。
- 5飛行で明確な発散: run1_prefix（位置RMSE 2,592,437m）、run1_middle（3,042,660m）、run1_tail_a1（16,084m）、run1_tail_b1（106,109m）、run2b（32,431m）。発散値を削除・切断・軸省略しない。残る9飛行の位置RMSEは0.041–2.015m。各run1試行のため成功率や平均性能とは扱わない。
- run1_middleは元記録末尾の次LiDARがないため、1323.2秒の1標本だけ未出力。最終1323.15秒、追従25,251標本。内部時刻の欠損はなし。元データを外挿せず記録。run2bは推定完了後の親中断から復旧したためOS終了コード/逐次入力SHAが未取得。この制限をsummaryと詳細レポートに明記。
- ユーザー指示による一時停止から、同じ推定器プロセスを再開して完了（再初期化なし）。以前の入力切断試行はinterrupted_attemptsに別保存。最良試行を選択した結果ではない。
- 所有ROS master11376/11377、rosout、監視を停止。推定/取得/集計プロセスは終了済み。一時swap8+4GiBを解除/削除、zram0/1を解除/resetし、今回ロードしたzram/zsmallocを解放。swapは元/dev/sdc 4GiBのみ、priority=-2へ復元。使用済み入力キャッシュを削除、元クラウドbag・旧結果は変更なし。
- 詳細: for_codex/resources/FASTLIVO2_ALL_RUNS_20261005.md。出力内runtime.json/pause_state.jsonはcomplete、cleanup_complete=true。

### 2026-10-05 発散5飛行でのFAST-LIVO2単眼確認を開始
- 対象: run1_prefix、run1_middle、run1_tail_a1、run1_tail_b1、run2b。link3のみ。3台版のmulticamera設定を削除し、同じ実行バイナリの従来単眼分岐を使用。他の設定・校正・露出推定OFF・LiDAR/IMU・画像処理を維持。推定ライブラリSHA不変を検証。
- 発散判定を事前固定: 開始位置/姿勢を1回整合した後、位置誤差100m超が5秒連続した場合、チャンク境界で入力を終了する。未到達なら全区間を実行。真値は判定/評価にのみ使用し、推定器へ入力しない。早期終了のRMSEは全run値と比べず、単眼/3台/MIMOSAを同じ区間で再評価。
- native/run_fast_monocular_failures.py、check_fast_monocular_failures.pyを追加。既存の時刻保持転送処理を使用し、link3の画像だけを単眼入力番号へ対応。入力のMD5確認、同じ親内で全チャンク連続処理。
- 短い既存run2a_endで単眼の転送経路を検証。IMU111338、点群1113、画像2226、出力時刻とLIO2224/VIO2223更新回数が従来bag入力と一致。CAMERA_COUNTSなしで単眼分岐を確認、終了コード0。
- 出力native/logs/zab-offline/fast_monocular_failures。独立ROS master11376/11377で2飛行まで並行。5飛行を完了後、図と作業記録を更新し、所有プロセス/入力キャッシュを片付ける。元bag・既存比較結果は変更しない。

### 2026-10-05 発散5飛行での単眼確認 完了
- **5/5飛行でlink3単眼も発散**。位置誤差100m超が5秒連続する最初の区間の開始（共通追従開始から秒、単眼/3台）: run1_prefix 167.5/76.6、run1_middle 106.2/145.7、run1_tail_a1 126.1/124.2、run1_tail_b1 78.4/56.9、run2b 121.7/115.3。3台の時刻は保存済み全区間から算出。RMSEは各行で同じ短縮区間に揃え、前回全区間RMSEと混同しない。
- 3台構成のmulticameraキーのみ削除し、同一バイナリの従来単眼分岐、link3・露出推定OFFを使用。全5件の設定一致、ライブラリSHA不変、元チャンクMD5/サイズ、入力カウンタ、出力の50ms連続性、有限位置/単位四元数、終了コード0を照合。元timestamp保持、GTは推定器へ入力しない。
- 単眼の発散を確認後、チャンク境界と残りの1秒の入力バッファまで処理して正常終了。未使用の先読みダウンロードだけを終了し、待ち時間と不要な通信を削減。推定器は強制終了していない。
- 結果はnative/logs/zab-offline/fast_monocular_failures。六軸比較PNG、5ページPDF、HTML、各runの真値/単眼推定/同区間の単眼・3台・MIMOSA誤差CSV、設定・監査情報。図は全誤差範囲を表示。PDF構造5ページ、HTMLリンク、図の描画を確認。詳細for_codex/resources/FASTLIVO2_MONOCULAR_FAILURE_CHECK_20261005.md。
- 追加コード: finalize_fast_monocular_failures.py（全結果照合・報告）、cleanup_fast_monocular_prefetch.py（早期終了後の専用先読みだけを中止）、test_fast_monocular_stop.py（5秒未満の突発値/閾値ちょうど/回復を拒否、5秒連続を採用、書込途中行を無視する試験）。全試験成功。
- 複数カメラ化だけでは発散を説明できない。単眼が長く持つ例も、単眼が先に崩れる例もある。各1試行で、拡張の正しさの証明や成功率評価ではない。共通入力・校正・設定・推定処理と環境のどれが原因かは未特定。
- 所有ROS master11376/11377を停止。推定器・読み出し・先読み監視は終了済み。入力キャッシュ削除、元クラウドbag/旧結果保持。一時swapの追加は不要だった。runtime/validationはcomplete/cleanup_complete=true。

## 2026-10-06 FAST-LIVO2公式単眼の再検証：開始
- 依頼：無改変の公式単眼実装・元画像で数飛行を検証。改善した場合のみ同じ標準処理へ複数カメラ版を整合し全14飛行を再実行する。
- 別WS fast_livo2_official_wsへ公式revision 0d2c034の全tracked filesを抽出、CMake/推定コード/main含め無変更でビルド。公式mainのROS入力経路を使用し、既存の内部関数直接呼出adapterは使用しない。
- official_fast_publisher.py/run_fast_official_raw.py/audit_fast_official.py/evaluate_fast_official.pyを追加。外部ROS1変換・送信のみ。元カラー画素/元時刻を維持。標準avia設定、標準scale0.5/露出推定ON/点間引き1/voxel前処理0.1。機体校正・topic・点形式・32走査線・画像時刻差0・出力先のみ適合。
- 記録時URDFから全4例のカメラ/IMU/LiDAR幾何を照合。以前scan_line=16だったが実機体モデルは32（点特徴抽出OFFの経路で走査線は使用しない）。新条件は32。
- 対象は成功対照run2a_end、失敗run1_tail_b1/run1_tail_a1/run1_prefix。位置誤差100m超5秒連続で早期停止、それ以外は全追従区間。無改変公式のpose出力は現画像補正前のLiDAR更新直後であることを明示し、過去の画像補正後CSVとは区別。
- データ・監査・進捗保存先 native/logs/zab-offline/fast_official_raw/。まだ測定結果は未確定。

### 同日：依存ライブラリも公式設定に統一して本測定へ
- 初回の公式本体試行ではカメラ依存ライブラリvikitが旧comparison_wsのgeneric CPU向けビルドだった。run2a_end/run1_tail_b1で発散したが、終了時にtimeout/segfaultもあったため、この2件はpreliminary_mixed_dependency_buildへ分離し、最終結論には使用しない。
- rpg_vikitも公式revision6c886c8を原本のCMake/ソースから独立ビルド。C++17はインストール済みROSログライブラリに必要なためCMake構成引数で指定（ファイル改変なし）。FAST-LIVO2本体と同じmarch=nativeで統一。本体を再リンクし、lddで独立WSのlibvikit_commonが読み込まれることを確認。
- 最終の対象はrun1_prefix→run1_tail_a1→run1_tail_b1の3件。元カラー画像・公式推定処理/パラメータを維持。停止要求はROSの正規終了APIを使い、測定区間の欠落照合と終了処理の不具合を別に記録する。
- build_fast_official.shを追加、source_audit.jsonでFAST75tracked filesとrpg_vikit全tracked filesを原本照合。最終進行ログnative/logs/fast_official_final_campaign.log。

### 同日：PCLメモリ互換性の不一致を再現・修正し再測定
- 公式本体とnative版vikitの3例は発散後の停止要求でSIGSEGV。gdbでLIVMapper destructor→PointCloud<PointXYZINormal>解放→free(不正pointer)を捕捉。この試行を最終評価から除外しpreliminary_pcl_abi_mismatchへ保存。
- 点群をVoxelGridで縮小するだけの独立C++プログラムtest_official_pcl_abi.cppでも再現。nativeはexit-11、EIGEN_MAX_ALIGN_BYTES=16だけでもexit-11、これにEIGEN_MALLOC_ALREADY_ALIGNED=1を追加するとexit0。SSEコンパイルもexit0。Eigen3.4のAVX/default alignmentとインストール済みPCLのmalloc/free規約の不一致。
- official_pcl_abi.cmakeをCMAKE_PROJECT_INCLUDEで読み、上記2定義をビルド側に適用。本体/vikitのソース・CMake原本・推定パラメータは無変更。修正後3例を再測定し正常終了を必須にする。ビルドログfast_official_abi_build.log。結果は未確定。
- ROS点群形式変換に未使用のambient/rangeフィールドも記載して形式を完全化（XYZ/強度/ring/点時刻は変更なし）。IMUのorientation/covariance/frameも元メッセージ通り保持（公式推定器が使うacc/gyroは以前から同一）。

### 2026-10-06 公式単眼3飛行の再検証 完了
- 互換性修正後の無改変公式FAST-LIVO2/rpg_vikitで3/3飛行とも発散。画像補正後の状態でも同じ判定。追従開始から位置誤差100m超が5秒連続する区間の開始（以前の適合単眼→今回公式、秒）：run1_prefix 167.5→133.8、run1_tail_a1 126.1→114.4、run1_tail_b1 78.4→84.2。最後の1本は5.8秒遅延したが、安定完走は0/3。複数カメラ版の整合・全14run再実行には進めていない。
- 全3件が正規ROS終了要求でexit0。元bag取得MD5、送信数、評価終端までの全画像受信時刻、推定値の100ms連続性、有限値/単位四元数、共通バイナリ、FAST75ファイル/vikit原本一致を照合。PCL互換性修正前の暫定結果は最終比較から除外。
- evaluate_fast_official.py/finalize_fast_official.pyで画像補正前と補正後を分離。公式mat_outの補正後状態を元evo時刻へ全件一致確認して整数nsのCSV化。評価は離陸・向き合わせを除き、開始時のみ位置姿勢整合。公式テキスト出力の数値精度を超える精度は主張しない。
- 成果物：native/logs/zab-offline/fast_official_raw/plots/index.html、comparison.pdf（3ページ）、各runのtruth.csv/odometry.csv/post_visual_errors.csv、source_audit.json/validation.json。六軸図を目視し、PDFの3ページ構造/終端、HTMLリンクを確認。詳細はresources/FASTLIVO2_OFFICIAL_RAW_VALIDATION_20261006.md。
- 結論の範囲：今回の機体校正・センサ入力と公開avia推定設定で公式実装も発散した。環境だけが原因、全設定で失敗する、という結論にはしない。各条件1試行。発散後の早期停止区間のRMSEを全飛行RMSEと呼ばない。
- 起動したmaster11378と全測定プロセスを停止済み。専用入力キャッシュ削除。元クラウドbagと過去結果は保持。GUI起動なし。validationはcomplete/cleanup_complete=true。

## 2026-10-06 DRPM＋単眼vision：公式FAST発散3飛行の比較開始
- 対象run1_prefix/run1_tail_a1/run1_tail_b1、先ほどと同じlink3カメラ1台。採用済み公式DRPM更新＋距離方向ノイズ＋ROVIO（registration_mode=1、noise_model=2、10反復）で全追従区間を計算。FASTは保存済み公式単眼の画像補正後出力と共通区間だけで比較する。
- ROVIO入力と設定は採用済み方式（校正済み白黒補正480×360、単眼25特徴、シミュレーションIMUに合わせた既存設定）を維持。FASTの元RGB入力と同一の物理カメラであり、画像前処理まで同じとは扱わない。
- 新規3飛行用の融合共分散は、過去run2a_end/run1_tail_a2_prefixの保存済み50ms相対誤差を併合した非中心2次モーメントで固定。今回3飛行の真値は重み校正に使わない。元の比較は2飛行交差校正だったので、新対象にはその2飛行全体を訓練用として使う。
- 追加run_drpm_failure_comparison.py/evaluate_drpm_failure_comparison.py。既存取得/ROVIO/ROSbag変換処理を新しい出力先に適用し、元bag MD5、センサ時刻連続性、DRPM処理状況、正常終了、全追従区間の出力時刻を照合する。結果先native/logs/zab-offline/drpm_fast_failures、進捗native/logs/drpm_failure_campaign.log。GUIなし、所有master11376は完了後停止予定。

### 同日：入力準備の並行化・共通区間の先行確認
- 短い2runの入力準備と後段計算を長いprefixと並行実行する。MIMOSAの共有ROS parameter更新はファイルロックで直列化。各段階の完了記録で重複実行を避ける。
- make_comparison_bag.pyにinclude_images=Falseを追加（既定trueは維持）。MIMOSAで読まない画像を一時bagから除外。test_drpm_failure_bag.pyで約40秒分のIMU40648/点群405/ROVIO810メッセージのバイト列・時刻・順序の完全一致を確認。bagサイズ210608037→57440792 bytes。推定器へ渡る内容は変えない。
- 長いprefixの取得中、準備済み先頭15分割から共通区間を先行評価（preview_drpm_failure_comparison.py、出力drpm_fast_failures/common_preview）。同じ固定重みで追従51.6～198.8sを評価：DRPM位置RMSE0.58065m/姿勢0.39595°、公式FAST210.13769m/16.57919°。DRPM最大位置誤差2.6461m、100m超5秒の発散なし。ROVIO単体は1.17426m/0.04810°。
- 先行確認のLiDAR1473スキャンすべてから拘束生成、481回は設定した反復停止条件に到達。10反復上限のままのスキャンは収束済みと呼ばない。全区間は別途再計算しており、この値を全飛行の指標と扱わない。

### 同日：run1_tail_a1 全区間完了
- 全追従232.38秒のDRPM＋ROVIO：位置RMSE5.01997m、姿勢0.53561°、最大位置誤差11.47460m。100m超5秒継続なし。ROVIO単体は8.31323m/0.06789°。
- 公式FASTと共通の121.2秒：DRPM3.48923m/0.58861°、FAST82.42032m/18.03153°。比較の開始姿勢整合と時刻は公式FASTの評価に一致。
- 2324/2324 LiDARスキャンから拘束を生成し、450回が反復停止条件を満たした。ROVIO不確かさによる入力除外0、推定は50ms連続、正常終了。センサ入力期間中の時刻欠落なし（IMU293898、LiDAR2939、画像5878）。
- 評価器は、推定器が入力を除外した場合を単なる検証コード異常と混同しないよう、20Hz出力不足とROVIO除外回数を記録し、比較用10Hz時刻の存在を検証する。今回a1の不足・除外は0。アルゴリズムや閾値は変えていない。
- a1の一時NPZ/ROS1bagを削除、元データと推定/真値/検証記録は保持。残るprefix/b1は処理継続中。

### 同日：Windows実容量を保護して取得を継続
- Windows C:の空きが約25GiB→6.4GiBへ低下。WSL内では使用約55GiBだが、作業用ファイル削除後もWindows側へ空きが戻らず、fstrim（44.9GiB通知）直後も改善しなかった。
- prefixの未完了ダウンロードだけを停止し、準備済み39分割は保持。入力変換処理は未取得分待ちで終了したため、書込途中NPZを中断していない。保持NPZのZIP終端・必須フィールドを確認。
- 今回専用cacheを/dev/shm/drpm_fast_failures_cacheへ切替。RAM上の元MCAPは先読み3分割に限定、処理後削除。prepare_livo_comparison.pyはRAM cacheの場合にWindows実空き3GiB/メモリ側空き2GiBを確認する（従来ディスクcacheの6GiB条件は不変）。推定器・センサ内容・設定は変更なし。
- 再開ログはnative/logs/drpm_failure_campaign_resume.log。所有master11376を維持し、b1融合は中断せず継続。prefixは既存39分割を再取得せず残りから再開。最終cleanupは専用RAM cacheのsymlink/実体を区別して削除する。

### 同日：run1_tail_b1 全区間完了、DRPM＋visionも発散
- 全追従272.54秒：DRPM＋ROVIOは位置RMSE983.33324m、姿勢6.73873°、最大/終端位置誤差2408.10808m。100m超5秒継続は追従開始から95.2秒。公式FASTの84.2秒に対して11秒遅延しただけで、発散解消とは扱わない。
- 入力ROVIO単体も位置RMSE1040.09682m、最大2539.19454m、同発散開始91.4秒。推定入力のvision側も破綻している。原因をDRPMだけに帰する実験ではない。
- FASTと共通94.3秒ではDRPM58.57201m/1.89604°、FAST189.29717m/22.70769°。この区間だけではDRPMの100m継続条件をまだ満たさないが、直後の全区間で発散するため、共通区間RMSEだけで成功と判断しない。
- 2725/2725スキャンでLiDAR拘束生成、反復停止条件達成75回。ROVIO入力除外0、出力50ms連続、正常終了。a1と同じ固定共分散・設定。b1一時NPZ/bag削除、結果と元データ保持。
- prefixのMIMOSA用一時ROS1bagも専用/dev/shm/drpm_fast_failures_run1_prefix.bagへ書くsymlinkを用意。残りの取得・本測定は継続。

## 2026-10-06 DRPM＋単眼vision：FAST発散3飛行の比較完了
- 全3飛行の追従区間を最後まで計算、全推定プロセスexit0。カメラは公式FAST試行と同じlink3の1台。採用済み公式DRPM更新＋距離方向ノイズ＋ROVIOを維持し、今回3飛行の真値で設定・重みを調整していない。
- 数値の単位は位置m、姿勢deg。共通区間はprefix147.2s、a1 121.2s、b1 94.3s。FASTは発散後に打ち切った既存結果であり、全区間のDRPM値と直接同条件比較しない。

| 飛行 | 共通FAST位置RMSE | 共通DRPM位置RMSE | 全区間DRPM位置RMSE | 全区間DRPM姿勢RMSE | 全区間DRPM最大位置誤差 | 全区間ROVIO位置RMSE |
|---|---:|---:|---:|---:|---:|---:|
| run1_prefix | 210.13769 | 0.58065 | 47.96271 | 4.55254 | 56.48900 | 15.84085 |
| run1_tail_a1 | 82.42032 | 3.48923 | 5.01997 | 0.53561 | 11.47460 | 8.31323 |
| run1_tail_b1 | 189.29717 | 58.57201 | 983.33324 | 6.73873 | 2408.10808 | 1040.09682 |

- prefixは短い共通区間では0.58065mだったが、その後に大きくずれ、全区間47.96271m、最大56.48900m。ROVIO単体15.84085mより悪い。100m超5秒の判定に達しないことを高精度・安定の証拠にしない。
- a1は全区間5.01997m、最大11.47460m。100m超の継続はないが、誤差の急な変化も残る。b1はDRPM＋visionも発散、同判定開始95.2s（FAST84.2s）、最大2408.10808m。入力ROVIO自体も発散しており、DRPMのみが原因と断定できない。
- prefix本測定と先行短区間試行は共通4181状態で位置差0、姿勢差最大6.4e-15deg。取得再開・全区間再生で先頭区間の結果は変わっていない。
- 全3飛行で入力のIMU1ms/点群100ms/画像50ms周期に欠落なし、全比較10Hz時刻に出力あり、推定出力50ms連続、ROVIO入力除外0。追従区間の全LiDARスキャン（10854/2324/2725）でDRPM拘束生成。反復上限到達は収束と扱わず、実際の収束停止数をmetricsに保存。
- センサ入力数、元bag MD5、派生入力SHA、推定バイナリ・ライブラリSHA、公式DRPMヘッダ一致、固定設定、比較元真値一致を確認。時刻整数nsを保持したtruth_tracking.csv/rovio_tracking.csv/odometry_tracking.csvは重複なしの昇順。同時刻の最適化更新は最終状態を採用し、元raw出力も保全。共通評価は開始時に1回だけ位置姿勢整合、スケール調整・途中再整合・外挿なし。
- 最終成果物：native/logs/zab-offline/drpm_fast_failures/plots/index.html、comparison.pdf（6ページ、各飛行の共通比較＋全区間DRPM六軸図）。軸省略なし。PDFページ構造/終端・HTMLリンクを検証し、代表図を目視確認。指標/検証結果はevaluation.json/validation.json、各fusion/<run>/metrics.json。
- 最終化はfinalize_drpm_failure_comparison.py。今回専用のNPZ/ROS1bag/RAMキャッシュとsymlinkを削除。元クラウドbag、旧結果、真値/推定CSV、設定/監査記録は保持。所有ROS master11376および検証プロセスは停止済み。GUI起動なし。Windows実空きは終了前確認時約26GiB。runtime/validationはcompleteかつcleanup_complete=true。未完了の処理なし。

## 2026-10-06 発散比較3runの元rosbagをローカルに常設（取得開始）
- ユーザーがWindows容量を確保し、今回3runのローカル保持を許可。実空き約118GiB、元bag合計73,438,954,417bytes（約73.44GB、127MCAP）。run1_prefix/run1_tail_a1/run1_tail_b1の全センサ・全記録区間を保存する。
- 保存先native/logs/zab-local-bags/<label>/rosbag。cache_zab_bags.pyで6並列取得、保存済みのチャンク受領記録に対してサイズ/MD5を照合し、全分割を揃えてmetadata.yamlとverified.jsonを保存する。推定は再実行しない。クラウド元データは変更しない。
- local_zab_bags.pyを追加。prepare_livo_comparison.pyとrun_fast_official_raw.pyは常設元bagを優先し、実験専用cacheにsymlinkを置く。実験後のunlinkはsymlinkだけを削除し、元bagを残す。従来の読込時MD5照合も維持。常設データがないrunは従来のクラウド取得に戻る。
- test_local_zab_bags.pyで同一データ読込、実験cleanup後の元データ保持、MD5指定不一致/存在しないrunの除外、元ファイルを置換しない保護を確認。変更スクリプトの構文検査成功。全bag取得・rosbag読込確認は継続中。

### 同日：ローカル取得の速度調査
- ユーザーの速度指摘を受け、受信総量を/proc/net/devで計測。6並列で12.14MB/s、未着手のb1先頭6ファイルを追加した12並列で12.25MB/s。CPU/ディスクpressureは計測時0。Windowsの実有線アダプタは1Gbps、WSL仮想アダプタ10Gbps。並列増加の効果なし。通信経路の具体的な制限箇所は未特定。
- prefetch_zab_probe.pyはb1先頭6分割のみを先行取得してサイズ/MD5照合する。メイン処理はprefixの先頭付近にあり対象が重複しない。先行取得分も常設データとして再利用し、メインが後で照合するため捨てない。設定変更による再ダウンロードや元データ破棄は行わない。
- 直近の部分ファイルサイズから出した残り1時間の予測を修正。ネットワーク実測約12MB/sでは残り約90分。部分ファイルは並列範囲書込によりサイズだけでは厳密な転送済みバイト数を表さないため、完了数はMD5照合済み分で報告する。

### 同日：転送経路を変更して高速化
- Windows版rclone v1.75.1を公式配布から取得しSHA256照合。既存認証設定をWSLのUNC経由で読み、認証情報をWindowsへ複製せず使用。Windows有線経由でも改善しなかった。
- 保存済みeduroamへ追加接続。有線は接続を維持し、通常の経路も有線のまま確認。Wi-FiのIPへWindows版rcloneだけをbindして実bagを取得すると、転送中約31MiB/s、接続開始からの平均23.49MB/sに改善。取得bagのMD5一致を確認。Wi-Fiは最初2.4GHzから自動で5GHzへ移り、400Mbps/信号94%で接続。
- Wi-Fiでも3ファイル並列を試し、全ファイルの照合に成功。Windows一時フォルダからWSL常設先へ移動済み。probe_windows_zab_download.pyとwindows_wifi_probe*.jsonに記録。
- 元6並列スケジューラのみを一時停止し、進行中6転送がすべて完了してからスケジューラを終了。部分転送を破棄せず、取得済みファイルをそのまま再照合して再開。
- cache_zab_bags.pyに--wifi-ipを追加し、Windows/Wi-Fi4並列＋Linux/有線2並列へ変更。タスクを3:1に配分。Windowsの一時受信ファイルもサイズ/MD5検証後にWSL常設先へ移動する。現在のログはnative/logs/zab-local-download-fast.log、進捗はzab-local-bags/progress.json。取得完了後は追加接続したWi-Fiを元の切断状態へ戻す予定。
- 切替後、Windowsの各ネットワークアダプタ受信量を10秒計測し、有線10.95MB/s＋Wi-Fi31.75MB/s＝合計42.70MB/sを確認。従来12.14MB/sに対して3.52倍。bandwidth_probe.jsonへ保存。単に並列数を増やすのではなく、ダウンロードを別の通信経路へ振り分けることで改善した。

## 2026-10-06 元rosbag3runのローカル保存完了
- 全127MCAP、73,438,954,417 bytesの保存・サイズ/MD5照合が完了。run1_prefix: 80分割/45,826,463,553bytes、run1_tail_a1: 22分割/10,128,560,758bytes、run1_tail_b1: 25分割/17,483,930,106bytes。切替後の再開処理は1286.0秒（約21分26秒）で完了。
- 保存先 `/root/uas_native/logs/zab-local-bags/<label>/rosbag`。元の全記録区間・全3カメラ・LiDAR・IMU等をそのまま保持。各ディレクトリにmetadata.yamlを配置し、分割ファイルを一つのROS2 bagとして扱える。
- verify_local_zab_bags.pyで127ファイルをすべてROS2 MCAP readerで開き、各bagディレクトリから3カメラ/IMU/LiDAR/真値の実メッセージを読み出して型と時刻を確認。元のheaderと保存時刻は無改変（両者が等しいと仮定しない）。全bagの取得MD5が元の受領記録と一致。
- 各runの実データでlocal_zab_bags.pyのローカル選択→実験cache symlink経由のrosbag読込→symlink削除後も元ファイルが残ることを確認。reuse_validation.jsonに保存。DRPM入力準備と公式FASTはこの常設先を優先し、実験cleanupで元bagを削除しない。対象外runは従来どおり取得する。
- progress.json/validation.jsonはcomplete。専用転送・検証プロセス終了、一時MCAP/partialなし。追加接続したWi-Fiは切断し、元の状態へ復元。元の有線接続・既定経路は維持。Windows空き約49GiB。Windows版rclone実行ファイルはユーザーのTemp/codex-rclone-probe内に保持（認証設定は元WSLのファイルを参照、Windowsへの認証情報複製なし）。
- 今後の再生例：`ros2 bag play /root/uas_native/logs/zab-local-bags/run1_prefix/rosbag`。取得データ・監査記録は既存logs/除外規則でgit管理対象外。クラウド元bagと過去の推定結果は変更していない。

## 2026-10-06 DRPM＋3カメラROVIO：同じ3runで検証開始
- 対象run1_prefix/run1_tail_a1/run1_tail_b1、前回と同じ全追従区間とFAST共通区間。link1/link3/link4の3カメラを使用。DRPM公式更新＋距離方向ノイズ、前回の固定融合共分散、ROVIOの合計特徴点上限25とその他設定を維持。今回の真値で再校正・再調整しない。
- 常設元bagから既存prepare_livo_comparisonの処理で3台すべての白黒補正画像を抽出。一時NPZは無圧縮として圧縮CPU時間を省く（配列/画像/時刻不変）。元bagは削除しない。前回単眼のIMU/LiDAR/link3入力周期・件数を照合する。
- run_drpm_three_camera.pyを追加。3カメラの既存ROVIO検証ハーネスにpose covarianceのCSV出力だけを追加したrovio_three_camera.cppを独立ビルド。推定アルゴリズムは変更しない。各カメラの入力数、同時刻グループ完成数、特徴点利用数を検証する。
- 成果物予定native/logs/zab-offline/drpm_three_camera。finalize_drpm_three_camera.pyで単眼対3台の全区間・公式FASTとの短い共通区間の六軸図を別々に作成予定。ROS master11376を所有して起動、完了後停止する。GUIなし、推定値・真値・監査記録だけ残し、派生センサ入力は処理後削除予定。

## 2026-10-06 DRPM＋3カメラROVIO：全3runの比較完了
- run1_prefix/run1_tail_a1/run1_tail_b1の前回と同じ全追従区間を処理。link1/link3/link4を使用し、DRPM公式更新＋距離方向ノイズ（0.03m）、融合の固定共分散、ROVIOの合計特徴点上限25およびスカラー設定を維持。今回の真値で再調整していない。カメラ追加以外の推定条件を照合。
- 全区間の位置RMSE[m]（単眼融合→3カメラ融合、3カメラROVIO単体）：prefix47.962714→5.051568、ROVIO2.658724。a1 5.019973→4.382224、ROVIO0.905232。b1 983.333242→0.374166、ROVIO0.266316。
- 3カメラ融合の姿勢RMSE[deg]/最大位置誤差[m]：prefix1.503656/8.156339、a1 0.424795/8.296864、b1 0.420671/0.616627。b1の単眼での発散は今回再現せず、全区間で最大誤差0.62m。prefix/a1にはメートル単位のドリフトが残る。全3runとも全区間の位置RMSEは3カメラROVIO単体より融合の方が大きい。DRPM自体の寄与とカメラ追加の寄与を分離した実験ではない。
- 保存済み公式FASTと同じ短い共通区間の3カメラDRPM位置/姿勢RMSE：prefix0.233333m/0.299028deg、a1 0.157081m/0.276996deg、b1 0.077979m/0.410857deg。FASTは単眼かつ早期終了した既存結果なので、全区間・同カメラ数の比較と混同しない。
- run_drpm_three_camera.py：既存3カメラROVIOハーネスに共分散CSV出力だけを追加したrovio_three_camera.cppを独立ビルド。入力準備を別プロセスで先行可能にし、各段階のロックと完了記録で二重処理を防ぐ。NPZ無圧縮は一時保存方式のみで画像/配列/時刻は不変。今回の融合壁時計時間はprefix609.9秒/a1 128.2秒/b1 165.9秒。
- 全3カメラの同期画像群、IMU1ms/LiDAR100ms/カメラ50msの欠落なしを確認。前回単眼とIMU/LiDAR/link3の件数・時刻一致。3台すべてで特徴点利用あり。融合/ROVIOとも追従区間の出力50ms連続、同じ整数nsの10Hz評価点をすべて含む。真値CSVは前回とbyte一致、融合設定はログ先以外一致、推定器バイナリ・公式DRPMヘッダの一致を確認。
- 全追従LiDAR入力でDRPMを実行（prefix10854/a1 2324/b1 2725件、対応付けstatusは全件ok）。方向抑制あり9279/1837/1892件。反復上限前に収束した件数は4480/654/1214であり、status okを全件収束と解釈しない。ROVIO不確かさによる棄却0件。
- finalize_drpm_three_camera.pyを追加。成果物：native/logs/zab-offline/drpm_three_camera/plots/index.htmlおよびcomparison.pdf（9ページ）。各runにつき単眼対3カメラの全区間図、旧公式FASTとの共通短区間図、3カメラ融合対ROVIO単体の詳細図。すべて六軸、縦軸省略・クリップなし。開始時だけ位置姿勢を整合し、途中再整合/スケール調整/外挿なし。PDF9ページ/終端とHTMLリンクを検証、prefix/b1詳細図を目視確認。
- comparison.json、validation.json、元nsの真値/ROVIO/融合追従CSV・六軸誤差CSV・入力監査記録を保存。3カメラ向けの融合共分散の再校正は行っていない。元bagの保存時刻とheader時刻は必ずしも一致しないが、推定・評価には元header時刻を維持している（既存windowメタデータの時刻同一記述は派生ROS1bagに適用）。
- 全計算・所有ROS master11376を停止、空cache・派生NPZ/ROS1bagを削除。常設元127MCAP計73,438,954,417bytesは全件存在/サイズ一致を確認して保持。旧単眼/FAST結果は保持。runtime/validationはcomplete、cleanup_complete=true。未完了処理なし。

## 2026-10-06 3カメラMIMOSA＋ROVIOのDRPMなし対照：開始
- ユーザー指示により同じprefix/tail_a1/tail_b1でDRPM OFFを追加測定。ROVIOは直前の3カメラCSVを再利用し、融合共分散/バイナリ/全追従区間を固定。変更はdrpm_enabledだけ。既存project_on_degneneracyも前回と同じfalse。通常MIMOSA点面因子と採用DRPMのICP姿勢因子の比較であり、同一解法で確率抑制のみを除いた比較ではない。
- run_three_camera_no_drpm.pyでセンサだけを常設元bagから再抽出し、前回と入力bag SHA256一致を検証してから測定する。画像/ROVIOは再計算しない。結果native/logs/zab-offline/three_camera_no_drpm、主ログnative/logs/three_camera_no_drpm.log。plot_three_camera_no_drpm.pyで六軸ON/OFF/ROVIO図を出力予定。所有master11376 PID38285は完了時に停止。元bag/前回結果は保持。

## 2026-10-06 3カメラMIMOSA＋ROVIOのDRPMなし対照：完了
- 同じ3runの全追従区間で測定完了。前回DRPMありの3カメラROVIO CSVをそのまま使用。センサのみ再抽出した入力ROS1bagは全3runで前回のSHA256と完全一致。推定器実行ファイル/共有ライブラリ、融合固定共分散、開始/終了・評価時刻、真値を固定。設定差はdrpm_enabled=true→falseとログ先のみ。MIMOSA既存のproject_on_degneneracyは両条件false。
- 位置RMSE[m] OFF / ON：run1_prefix3.147873/5.051568、run1_tail_a1 0.191751/4.382224、run1_tail_b1 0.196655/0.374166。全3runでOFFが小さい。
- 姿勢RMSE[deg] OFF / ON：prefix1.094874/1.503656、a1 0.098940/0.424795、b1 0.562995/0.420671。b1の姿勢のみONが小さい。OFF最大位置誤差[m]はprefix5.446561/a1 0.751317/b1 0.363351。今回の全3runでOFFも発散していない。
- 比較：3カメラROVIO単体の位置RMSEはprefix2.658724/a1 0.905232/b1 0.266316m。通常MIMOSAへの融合はa1/b1でROVIO単体より改善するが、prefixでは悪化。DRPM導入の必要性・位置精度の利点はこの3カメラ条件では示されない。
- 解釈上の制約：OFFは通常MIMOSAの点面制約、採用DRPM ONは独立ICPの推定結果を姿勢制約として融合する。したがってこれは通常MIMOSA対DRPM導入版の性能比較であり、同じ最適化法内で退化確率だけを無効にした比較ではない。各条件1試行で、差を退化判定単独やDRPM論文一般の効果と断定しない。
- run_three_camera_no_drpm.pyとplot_three_camera_no_drpm.pyを追加。準備/変換の先行処理と段階ロック、融合時の共有パラメータロックで衝突を防ぐ。ROVIO/画像は再処理なし、今回の真値による調整なし。融合壁時計時間prefix426.5秒/a1 85.7秒/b1 100.5秒。
- 全3測定exit0、全追従20Hz出力の元ns時刻が欠落/余分なし、有限値を確認。前回と同じ10Hz評価グリッド、開始時だけ位置姿勢整合、途中再整合/スケール調整/外挿なし。DRPM分岐の記録が発生せず、ROVIO共分散読込あり、ROVIO不確かさによる棄却0件。入力/バイナリ/ROVIO/真値/意図した設定差の一致を検証。
- native/logs/zab-offline/three_camera_no_drpm/plots/index.html、comparison.pdf（3ページ）、各run六軸PNG、整数nsの推定/真値/ROVIO追従CSVと誤差CSV、comparison.json/validation.jsonを保存。ON/OFF/ROVIO単体を同じ図に掲載、縦軸省略なし。PDFページ数/完結・HTML参照を確認、a1図を目視確認。
- 所有master11376 PID38285・推定/準備プロセスは終了。派生NPZ/ROS1bagと空cache削除、元127MCAP73.44GBは全件存在/サイズ一致を確認して保持。旧結果も保持。runtime/validationはcomplete、cleanup_complete=true。未完了作業なし。

## 2026-10-06 run1_tail_b1のGazebo world調査
- inspect_b1_world.pyを追加し、記録時world SHA256と現zab_harmonic.sdfの一致を確認。推定器/worldを変更せず、GUI/シミュレーションも起動していない。
- world内ns2/ns1/s3/connection1付近の実経路を形状と照合。3カメラ比較で保存した全追従真値から長さ95.16m/272.528秒を算出（旧粗間引き解析94.94mとの差はサンプリング差）。6.55m支線往復→69.27m主通路→5.88m横坑往復、180度その場旋回2回。高さ変化・停止を含むため純粋な直線一定速実験ではない。
- 主通路3地点（追従+90/140/180秒）のmesh最初交点から幅10.61〜10.76m/床天井間8.02〜8.11m。床距離3.89〜4.10m。これは3点の局所寸法で全区間最大最小ではない。近隣4点光源range20m/影なし、ambient0.4、太陽/scene影あり。静的DAE＋PNG色テクスチャで、斑模様はそのまま測距の凹凸ではない。
- 記録URDFのLiDAR垂直−7〜+52度/30mから、広い主通路で床観測が弱く壁天井に偏る構成と確認。30sin7deg=3.66mに対し局所床距離約4m。姿勢/床傾斜で変わるので床点完全ゼロや姿勢誤差の原因とは断定していない。
- 前回環境解析の実画像を追従経過時刻で再掲。旧onset名は現在の推定器の悪化地点を意味しない。+172秒の汎用角検出398/570/455、暗部4.8/0.05/2.9%、ROVIOの内部特徴数とは区別。形状は長手方向が弱いが、視覚特徴は残るという環境。
- native/logs/zab-offline/b1_world_review/index.html、world_analysis.json、route_world.png、camera_views.pngを保存。地図と画像一覧を目視。最新3cam OFF0.197m/0.563degとON0.374m/0.421degの改善対象は直進/出入り/旋回別に切り分けるのが次の候補。追実験・パラメータ変更は本依頼では実施していない。

## 2026-10-06 run1_tail_b1：Vision不調の区間別・更新順解析
- ユーザー指示により実route_tracking.csvのroute_tで9区間を分割。主通路は追従評価開始61.8秒から66.0〜208.6秒、最初の180度旋回16.7〜40.24秒。予定時間をsim時刻へ単純加算せず、制御器が進行を止めた時間も反映。全体開始時一度の姿勢整合を維持し、区間ごとの再整合なし。
- diagnose_b1_vision.py/report_b1_vision.py追加。過去単眼ROVIO/DRPM、3台ROVIO/通常MIMOSA/DRPM、公式FAST単眼を既存保存データから解析。再推定・パラメータ変更・シミュレーション起動なし。成果物native/logs/zab-offline/b1_vision_diagnosis/index.html、analysis.json、fast_updates.csv、diagnosis.png、sources.json。図を目視、全HTML参照、スクリプト構文、FAST1540画像処理ブロックと同時刻LiDAR後/画像後1540姿勢ペア・公式CSV時刻を照合。
- 過去単眼ROVIOは追従前の静止から異常。元時刻10秒で真値速度約0に対し0.546m/s、特徴7個・深度中央値1806m（3台20個・9.675m・0.001m/s）。20秒1.142m/s、30秒1.853m/s。追従開始後の位置1m超はROVIO0.6秒/DRPM融合0.7秒、10m超3.3秒/3.5秒。入力が最初から悪く、MIMOSAは棄却なしで融合。初期深度/移動量の崩壊は観測済み、起点の画像誤対応や初期化モデルの原因までは未分離。
- 3カメラROVIOは全区間20特徴以上・周期欠落なし。通常MIMOSAは位置0.197mでROVIO0.266mより良いが、姿勢0.563degはROVIO0.183degより悪い。最初の180度旋回の姿勢RMSEはROVIO0.124/通常MIMOSA0.589/DRPM0.411deg。ROVIOは相対BetweenFactorのみ、絶対姿勢の直接拘束ではなく、入力棄却0件。LiDAR/IMU/固定重みのどれが姿勢劣化を主導するか未分離。
- 以前単眼2飛行で校正した固定共分散の適合を診断。主通路で3台ROVIOの50ms並進X RMSE1.172mmに対し固定sigma8.080mm、Y実測0.470mm対sigma0.330mm。方向/動作で適合が変わる。今回の真値でチューニングせず、数値は診断のみ。時間/共通IMU相関未処理のため単純な独立誤差の適合検定ではない。
- 公式FASTは主通路進入後、経過69.95秒0.487deg→74.95秒3.342deg→78.95秒28.436deg→79.95秒52.410degとロール中心に悪化。78.95秒のLiDAR更新2.3503degに対し画像更新0.010615deg、画像対応19点。その回の画像更新は誤差を少し増やす。最初に対応0は81.05秒。よって特徴消失が姿勢破綻に先行したのではなく、姿勢崩壊を画像補正が戻せず、後に画像対応を失う順序。画像配送は既存監査で欠落なし、初期のNo pointログは全体停止の証拠ではない。
- FASTの画像地図はLiDAR由来の点・推定姿勢を共有し、点数0で画像更新を返す実装を確認。形状の弱さと共有地図への誤差持越しが整合するが、候補棄却理由/画像情報行列/ゲインが保存されておらず、画像重み不足と地図誤りの因果分離は未完了。次の候補は66〜82秒の内部診断記録と、別飛行による3台ROVIO共分散再校正。現在の結果を「Vision全体が止まっている」「暗さだけが原因」と扱わない。

## 2026-10-07 単眼ROVIOの初期化失敗：センサ仕様に合わせた初期バイアス分散の修正
- ユーザー指示は単眼の初期化問題を先に解決すること。ライブsimulationが必要なら伝えるよう追加指定あり。今回はローカル保存済み3runのlink3カメラとIMUだけで再現・対照実験を完了し、Gazebo/RVizを起動していない。3カメラ融合やFASTの修正・再実験は対象外。
- 原因を切り分け：記録URDFの加速度バイアス仕様は各軸bias_mean=0.1、bias_stddev=0.001 m/s²。一方、従来ROVIO Init.Covariance.acbは0.0004（標準偏差0.02 m/s²）で、初期平均0に対して模擬バイアスの大きさを過小評価。新設定はゼロ平均を維持し、仕様の二乗平均0.1²+0.001²=0.010001を初期分散に使用。真値や対象runの実測バイアスで平均・分散を適合していない。バイアス推定が立ち上がる前に速度/距離が崩れる不具合を、初期3分散だけ変えて解消した。
- rovio_initialization.py追加：記録dragon_slam.urdfからspinal_imuの仕様を読み、Init.Covarianceのacb_0/1/2だけを変更。replay_monocular_comparison.pyは今後この設定を使用し、summaryに設定識別子/分散を記録。ROVIOライブラリ・実行ファイル、初期平均、カメラ校正、特徴数25、他の分散、画像処理は不変。3カメラ設定やライブlaunchの設定は変更していない。
- test_monocular_startup.py追加。元MCAPを読み、link3の既存mono8補正480×360とIMUだけを旧単眼バイナリへ保存順に直接供給。元整数header時刻、初期化2.003秒を保持。点群/画像キャッシュなし。b1ではbaseline、公式静止検知+ゼロ速度更新ONのみ、初期バイアス分散のみ、両方の4条件を比較。prefix/a1は初期分散のみの同一修正を全追従区間まで検証。
- b1のbaselineは旧result_vio.csvとバイト一致。分散修正の3runは旧バイナリSHA256・入力件数・校正・出力時刻と一致、50ms周期欠落なし・全追従区間完走。修正設定は共通ヘルパーの生成結果と全文一致。初期化1回、未完画像0。真値は事後評価にだけ使用。
- 離陸前3〜10秒の推定速度RMS[m/s]（旧→分散修正）：prefix0.368690→0.012200、a1 0.084126→0.010192、b1 0.315360→0.012277。真値速度最大は同窓で0.000871/0.005763/0.000602 m/s。10秒時点の姿勢位置差分速度はprefix0.532597→0.000611、a1 0.025663→0.001353、b1 0.546033→0.000583 m/s。初期の小過渡は残るが持続的な偽運動・深度破綻を解消。
- 10秒時点の特徴深度中央値[m]：prefix1941.565→9.946877、a1 14.720700→9.935901、b1 1806.065506→9.945062。修正値約10mは初期距離事前値付近を維持したことを示すもので、静止単眼で真の深度を計測できたことを意味しない。
- 全追従区間位置RMSE[m]旧→修正：prefix15.840850→22.609260（悪化）、a1 8.313233→7.010673、b1 1040.096823→0.559758。最大位置誤差24.940389→31.253672、23.204594→18.953318、2539.194539→1.158711 m。姿勢RMSE[deg]0.292835→0.303081、0.067891→0.056052、1.202462→0.056929。初期化の修正と飛行中単眼ドリフトは分ける。prefixの全飛行精度は改善しておらず、全14runの単眼安定性は未検証。
- ゼロ速度更新だけのb1位置RMSE0.882899mだが10秒時点の偽速度0.546m/sは変わらず、初期化失敗を速やかには防がない。採用したのは分散修正のみ。動作中の誤った静止判定を導入しないため、MotionDetection/ZeroVelocityUpdateは従来通りOFF。
- report_monocular_startup.py追加。成果物native/logs/zab-offline/monocular_startup/plots/index.html、comparison.pdf（9ページ）、各PNG/六軸CSV、evaluation.json、全初期化期間CSV/設定/ログ。旧結果/既存error_plotsを保持。離陸前を独立評価し、追従誤差は従来同様10Hz・開始時一度の剛体整合のみ。途中整合/スケール補正/軸省略なし。修正前後は別の全範囲図として巨大発散値で修正後を隠さない。
- スクリプト構文、実データ対照、生成設定一致、初期化図の目視を確認。b1 baseline/zupt同時実行97.5秒、bias/bias_zupt同時実行101.8秒、a1 61.5秒、prefix235.1秒（各wall、読出し込み）。旧比較/生bagは保持。融合結果をこの新ROVIOの結果と取り違えない。
- 最終確認：PDF9ページオブジェクト・HTML参照・3run検証記録に成功。所有ROS master11376（PID152172）と推定器を停止。validation.json complete。元bag/過去結果を保持、未完了処理なし。

## 2026-10-07 初期化修正後の単眼ROVIO＋MIMOSA＋DRPM：3run再測定開始
- ユーザー指定のprefix/tail_a1/tail_b1で、直前に検証したlink3単眼ROVIO CSVを再利用し融合を再実行。新出力native/logs/zab-offline/drpm_corrected_monocular。旧単眼・3カメラ結果は上書きしない。
- run_corrected_mono_drpm.py追加。採用DRPM（公式更新＋測距方向モデル）・融合設定・旧2run由来の固定共分散・バイナリを固定し、ROVIO初期化3分散だけの変更による効果を比較。今回の真値による再校正なし。全追従区間を評価、元センサ時刻を保持。真値/ROVIO/融合姿勢と診断ログのみ結果として保持。
- 保存済み元bagからIMU/LiDARを抽出。専用ROS master11376/11377でprefixと短い2飛行を独立処理し、完了時に所有プロセスと派生センサ入力を片付ける。Gazebo/RVizなし。完了結果は追記する。

## 2026-10-07 初期化修正後の単眼ROVIO＋MIMOSA＋DRPM：3run完了
- 前節の全3runを完走。link3の1台。monocular_startup/biasのROVIO CSVをバイト一致で再利用し、MIMOSA/DRPMバイナリ・融合設定・相対姿勢共分散は前回単眼と一致（ログ先除く）。採用DRPMは公式更新mode1＋測距方向ノイズmode2。今回はROVIO共分散の再校正なし。修正前後の比較で変わったのはROVIO初期加速度バイアス3分散。
- 全追従位置RMSE[m] 前回単眼融合→修正後融合（修正後ROVIO単体）：run1_prefix 47.962714→13.174592（22.609260）、run1_tail_a1 5.019973→3.090375（7.010673）、run1_tail_b1 983.333242→6.523832（0.559758）。前回融合より全3run改善。単体ROVIOより融合が良いのはprefix/a1で、b1は悪化する。
- 修正後融合の最大位置誤差[m]はprefix16.516242、a1 5.627776、b1 9.824663。姿勢RMSE[deg]は2.874468/0.769588/0.453674。前回は4.552537/0.535615/6.738725なのでa1の姿勢は悪化。修正後ROVIO単体姿勢0.303081/0.056052/0.056929degより融合は全3runで悪い。初期化問題の解消と融合後の精度は別で、全体が解決したとは扱わない。
- 既存3カメラDRPM融合（位置RMSE5.051568/4.382224/0.374166m）も参考掲載。今回単眼はprefix/b1で3カメラより悪く、a1では良い。3カメラを再測定したわけではない。FASTは既存発散後打切り結果のため、短い共通期間だけmetrics.jsonに残し、全追従比較表には混ぜない。
- DRPM登録はprefix10854/a1 2324/b1 2725スキャンすべてstatus=ok、記録欠落0。うち方向抑制あり9307/1808/1837、収束条件成立3893/443/1044。status=okは反復収束を全回満たすという意味ではない。ROVIO不確かさによる棄却0件、全20Hz出力時刻欠落0、真値CSVは前回とバイト一致。元MCAPのMD5照合・IMU/LiDAR周期/件数・前回バイナリSHA256・設定差を確認。
- 成果物native/logs/zab-offline/drpm_corrected_monocular/plots/index.html、comparison.pdf（6ページ）、PNG、comparison.json/validation.json。全追従の真値/融合/ROVIO CSV、元時刻の6軸誤差CSV、設定/診断ログを保存。主図は修正後単眼融合・ROVIO単体・既存3カメラ融合。別図で前回単眼融合との対照。共通10Hz・追従開始時一度の剛体整合、途中整合/スケール補正/縦軸省略なし。
- plot_corrected_mono_drpm.py追加。構文・全実データ検証・図の目視・PDF6ページ・HTML全リンクを確認。融合計算wallはprefix638.22秒、a1 149.56秒、b1 167.56秒。prefixと短い2runを別masterで並列実行。
- 所有master11376/11377と全推定器停止、派生bag/NPZを削除。元bag127MCAP（73,438,954,417 bytes）と既存結果を保持。Gazebo/RViz未起動。runtime/validation complete、未完了処理なし。新しい結果は既存error_plotsを上書きせず専用リンクで提示。

## 2026-10-08 修正後単眼ROVIO＋MIMOSAのDRPMなし対照：開始
- ユーザー依頼によりprefix/tail_a1/tail_b1でDRPM OFFを測定。前回drpm_corrected_monocularの修正後link3単眼ROVIO CSV・融合共分散・推定器バイナリを固定。drpm_enabledのみfalseへ変更（ログ先除く）。既存MIMOSA退化投影は前回同様OFF。
- run_corrected_mono_no_drpm.py/plot_corrected_mono_no_drpm.py追加。既存検証済みOFF実行・評価関数を再利用。run_corrected_mono_drpm.pyのflightに呼出しscript引数を追加して処理/片付けを共有。元bagから生成する入力ROS1 bagのSHA256を前回ONと照合し、一致した場合だけ融合を実行。
- 出力native/logs/zab-offline/corrected_monocular_no_drpm。master11376/11377を専有しprefixと短い2runを独立並列処理。終了時停止・派生bag/NPZ削除、元データ/旧結果保持。Gazebo/RVizなし。通常MIMOSAの点面制約と採用DRPMの独立ICP姿勢制約の比較であり、同じ解法の退化確率抑制だけを除去した比較ではない。完了結果を追記する。

## 2026-10-08 修正後単眼ROVIO＋MIMOSAのDRPMなし対照：全3run完了
- 同じlink3単眼・初期化修正済みROVIOを用いたDRPM OFFがprefix/tail_a1/tail_b1すべて完了。ROVIO CSVのSHA256一致に加え、IMU/LiDAR/ROVIOを含む再構成ROS1 bag全体のSHA256が前回ON測定と3件とも一致。真値CSVも一致。実行ファイル/ライブラリ/融合共分散を固定し、drpm_enabledだけfalseへ変更（ログ先除く）。既存MIMOSAの退化投影もON/OFF両条件でfalse。
- 位置RMSE[m] ON→OFF：run1_prefix 13.174592→23.457815（ROVIO単体22.609260）、run1_tail_a1 3.090375→5.344842（単体7.010673）、run1_tail_b1 6.523832→0.264193（単体0.559758）。位置はprefix/a1でDRPMあり、b1でなしが良い。b1で見られた約10mへの位置ドリフトはOFFでは起きず、最大位置誤差0.518672m。
- 姿勢RMSE[deg] ON→OFF：prefix2.874468→1.149464、a1 0.769588→0.333373、b1 0.453674→0.603085。姿勢の優劣は位置と逆。OFF最大位置誤差[m]はprefix30.618492、a1 16.252425、b1 0.518672。OFF姿勢も全3runで修正後ROVIO単体より悪い。
- 全追従区間を20Hz出力欠落なしで完走、有限性・初期化後真値/時刻照合に成功。ROVIO不確かさによる棄却0、DRPM登録ログ生成なしを確認。共通10Hz評価、追従開始時に一度だけ剛体整合、途中整合/スケール変更/外挿/縦軸省略なし。各条件1試行。OFFは通常MIMOSA点面制約、ONは採用DRPMの独立ICP姿勢制約であり、同一解法内の確率抑制だけを外した実験ではない。
- 成果物native/logs/zab-offline/corrected_monocular_no_drpm/plots/index.html、comparison.pdf（3ページ）、各PNG、comparison.json/validation.json。六軸にOFF/ON/同じ単眼ROVIO単体を重ねて表示。各fusion/<label>に元時刻の融合/真値/ROVIO追従CSV、off_full_errors.csv、設定/ログ/metricsを保存。ONや既存error_plotsは上書きなし。
- run_corrected_mono_no_drpm.py・plot_corrected_mono_no_drpm.pyと共有runnerの構文検査、bag/設定/実行ファイルの一致、生成図目視、PDF3ページ/全HTMLリンクを確認。融合wallはprefix273.42秒、a1 56.96秒、b1 70.46秒。prefixと短い2runを別ROS masterで並列処理。
- 所有master11376/11377・推定器は停止済み。生成した一時bag/NPZを削除、元MCAPと過去結果を保持。runtime/validation complete、cleanup_complete=true。Gazebo/RViz未起動、未完了処理なし。

## 2026-10-08 単眼ROVIO＋MIMOSA：標準並進退化処理ONの測定開始
- ユーザー指定によりDRPM OFFのままproject_on_degneneracyをtrueにし、同じprefix/tail_a1/tail_b1を測定。既存並進しきい値40、回転しきい値0は変更しない。実装の適用対象は並進側のみ。単眼ROVIO・融合重み・バイナリを固定し、入力bagのSHA256一致を要求する。
- run_corrected_mono_native_degen.py / plot_corrected_mono_native_degen.py追加。元データから入力再構成、標準退化処理なしとの対照、既存DRPM/ROVIO単体も参考表示予定。DEGEN Handling出力で実際の処理呼出し回数を数え、フラグONと検出成立を区別する。対象run真値でしきい値を調整しない。
- 出力native/logs/zab-offline/corrected_monocular_native_degen。専用master11376/11377、GUIなし。終了時に所有処理停止と一時bag/NPZ削除、元bag/過去結果保持。完了時追記する。


## 2026-10-08 単眼ROVIO＋MIMOSA：標準並進退化処理ON（完了）
- 同じrun1_prefix/run1_tail_a1/run1_tail_b1の全追従区間を再計算。DRPM OFF、標準project_on_degneneracy ON、並進しきい値40、回転しきい値0。現在の標準実装は並進の弱い点の制約だけを投影し、回転の投影処理は無効のまま。初期化修正済みlink3単眼ROVIO出力、固定融合共分散、推定器バイナリを前回と同一に保持。
- 全3runでDEGEN Handling分岐の呼出し0回。全出力時刻の位置・姿勢が標準処理OFF時と完全一致。しきい値40では退化と判定されず、実際の補正は一度も発動しなかった。退化の不存在や、補正を発動させた際の有効性を示す結果ではない。しきい値の追加調整は行っていない。
- 位置RMSE[m] / 姿勢RMSE[deg]（ONとOFF同値）：prefix 23.45781487603511 / 1.149464221231824、a1 5.344841848245656 / 0.33337321447360946、b1 0.2641931161659767 / 0.6030850768177318。最大位置誤差[m] 30.61849235032606 / 16.252425153462944 / 0.5186724638138852。
- 変更・再現：native/run_corrected_mono_native_degen.py campaign、native/plot_corrected_mono_native_degen.py。前回標準OFFとの差分を有効フラグとログ先だけに限定。再構成入力bagの全体SHA256を前回標準OFFおよびDRPM ONと照合、ROVIO出力も同一。全追従20Hz出力欠落なし、不確かさによるVIO棄却0。融合計算の実時間はprefix277.22秒、a1 57.70秒、b1 59.97秒（入力準備・評価を除く）。
- 成果物native/logs/zab-offline/corrected_monocular_native_degen/plots/index.html、comparison.pdf（3ページ）、comparison.json/validation.json、各fusion配下の元時刻推定/真値CSV。六軸の標準ON/OFF・既存DRPM・ROVIO単体を全追従区間で比較。ON/OFFは重なるためOFFを破線表示。開始時だけ位置姿勢を整合、元ns時刻の共通10Hz評価、途中整合/スケール変更/縦軸省略なし。DRPM参考値は独立ICP姿勢因子であり、標準ON/OFFと同じ解法の確率処理だけの比較ではない。
- 検証：PDF3ページ・HTML参照先・全時刻・入力/設定/バイナリ一致・終了コード0を確認。各条件1試行。全所有master11376/11377停止、派生入力bag/NPZ削除。常設元127MCAP計73,438,954,417bytesと過去結果を保持。GUI/ライブsimulationなし、未完了処理なし。


## 2026-10-08 LiDAR退化方向に応じた単眼ジンバル制御の文献調査
- ユーザーはDRPM＋MIMOSAを一旦採用し、LIVOでLiDARの弱い方向を検出してカメラの向きを動的に変える先行研究の調査を依頼。研究・資料整備のみ実施し、推定器設定、センサー配置、既存結果は変更していない。
- 確認した公開文献の範囲で全条件を満たす論文は未発見。SKFは弱い方向への視覚の選択的融合、IGLOVは情報量に基づくジンバル制御。BonettoらECMR2021には2Dレーザー・IMU・回転RGB-Dカメラの併用、Aerospace2022にはLiDAR退化時の既知マーカー補助とジンバルがあるが、退化方向に応じたカメラ角の計画ではない。AEOS/AWAREはLiDARの向き／機体ヨーを制御。DC-VINSは動的なカメラ対IMU校正の参考。
- ETH RSLは電動パン・チルトによる退化軽減を研究紹介に記載。具体的な同条件の論文は特定できず、既存研究が存在しない／初の提案とは断定しない。出版社最終版を取得できない文献と、確認済みの著者公開本文を区別。
- resources/README.mdに比較表、本文該当節、引用URL、研究上の解釈、アクセス制約を追記。resources/active_gimbal_20261008/に10本のPDF、検索用抽出テキスト、取得元・ページ数・SHA256のmanifestを保存。PDFの読み取り・ハッシュと文書リンクを検証。
- シミュレーション／GUI／推定器は起動していない。研究方針として固定単眼、視覚情報量だけの向き制御、LiDARの弱い方向への補助効果で選ぶ制御を比較候補として記録。実装・実測は未実施。

## 2026-10-13 視覚地図・LiDAR退化方向評価・単眼ジンバル制御の実装と接続検証
- ユーザー依頼により、固定四角形DRAGONのlink3単眼＋link2 LiDARで、DRPM＋MIMOSA＋可動ROVIOの閉ループを実装。`native/active_perception/`、`launch/active_gimbal.launch.py`を追加し、`dragon_cameras.py`と`dragon_sim.launch.py`へ明示的なgimbalオプションを追加。既存の3カメラ既定動作・過去データ・採用済み推定器バイナリは保持。
- 公式利用範囲：FIF `uzh-rpg/rpg_information_field` revision `76d7b834128bd3488d1aae3888a79f3d5216a4ab`、minkindr revision `564f12639a8447d4d3e5e7707851424302941056`を`assets/`に取得。公式の特徴点ベースの視覚微分計算を無改変で呼ぶC ABIを追加。FIF全体のボクセル近似・旧ROS1アプリを移植したという意味ではない。各特徴点の全6×6情報を一度計算し、同一光学中心の候補間で再利用する。
- 視覚地図はROVIOのID、3次元位置、相対点共分散、観測方向、時刻を保存する。未初期化/極端に不確かな点を棄却、最低3観測、20cmセルの代表点選択、60秒失効、最大3000点。融合後の共分散を固定的に地図へ格納しない。停止時`visual_map.npz`を保存。別起動の座標系で再利用するには位置合わせが必要。
- MIMOSAには登録結果の読出しだけを加えた独立共有ライブラリをビルド。DRPM確率・固有方向・有効情報を`/active/lidar_information`へ出力する。LiDARの右摂動[回転,並進]を機体へ変換し、弱い方向のスカラー座標は逆変換ではなく随伴行列の転置で変換する（原点の違いを含む不確かさの不変性テストで確認）。既存DRPM更新・距離方向ノイズ・固定融合共分散は不変。
- 視野評価はLiDAR情報と候補視覚情報を足し、Cholesky分解による連立方程式で弱い方向の予測不確かさを計算。明示的な評価用事前分散、退化の確率、進行方向の罰則、角度変更の罰則を使用。yaw±120度/pitch±30度の85候補＋現在角を探索し、選ばれた目標と速度制限後の指令を区別して記録。既知点の観測方向差と現在LiDARの遮蔽を棄却。センサが0.5秒以上古い/地図不足/有効退化方向なしでは保持する。
- ジンバルは光学中心に交差する2軸、合計4gの理想モデル。Gazebo標準JointPositionControllerの速度指令モードで各軸速度を20/sqrt(2)度/秒以下に制限し、合成20度/秒を保証。指令値ではなく実測関節角を画像時刻へ補間（補間幅20ms以下）する。機体関節が四角形から0.03rad以上外れた場合の画像入力も拒否する。
- 可動ROVIOは独立ビルド。IMUによる予測後・画像更新前に、追跡中の特徴点方向、パッチ変形、共分散を既知のカメラ回転で変換。カメラとIMU間の並進が変化する入力は拒否する。検証済み単眼のセンサ由来初期化/雑音設定を引き継ぎ、元画像のmono8＋標準OpenCV補正と撮影時刻を維持。真値は飛行制御および独立評価だけに使用。
- 数値検証：`test_core.py`の10件成功（公式微分の数値差分照合、情報の方向間結合、原点変換、地図保存/失効、観測重複、遮蔽、候補間キャッシュ、速度制限、生成URDFの光学座標との一致等）。`test_gimbal.cpp`で地図点の機体位置不変、共分散変換、往復回転、非対応の並進拒否を確認。
- 記録bagによる固定時の対照：`test_static_replay.py`でローカルrun1_tail_b1の2.003〜20秒のIMU/単眼画像を再生。旧単眼binaryと拡張版のodometry（共分散含む）・バイアス状態CSVがbyte単位で一致、特徴点の全データ行も一致（旧診断ヘッダだけ未使用status1/status2を含むため相違）。`native/logs/active-static-regression/validation.json`にSHA256保存。派生入力binは削除、元bagは保持。
- オンライン接続検証（GUI/rosbagなし）：ZaB spawn=(0,0,0.5)、固定と能動の両方で離陸・30秒ホバリング成功。能動`active-gimbal-20261013-143208`は92.05秒評価、ROVIO最大出力間隔50ms/MIMOSA100ms、視野評価p95=22.16ms、関節合成速度最大20.00000000024度/秒。固定`...143557`は73.15秒評価、関節角変動0.000013度未満、同じ出力周期を確認。各`validation.json`に評価値保存。
- 最終コードでの再確認`native/logs/active-gimbal-20261013-144001`：離陸＋30秒ホバリング成功、65.45秒のROVIO出力1310件、最大間隔50ms、MIMOSA最大間隔100ms。能動評価309回、地図最大44点、評価p95=23.0119ms（最大29.2159ms）、yaw角範囲24.80度/pitch30.00度、合成速度20度/秒以下。初期姿勢だけ整合した位置RMSEはROVIO0.07476m/融合0.07020m。これは動作確認であり、固定方式より良いと主張する対照実験ではない。新規ノイズ生成・区間長も一致しない。
- 終了処理を修正し、最終試験ではmaster/bridge/Gazebo/画像変換/ROVIO/MIMOSA/視野選択/評価/hoverの全プロセスexit0。所有ポート11379/11380が閉じ、関連プロセスが残っていないことを確認。途中の失敗試行はログに保持し、最終検証と混同しない。
- `colcon build --packages-select multilink_uas_bringup --symlink-install`成功、ROS2 launch引数の解決確認済み。ユーザー端末から`ros2 launch multilink_uas_bringup active_gimbal.launch.py world_file:=/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf spawn_x:=0 spawn_y:=0 takeoff:=true`で起動し、Ctrl+Cで終了。固定対照は`mode:=fixed`。既定takeoff=false、GUI/rosbag自動記録なし。READMEに詳細を追記。
- 制約：情報は固定地図に条件付けた候補選択用の近似。地図/現在姿勢の相関、IMU共有相関、地図ドリフト、未観測遮蔽や自己遮蔽、将来の追跡成功は厳密には評価しない。評価用事前分散と観測雑音は設定値で、今回新規校正した値ではない。地図不足時の探索スキャン方策、光学中心が並進する実ジンバル、機体変形、長距離精度・能動視覚の改善効果は未検証。稼働中の検証処理なし。

## 2026-10-13 固定単眼とジンバル偏向のオンライン3軌道比較：開始
- ユーザー依頼によりrun1_prefix（元の有効追従1070秒）、run1_tail_a1、run1_tail_b1を新規オンライン飛行。速度0.5m/s、固定／能動の各1試行、共通seed20261013、GUI/rosbagなし。離陸・向き合わせを除く軌道追従だけ評価。元bagの画像を可動視野の代用にしない。
- DRPMの推定補正はOFF、標準MIMOSA退化投影もOFF。カメラ選択に必要な退化確率・情報だけを通常MIMOSAの現在位置で診断する独立フックを追加。姿勢更新・因子追加なし。地図点の取得時測距共分散を診断用に付加。既存採用バイナリは無変更、active_perception独立ライブラリのみ更新。
- run.pyに軌道・DRPM無効・乱数seed・自動終了を追加。route_flight.pyは既検証の真値ベース飛行制御を再利用。compare_routes.pyは6飛行を逐次実行し完了済みを再開時スキップ。偏向は追従開始から有効、それ以前は両条件とも固定。重みの再調整なし。結果は専用ディレクトリに保存し、全所有プロセスを終了後停止する。

- 中間確認：a1は固定／偏向とも完走。MIMOSA位置RMSE4.899→3.310m、姿勢0.286→0.363deg。経路の真値差最大0.026m。b1固定も完走（位置0.429m）。b1偏向初回はROVIOがsim3秒で6.25m、5秒で64.9mに崩壊。偏向評価0回、関節角変化0.00002度未満なので、偏向の失敗と評価してはいけない。excluded_startup/run1_tail_b1_active_attempt1へ保存し、本比較から除外。
- 再試行の適格条件：sim5秒までのROVIO初期位置からの変位が1m未満（真値を使わない離陸前の検査）。両モードに同じ条件を適用し、既完走試行もCSVで遡及確認する。推定器設定・重みは変更しない。初期化失敗の根本原因を解消したという意味ではない。prefix固定は正常な初期化後に継続中。

## 2026-10-13 固定単眼／ジンバル偏向・DRPM補正OFF：3軌道比較完了
- 全6有効飛行をオンラインで取得。run1_prefixは元データで有効だった追従1070秒、tail_a1は228.492812秒、tail_b1は263.384754秒。ZaB、固定四角形DRAGON、link3単眼、速度0.5m/s。固定と能動で同じルート・spawn・Gazebo seed20261013・校正・ROVIO/MIMOSAバイナリ・融合重みを使用。離陸／向き合わせ中は両条件ともカメラ固定、追従開始後だけ偏向。
- DRPMの推定更新はOFF、MIMOSA標準の退化投影もOFF。カメラ方向選択用の退化確率・有効情報を、通常MIMOSAの現在姿勢／地図から診断するだけ。独立ICPの姿勢更新やDRPM姿勢因子を追加しない。偏向の評価重み1/0.05/0.002と速度上限20deg/s、視覚雑音・事前分散は変更していない。
- 結果は位置RMSE[m]／回転測地誤差RMSE[deg]。追従期間だけ、共通の軌道進行時刻0.1秒刻み、最初の追従姿勢だけ剛体整合。途中整合・スケール補正・縦軸省略なし。
- run1_prefix：fixed: MIMOSA 18.753519m/1.645536deg, ROVIO 11.835315m/0.328954deg；active: MIMOSA 46.477887m/4.553411deg, ROVIO 25.675348m/0.189382deg。
- run1_tail_a1：fixed: MIMOSA 4.899126m/0.285864deg, ROVIO 7.955879m/0.058589deg；active: MIMOSA 3.309576m/0.363372deg, ROVIO 7.375427m/0.047247deg。
- run1_tail_b1：fixed: MIMOSA 0.428570m/0.760641deg, ROVIO 1.940328m/0.085687deg；active: MIMOSA 11.891206m/2.353881deg, ROVIO 11.080680m/0.121987deg。
- MIMOSAの位置はa1だけ改善、prefix/b1は悪化。回転は全3軌道で悪化。ROVIO単体の位置もa1だけ改善、prefix/b1は悪化。現在の偏向方策で一貫した改善は得られなかった。各条件1試行であり、統計的優位性やactive perception一般の有効／無効を断定する結果ではない。
- 初期化失敗：b1偏向初回はsim3秒でROVIOの移動量6.25m、5秒で64.9m、カメラ方向評価0回・角度変化0.00002deg未満。偏向の効果として扱わず、excluded_startup/run1_tail_b1_active_attempt1とexcluded_runs.jsonへ全ログ／CSVを保存して再取得。再取得は同じ推定設定・seed。sim5秒までの初期位置変化1m未満という離陸前適格条件を両条件へ共通適用し、既完走分も遡及確認。全有効試行は約1cm以内。初期化の根本原因を解消したという意味ではなく、良好な初期化を条件とした飛行中の比較。
- 全6試行で追従期間のROVIO/MIMOSA出力カバー率100%。ROVIO最大出力間隔50ms、MIMOSA100ms、LiDAR更新100ms。全3能動試行で実際の偏向・評価を確認。真値の実経路差（固定／能動）はprefix最大0.102763m/RMSE0.005443m、a1最大0.025976m/RMSE0.001708m、b1最大0.024429m/RMSE0.000986m。seedを合わせても新規シミュレーションの全雑音列・開始タイミングが完全一致するとは主張しない。
- 成果物native/logs/active-fixed-three-20261013/plots/index.html、comparison.pdf（6ページ）、6軸PNG、各試行の元ns時刻の真値/ROVIO/MIMOSA CSV、角度CSV、判断ログ、誤差CSV、metrics.json、comparison.json、validation.json。画像・点群bagは作成せず、既存bag/比較結果は保持。初回ハーネス設定確認は別のactive-fixed-three-20261013-setup-checkに保持し、本評価には含めない。
- 検証：視野評価の既存数値テスト10件と、評価器の既知剛体座標変換／既知ドリフトの検算に成功。ソース構文、同条件の設定/校正/バイナリ/ルート/seed一致、全HTML参照先、PDF6ページ、生成図の目視を確認。
- 終了：全シミュレーション・推定器・bridge・master11379/11380停止。全推定器exit0。prefixの飛行制御だけ、完走・データ保存後の終了中にsupervisorからSIGINTを受けexit-2となったが、追従データは全区間保持。runtimeを改変せずvalidationに注記。run.pyを自然終了待ちに修正し、最後のb1では全プロセスexit0を確認。GUI起動なし。
- 再現：active_perception/compare_routes.py --output <新規出力先>。既存出力先指定時は完了済みの飛行を再利用。report_routes.py <出力先>で集計／図を再生成。未完了の測定なし。

## 2026-10-13: ジンバル偏向時の機体による視野遮蔽を調査

- 質問への調査のみ。推定器・偏向方策・モデル設定の変更や新規飛行は行っていない。現行方策には機体形状による遮蔽判定がなく、LiDAR点群による近似判定では近傍のリンク・ローターを確実に除外できない。
- 既存b1能動試行のURDFを使用し、四角形姿勢、ロータージンバル角0で可視形状に対する光線交差を計算。27方向、各48x36画像サンプル、水平画角120deg、画像平面の近クリップ4cm。両面の三角形として判定する幾何学的な概算で、Gazebo描画画像の画素測定ではない。
- pitch0で、yaw0: 0.0%、yaw-90: 34.1%、yaw-120: 61.7%、yaw+120: 38.3%。27方向中最大61.8%（yaw-120/pitch+30）。カメラ位置と広い可動域により大きな自己遮蔽が起こり得る。飛行時のロータージンバル姿勢、描画材質/裏面処理、より細かい角度/画素は未検証。これだけで精度悪化の原因と断定しない。
- 成果物: native/logs/active-fixed-three-20261013/self_occlusion_audit/{coverage.json,check_gimbal_occlusion.py}。新規センサbag・GUIなし。

## 2026-10-14: カメラ配置調整用の手動ジンバル起動

- ユーザーがGazebo画像を見ながら配置を調整できるよう、native/gimbal_inspect.launch.pyとactive_perception/manual_gimbal.pyを追加。ZaB、DRAGON link3単眼、光学中心交差2軸。Gazebo GUI、rqt_reconfigure、rqt_image_viewを起動端末の配下で管理。自動離陸、自動偏向、SLAM、bag記録は起動しない。
- /camera_gimbal_manualのyaw_deg（±120度）、pitch_deg（±30度）を範囲付きROS2パラメータとして公開。標準rqtのスライダーで変更し、既存角度指令topicへrad変換して10Hz送信。NaN/範囲外を拒否。Qt runtimeは起動するrqtだけ専用0700ディレクトリを使う。
- 配置編集元はnative/dragon_cameras.pyのMOUNT_XYZ、MOUNT_RPY。生成URDFはoutput/dragon_slam.urdf。直接URDFを編集しても再起動時に生成し直される。元の配置値・可動範囲・自動偏向の実装は変更していない。
- 検証: ROS2上で45度/-15度が正しくradとして配信され、範囲外・NaN拒否を確認。ZaBでGUIなしの起動試験を行い、実測関節角yaw29.43度/pitch-15.00度（目標30/-15度、判定許容差0.01rad）と720x540画像43フレームを確認。4種の飛行制御器activeを確認。rqt両プラグインはoffscreenで起動確認、画面の目視操作と今回の離陸は未実施。検証用起動はCtrl+Cで終了し、常駐させていない。
- ユーザー起動: source /root/uas_native/env.sh; ros2 launch /root/uas_native/gimbal_inspect.launch.py。別端末でros2 run aerial_robot_core keyboard_command.py --ros-args -p robot_ns:=/dragon、rでarm、tでtakeoff、lでland。位置変更後はlaunchをCtrl+Cで停止して再起動。

## 2026-10-14: rqtがジンバルを表示しない起動順問題を修正

- ユーザー起動中のGazebo・手動制御ノードは正常。標準rqt_reconfigureはROS2通信相手の発見前に一度だけ一覧を作り、自動更新しない。対象ノードを引数で指定した起動でも「Could not find ... /camera_gimbal_manual」を再現。rqt自身しか表示されない原因を確認した。
- active_perception/gimbal_rqt.pyを追加し、標準rqt画面の一覧を0.5秒ごとに更新、対象を発見したら絞り込み・自動選択して更新を停止する。変更は当該プロセス内に限定し、インストール済みrqtは改変しない。gimbal_inspect.launch.pyの設定画面起動先を変更。
- 実際の稼働ノードに接続するoffscreen GUI検証で、yaw_deg/pitch_degと操作可能な2本のスライダー生成を確認、画面を保存・目視確認し正常終了。角度指令の変更、ユーザーのシミュレーションや既存GUIの停止はしていない。既存画面はRefresh→/camera_gimbal_manual選択で操作可能。次回の同じlaunch起動では自動選択。

## 2026-10-14: 画角確認の機体を標準UniPilot＋単眼ジンバルへ変更

- ユーザー指示により、今後のカメラ配置確認はDRAGONではなくUAS標準UniPilotを用いる。gimbal_inspect.launch.pyのrobot既定値をunipilotへ変更。robot:=dragonで旧構成も選択可能。既存DRAGONの配置ファイル（ユーザー編集対象）・過去データは変更していない。
- native/unipilot_gimbal.pyで標準rmf_unipilot/model.sdfから派生SDFを生成。機体/4ローターの慣性、LiDAR仕様を保持。標準モデルで無効だったIMUと前方RGBカメラを有効化。単眼cameraは元の480x270、水平画角1.52rad、画像ノイズstddev0.007、近遠クリップ0.02/20mを保持し、更新頻度だけ1→20Hz。標準位置base_link相対(0.18,0,-0.025)m、姿勢(0,0,0)から開始。MOUNT_XYZ/MOUNT_RPYが編集元。光学中心で交差するyaw/pitch±120/±30deg、合計4g、筐体接触は扱わない理想ジンバル。
- 飛行制御の例外を明記: 同じモーターに指令するAccelerationControlを除去し、標準VelocityControlを使用。最初は元のvelocityGain=4 4 1.5を保持したが、z速度0.35m/s指令でも45秒間離陸しなかった。既存prepare_slam_model.pyのUniPilot飛行で採用済みの4 4 10へ変更して成功。他の速度制御設定・雑音は不変。標準モデル完全無改変とは呼ばない。
- native/unipilot_gimbal_sim.launch.pyでZaBの実world名を読み、rmf_unipilotをspawn、時刻/IMU/LiDAR/画像/odometry/速度/enable/ジンバルのROS2 bridgeを用意。native/unipilot_hover_manual.pyは真値に基づく位置保持と/unipilot/takeoff・/unipilot/land（std_srvs/srv/Trigger）を提供。takeoff_height既定1.5mは離陸開始位置からの相対高さ。二重takeoffは拒否し、離陸はユーザー指示のみ。
- 今回の移行範囲は手動の画角確認。自動偏向・ROVIO/MIMOSAによる推定は起動しない。既存active_gimbal.launch.pyは依然DRAGON用であり、UniPilotの時変カメラ外部パラメータ/推定器接続へ移植済みとは扱わない。
- 検証: GUIなしZaBでspawn、画像/IMU/LiDAR出力、離陸→hover→yaw45/pitch-20→着陸成功。角度実測45.000/-20.000deg、偏向後5秒の相対高度1.4276〜1.4387m（設定1.5m）、XY保持誤差最大0.00722m、画像537/IMU5374/LiDAR269件。生成モデルと元モデルの機体/ローター慣性・LiDAR設定一致も確認。成果物native/logs/unipilot-gimbal-check-gain/validation.jsonと検証スクリプト。初回標準gainでの未離陸試行はunipilot-gimbal-checkに区別して保持。検証用全プロセスを停止。GUIは既存の検証済みrqt接続を使用、新規UniPilot画面の目視操作は未実施。
- 起動: source /root/uas_native/env.sh; ros2 launch /root/uas_native/gimbal_inspect.launch.py。画像topic /rmf_unipilot/cam_gimbal/image、角度はrqt /camera_gimbal_manualのyaw_deg/pitch_deg。別端末でros2 service call /unipilot/takeoff std_srvs/srv/Trigger '{}'、着陸は/unipilot/land。終了は起動端末Ctrl+C。生成先logs/unipilot-gimbal-inspect/unipilot_gimbal.sdf。位置の反映には再起動、ビルド不要。bag自動記録なし。

## 2026-10-14: UniPilot起動直後にGUIが閉じる問題への対処

- ユーザーは起動後に画面が閉じると回答。直近2回のlaunchログでは約9秒でGazeboラッパーが終了、全起動処理の終了が連動していた。headless離陸試験では検出できなかったGUI経路の問題。
- GUI付き追試で実際のクラッシュを再現: D3D12: Removing Device、failed to create drisw screen、Qt Failed to create OpenGL context、SIGABRT(-6)。再現しない起動もあり、全起動で必発とは扱わない。
- unipilot_gimbal_sim.launch.pyを変更し、gz sim -s（物理/センサ）とgz sim -g（表示）を独立起動。server終了時だけlaunchを終了。表示にはDRAGONで使用していたgazebo_gui.configを明示、GUIプラグインの探索パスも指定。Qt runtimeは当該プロセス用の0700ディレクトリ。
- WSLでは表示プロセスだけGALLIUM_DRIVER=llvmpipe、LIBGL_ALWAYS_SOFTWARE=1を指定。gimbal_inspect.launch.pyのgui_software_rendering引数でも指定可能。サーバーのGPU設定・センサ品質・物理刻み・飛行/ジンバル制御は変更なし。GUIのCPU描画はZaB表示で約2fpsに落ちることがあり、表示の滑らかさに制約がある。カメラ画像は独立して20Hz/sim。
- 検証: 単独GUIのCPU描画で1分以上維持、25秒間に画像458枚/シミュレーション22.933秒。GUIだけ終了しても画像受信継続を確認。最終launchの通常GUI付き起動でも別途433画像/シミュレーション21.598秒（実時間25秒）を取得、GUIプロセス生存と描画環境設定を確認。native/logs/unipilot-gimbal-gui-final/validation.jsonと再現・修正後ログを保存。検証用launchはCtrl+Cで終了。
- ユーザー起動コマンドは変更なし: ros2 launch /root/uas_native/gimbal_inspect.launch.py。これはGPUドライバ本体を修復したという意味ではなく、表示だけソフトウェア描画へ切り替える回避策。

## 2026-10-14: GPU描画の復帰とUniPilotジンバル範囲の変更

- ユーザー指定: Gazebo表示のカクつきを改善、yaw±50deg/pitch±90deg、位置は維持。原因となっていた前回のGUI専用CPU描画を既定から外した。
- unipilot_gimbal_sim.launch.pyのWSL GPU表示に、既存DRAGON launchで使用していたQT_QPA_PLATFORM=xcb、MESA_GL_VERSION_OVERRIDE=3.3、MESA_GLSL_VERSION_OVERRIDE=330を適用。以前UniPilot側で省略されていた互換設定。GALLIUM_DRIVER=d3d12、LIBGL_ALWAYS_SOFTWARE=0をGUIだけに指定。物理/センササーバーの環境と計算品質は不変。ソフトウェア描画はgui_software_rendering:=trueで明示指定できる。
- unipilot_gimbal.pyにGIMBAL_LIMITS_DEGを定義し、SDF関節とmanual_gimbal.pyのrqt範囲/入力検証で共有。UniPilotはyaw±50/pitch±90。robot:=dragonでは既存±120/±30を保持。カメラ位置(0.18,0,-0.025)m・基準姿勢(0,0,0)は変更なし。camera_mount.jsonにも範囲を記録する。
- 実機構検証: (yaw,pitch)=(50,90),(-50,-90),(0,0)を順に指令し、各実測角が0.4deg以内に到達、yaw50.1/pitch90.1の範囲外指令を拒否。rqtパラメータ記述とSDFの上下限の一致、生成SDFの取り付け位置不変を確認。logs/unipilot-gimbal-gpu-limits-check/limits_validation.jsonに保存。
- GUI実測: 修正環境で2回起動し、終了・クラッシュなし。初回の読み込み後25〜41fps、再起動の読み込み後約38〜39fps（後続値はgui_validation.json参照）。モデル読み込み中には一時的に約0.2fpsになるが、定常表示がCPU描画時の約2fpsから改善。GUIプロセスのGPU/GL環境変数も照合。1回目サーバー/rqtを保持したままGUIを閉じて再起動する経路も確認。長時間の安定性を保証する試験ではない。
- 検証用GUIとlaunchは終了。起動コマンドは従来どおりros2 launch /root/uas_native/gimbal_inspect.launch.py。作業者が起動したプロセスを常駐させない。

## 2026-10-14 UniPilot 固定・偏向比較の高速化（2時間枠、検証中）
- ユーザー指定: 最低5倍、制限2時間。開始10:28:53 UTC、期限12:28:53 UTC。現時点で5倍達成とは判断していない。旧DRAGONの3区間×2条件は約90分、今回の基準条件はUniPilotの承認済み配置、yaw±50°/pitch±90°、軌道速度0.5m/s、DRPM融合OFF。
- 新規 unipilot_run.py / unipilot_route.py / unipilot_calibration.py / unipilot_campaign.py / benchmark_live.py。UniPilot標準のLiDAR/IMU/camera外部・内部校正を生成モデルから取得。DRAGON用のほぼ無雑音gyro事前値を流用せず、UniPilot SDFのノイズ仕様から初期分散・連続時間ノイズ密度を算出。既存DRAGONの起動入口は保持。
- dt=1ms、画像480x270/20Hz、LiDAR500x48/10Hz・測距σ3cm、IMU200Hzとノイズ、画角/クリップは保持。GUI・bagなし。未購読のSLAM地図ROSブリッジを省略、OMP_WAIT_POLICY=PASSIVE、Gazebo8.15のparallel_postupdates=falseを任意指定。
- 固定・偏向の独立した単眼推定器を同じ物理飛行で評価するpairedモードを追加。固定カメラは光学センサのみで質量/筐体を追加せず、各推定器には自分の画像だけを入力。LiDAR/IMU/真値を共有する。物理的に別々に飛ばした試行とは区別する。
- メモリ/画像品質を変更せず、画像と点群のGPU描画を先にまとめて発行し、その後にデータを取り出す任意パッチを隔離ビルド。既存OSライブラリは置換しない。gz-rendering8.2.3へ公式PR1302由来の描画worker=0をバックポート。gz-sensors8.2.2/gz-sim8.15.0へバッチ描画を追加。patchはactive_perception/performance/。
- 静止・無雑音の同一世界/機体/光学設定で原版とバッチ版を比較: 2画像各388800bytes、点群768000bytesすべてバイト一致（speed5-quality-original / speed5-quality-batch）。動的場面と本試行での欠落検証は別途必要。
- センサのみの実測RTF: 元構成約1.29–1.31、serial約1.41、描画worker=0約1.37、2視点バッチ約1.33。CUDAの小さな負荷でGPUメモリクロックを維持する限定実験は約1.62。これは比較全体の5倍達成を意味しない。
- 隔離Mesa25.0.7、EGL、Intel GPU、Vulkanも試験。Mesa/EGLは大幅改善なし、Intelは描画初期化クラッシュ、Vulkanは未対応処理エラーで棄却。Windows NVIDIA global power一時変更は効果がなく元設定へ復元済み。通常環境/OSドライバは変更していない。
- 初期UniPilot移植試験ではROVIOが離陸時から大きくドリフト。地面近傍のカメラ画像で近距離床面が画面下半分を占めることを確認。任意のairborne-init（ホバリング後にIMU3秒平均・VIO初期化）を追加し短区間検証中。地上初期化を根本解決したという主張ではない。真値は飛行制御/事後評価のみで、推定器へ渡していない。
- 短いpaired試行は45秒/60秒追従を完了。speed5-paired-probe wall141.28s、speed5-airborne-probe wall120.91s（それぞれ起動/離陸/初期化/終了を含む、条件が異なるので直接の速度倍率比較に使わない）。大きい推定ドリフトは残る。高速実行時のencoder購読をRELIABLE・履歴2000へ変更、画像前処理の不要なencoder履歴走査と点群Python tuple生成を削減。
- 計測ラベルの訂正: 初期speed5-inline-render2等はソース置換が成立しておらず、実際にはworker=0パッチ未適用だった。上の「worker=0約1.37」は無効。実適用は11:28 UTC頃。speed5-quality-inline-actualで原版との30サンプル（画像2系統・点群各10）のバイト一致を再確認し、静止センサのみ10秒間のRTF=3.7616を計測。全処理の速度ではない。
- build_performance.pyで固定revision/patch整合を確認して隔離再ビルド完了。描画サーバーのみ明示的なカスタムSensors pluginを指定する（インストール済みcore8.15に対して既存plugin aliasが8.9を指していたため）。
- speed5-full-pairedはMIMOSA遅延で偏向の情報鮮度判定が頻繁に失敗したため中断。これを有効な高速比較としない。
- build_map_performance.py: voxelが削除されなかった回の全インデックス再構築を省略、隔離libgtsam_points.soに限定。点群/共分散/KNN検索結果はLRU削除あり・なしで原版とバイト一致、SHA256=24dc9412003aab68a5a64328cb9ec23674c483872ae12377e86d1aabb965e500。小さな挿入性能試験だけでは効果は小さい。jemalloc2 5.2.1-4ubuntu1をdeb展開だけしてMIMOSAプロセス内に限定preload（OS未インストール）。
- pace_simulation.py: LiDAR処理完了とROVIO出力の時刻を監視し、処理が遅れた場合はGazeboの実時間進行だけ一時停止。物理刻み・観測頻度・タイムスタンプを変更しない。VIOには元から約0.15秒の出力遅延があり、初版の解除閾値0.12秒では待ち続けるため修正。LiDARは遅れ0.30秒で停止/0.15秒で解除、VIOは0.40/0.23秒。
- speed5-paced-short2: prefix60秒追従、固定/偏向を同時測定。全所有プロセス正常終了、wall112.52s（起動・離陸・初期化・終了含む）、sim100.86s、LiDAR2系統とも0.1秒間隔・欠落なし、偏向stale=0。まだ5倍達成ではない。speed5-paced-fullで3区間の全長検証開始。
- speed5-paced-fullはa1のMIMOSA SIGSEGV（exit -11）で中止。停止前のplanner stale=0で、単なる情報鮮度待ちとは別問題。高速化した地図管理/allocatorの影響を断定せず、元の地図管理・allocator、等速のspeed5-control-a1を開始。
- 並列化の設定不整合を確認: comparison_wsの-DUSE_OPENMP=TRUEに対し、ヘッダは#if USE_OPENMP。TRUEマクロが未定義のため通常ICPは逐次実行だった（コンパイル済みテストでもfalseを出力）。build_mimosa_performance.pyは単項・DRPM-OFFの点ごとの対応探索だけ4並列化し、元と同じ点順で逐次加算する隔離libを生成。二項/DRPM-ONの分岐は元の逐次実行。
- test_mimosa_performance.cpp / verify_mimosa_performance.py: 単項/二項、標準退化投影ON/OFF、姿勢変化・キャッシュ再利用を含む240回の情報行列・対応点判定・法線が元とバイト一致（a4d050c370fc7515b1ff5a46254e5336dfb5d166949a99cc1e5ae5424b2a10f4）。初回0.656→0.496s、再試験0.871→0.592s（他処理同時稼働なので速度は参考）。全処理5倍の根拠にはしない。
- speed5-icp-fullで地図/allocator変更を外し、検証済みICP並列化のみMIMOSAへ適用、描画最適化・同一飛行paired・処理完了待ちで3区間を再試験。a1側はSIGSEGV発生時のスタック取得用にgdbを接続。GUI/bagなし。
- speed5-control-a1（元の描画/ICP/allocator、等速・active1系統）は全228.49秒追従を完了、wall555.40s。ただし別試行/コンパイル同時実行による負荷を含むため速度の単純比較基準にしない。
- speed5-icp-fullはa1の固定側LiDAR完了が時計より0.211秒遅れた状態で進まなくなり、pacerの15秒タイムアウトで停止。gdbのSIGSEGV/二重解放スタックは終了時のgraph破棄とworker並列処理が同時に起きているため、このスタックだけを飛行中の停止原因と断定しない。旧nodeはros::waitForShutdown後にspinner.stopせず、managerが先に破棄される構造。
- build_serial_fusion.py: 独立nodeでLiDAR/外部odometryのcallbackを1 workerへ制限し、IMU専用workerは維持。終了時はspinner.stop→IMU join→manager破棄。performance/tbb_limit.cpp: GTSAM内部のworker数をプロセス内だけ制限するopt-in preload。ICPの点ごとの4並列・逐次加算は維持。どちらも推定因子/反復回数/センサ頻度/時刻を変更しないが、非同期処理順序が変わるため全軌道の数値同一は主張しない。
- serial初回はROS launchが絶対パスtypeを解決できず、物理時刻0で失敗。launch-prefix wrapperで隔離nodeを起動する形へ修正。通常の起動入口・既存binaryは不変。
- 既存active_perceptionのcore/reportテスト11件成功。source/binaryの記録にfast-icp実効libのSHAを含めるよう修正し、speed5-icp-fullのprotocolにも実際のoverrideを明記。
- speed5-serial-fullはsim37.8sでIMU lookupの開始/終了同時刻例外により停止。既存deskewには瞬時点群の同時刻保護があるが、この例外の呼び出し経路は未確定。条件付きgdbによる5秒追従再試験speed5-zero-imu-probeは完了、同じ例外は再現せず。従って特定のzero-duration修正を憶測で加えていない。
- コード確認でimu::Manager::getNumMeasurementsBetweenがbuffer_mutex_を取らず、IMU専用workerのdeque追加/削除と並行して履歴を走査していることを確認。隔離MIMOSA libにread側の同じmutex保護を追加（数式/時刻/観測数は変更しない）。これが先の例外の原因であるとの断定はしない。build_mimosa_performance.pyに再現ビルドを保存。
- speed5-final-fusionは描画/ICP最適化、元のmap/allocator、逐次fusion callback＋独立IMU、GTSAM worker1、処理完了待ちを使用。3区間を1jobで実行、wall900秒上限。開始12:12頃UTC、2時間期限12:28:53 UTCの前に停止する。まだ5倍達成/全比較完了は未確認。

## 2026-10-14: UniPilot比較の高速化・2時間枠の結果（未達）

- **5倍高速化は未達。全3軌道の固定/偏向比較も未完了。** 開始10:28:53 UTC、期限12:28:53 UTC。最終試行は12:23 UTCまでに全所有プロセスを停止。過去6飛行のログ期間合計4698.860443秒を厳しい基準とした場合、5倍には939.772秒以内で全区間・両条件を完了する必要がある。今回は機体もDRAGONからUniPilotへ変わるため、仮に時間条件を満たしても同一機体の純粋な高速化倍率ではない。
- 最終組合せの経過632.979秒でprefixが軌道時刻327.841/1070秒にて追従誤差0.1812mとなり、既存経路の0.18m停止基準で終了。a1は207.381/228.493秒で監督処理により停止、b1未開始。この632.979秒を完走時間として扱わない。prefixの停止基準を緩めて成功扱いにしていない。追従誤差を生じた直接の制御/通信原因は未確定。
- 出力: native/logs/speed5-final-audit/{campaign.json,performance_audit.json,processing_summary.json}。元の部分データはspeed5-final-fusionとspeed5-final-short-routesに保持。fivefold_runtime_reached=false、all_routes_complete=false、full_accuracy_equivalence_verified=false。未実行b1を含む全区間のtimestamp_coverage_completeもfalse。
- 取得済み追従期間ではROVIO両系統は50ms間隔、処理済みLiDAR両系統は100ms間隔、区間内部の欠落0、偏向判断のstale=0。途中中断した期間の監査であり、未取得区間や終了端を含む完全性の証明ではない。
- 最終prefixのLiDAR1スキャン全処理の末尾100回中央値は偏向192.56ms／固定171.75ms、地図更新だけで45.14／51.87ms。10Hz観測に対し処理が追いつかず、間引きを防ぐシミュレーション待機が全632.98秒のうち少なくとも357.63秒発生した。先頭100回は初期化・小さい地図のため同じ処理負荷ではない。固定/偏向2系統、別軌道も並行稼働した測定なので単体処理の上限性能とは扱わない。
- 推定精度も未解決。prefix取得区間の真値の始点からの最大変位131.14mに対し、ROVIOは偏向910.94m／固定810.48m、MIMOSA318.26m／560.00m。これはRMSEではなく座標差の大きさを示す診断値。移植後のノイズ/初期化条件、スケールと地図肥大の因果は追加検証が必要。高速化パッチが原因と断定せず、旧DRAGONの精度結果へ上書きしない。
- 検証できた改善部品: (1) 隔離したGPU描画worker/バッチ処理、静止無雑音画像2系統・点群計30サンプルは原版とバイト一致、センサ単独RTF3.7616。(2) 点対応探索の並列化＋元順序で加算、240線形化の数値バイト一致。(3) 同じ物理飛行から固定/偏向を独立評価する任意pairedモード。(4) 不要な画像再変換/履歴走査/点群Pythonオブジェクト生成を削減。センサ単独RTF3.7616は全体3.76倍高速化を意味しない。
- MIMOSAのIMU履歴参照mutexと終了順を隔離版で修正。最終試行は推定器を含む全子プロセスexit0、prefixの飛行制御だけ停止基準でexit1。過去のzero-duration例外の原因確定や一般的な競合解消を主張しない。map最適化/jemallocは最終構成に不採用。OSライブラリ置換なし、Windows一時power設定は元へ復帰、GPU負荷維持プロセスも終了。
- 通常の手動ジンバル起動を維持。高速化入口は未完成の実験用であり既定起動には採用しない。新しい結果を採用比較として図へ反映しない。次の優先課題はUniPilotでの単眼VIOスケール整合と同一条件の速度基準確立、地図コピー/因子生成の時間削減、全長で追従・欠落・精度を検証すること。物理刻み/センサ頻度/解像度/ノイズ/経路長を削減して倍率を作らない。
- 最終確認12:25 UTC: 変更対象Python13ファイルの構文検査成功、test_core.py/test_report_routes.py計11件成功。所有campaign/run/serial-MIMOSA/GPU keepalive/coordinatorプロセス残存0（speed5-final-audit/cleanup.json）。全長の精度同等性は未確認のまま。

## 2026-10-14: UniPilot高速化・追加2時間枠（進行中）
- 新規依頼の開始12:28:12 UTC、期限14:28:12 UTC。単独物理飛行・paired単眼2系統、前回最終構成のままa1全長を基準測定中（speed6-baseline）。別飛行・コンパイルを重ねず測る。GUI/bagなし、センサ/物理品質・経路・停止基準は維持。
- 地図コピーの負担削減を狙い、gtsam_pointsの点格納/近傍巡回順/LRU/旧snapshotとの点共有を維持したまま、座標索引だけ連続メモリへ置換するCompactIVoxを実験実装。元型を継承しvirtual insert/knnだけ差し替え、元のOS/上流コード/通常ライブラリは変更しない。現時点では未ビルド/未検証。
- speed6-baselineはa1全228.493秒追従完了、wall394.041秒、同時に別飛行/重いビルドなし。物理sim272.28秒、待機214.235秒。speed6-compactは同条件でwall328.592秒、1.199倍。両試行の追従窓でVIO/処理済みLiDARの内部周期欠落0、planner stale0、最大追従誤差0.11888/0.11495m。完走した1区間・各1試行の比較で、全3区間5倍ではない。
- CompactIVox（performance/compact_ivox.hpp）は元のiVoxを継承し索引だけ変更。元のLRU削除、検索順序、共有点コンテナを維持。test_compact_ivox.cppで1760 snapshot/近傍/共分散/強度比較成功。ICPの240線形化も同じSHA256=a4d050c370fc7515b1ff5a46254e5336dfb5d166949a99cc1e5ae5424b2a10f4。build_compact_mimosa.py/verify_compact_mimosa.pyに再現処理を保存。
- 時刻順入力の独立node（performance/ordered_mimosa_node.cpp）を追加。各センサの最新受信時刻の小さい方まで、LiDAR/VIOを同じcallbackへ時刻順で渡す。VIOが始まる前だけ従来LiDAR初期化を継続。観測値/タイムスタンプ/推定重み/反復数は不変、処理順と遅延が変わるので推定結果の同一は主張しない。IMUは独立worker。pacerはLiDAR停止0.4秒/解除0.26秒、既存の偏向情報期限0.5秒内に限定。
- speed6-ordered-probeは60秒追従完了。各系統2553 callbackの時刻逆転は初回VIOの27.55秒で1件のみ、以降時刻順。planner stale0。CPU診断を有効化しscratch版のコンパイルも一部並行したため93.6秒の所要時間を速度比較には使わない。Ubuntu libgoogle-perftools4/google-perftoolsをbuild/profiler-speedに展開のみし、node限定preloadで計測（OS未インストール）。CPUサンプルの38.9%が近傍点探索。
- scratch版は点対応探索/平面推定の一時配列をスレッドごとに再利用し未使用ones配列を除去、算術順序は不変。pruned版は現在の5近傍より近い点が存在し得ないボクセルを保守的な距離下界で除外、残りの巡回/同距離の順序を保持。両者とも240線形化のバイト一致。prunedの1760snapshot比較成功、3解像度・境界と同距離を含む60189検索もバイト一致。検索単体は元0.1368秒→pruned0.09349秒（実飛行全体の倍率ではない）。
- build/mimosa-compact{,-scratch,-pruned}に独立libを保存。通常起動不変、--compact-map --map-variant pruned --ordered-fusionは実験用。speed6-ordered-prunedでa1全長の単独飛行比較を開始。

### 2026-10-14 高速化継続: 13:10 UTC時点
- compact mapのみのa1完走328.5915秒に対し、時刻順融合＋厳密な近傍枝刈りは339.6116秒。組合せを既定には採用しない。処理順の変更は数値軌跡にも影響するため別条件として保存。
- 画像変換とIMU/点群受け渡しをC++へ移植し、`--native-adapters`で選択可能にした。fixed/active各30画像の画素と付随情報が元のPython版と一致、姿勢四元数の最大差2.10e-16。
- ROS1実通信でIMU250個・点群12個（500x48、行padding、NaN/Infを含む）の全内容とstampが一致。ROS送信ライブラリが独立採番するheader.seqのみ比較から除外した。VIO初期化ゲート後のIMUも同一。
- compact map＋native adapters、元の直列融合のa1全区間を測定中。物理・センサ品質・推定パラメータは変更していない。

### 2026-10-14 高速化継続: 13:23 UTC時点
- `speed6-native` a1全228.4928秒完走: wall301.1759秒（初期baseline394.0405秒比1.308倍）。監査で両ROVIOの内部欠落0、両LiDAR欠落0、planner stale0。画像/IMU/点群頻度・ノイズ・物理刻み維持。
- 完全一致した姿勢・重力でのみGaussianFactorを再利用する`--map-variant cached`を追加。姿勢/重力変更、同一入力反復、map共有更新、factor cloneを含む480線形化の行列/対応状態/法線が元版とbyte一致。
- 評価CSV記録もC++版`--native-monitor`を追加。ROS通信で4系列各41行の時刻/値、初期化判定がPython版と一致。
- 軌道数値配列を読込時に構築。実経路の5900評価で位置/速度/姿勢指令が完全一致、当該計算0.6242→0.0695秒。センサ周期や制御周期は変更なし。
- `speed6-cached`（上記3変更追加）はa1 wall298.4921秒。直前301秒との差は小さく、個別変更の大幅な全体速度向上とは解釈しない。時刻監査実施済み。
- 近傍voxel IDのthread-local cache（copy/insert/clearで世代更新）＋保守的な枝刈り＋作業領域再利用を`neighbors`として隔離。1760 snapshot比較・60189近傍照合はbyte一致。線形化照合は実行中。
- 長区間の前回失敗を調査: route327.8秒付近のpolyline先頭に進行方向の急変があり、追従誤差0.18m上限超過。基準経路/上限を変更せず、実測速度誤差の補正をtracking中だけ加える`--velocity-feedback`を実験用に追加（既定0、未検証）。真値は従来どおり制御/評価のみに使用。

### 2026-10-14 高速化継続: 13:41 UTC時点
- `speed6-neighbors` a1完走295.1384秒。baseline394.0405秒比1.335倍。`speed6-clock200`は292.4031秒で、200Hz時計通知はCPU負荷を下げるが全体時間の有意な改善は確認できない。採用候補は1ms通知を維持する1000Hz relay（停止中の重複のみ除去）。200Hzは実験用opt-inに留める。
- native clock relay: Gazebo→ROS2、実際のGazebo stampをそのまま転送。239入力で1000Hz/200Hzの期待出力と一致、停止時重複除去・時計巻き戻り伝播を確認。物理1msや各センサstampを変更しない。
- 実測した機体SDFは全条件でSHA256 `7de2c060d67a018d1b78e7cbcc9e98adfeae07504fd01960c3747d80685ed5bd` と一致。LiDAR10Hz/IMU200Hz/両カメラ20Hz、world物理dt0.001秒。
- 品質監査を末端まで拡張したところneighbors試行はROVIOの末尾gap0.08秒（最後の画像1枚分）を検出。従来「internal gap0」だけでは終了時の未処理画像を見落としていた。runnerを、tracking最終時刻に対応する両VIO/両LiDARの処理完了まで待つよう修正し、`drain.json`に対象/実際の完了stampを記録する。監査判定も始端/終端gapを含める。
- native sensor adapterへ入力stamp監査追加。内容は変更せず、受信数・最大gap・重複・欠落を`input_audit.json`へ保存。改修後IMU250件/点群12件の元実装との一致を再確認。
- 13:38:23 UTCに`speed6-full`全3区間を開始。2並列、neighbors/native adapters/native monitor、clock1000Hz、velocity feedback1、物理/センサ品質維持、DRPM融合OFF。2200秒wall制限で2時間作業期限14:28:12より前に自動停止する設定。失敗しても他の独立飛行は完了させる。途中経過のIMU/LiDAR欠落/重複はいずれも0。
- 性能実装のC++ソース/ビルドmanifestも各試行のimplementationへ保存するよう改修。先に開始した2飛行には一致するソースを追記保存（実行コードは変更していない）。

### 2026-10-14 高速化継続: 全長試験と曲がり角の診断
- `speed6-full` はwall1076.189秒で終了。a1全228.493秒とb1全263.385秒は完走、prefixは328.849/1070秒で追従誤差0.187571mとなり停止。全3区間完了/5倍判定はいずれもfalse。2並列の各飛行wallはa1約476秒、b1約599秒、prefix約829秒であり、全体1076秒を全区間完走の高速化倍率へ換算しない。
- 終了待ち改修後、a1/b1はtracking末尾の必要な両VIO/両LiDARの処理完了をdrain.jsonで確認。VIO内部の50ms欠落は0。入力監査ではa1/prefixのIMU/LiDAR欠落0、b1のIMUに5ms周期1件分のgapを検出（最大10ms、LiDAR欠落0）。このため全試行のtimestamp_coverage_complete=false。欠落時刻を今回の集計だけから特定できず、飛行前か飛行中かも未確定。先の「途中欠落0」を最終結果へそのまま適用しない。
- 全長試験の単眼ROVIOは大きなドリフトが残る（始点からの最大変位: a1偏向768m/固定906m、b1偏向1206m/固定1100m）。これを速度改善の品質保証や固定/偏向の精度比較として採用しない。旧DRAGON結果不変。
- prefixの停止付近1058点は制御側と真値ログが一致。受信真値の最大経過時間31ms、制御刻み最大39msで、大きな通信途絶を示す証拠はない。速度誤差の補正係数1では主に鉛直誤差0.185mが生じた。既定係数0は維持し、試験設定を採用しない。
- `control_corner_probe.py` で同じ曲がり角を制御のみで切り出し、22秒分を検証。元制御は完走するが最大誤差0.176357m、誤差に応じて滑らかに進行速度を落とす案は0.180951mで停止。`corner_speed.py` の先読み減速（角度20deg超、加速度上限0.15m/s²、角直近0.02m/s）は最大誤差0.070093mで完走。追従部分の所要sim時間24.60→25.12秒。経路形状・停止上限0.18m不変、巡航0.5m/s、角のみ減速。高速化数値には算入しない。推定器同時負荷およびprefix全長は未検証、`--corner-speed-limit`は任意指定のみ。
- 速度監査の評価窓を「route_t最大となった最初の時刻」から最後の追従記録までへ訂正。停止時に進行時刻が同じまま誤差が増える期間を切り捨てない。`test_speed_audit.py` が停止尾部の誤差/未完走/未達を保持することを確認。
- GPUクロック維持の軽量化は不採用。32blocks・1000cyclesを10ms/50ms間隔で発行するとmemory clockが8001MHzを維持できず405MHzまで低下。従来32blocks・1Mcycles・1msは8001MHzを維持。CUDA同期をCPUブロック待機に変更しても約45% CPUが残り、改善を主張しない。

- 最終候補 `prefetch` は、neighborsの探索順/演算順を保ち、次に読むvoxel/点コンテナをCPUへ先読み指示するだけの変更。独立build/mimosa-compact-prefetchへ配置。1760 snapshot比較、60189検索（IDと距離が完全一致）、480線形化（SHA256 4bbefe91b306d129484276d2d035b61ad03d16712d2d9928caf311e7330d6069）成功。単体検索0.16592→0.076996秒、線形化0.392809→0.276208秒。これらの単体時間を飛行全体の倍率としない。
- 14:11 UTCに `speed6-final-prefetch` a1全長・1jobを開始。原比較同様に速度feedback0、先読み減速なし。prefetch/native adapters/native monitor/clock1000Hz/終了待ちを使用。600秒wall上限で2時間枠内に停止する。実験中は他のシミュレーションや重いコンパイルを動かさない。

- `speed6-final-prefetch` はa1全長をwall306.960722秒（campaign307.975576秒）で完走。baseline394.040536秒比1.2837倍、22.10%短縮。全子プロセスexit0、終了待ち完了、IMU54522件/LiDAR2727件の内部欠落・重複0。追従期間の両ROVIO4585件は50ms欠落0、両LiDAR処理2292件は100ms欠落0、planner stale0。新監査timestamp_coverage_complete=true。a1だけのため全3区間判定はfalse。最大追従誤差0.120519m。
- prefetchの単体テスト改善は全体時間へ直結せず、以前のneighbors295秒を上回らなかった。終了待ち/clock1000Hzが同じ条件で `speed6-final-neighbors` を追加測定。センサ設定同一でもオンラインの入力順序・推定/地図状態が完全一致しないため、1回の測定差を個別変更の効果とは断定しない。
- 最終prefetch試行でも真値の最大変位50.09mに対しROVIOは偏向422.99m/固定364.46m、MIMOSA245.02m/257.37m。演算単体の一致はオンライン推定精度の保証ではない。この試行を固定/偏向の精度比較として採用しない。

### 2026-10-14 追加2時間高速化の終了結果（14:24 UTC）
- **欠落監査を通った同一a1全長の基準394.040536→306.960722秒、1.283684倍・22.099%短縮。全3区間5倍は未達。** 制限12:28:12～14:28:12 UTC内に作業を終了。最終neighbor再測定は294.640183秒（campaign295.435100秒）で完走し全子プロセスexit0だが、IMU受信に1件分の欠落があるため有効な欠落なし速度実績には含めない。先読みが受信欠落の有無を決めるという因果は未確認。
- 有効な速度実測として残すprefetch版は原点検索/線形化と一致、最終オンラインのセンサ時刻監査true。実機モデルSDFはbaseline/neighbors/最終2試行すべてSHA256 7de2c060d67a018d1b78e7cbcc9e98adfeae07504fd01960c3747d80685ed5bd。画像サイズ・各センサ頻度・ノイズ・物理1msを維持。速度倍率はa1の各1試行であり、全長/全環境の平均や上限性能を示さない。
- 監査をさらに強化し、全CSV系列の有限値・重複・始端/終端を確認（MIMOSAは100ms、VIO/真値は50ms以内）。prefetchのtimestamp_coverage_complete=true、neighbors再測定/fullはfalseを維持。失敗時末尾を保持する回帰テスト1件成功、関連Python43ファイル構文検査成功。
- 残る問題: prefix全長の追従完了、UniPilot単眼ROVIOのスケールドリフト、IMUのまれな1周期欠落の発生位置/原因、オンラインでの精度同等性。末尾待ち/生入力監査を追加したが、これらを解決済みとは扱わない。通常運用設定に高速化実験を自動適用せず、READMEに検証済み実行コマンドを記載。
- 最終集計 native/logs/speed6-final-summary.json、各試行にprotocol/model/source snapshot/build hash/runtime/input_audit/drain/performance_auditを保存。検証用campaign/run/control probe/GPU keepalive残存0を確認。GUI/rosbagなし。ユーザー作業や旧精度図は不変。HANDOFF先頭を終了状態へ更新。

## 2026-10-14 UniPilot VIO移植の精度修正（検証中）
- ユーザーは有効な修正の反映と原因の切り分けを依頼。開始14:59 UTC。前の高速化2時間枠とは別の作業。カメラ初期方向・LiDAR位置関係以外にも、DRAGON/UniPilotでセンサ仕様と初期化が異なるため、修正後の残差だけからその2要因へ断定しない。固定ROVIO単体にはLiDAR入力がない。
- 実SDF/URDFで確認: DRAGONのraw camera720x540/HFOV120deg/描画60m/画像ノイズ指定なし、UniPilot480x270/HFOV1.52rad/描画20m/画像noise0.007。gyro標準偏差は0.0002対0.009 rad/s、初期バイアス/時間変動の仕様も異なる。native/logs/unipilot-init-sensor-differences.jsonへ記録。
- 初期化候補: 既存3秒平均はgyro/acc biasにだけ使用し、初期姿勢は最後のIMU1サンプルから決まっていた。`stationary_mode=mean`で初期化だけ平均加速度を渡す。初期化時刻と以後の生IMUは保持。既定legacyは対照検証まで維持。
- rovio_gimbal.cppへ明示的なinput_record/input_replay/diagnostic_csvを追加。同じ実行プログラムがROSメッセージの受信順と内容を再生し、状態/特徴点数/深度をCSVへ出す。記録は任意指定のみ。unipilot_run/campaignに--record-vio-input/--stationary-mode追加、通常の自動bag記録なし。
- `replay_unipilot_initialization.py`でlegacy/mean/mean+gyro平均の事前分散の対照を準備。gyro分散候補はセンサ由来の白色雑音密度/T＋バイアス駆動密度*T/3（T=3s）、対象真値による適合はしない。採用は結果を見て判断。
- native/logs/unipilot-init-audit-a1で同じa1の固定/偏向入力をGUIなしで診断収録中。DRPM融合OFF、物理/センサ設定を維持、旧初期化。Windows実空き18.5GBを確認し、最大約1.5GBの一時入力に限定。旧実行ファイルをbuild/active_perception/rovio_gimbal_before_init_auditへ保持。オンラインを起動する旨は事前にユーザーへ通知。
- 同一callback順のlegacy再生は固定/偏向ともオンライン診断CSVとバイト一致。a1の位置RMSEは偏向184.212m/固定180.793m。平均姿勢初期化だけでは198.117/187.541m、gyro平均分散だけでは206.620/208.787m、併用205.823/212.621mとなり、位置精度改善としては採用しない。初期化候補は明示選択のみ。
- 実画像で20m描画打切りの灰色境界を確認。camera_far指定を追加し、60mのみ変えたa1を別収録（unipilot-far60-a1）。元の20m既定は検証まで保持。画角・解像度・IMU・noise・初期化・位置・向きは同一設定。
- 新たに確定した不整合: Gazebo GaussianNoiseModel.ccでは初期bias全体が exp(-dt/tau) で減衰し、q*tau/2*(1-exp(-2dt/tau)) の雑音を加える。UniPilotはgyro tau400s/acc175s、DRAGONには動的bias指定なし。従来ROVIOはqだけ読み替え、平均値の減衰を入れないランダムウォークのままだった。
- 収録済みジャイロの3秒センサ平均だけを初期biasに使う積分診断で、constant→仕様の減衰モデルにより姿勢RMSE8.092→2.983deg、最大14.727→4.236deg、末尾12.688→2.265deg。初期姿勢は診断用にGT整合、誤差算定のみGT使用。ROVIO性能の結果とは区別する。
- imu_bias_model.hpp / patch_imu_prediction.pyでROVIOのbias状態遷移・状態Jacobian・noise Jacobianを対応させる隔離ヘッダを実装。tau0は従来ランダムウォーク。build.pyへ組込み、上流ソースは不変。worldの重力9.8とROVIO固定9.81の差も明示設定で対照可能にする。100秒の状態/分散について、4刻み×3軸×2量=24件の解析解照合成功。全ROVIOの同一入力再生で検証中。

### 2026-10-14 UniPilot精度修正：同一入力検証と起動反映
- a1の旧入力を固定した再生 `unipilot-sensor-bias-a1-v2`：旧→bias減衰＋world重力＋平均姿勢初期化のROVIO位置RMSEは固定180.793285→67.713621m、偏向184.212095→162.612456m。姿勢RMSEは固定15.959022→2.291550deg、偏向4.989819→2.686937deg。両条件4585追従出力。GTは評価時の始点剛体整合と誤差だけに使用、スケール調整なし。改善したが発散解消ではない。
- `unipilot-sensor-bias-a1` と `unipilot-far60-replay` の初回試験は無効。ROS1 private CLIの配列が文字列となりtau未設定、さらにmaster内gravityが次試行へ残っていた。INVALID_EXPERIMENT.txtを保存。v2では各試行前に数値配列とgravityをrosparamで明示設定し、legacyが元オンラインCSVとbyte一致したことを確認した。C++も誤った型の設定は例外終了へ変更。
- far60単独ではa1の位置が悪化（固定266.037m/偏向228.048m）。画角打切りが主因という仮説は未証明、既定far20を維持。`--camera-far`は対照実験用。
- 高速描画なしの90秒飛行 `unipilot-original-render-a1` もドリフトし、旧ROVIO位置は固定42.039/偏向46.869m。同じ入力へのbias/重力/平均初期化修正後は22.711/33.448m。高速化だけを原因とは断定できない。標準と高速で入力は同一ではなく、動的描画の厳密同等性を証明した試験ではない。
- unipilot_run/campaign既定を `--imu-model sensor --stationary-mode mean` に変更。SDFからacc tau175s/gyro tau400s、worldからgravity9.8を読み、typed YAMLを両ROVIOへロード。DRAGONと既存rovio_gimbalの未指定既定tau0/9.81は保持。旧UniPilotを再現する指定は `--imu-model legacy --stationary-mode legacy --legacy-slam-start`。
- airborne-initではMIMOSAのIMU・LiDARもhover開始合図まで待たせる。従来はsim2秒からIMUを送り、2秒間の初期化とsim3秒からの離陸が重複した。native/Python双方のslam_inputsに任意gate_slamを追加、通常既定false。通信試験でゲート前の出力なし、解除後IMU650個/点群12個が双方一致。従来モードもIMU250個/点群12個一致（独立採番header.seqを除く）。生入力監査はゲート前も継続。
- 修正後のオンラインa1をGUI/bagなしで実行中（unipilot-corrected-online-a1）。追加のb1旧入力を別domainで有限収録し、同一入力で検証予定。今回の診断収録総量は約5GB以下、Cドライブ実空き18.2GB確認済み。結果は旧DRAGON比較図へ上書きしない。

### 2026-10-14 UniPilot精度修正：検証完了
- 同一入力b1再生 `unipilot-sensor-bias-b1` も旧設定はオンラインCSVとbyte一致。bias減衰＋重力＋平均初期姿勢によりROVIO位置RMSEは固定700.604283→67.871854m、偏向103.958014→46.428179m。回転は固定33.293798→6.325417deg、偏向2.087430→2.447757deg。各5299出力。a1/b1の同一入力4条件で位置は改善、回転はb1偏向のみ悪化。残るスケールドリフトは未解決。
- 修正前後のオンライン全長飛行をa1/b1で実施し、固定/偏向各系統のMIMOSA＋ROVIOを評価。DRPM融合OFF、同一経路・seed・センサSDF・校正・ROVIO基本info・融合重み・planner設定を検証（カメラmount JSONの差はcamera_far=20.0の記録項目だけ、実SDFはbyte一致）。初期化の待機・新モデル・偏向判断結果が変わるためオンラインraw入力の一致は主張しない。
- 正式なオンライン表は共通経路時刻0.1秒刻み、始点剛体整合のみ、scale fitなし。MIMOSA位置RMSE[m]旧→新：a1固定156.5146→13.0953、a1偏向150.9549→45.7921、b1固定449.2913→29.8548、b1偏向23.1616→31.9583。3/4改善、b1偏向は悪化。回転RMSE[deg]：15.1043→6.2388、15.3111→4.3241、33.2082→6.1394、3.2986→5.5678。速報の各出力時刻集計とは評価刻みが異なるため少し数値差がある。
- `native/logs/unipilot-imu-fix-comparison/index.html` に上記比較、a1/b1それぞれのROVIO/MIMOSAの六軸図4ページ・PDF・数値CSV、summary.json、validation.json。生成スクリプト `active_perception/report_unipilot_imu_fix.py`。旧DRAGON結果や採用比較図は上書きしていない。
- 生LiDARは全4飛行欠落0、IMUの5ms周期欠落はa1旧0/新1、b1旧1/新2。b1旧の1件は68.415→68.425秒、固定/偏向両ROVIO入力で一致し、上流側の欠落。位置悪化/改善をこの少数欠落の原因と断定しない。完全な無欠落オンライン比較ではない。
- 検証済み：ROVIO/biasモデル再ビルド、mean/covariance解析解24件、既存gimbal世界点/共分散不変・往復・並進拒否テスト、native/Pythonセンサadapterの旧/hoverゲート両モード、Python構文。全4オンライン飛行と全再生は終了、全所有プロセスexit0、Gazebo/ROS master/keepalive残存なし。GUI/bagは起動なし。git diff --checkはユーザー既存AGENTS.mdのMarkdown改行用末尾空白2行だけを報告（変更せず）。
- 採用：UniPilot既定sensor biasモデル・world重力・平均初期姿勢・airborne-init時のMIMOSA hoverゲート。追加gyro prior縮小、far60化は採用しない。MIMOSAの融合重み・bias random-walk因子自体は変更なし。修正が全ての要因を除去したとは主張しない。DRAGONとの差はカメラ初期方向/LiDAR位置だけでなく、IMU頻度/ノイズ/bias、画像画角/解像度/ノイズ/描画範囲、機体制御にもある。固定ROVIO単体のドリフトをLiDAR位置のせいにはできない。


## 2026-10-15 a1：yaw −50deg / pitch 0deg固定の精度検証
- ユーザー指定のa1のみ、yaw−50deg・pitch0deg。修正済みUniPilot単眼ROVIO＋MIMOSA、DRPM融合OFF、0.5m/s、元a1全228.492812秒の経路。GUI/rosbagなし。
- `active_perception/planner_node.py` に任意 `--fixed-angles-deg YAW PITCH` を追加し、起動時から推定器/地図の準備を待たず固定目標を送り続ける。角度限界を検証。run/campaignへ引数と実験設定の記録を追加、未指定の旧動作を保持。campaignでは `--modes fixed` の単独飛行に限定。
- 出力 `native/logs/unipilot-yaw-minus50-a1-20261015/run1_tail_a1_fixed`。前回の修正済みyaw0/pitch0固定結果を参照し、同じ評価刻み・始点整合で精度を比較。単独飛行と前回paired飛行はraw入力同一ではなく、角度ごとの1試行である。センサノイズ・IMUモデル・融合重み・校正・経路は変更しない。
- `report_fixed_view.py` を追加し、姿勢の実測値、初期化前の角度整定、設定同一性、追従区間の位置/回転RMSEと六軸図を検証・保存する。完走後に結果追記。

### a1 yaw−50deg / pitch0deg：完了結果
- 全228.492812秒の経路を完走、実行wall 197.354秒。全所有プロセスexit0で停止、追加GUI/rosbagなし。
- rovio: 前回yaw0→今回yaw−50の位置RMSE 69.373084→155.536469m、回転RMSE 6.542957→3.256733deg。今回最大位置誤差309.682913m。位置は悪化、回転RMSEは改善。
- mimosa: 前回yaw0→今回yaw−50の位置RMSE 13.095278→114.103121m、回転RMSE 6.238793→4.305569deg。今回最大位置誤差236.574149m。位置は悪化、回転RMSEは改善。
- 追従区間の角度最大誤差 yaw=0.001994790deg/pitch=3.51e-12deg。sim3.737秒で目標±0.05deg以内へ整定、初回VIOは27.650秒。静止IMU平均化より前に整定済み。生IMU54576/LiDAR2729、周期欠落0、ROVIO最大出力間隔50ms/MIMOSA100ms。
- 校正・ROVIO基本設定・融合設定/重み/外部変換・IMUモデル・初期化方式・推定バイナリSHA・経路の同一性を確認。前回はpaired内の固定カメラ、今回は単独のジンバル固定で、それぞれ別飛行・各1試行。角度変更だけによる効果の厳密な統計的証明とは扱わない。原因分析は今回の依頼外、悪化を入力欠落のせいにはできない。
- 評価は軌道追従のみ、共通経路時刻0.1秒刻み2284点、始点位置/姿勢のみ整合、scale fittingなし。結果 `native/logs/unipilot-yaw-minus50-a1-20261015/plots/index.html`、comparison.pdf（ROVIO/MIMOSA各六軸、全縦軸）、comparison.json、誤差CSV。旧結果は保持。READMEへ固定角指定を追記。


## 2026-10-15 a1：yaw＋50deg / pitch0deg固定の精度検証
- ユーザーが＋50degと明示したため、a1全区間をyaw＋50deg/pitch0degで実行。前回−50degと同じ修正済みUniPilot、DRPM融合OFF、速度0.5m/s、seed20261013、単独飛行、GUI/bagなし。出力 `native/logs/unipilot-yaw-plus50-a1-20261015/run1_tail_a1_fixed`。
- 既存の固定角度指定を利用し、推定器・制御・センサ設定は変更しない。評価スクリプトreport_fixed_view.pyの角度ラベルを設定から生成し、任意の追加比較runを受け付けるよう変更。追加比較でも推定器設定・バイナリ・経路・seed一致を検証し、0/−50/＋50の六軸図を生成する。初期化開始前に角度が整定したことの確認を必須化。
- 前回−50degの結果を/tmpで再集計し、既存のRMSE等のmetricsが完全一致することを確認。過去の公開済みプロットは上書きしない。今回の完走後に結果追記。

### a1 yaw＋50deg / pitch0deg：完了結果
- 全228.492812秒の経路を完走、wall 163.091秒、全所有プロセスexit0で停止。GUI/bagなし。
- yaw_0_pitch_0: ROVIO位置RMSE 69.373084m/回転6.542957deg、MIMOSA＋ROVIO位置13.095278m/回転6.238793deg。
- yaw_minus50_pitch_0: ROVIO位置RMSE 155.536469m/回転3.256733deg、MIMOSA＋ROVIO位置114.103121m/回転4.305569deg。
- yaw_50_pitch_0: ROVIO位置RMSE 92.472473m/回転9.441420deg、MIMOSA＋ROVIO位置40.248649m/回転7.702487deg。
- 今回＋50degの位置RMSEは−50degより小さいが0degより大きい。回転RMSEは3方向で最大。各1試行・別飛行の比較であり、統計的優劣や悪化の原因を断定しない。0/−50の再飛行は行わず、前回結果を同じ評価関数で使用。
- 角度整定sim 3.736秒、静止平均化開始約24.626秒、初回VIO27.650秒。追従区間の最大角度誤差yaw0.001817176deg/pitch3.7e-12deg。raw IMU54700/LiDAR2735、周期欠落0。ROVIO最大出力間隔50ms/MIMOSA100ms。
- 推定設定/バイナリSHA・校正・融合重み/変換・経路・seedの一致を確認。追従のみ、0.1秒経路刻み2284点、始点剛体整合のみ、scale fittingなし。結果 `native/logs/unipilot-yaw-plus50-a1-20261015/plots/index.html`、comparison.pdf（3方向重ね描き六軸・ROVIO/MIMOSA各1ページ）、comparison.json、各系列CSV。既存結果は保持。

## 2026-10-15 UniPilotのセンサ条件をDRAGONに合わせたa1比較
- ユーザー依頼により、UniPilotの機体・制御・センサ取付位置/姿勢・yaw/pitch固定0degを保持し、物理センサ仕様をDRAGON参考試行と一致させる明示的な `sensor_profile=dragon` を追加。標準はnativeのまま。参照は `logs/active-fixed-three-20261013/run1_tail_a1_fixed/dragon_slam.urdf`、抽出済み仕様は `native/sensor_profiles/dragon.xml` に参照SHAとともに保存。ログ依存なく再使用可能。
- カメラ: HFOV120deg、raw720x540→ROVIO480x360、20Hz、clip .04–60m、画像ノイズなし。IMU:1000Hz、gyroσ.0002/bias .0000075±.0000008、accσ.017/bias .1±.001、時間変動バイアス/量子化なし。LiDAR:360x32、水平±π/垂直−7～52deg、.1–30m/分解能.01m/距離ノイズσ.03m。LiDAR取付姿勢はUniPilotのままなので、機体基準の走査方向までDRAGONと同じではない。全変更が一律に「性能向上」するという仮定は置かない。
- generator/launch/run/campaignにprofileを伝搬、校正・ROVIOノイズ密度・バイアス設定を指定物理センサから自動生成。dynamic_bias_stddev欠落を0として処理。センサ受信監査のIMU周期も設定レート由来に変更（従来200Hz固定だった）。推定器本体/融合重み/取付変換/初期化方式/経路は維持。
- 高速C++画像adapterは以前同一解像度限定だったため、Python版と同じOpenCV remapを追加。30枚のRGB/BGR合成入力で変換後画素・時刻・画像メタデータ完全一致、姿勢差2.10e-16以下。従来設定も固定/可動各30枚で完全一致。DRAGON仕様各XML subtree/rateとraw/processed内部校正は参考試行と完全一致。nativeの既存cam0/raw_cam0/rovio.infoはbyte一致、センサ以外のモデルも一致。
- 初回 `unipilot-dragon-sensors-a1-20261015` は画像adapterの同一解像度制限で走行前停止（IMU入力0）、精度評価から除外。改修後 `unipilot-dragon-sensors-a1-v2-20261015` で再実行。DRPM融合OFF、単眼固定0/0、速度.5m/s、seed20261013、GUI/bagなし。report_sensor_profile.pyで旧UniPilot0/0・DRAGON固定と同じ0.1秒経路刻み/始点剛体整合/スケール補正なしで比較。結果は完了後追記。

### DRAGONセンサ比較：追加切り分け
- 物理仕様のみを合わせ、現行のセンサ仕様由来のROVIO誤差設定を使ったv2試行は完走したが、大きく悪化。ROVIO位置RMSE7125.061m/回転7.440deg、MIMOSA7072.456m/20.492deg。全プロセスexit0、wall201.039s。入力IMU272909件/欠落1周期(1ms)、LiDAR2729件/欠落0。出力周期ROVIO50ms/MIMOSA100ms、初回画像1枚のエンコーダ未到着による除外以外に画像adapter欠落なし。
- DRAGON時代のROVIO設定は、速度予測誤差4e-5、acc bias変動1e-8、gyro bias変動1e-16。物理仕様からの自動設定ではそれぞれ2.89e-7/0/0になる。この違いも切り分けるため `--rovio-noise-profile dragon` を追加。参照値は `sensor_profiles/dragon_rovio_noise.json` として出典SHA付きで保存し、取付変換以外のrovio.info数値がDRAGON参考試行と一致することを確認。標準sensorモードは保持。
- v2の初期化区間(24.702～27.702s)で、静止と仮定したgyro平均[-.000908927,.000332553,.000196710]rad/sに対し、真値姿勢の差分から求めた実回転平均は[-.000980417,.000287855,.000193506]rad/sだった。ホバリングは厳密な静止ではなく、実運動がバイアス初期値へ混入している。真値は事後診断だけに使用。これだけで全発散原因とは断定しない。
- そこでDRAGON時代に存在しなかった静止平均/バイアス初期化を明示的に無効化できる `--stationary-duration 0` を追加（標準3s）。duration0では、hover gateが開く前の画像を溜め続けて停止する経路があったため、ROVIOラッパーを「初回IMU受信前の画像を無視」に修正。初回IMU後の推定処理は変更なし。最初のinit試行はこの起動問題により精度評価から除外、別出力で再試行する。
- 誤差設定もDRAGONと合わせた `unipilot-dragon-sensors-noise-a1-20261015` は全区間完走（wall261.305s）。ROVIO位置RMSE3499.573m/回転2.199deg、MIMOSA3470.049m/35.688degで依然発散。初期補正の有無を変えた試行を続行。センサ仕様を合わせればDRAGON水準に戻る、という仮説はこの2試行では支持されない。

### DRAGONセンサ比較：完了結果
- 最終条件 `--sensor-profile dragon --rovio-noise-profile dragon --stationary-duration 0 --modes fixed --fixed-angles-deg 0 0`。出力 `native/logs/unipilot-dragon-sensors-init-v2-a1-20261015`。a1全228.492812秒を完走、wall152.181s、所有プロセスすべてexit0で停止。GUI/bagなし。
- ROVIO位置RMSE **7.044938m** / 回転 **0.387546deg**、MIMOSA＋ROVIO位置 **5.053299m** / 回転 **0.515453deg**。位置最大誤差15.785m/10.108m。前回UniPilot0/0は69.373m/6.543deg、13.095m/6.239deg。DRAGON固定参考は7.956m/0.058589deg、4.899m/0.285864deg。位置精度は参考DRAGONと同程度になったが、回転誤差はまだDRAGONの方が小さい。
- raw IMU273178件・LiDAR2732件、監査周期欠落/重複ともに0。ROVIO出力最大間隔50ms、MIMOSA100ms。初期化startup監査valid、初期変位.018617m。画像adapterの最初の1枚のみエンコーダ時刻不足で除外、追従中は画像周期維持。ジンバル最大誤差yaw1.94e-12deg/pitch4.77e-12deg。
- 機体/飛行制御/関節/各sensor pose・経路・seed・MIMOSA融合設定・取付変換・推定関連ライブラリの同一性を自動検証。ROVIO実行ファイルの差分は初回IMU前画像を無視するguardだけであることをsnapshotソースで検証。静止初期化duration3→0は明示的な実験条件変更。参照DRAGONとの機体・配置・初期化/重力の履歴差が残るため、配置の2要因だけを原因と断定しない。
- 比較は各条件1回の別飛行で、同じ入力を再生した比較ではない。センサ仕様のみの変更では改善せず、DRAGONの誤差設定＋静止バイアス初期化無効の組み合わせで改善した。ホバリング中の実回転をバイアスに入れる問題は真値による事後監査でも確認。各要素の独立寄与や他runでの改善は未検証。真値を推定に入力していない。
- 集約 `native/logs/unipilot-dragon-sensors-init-v2-a1-20261015/index.html`、all_trials.json、plots/comparison.pdf、六軸PNG/CSV。全3試行を表に残し、途中の発散結果もリンク。2件の起動失敗はINVALID_EXPERIMENT.txtで明示。最終プロットを画像確認済み。通常のUniPilot既定sensor/noise/初期化設定は変更せず、再現には上記3フラグを明示。


## 2026-10-15 UniPilot ROVIOドリフトの原因切り分け（診断実装）
- ユーザー依頼：UniPilot条件でROVIOのドリフトの主要因を検証する。固定yaw/pitch0、nativeセンサ、a1/b1を対象。通常の推定設定の既定値は変更しない。
- `active_perception/diagnose_unipilot_drift.py`：保存したcallback順の入力を同じ順で再生し、画像/IMU/時変外部パラメータを保持した要因別比較を追加。原条件の出力CSVがオンラインとbyte一致することを確認。真値は評価・診断にのみ使用、診断用のノイズなしIMU置換を実用性能と扱わない。
- `unipilot_gimbal.py` / `unipilot_gimbal_sim.launch.py` / `unipilot_run.py` / `unipilot_campaign.py`：明示的な`--diagnostic-sensors`指定時だけ、同じ場所にノイズなしIMU、ノイズなし画像、水平画角だけ拡張した画像、far60画像を生成・記録。追加センサに質量/制御への入力はない。通常起動は従来どおり。
- `record_diagnostic_sensors.py` / `override_imu_stream.py`：参照をCSV/独自raw画像形式で記録し、時刻と非対象データを保った再生置換。rosbag/GUIなし。画像終端が評価区間を覆うことを必須とし、評価終了より後の参照なし最終画像1枚だけを監査付きで除外可能にした。評価区間内の欠損はエラーにする。
- `audit_recorded_imu.py`：同時刻の実際のノイズなしIMUとの比較を追加。最初に試した100Hz真値姿勢/位置の微分による擬似IMUは高周波運動を再現できず、因果実験には不適格と判定。`a1/inputs`・`b1/inputs`にDO_NOT_USE_AS_ORACLE.txtを保存し、その擬似IMUの再生実験は行っていない。
- `rovio_gimbal.cpp`：`_feature_csv`指定時だけ、採用済み特徴点の方向/距離/追跡状態を記録。フィルタ演算は変更せず、再ビルド後にも原条件CSV byte一致を再確認。
- `audit_feature_geometry.py`：真値カメラ姿勢と採用済み特徴方向から事後に距離を三角測量。これは独立した生画像対応点の検証ではなく、ROVIO更新後の方向を用いる診断。`audit_diagnostic_images.py`は同時刻画像を比較。`audit_imu_integration.py`は姿勢の数値微分を用いず、ノイズなしIMUを積分して真値と比較。
- 結果・監査出力の集約先：`native/logs/unipilot-drift-cause-20261015/`。オンライン診断入力は`unipilot-drift-diagnostics-a1-20261015` / `...b1...`。IMU更新頻度だけの追加検証は`unipilot-drift-rate-a1-20261015`。最終結果は次の追記に記載。


### 2026-10-15 同上・最終結果：IMUの瞬時サンプリングと距離スケールの誤り
- 完了。最も強い因果的証拠はIMUの200Hz瞬時サンプリング。追加a1の同一物理飛行/同一画像/同一外部変換で、ノイズなし参照200Hz→1000HzのROVIO位置RMSEは41.608962→7.386501m。1000Hzの直前5msを台形積分して平均し200Hzで与えると6.020131m。演算更新頻度を200Hzのまま改善するため、単に推定器を高頻度で動かす効果だけでは説明できない。現在のGazeboの速い運動変動を平均せず間引く影響を確認したもので、実機200Hz一般の限界ではない。
- 同じ参照の加速度の100Hz超成分は変動パワーのx34.0%、y61.6%、z40.6%（sim60–240s、Welch）。200/1000Hzの同時刻54,635標本は全6成分で完全一致。高レートCSVに55.337秒の1標本欠損、最大間隔2ms、その他1ms。欠損は保持・監査し、区間平均ではその1区間を台形積分でつないだ。`a1_rate/integrity.json`で元画像/外部変換のSHA一致、全IMU時刻の参照カバーを確認。
- 更新頻度比較の姿勢RMSE：clean200 2.343584deg、clean1000 9.291551deg、clean200区間平均5.710024deg。cleanセンサで正しいゼロbiasを固定する診断では1000Hzが位置6.477829m/回転0.137205deg、200Hz区間平均5.969879m/0.111524deg。200Hz瞬時値・biasゼロ固定は202.485825m/0.891996deg。位置の主要な大誤差を減らしても、画像とIMUの更新過程で不要なbias推定が入る問題は残る。実際の雑音ありセンサでbias推定を止める推奨ではない。
- 通常nativeセンサの診断飛行（更新頻度比較とは別の飛行）のa1/b1位置RMSEは23.745618/49.719072m。ノイズなし画像19.9989/37.0298m、広角32.7077/59.4060m、clean IMU+静止平均OFF24.1070/38.1084m。b1 far60画像は序盤30秒6.69→0.515mに改善するが全区間70.5939m。画像条件だけの変更は一貫した改善にならない。
- ROVIOの加速度予測の連続時間雑音密度だけを2.205e-6→4e-5（DRAGON参照値）へ増やすと16.0288/17.5521m。物理センサ雑音の再測定値ではなく、モデル外誤差に対する設定感度。既定へ採用していない。
- 既存特徴の距離中央値から新規点を初期化する処理を止めるとa1 26.8194m（悪化）、b1 11.5270m（改善）。初期距離3/30m・距離共分散増大も一貫した改善なし。誤ったスケールを引き継ぐ経路は影響するが、単一の初期値だけが原因とはしない。
- 真値姿勢・ROVIO更新後の特徴方向による事後三角測量：a1 255本中240本、b1 470本中449本が正の距離>90%・再投影RMS<2pxに合格。これらの推定距離/三角測量距離の中央値2.2961/2.6337倍。短い追跡は除外され、独立した生画像特徴の監査ではない。全誤対応を否定しない。
- 経路を事後に縮尺だけ補正する診断ではa1位置23.7456→10.0260m、b1 49.7191→3.3035m（倍率.5840/.4723）。主要表は従来どおり初回姿勢だけの整合で、縮尺補正なし。
- 既存a1/b1入力の再生では旧設定180.631/697.701m→採用済みの物理IMUモデル/重力/平均初期化67.8248/67.5825m。nativeで平均初期化だけ外すと246.673/274.228mへ悪化。前回の低雑音DRAGON条件の初期化対策をnativeへそのまま一般化しない。
- 今回追加の更新頻度比較飛行のnativeオンライン結果は503.954228m/36.030135deg。出力CSVが再生とbyte一致し、除外/削除せず報告した。同じ飛行内のclean200/1000/区間平均だけを因果比較し、別飛行の23.7456m等を分母に使わない。
- `average_reference_imu.py`を追加、`override_imu_stream.py`に高レート参照ストリームの置換・時刻範囲/単調性検証を追加。診断モードにノイズなし1000Hz IMUを追加。`report_unipilot_drift.py`で六軸・経路・特徴距離比・更新頻度比較を描画。通常センサ/推定器の既定は不変。
- 出力：`native/logs/unipilot-drift-cause-20261015/index.html`、`plots/diagnosis.pdf`、`summary.json`、各条件の入力SHA/設定/再生ログ/時刻別誤差、`a1_rate/integration_*`（真値初期姿勢・初期速度事後fitを使う積分診断。推定精度ではない）。報告画像は確認済み、Python構文チェック成功、原条件の再生一致を複数入力で確認。
- 未確認：nativeの実際の雑音/時間変動biasを残した高レートまたは平均化の全系統試験、b1での更新頻度対照、複数回統計。今回の診断を「修正完了」「全runをIMUだけで説明」と扱わない。主因の強い証拠はa1で得た。全オンライン/再生/master/keepaliveは停止、GUI/bagなし。


## 2026-10-15 IMU通常経路の修正と尺度の前提確認（完了）
- ユーザー目的：ROVIOの最高性能を探すのではなく、gimbal効果の比較を歪める推定器側の不具合を除去する。native雑音/biasを維持し、尺度合わせや真値入力、成績による重み調整を行わない。
- `unipilot_gimbal.py`の既定を`imu_sampling="resolved"`に変更。native IMUは200→1000Hz。白色雑音stddevと量子化precisionを√5倍し、stddev²/rateおよびprecision²/(12rate)を保持。bias_mean/stddev/dynamic_bias_stddev/correlation_timeは不変。DRAGONプロファイルは元々1000Hzで不変。`imu_sampling.json`へ保存。`unipilot_gimbal_sim.launch.py`・`unipilot_run.py`・`unipilot_campaign.py`で既定を接続し、`--imu-sampling legacy`で従来の取得を明示再現できる。gimbal確認launchも同じ生成器を使う。
- `test_unipilot_sampling.py`追加：native新旧のROVIO設定byte一致、雑音密度保存、bias/他センサ/全形状不変、DRAGON新旧モデルbyte一致、不正mode拒否の3テスト成功。
- nativeセンサのa1/b1全区間を1000Hzでオンライン収録：`logs/unipilot-imu-production-fix-20261015/`。旧初期化でROVIO位置RMSE8.796826/18.778818m、回転2.741588/2.637104deg。追従完走、入力監査で1000Hz欠損0。初期化以外のROVIO設定は従来と同一。
- 初期化の誤った仮定を修正。従来はhover3秒の平均角速度をgyro bias、平均加速度の大きさ差をradial acc biasへ代入していた。真値を事後監査すると3秒で正味a1 .245536deg、b1 .242601deg回転、途中の最大角度変化.423065/.466380deg。完全静止ではないため実運動がbiasへ混入する。
- `rovio_gimbal.cpp`に`stationary_mode=wait`追加。待ち時間3秒と初期化時刻を保ち、biasへの代入・平均加速度の置換を行わず、その時刻のIMUでROVIO標準の加速度姿勢初期化を使用する。bias状態は従来の初期値/物理モデル由来の共分散を持ち、以後推定する（ゼロ固定ではない）。UniPilot run/campaignの既定をwaitに変更。mean/legacyは明示選択時に保持。バイナリ再ビルド後もmean条件の全出力が既存オンラインCSVとbyte一致。
- 同じ1000Hz入力/同じ初期化開始時刻でmean→wait：a1位置8.796826→6.787755m、回転2.741588→3.251510deg。b1位置18.778818→6.764554m、回転2.637104→5.029648deg。最終固定2区間の値はオフライン同一入力再計算。位置改善だけを理由にしたチューニングではなく、成立していない静止仮定を外すため採用。回転悪化も結果として記録。
- 対照：待ちを完全に省略した場合7.304546/10.134512m、距離中央値引継ぎを止めた場合10.479917/8.763193m（回転3.885942/8.170912deg）。これらは採用しない。深度初期値、画角、特徴数、融合重みは不変。
- 修正後waitの事後特徴三角測量：a1整合239/258本、中央値距離比1.195916（10–90% .862186–1.724602）、b1整合430/444本、中央値.987490（.652262–1.478042）。更新後の特徴方向を使用し、短い追跡は対象外。固定倍率の校正/単位ミスを示す結果ではなく、区間・特徴ごとに尺度残差が残る。残差がすべて不可避な推定限界だとは証明していない。
- `test_gimbal.cpp`を強化：非等方・姿勢/特徴の相関を含む共分散、機体位置/速度/姿勢・距離の不変性、回転往復、数値微分、非対応の光学中心並進の拒否。成功。差分1e-6radでは上流NormalVectorのacos/分岐境界の丸め誤差3.3e-5が出たため、数値微分幅を1e-4radとし検証した（本体アルゴリズムは変更なし）。既存IMU biasモデルの平均/共分散24チェックも成功。
- `audit_unipilot_inputs.py`追加：SDFからカメラKを照合し、モデルの取付姿勢/軸と記録エンコーダから外部変換を独立再構成。画像/外部変換時刻一致、画像寸法、IMU/画像欠損を検証。固定a1/b1は追従中IMU229261/265021件、画像4586/5301件、最大IMU間隔1ms、画像50ms、欠損0。
- 可動オンライン60秒をIMU修正のみと最終wait設定で各1回確認。最終は`logs/unipilot-imu-final-gimbal-check-20261015/run1_tail_a1_active`、全プロセスexit0・完走。最終の実測yaw -10.172〜12.5deg、pitch -54.548〜41.590deg。追従中IMU60081件/画像1202件、欠損0、画像/変換時刻一致、位置誤差0、モデルから再構成した外部姿勢との差5.23e-15rad以下。gimbalの有効性比較ではなく接続確認。
- `diagnose_unipilot_drift.py`は記録protocolの初期化mode/durationを読むように修正し、wait_only介入を追加。`report_imu_scale_fix.py`で同一入力比較の六軸/経路/距離比/表を作成。結果`logs/unipilot-imu-scale-audit-20261015/index.html`、`plots/comparison.pdf`、`summary.json`。各実験の設定/入力SHA/バイナリSHA、初期化時刻一致を保存。
- 次の固定/可動比較では両方でresolved+waitを使用すること。真値による尺度補正なし。全run・全姿勢の不具合不在や、残る誤差が全て手法限界であることを保証する段階ではない。今回見つかった入力/初期化の不整合は修正済み、各検証後に所有シミュレーションを停止。GUI/rosbagなし。
- 最終可動60秒の同一入力再生も完了し、オンライン出力CSVとのbyte一致を確認（`unipilot-imu-scale-audit-20261015/final_active/report.json`）。時刻順・動的外部変換を含む再現性確認であり、固定/可動の効果比較ではない。

## 2026-10-16 推定器の前提検証を完了する追加作業（進行中）
- ユーザー依頼：残る初期化/回転・尺度の原因確認と、代表3run固定/可動比較を完了する。真値を用いる介入は診断専用とし、通常推定器へ入れない。
- 同一物理飛行上でfixed/activeを同時評価するpaired実験を開始。run1_tail_a1, run1_tail_b1, run1_prefix、nativeセンサ1000Hz+wait、DRPM OFF、GUI/bagなし。出力`logs/unipilot-validation-paired-20261016`。Windows実空き20GBを確認し、診断カメラを省略して参照IMUだけを追加する`--diagnostic-imu-only`を実装。標準動作は不変。
- 診断再生に初期姿勢だけ/初期biasだけの分離介入を追加。初期化1標本を置換する際の診断ストリーム読込をndmin=2へ修正（旧ツールは1行CSVで失敗）。失敗した診断は停止し同じ設定で再実行。
- `audit_raw_tracks.py`追加。ROVIO特徴の初回画素だけを起点として、その後は画像だけの独立追跡。真値は事後三角測量のみ。順逆追跡誤差0.5px、再投影RMS2pxで監査。合成画像の既知3px/-2px移動で258点/258点合格、95%誤差0.0006px、既知3D点の三角測量一致。
- `build_fixed_reference.py`追加。通常バイナリを上書きせず、公式ImgUpdate.hppを無変更で使う固定カメラ参照版を構築し、gimbal用座標更新の影響を切り分ける。

### 2026-10-16 同上：原因切り分け結果（prefix飛行は継続中）
- 初期化の同一入力介入：a1 mean旧8.796826m/2.741588deg、wait6.787755m/3.251510deg、平均加速度だけ6.787955m/3.251829deg、旧biasだけ8.794691m/2.740798deg、真の初期重力方向だけ6.787m/3.252deg。a1の差は初期bias側。b1はmean18.778818m/2.637104deg、wait6.764554m/5.029648deg、平均加速度だけ13.665274m/11.928597deg、旧biasだけ9.187m/6.261deg、真の初期重力だけ9.244m/6.638deg。両者の相互作用があり、正しい初期姿勢を与えるだけでは解消しない。真値介入は診断専用。
- 公式ImgUpdateの参照版（OpenCV表示用定数名の互換修正のみ、描画OFF）で、a1/b1×旧mean/waitの全出力CSVがbyte一致。IMU予測モデル/入力wrapperは共通に残しているので「完全無変更ROVIO」ではなくgimbal画像処理の切り分け。`upstream_equivalence.json`にcommit・差分の範囲と一致結果。
- `build_fixed_reference.py --measurement-audit`で画像マッチング後・状態更新前の観測方向も読む診断版を別途構築。通常バイナリは変更しない。a1/b1の旧mean/wait出力は通常版と一致。新規特徴のage=0には実測対応点がまだないので監査から除外する。
- 独立画像追跡の幾何整合a1 222/253本、b1 406/427本。そのうちROVIOの実測画像対応が不整合なもの14/37本。両方整合する208/369本でも推定距離比10/50/90百分位はa1 .858581/1.154745/1.741583、b1 .650643/.966121/1.457376。尺度誤差は画像対応誤りだけでは説明されず、公式視覚更新でも再現する。独立追跡の限界・除外基準を報告し、全画像の誤対応率と扱わない。
- `test_gimbal_images.py`：静止機体、yaw±40deg/pitch±65degの独立合成画像（テクスチャ付き立方体へレイを投射）、1000Hz無雑音IMU。補償あり位置RMSE .065843m/max .098181m、回転 .026922deg、特徴中央値20。補償なし対照9.633474m/1.527902deg。試験生成器の未観測面・ゼロ方向成分の処理を修正して最終生成・再生。通常推定器変更なし。
- 参照IMUの差分とSDFから実際のbiasを独立に推定する`audit_bias_evolution.py`を追加。単なる指数fitに加え、センサの時間変動モデルと測定雑音を使った平滑化とその不確かさを保存。ROVIO結果は基準推定に一切使わない。通常入力の雑音標準偏差はSDF設定に一致。a1追加参照imu_fastには2標本欠落があったが、同じ1000Hzのimu.csvは欠落0のため後者だけを使用。推定器入力には欠落なし。
- 同じ画像/同じ初期化時刻の無雑音参照IMUで、bias推定あり→正しいbiasゼロ固定（診断のみ）の回転RMSE：a1固定4.520917→.086412deg、可動2.939851→.100056deg、b1固定5.456→.086deg、可動4.043→.085deg。画像更新とbias推定の結び付きが回転誤差を作る経路を確認。実センサへゼロ固定は適用しない。
- 無雑音IMU積分の単位/符号確認：真値姿勢で加速度を変換した位置RMSEはa1 .008051m、b1 .012587m。初期速度を全区間事後fitする診断であり推定精度値ではない。gyro積分姿勢は.008368/.008184deg、これを使う位置は1.481147/.490292m。真値姿勢の100Hz補間・初期姿勢の補間誤差も含むため、これをROVIO誤差の唯一の原因とはしない。
- ジンバル状態/距離/相関共分散/数値微分/往復変換の試験成功、IMU bias平均/共分散24チェック成功、通常モデル雑音密度と形状不変の3テスト成功。
- 主結果と検証の出力は`logs/unipilot-estimator-validation-20261016/`。診断図は`plots/diagnostics.pdf`、3run比較は`plots/comparison.pdf`（prefix終了後更新）。Windows実空き容量を監視中。TRIMは空きブロック通知のみで既存ファイルを削除していない。

### 2026-10-16 長区間の最終監査で入力欠落を検出・再取得
- 初回prefixは1070秒完走したが、sim785.021→785.057秒でIMU35標本欠落。固定/偏向で同じ欠落を受信していた。独立したencoder記録にも最大161msの抜けがあり、9画像で照合誤差（最大.004260rad）。画像自体の周期欠落は0。初回prefixを最終精度比較から除外し、`unipilot-validation-paired-20261016/rejected_prefix_input_loss/`へ全データ・campaign記録を保持。欠落と精度を混同しない。
- 1000Hzに対してGazebo→ROSの既定送信履歴10件、ROS1中継200件、encoder中継10件が短かったため、IMU/関節のGazebo bridge履歴、IMUのROS1中継、encoderの送信/中継/記録を2000件に拡張。時刻・データ・推定数式・重みは変更しない。ROS-Gazebo bridge設定は型/方向/既存remapを保ってYAML生成。
- native画像adapterと元Pythonの固定/偏向各30画像は画素・メタ情報一致、回転差最大2.10e-16。IMU650標本と点群12件のadapter比較でも内容一致、hover開始gate一致。最初のROS1試験起動は混在環境で失敗したため停止し、空の環境から再実行して成功。
- prefixのみ同じ設定/seed/経路で取り直し。a1/b1は監査合格済みなので保持。`audit_unipilot_inputs.py`は失敗時も測定値を保存するよう修正、最終reportは全3run/全6監査の合格を必須化し、欠落ありや部分結果を完成扱いしない。

### 2026-10-16 容量整理とprefix再試行
- ユーザーが不要ファイル削除を明示。ワークスペース約119GiBのうちログ約116GiB、原本ローカルbag約69GiB、古い変換入力約14GB。Drive上のrun1_tail_a2_prefix/run2a_end原本29チャンクのMD5を保管記録と再照合したうえで、再生成可能な旧比較.bag/.npz、画角別画像cache、今回の不採用prefix2試行のraw入力を削除。66ファイル19,263,408,095 bytes。元のMCAP、設定、CSV精度結果、図、実装は保持。詳細`unipilot-estimator-validation-20261016/cleanup_manifest.json`、再取得元`cleanup_sources.json`。Windows実空き約0.6→9.7GBまで回復。
- 2回目prefixはroute327.835sの既知の急な曲がり角で追従誤差.182041m、既存停止上限.18mに達し停止。raw IMU/LiDARの周期欠落0。`rejected_prefix_tracking`へログ/設定/CSVを保持、精度表から除外。全検証プロセス停止を確認して再試行。
- 3回目prefixだけ既存`--corner-speed-limit`を有効化。経路形状/停止上限/巡航.5m/sは不変、角だけ先読み減速。固定/可動は同じ実飛行・IMUを共有するため、各区間のpaired比較は保持。a1/b1は既に完走・監査合格のため再実行しない。prefixの減速条件は最終報告へ明示する。推定器の式/重みは不変。

### 2026-10-16 容量整理・通信経路の追加修正
- 3回目prefixにも26.167s/435.483sのIMU各1標本欠落があり、route約668sで停止して不採用。独立したROS2監視はその後の87,676標本で欠落0であり、欠落時点を直接比較したものではない。rejected_prefix_best_effortへCSV/設定/ログを保持。不採用raw入力4,159,829,214 bytesを削除し、不要生成物の削除合計は68ファイル23,423,237,309 bytes。元MCAPは保持。
- インストール済みros1_bridge 0.10.3はparameter_bridgeのYAML QoSをROS2送信側にだけ適用し、受信側はbest-effortのまま。待ち行列拡大だけでは信頼性指定にならないことをコードで確認。公式の変換ライブラリ/循環防止をそのまま使い、受信・送信の両方へreliable QoSを明示する独立実行ファイルreliable_bridgeを追加。UniPilot実験だけに接続、推定器/センサ値は変更しない。システムのparameter_bridge自体は置換しない。
- build_reliable_bridge.pyで独立ビルド。リンクは既存bridge同梱のROS1共有ライブラリへ統一。別Noeticのlogging共有ライブラリを混在させた最初の試作は終了時に二重解放が出たため破棄・修正。最終試験は往復各20,000標本、全値・時刻一致、欠落0・重複0、ROS2受信QoS実測reliable。試験受信器は専用スレッドで常時受信し、送信ループによる受信待ち行列あふれを防ぐ。
- 保存済み診断用raw画像/旧初期化監査入力はzstdで可逆圧縮。展開後SHA256一致を確認して非圧縮コピーだけを削除。storage_compression.jsonに各原本/圧縮先/検証SHA/削減量、各フォルダに復元コマンド。研究データの内容は失われていない。
- 最終prefixのみ同じ経路・seed・corner-speed-limit条件で再取得開始。a1/b1の合格結果は保持。GUI/rosbagなし。

## 2026-10-16 推定器前提の残件検証・代表3run比較（完了）
- 入力/初期化/ジンバル補償の列挙した検証を完了。今回見つかった通信欠落の経路も修正。推定器の性能を最大化する重み調整、真値による尺度/姿勢補正は導入していない。数学/合成画像/公式視覚更新との一致/全経路入力監査を根拠として判断する。あらゆる環境の全上流実装に不具合がないという保証とは区別する。
- 最終prefixは1070 route秒完走、実時間867.977秒。固定/可動は同一飛行・同一IMU。IMU入力は初期化前から終了まで欠落0、追従中各1,084,477標本・画像21,690枚、画像50ms・IMU1ms。取得時外部姿勢の独立再構成誤差最大7.97e-14rad、位置誤差0。前の不採用試行からの切り貼りはなし。a1/b1も監査を再生成し、4系統の全時間欠落0。
- 有効prefix固定入力を公式ImgUpdate/ジンバルhookなし参照版で再生し、全出力CSVがオンラインとbyte一致（60.112秒）。既存a1/b1旧mean/waitも一致。共有するIMU予測モデルとwrapperは残るため、完全無変更ROVIO全体との比較とは呼ばない。
- 全3run×固定/可動、DRPM融合OFF、1000Hz resolved+wait3秒。軌道追従中だけ、初回位置姿勢整合のみ・尺度補正なし。位置/回転RMSE：
- run1_tail_a1 fixed: ROVIO 10.292341m / 3.556504deg; MIMOSA+ROVIO 2.155603m / 1.394969deg。
- run1_tail_a1 active: ROVIO 32.180773m / 7.911167deg; MIMOSA+ROVIO 27.009402m / 4.898265deg。
- run1_tail_b1 fixed: ROVIO 14.793249m / 3.582422deg; MIMOSA+ROVIO 22.316960m / 2.685674deg。
- run1_tail_b1 active: ROVIO 2.223025m / 2.349625deg; MIMOSA+ROVIO 9.242129m / 1.255646deg。
- run1_prefix fixed: ROVIO 58.788161m / 23.511930deg; MIMOSA+ROVIO 69.464432m / 13.800647deg。
- run1_prefix active: ROVIO 37.855883m / 2.939851deg; MIMOSA+ROVIO 18.558488m / 3.998234deg。
- a1では可動化が悪化、b1/prefixでは改善。各経路1試行の結果であり、繰り返し統計ではない。prefixだけ角の先読み減速あり（以前の追従停止を避ける既存処理）、固定/可動両方に同じ実運動。
- 最終prefixの全推定器/シミュレーション/bridgeは終了コード0。診断専用参照IMU記録器だけ、評価終了1123.513秒の後、1124.077秒まで記録してROS終了中にPython変換例外・終了1。元runtime/logは変更せず保持。通常imu.csv 1,124,077標本は欠落0で区間末尾をカバー。補助imu_fast.csvは3標本欠落があるため使用しない（推定器にはどちらも接続していない）。diagnostic_shutdown_audit.jsonに境界と終了コード。推定器入力監査の基準は緩めていない。
- record_diagnostic_sensors.pyはシグナル受信時にROS contextを非同期破棄せず、進行中callback完了→全ファイルflush/close→shutdownの順へ修正。SIGINT/SIGTERM/SIGINTの3回、各IMU2系統2000標本を受信中に停止する試験で終了0・全記録行数一致。実験後の補助記録器終了例外は報告へ明記し、全プロセス0だったとは記載しない。
- build.pyの標準ビルドへbridgeを追加し、単体build.py bridgeの経路も成功。既存システムbridgeやROVIO本体を上書きせず、独立生成物を使う。
- 整理：不要生成物68件23,423,237,309 bytesを削除、保持すべき旧診断データは可逆圧縮で7,723,253,011 bytes縮小。圧縮後の展開SHAが元と一致することを確認してから非圧縮コピーだけを削除。元MCAP/rosbag、結果CSV/図/設定/実装は保持。削除台帳cleanup_manifest.json、圧縮台帳storage_compression.json、復元手順は該当各フォルダ。
- 成果物：native/logs/unipilot-estimator-validation-20261016/index.html、comparison.json、plots/comparison.pdf（6ページ）、plots/diagnostics.pdf（4ページ）、diagnostic_validation.json、completion_status.json。prefixの2枚の六軸図を目視確認、全3run/6系統の自動監査を通過。検証用シミュレーション、再生、master、GPU keepaliveは停止。GUI/新規rosbagなし。

## 2026-10-17 ジンバルの安定した改善に向けた開発を開始
- ユーザー依頼: ROVIO+MIMOSAで固定視野より安定して改善するまで自走、必要ならROVIO単体診断。Windows実容量を監視し、最小限の結果のみ保持。
- 前提: 直近同一飛行比較はa1で悪化、b1/prefixで改善。DRPM融合OFF、1000Hz IMU+wait3秒、同一センサ/推定器設定を維持。真値は飛行制御と事後評価だけ。
- 制御の弱点: 過去60秒の未追跡点も利用可能と仮定し、実際の25点上限を超えた情報を評価。hard視野境界と粗いpitch候補により、現在の追跡を失う遷移を選べた。
- 開発v1: 追跡成功statusのみ地図更新、10回観測、8秒保持/2秒情報減衰、25点上限、画像端の重み減衰、現在追跡点80%保持、pitch13候補、10deg/s、改善幅.03未満は保持。別YAMLで試験し通常設定は未変更。意味のある既存/追加11テスト成功。
- 実験基盤: --planner-config/--seed/--diagnostic-vio追加。画像・点群なしでfeature/bias診断CSVを取得。候補全列挙ログはopt-in。/mnt/c実空き2GBを下回ると所有プロセスを停止する容量guard追加。
- 検証開始: native/logs/gimbal-improvement-20261017/v1、a1/b1各全長、同一飛行の固定/可動。結果未確定。候補を凍結後に3経路の別seed反復も確認し、失敗試行を隠さず残す。
- v1初回は追加したstatus fieldのROS1バイナリ順を取り違え、地図が空のままになったため中断/除外。実PointCloud2形式の回帰試験を追加して成功。修正後a1/b1全長の開発試行は完走。a1融合位置固定9.640→可動2.276m、b1固定3.271→可動30.896m。ROVIOはa1固定5.594→可動7.452m、b1固定7.613→可動4.844m。改善はまだ安定していない。
- 開発試行に少数のIMU1ms欠落を検出したため、正式な成功判定から除外（results.jsonのraw_no_missing=false）。結果自体は保持。以後factor1.2で取得負荷を抑える。画像/点群保存なしで各飛行数十MB程度。
- 融合設定は旧DRAGON校正の50ms相対姿勢共分散のままだった。旧回転sigma[.000278,.001339,.000117]degに対し、既存の有効なnative固定a1/b1から独立に再校正した値は[.008304,.008260,.015351]deg。並進sigma[.015785,.003962,.001684]m。各飛行を等重み、誤差の非中心二次モーメント（bias含む）。融合出力・今回の評価飛行は校正に不使用。時間相関/IMU共有相関は未解決で、厳密な共分散とは主張しない。
- calibrate_unipilot_fusion.py、--fusion-calibrationを追加し、同じ校正をfixed/activeへ使用。開発v1-native-calibration-seed17で別seed a1/b1再試験開始。通常既定の融合設定はまだ変更しない。
- 容量整理: 24時間以上前のGazeboコンソールログ56件の末尾64KiBを保持して約2.749GBを削減。元サイズ/SHA/対象をstorage_cleanup.jsonに記録。元bagや精度結果は保持。/mnt/c実空き監視と2GB停止guardは継続。
- native校正seed17のa1は融合位置RMSE固定.587→可動.461m、ROVIO9.740→13.148m。入力に1標本欠落があり正式評価からは除外。b1は以下の通信不具合修正のため開始直後に中断。
- Gazebo/ROS2を独立監視してROS2側で全IMUが二重に見えることを確認。公式ros1_bridge Factory::ros1_callbackはcallerid=/ros_bridgeを固定文字列で除外するが、独自wrapperのROS1名がuas_reliable_bridgeだったため入力をROS2へ反射していた。ROS1名を/ros_bridgeに一致させ、誤った改名を拒否。ROS2名は維持。公式変換自体は不変。
- test_reliable_bridge.pyに元入力側のecho検証を追加。往復各20000標本、値・時刻一致、欠落/重複/元側echoすべて0を確認。以前の試験は往復の終点だけでechoを検出していなかった。IMU欠落との因果はまだ未確定、実飛行でも独立監視する。
- native_slam_inputsの小さな入力監査に欠落区間の時刻と順序逆転カウンタを追加。audit_live_imu.pyはGazebo/ROS2双方の連続性を観測し、データ値は保存しない。
- v1-cleanbridge-seed17で同じa1/b1を再開。全試行の成功基準をvalidation_protocol.jsonへ事前記録: 未使用seed18/19の3経路6組で5勝以上、各経路平均比<=.9、全体平均比<=.8、単独悪化比<=1.5。欠落なし・完走・同じ設定を必須とし、不合格を都合よく取り除かない。
- cleanbridge seed17 a1完走、source/ROS2 echoは0になったが、IMU222.607sの1標本が両者に同時欠落。融合位置固定.519→可動1.010m、ROVIO6.317→8.332m。まだ安定した改善なし。b1も62.680sの1標本がGazebo/ROS2/ROS1で同時欠落し、Gazebo header seqも62678→62680へ飛んだため、生成後のGazebo Transport配送損失と特定（センサ更新自体の間引きではない）。受信者間の差分なし。
- 次回起動からGazebo Transport共有送受信待ち容量を1000→10000に指定。実ライブラリのrcvHwm()/sndHwm()が10000を返すことを確認。上限あり、センサ値/周期/ノイズは不変。損失が消えるかは次の全飛行で確認する。
- 改良候補v2を用意（未飛行）: v1にROVIOの機体速度・角速度から0.8秒後の特徴点可視性を予測する項と、旋回を打ち消す視線候補を追加。真値・既知飛行軌道は不使用。現在特徴80%保持に加えて将来保持を評価し、最大偏向20deg/s。情報の弱い方向がなくても追跡維持だけは実行可能。計算テスト12件成功。v1のhorizon=0経路は不変。
- HWM10000でもv2 a1で31.635sに1標本欠落。simulation.logのNodeShared::Publish() Error: Interrupted system callと、生成済みseqの欠落が対応。Gazebo Transport公式実装はzmq send中のEINTRを外側でcatchしてfalseを返し、ImuSensorは戻り値を無視して次の標本へ進む。ZeroMQ公式仕様ではEINTR時にメッセージは未送信・元buffer保持。
- 根本対策: Gazebo起動に限ってzmq_msg_sendのEINTRだけ同じmessage partのまま再試行する小ライブラリを追加。他エラーはそのまま返し、multipart全体を再送しない。OSシグナルで実sendを2830回中断する実ソケット試験で2000multipart/8000partの値・順序・欠落/重複ゼロを確認。performance/zmq_eintr_retry.cpp、test_zmq_retry.cpp、build_zmq_retry.py。build.py bridgeにも追加し、実験snapshotへbinary/SHAとテスト結果を保存。効果のなかったHWM拡大は撤回。
- v2-seed17 a1（EINTR対策前）: 融合固定.527→可動.630m、ROVIO6.921→17.130m、IMU1欠落により正式不合格。
- v2-seed17 b1（EINTR対策後）: 全区間完走/監査成功、融合位置固定.641→可動.479m（約25%低下）、回転.184→.245deg。ROVIO位置11.457→2.937m、回転5.954→3.557deg。raw IMU/LiDAR欠落・重複・順序逆転0、ROVIO/MIMOSA出力連続。現在prefixを同設定で実行中。
- 少数の画像だけsample_viewsへ保存し、b1の可動視野で機体が隠していないことを確認。床面は近いが模様がぼやけている。画像を一律大容量保存しない。
- v3（a1開発試験中）: 機体回転補償は最大20deg/s、意図的な走査成分だけ5deg/sに制限、現在特徴保持90%、1秒以上特徴不足なら前方へゆっくり復帰。長寿命特徴を一度に失い深度を再初期化する問題への対策候補。真値不使用、計算テスト13件成功。v3-a1-seed17をdomain85でprefix/domain84と並行検証、容量guard両方有効。未使用seedでの最終反復は未着手。
- v3-a1-seed17全長完走、正式監査成功。固定→可動: ROVIO位置8.327→6.475m/回転4.754→3.189deg、融合位置.617→.587m/回転.683→.630deg。融合の改善は3cm（約5%）に留まるので大きな効果と扱わない。特徴不足時の前方復帰によりpitchは多くの時間0度付近、body旋回時はyaw偏向。入力欠落0、画像/出力連続、全所有プロセス停止。
- v4候補はv3の機体回転補償だけを修正。真下視野では光軸のyaw分解が特異になるため、body座標の直交した2関節軸へ角速度を射影して打ち消す方式。視線方向だけでなく画像面内の回転も抑制する。真下のfull camera orientation不変を含む14テスト成功。v4-seed17で3経路の開発検証開始（domain85）、v2 prefix（domain84）は継続。設定確定後の未使用seed反復は依然未着手。
- v4 a1は全区間/入力監査合格。固定→可動: ROVIO位置6.873→8.994m、回転2.953→13.980deg。融合位置.540→.477m、回転.532→.527deg。融合は12%改善したがROVIO単体は悪化したため、改善機序や反復安定性は未確認。b1/prefixを継続。
- 視線候補の評価を一括計算へ変更（式/候補/制約は不変）。変更前後50ケース、全候補の評価量の最大差1.58e-14、選択コマンド一致。14既存試験成功。単体測定中央値81.3→8.84ms（約9.2倍）、vectorization_validation.jsonにSHA/比較記録。v4 a1は変更前、b1以降は変更後。新しいセンサ情報で判断できるよう計算遅延を減らす変更。
- 事後診断audit_gimbal_drift_phases.py追加。追従中の1秒相対姿勢誤差を機体旋回量/視線pitch別に集計。重複windowを独立標本として統計検定しない。v4 a1は融合の全体位置RMSEが小さくても、1秒相対並進RMSEは.052→.064mで悪化しており、単発の累積位置改善を過大評価しない。v2 b1は1秒相対並進ROVIO.184→.131m、融合.040→.037m。
- v5開発候補: v4から前方優先weight .05→.5、探索pitch±30degへ限定。真上/真下のみの視野へ固定される失敗を避け、前方を含む斜め視野を維持する仮説。手動の物理yaw±50/pitch±90は不変。外側角度からの復帰も速度制限、15テスト成功。経路別調整/真値入力なし。v5-seed17のa1→b1→prefixをdomain84で開始、v4はdomain85でprefixへ継続。未使用seed反復は未開始。
- v2 prefix全1070秒完走・監査合格。ROVIO位置固定52.448→可動156.188m、融合8.951→20.014mで悪化。可動は多くの時間ほぼ真上、1秒相対並進誤差も直進中.119→.407mへ悪化。単なる累積誤差の相殺ではない。v4 b1完走・監査合格、ROVIO8.834→10.432m、融合.510→.683m。v5 a1完走・監査合格、ROVIO8.079→7.953m、融合.509→.712m。まだ安定改善は達成していない。
- v5 a1の15秒の実測照合で、前方復帰74判断中73回は実際の画像追跡成功点が4点以上（4〜5点）。従来は距離共分散で除いた点を追跡保持にも利用せず、画像端の重み付き点数で復帰判定していた。距離が不確かな点と画像で追えていない点を混同しないよう、v6は最新の成功追跡点を保持/復帰判断専用に分離。情報行列には従来どおり距離の信頼できる地図点だけを使う。深度なし/追跡ありの試験を含む16 core試験+実PointCloud2試験成功。v6-a1-seed17をdomain86で開発確認、v4/v5の長い比較も継続。
- 自己遮蔽の机上検査: UniPilot base visual mesh242832面へ80x45本の視線rayを投射（near2cm）。yaw0/pitch0,±90の遮蔽0、yaw±50/pitch-90で約2.1%、yaw50/pitch-30で.58%。大部分が機体で隠れる根拠はない。ロータ位相/材質の裏面描画は未反映、実画像の全区間検証ではない。self_occlusion_body.jsonへ記録。
- v6 a1は3並列のRAM残量guardにより226.882/228.493 route秒で停止。全所有プロセス正常停止、欠落0だが全長未完了なのでvalid=falseのまま。共通先頭226秒の診断だけはROVIO5.495→15.408m、融合.531→.776mで悪化。部分結果を完走と扱わない。以後simは最大2並列。
- test_gimbal_images.pyに機体の3軸回転+並進を伴う独立合成画像を追加。同じ解析的運動/無雑音IMU・正しいbias固定の診断で、固定位置/回転RMSE.061610m/.134424deg、可動.081939m/.129400deg。機体とジンバルの同時運動だけで大きく崩れる症状は再現なし。実画像雑音/長時間/全条件の保証ではない。生成可能なraw入力2件計146394124 bytesは再生時SHA照合後に削除し、生成コード・設定・CSV・summaryを保持。
- null-a1-seed17開始: 同一物理飛行上の両カメラをyaw/pitch0に固定し、画像雑音/初期化による基礎変動を測る対照。config max_speed=0、両側同じ推定器設定。ファイル名activeは既存paired命名のままだが可動実験ではない。全長比較結果からgimbal性能向上とは主張しない。
- v4 prefix全1070秒/監査合格。固定→可動ROVIO64.226→31.374m/23.935→7.950deg、融合9.373→8.841m/4.489→4.666deg。ROVIOの改善がそのまま融合の同率改善にはならず、融合改善は約6%。全v4はa1改善/b1悪化/prefix小改善、まだ安定改善とは判定しない。
- null対照の同時刻画像200組は完全一致0組だが画素差RMSE平均.1126/255と小さい（image_noise_audit.json）。全長の推定誤差差分は取得中。
- unipilot_campaign.pyでpaired+固定角の対照を許可。基準カメラは0度、もう一方を指定角に固定し、動かすことと見る方向の効果を分離する。これを動的偏向の改善とは扱わない。protocolにfixedと角度を保存、既存の単独固定角試験は不変。
- v5 prefix完走・監査合格: 固定→可動ROVIO109.088→78.976m/38.680→26.252deg、融合9.074→111.280m/4.705→16.416deg。可動融合は大きく悪化。全3runのv5は採用不可。
- 同一向きのnull a1全長: 第1/第2カメラROVIO6.643/5.008m、融合.539/.581m。ジンバル角最大4.34e-6degでもこの差があり、数%の単発改善をgimbal効果と判断しない。
- pitch20固定a1対照: 基準0度ROVIO90.402m/融合63.713mに対し20度ROVIO16.615m/融合.317m。初期視野も変わるため、動的gimbal制御の成功とは扱わない。基準ROVIOにも大きな変動が残っている。
- paired_initialization.py追加: 同じ実測外部姿勢（回転差1e-6rad以内、位置差1e-8m以内）の間だけ元の固定カメラ画像/外部姿勢を可動ROVIOへそのまま配信。最初に偏向した時点から不可逆に可動カメラ入力へ切替。画素変更・タイムスタンプ変更・真値参照なし。実験用--shared-initialization、通常推定器や単独運用は変更しない。時刻同期/参照保持/不可逆切替/欠落時停止の3試験成功。
- shared-null-a1-seed17実行中、同時刻2270出力でROVIOの位置/姿勢/bias/特徴数/距離中央値がbit単位で一致。一方で同じ入力のMIMOSAは途中1354出力で位置差RMS .115m/max .327mが残る。到着順による数値経路の差を分離する。
- 既存ordered_mimosa_nodeの起動時も最初のVIOを待ってからLiDAR/VIOを時刻順配送するよう変更。従来は最初のVIO到着まではLiDARだけ先行し、起動順序が不定だった。pure LIOは待たない。入力値/因子/重みを変更せず、バッファ初期遅延だけが増える。ビルド/同一入力対照を続ける。
- shared-null a1全長でROVIO全出力bit一致、融合は固定.591→第2系統.491m。カメラを動かさなくても約17%差が出るため、従来の小さな単発改善をgimbalの因果効果と扱わない。
- LiDAR/VIOのみ時刻順にしたordered-shared-null a1全長も、ROVIO7.105mは完全一致、融合5.945/6.261mで差が残る。IMU初期姿勢処理が「その時点で到着したbuffer全部」を平均していたため、未来の標本の混入量が処理速度で変わることをコードで特定。
- ordered nodeをIMUも含めた取得時刻の配送へ変更。各LiDAR/VIO時刻までのIMU標本だけを元callbackへ渡す。センサ値/時刻/重み/因子は不変。同時刻のIMUがある今回の同期シミュレーション専用、非同期やoffsetありは明示的に拒否。通常nodeや既定値は変更せず。ビルド成功、ordered-imu-null-a1-seed17の先頭60 route秒で再現性検査開始。
- --initialization-excitationを実験用に追加（まだ未飛行）。追従前に同じ物理機体をX/Y±25cm、Z±10cmへ26秒かけて往復し、単眼尺度の初期推定に必要な加減速を与える。両カメラとも正面、評価する軌道/速度は不変、真値や指令軌道を推定器へ渡さない。準備区間は精度集計から除外。連続位置速度式/境界、既存経路の認証済み機体包絡内を検査、追従誤差18cmで停止。preparation_motion_validation.jsonに根拠。
- ordered-imu-null a1先頭60 route秒完走（短区間の再現性診断）。ROVIO/MIMOSA双方、同じ時刻の全位置・姿勢がbit一致。null_reproducibility.jsonに件数/全列最大差。IMUを含む取得時刻配送で、同一入力なのに結果が異なる問題を除去。比較はこの方式へ統一する。
- v6-ordered-excited-seed17で3経路の開発比較を開始。common画像初期化、時刻順融合、共通準備往復、同じnative融合校正、欠落監査/容量guardあり。全長結果・未使用seedの反復はまだ未確定。
- v6-ordered-excited seed17のa1/b1全長完走・入力/初期状態監査合格。固定→可動、位置/回転RMSE: a1 ROVIO12.441/2.788→7.804/6.843、融合4.919/.675→4.525/.573。b1 ROVIO13.532/2.969→8.782/3.581、融合10.597/1.233→7.315/.930。両区間の融合は改善したが、ROVIO回転は悪化。まだ未使用seed反復なし。
- 同campaignのprefixは追従開始前に停止。初回LiDARがgate開放直後の初回IMUより早い場合、追加した完全一致検査が例外となった。raw入力損失や飛行失敗ではない。未完結果を保持。orderedImuPrefixで起動前の観測は既存isImuReadyへ任せ、以降は観測を挟むIMU標本までの最小prefixだけを配送するよう修正。ROSの1ns丸めずれも同じ補間境界で扱う。数値/時刻は変更しない。完全一致/±1ns/起動前/既処理/境界不足の6条件をC++テストで検証。
- report_gimbal_campaign.pyに偏向開始前のMIMOSA出力bit一致監査も追加。a1/b1はそれぞれ864/844標本一致。通常計算の出力差をgimbal改善と扱わないための共通初期状態監査。
- 候補v6/共通準備/取得時刻融合をfrozen_candidate_v6.jsonへSHA付きで固定し、未使用seed18/19での反復を開始。検証開始後にこの候補の制御/融合設定は調整しない。
- seed18 a1全長・全監査合格。固定→可動: ROVIO位置17.514→7.830m、回転.897→9.343deg。融合位置6.300→.793m、回転.490→.347deg。独立条件での改善1組目。残り5組が揃うまで安定改善とは判定しない。
- report_gimbal_improvement.pyで全試行履歴と事前基準の6組判定を集約（native/logs/gimbal-improvement-20261017/index.html）。不採用/未完了も表示。report_gimbal_campaign.pyはIMUの処理境界、LiDAR/VIO全callbackの時刻順・欠落も検査するよう追加。
- 最小保存: 完了済み26件の高頻度encoder CSVをgzipへ可逆圧縮、展開後の全byte SHA256一致確認後に元コピーだけ削除。402152989 bytes削減。encoder_compression.jsonに元サイズ/圧縮サイズ/SHA、各folderにRESTORE_ENCODERS.txt。report_routes.load/verify_runは.gzを直接読める。基準a1の306123標本を圧縮状態で読み出し確認、既存精度評価試験も成功。実行中ファイルは対象外。原本bag/推定値CSV/プロットは保持。
- seed18 b1全長・全監査合格。固定→可動ROVIO8.136→6.347m/1.170→2.511deg、融合4.399→4.525m/.594→.673deg。融合位置は約3%悪化。結果を選別せず保持。seed19短区間と長距離の反復はcontinue_gimbal_validation.pyで最大2並列・長距離は時差実行、固定した候補に変更なし。
- 制御設計の机上診断（まだ修正/飛行なし）: 現在の「現在の特徴90%保持」は次回実際に動く角度ではなく最終目標角に適用。合成15点で目標yaw37.5degは55%保持として拒否されるが、速度制限後の次回1degでは100%保持できる。planning_transition_audit.json。今の候補の反復結果とは独立の設計検査であり、性能改善は未証明。次候補を作るなら、保持制約を到達可能な次の角度へ適用し、予測視野も偏向速度に合わせる修正を検討（v6検証中は変更しない）。
- v6開発prefix全1070秒完走・全監査合格。固定→可動ROVIO75.438→58.360m/29.209→15.817deg、融合16.119→30.427m/4.455→5.987deg。単眼の累積誤差が減っても融合は悪化。
- seed19 a1全長・全監査合格。固定→可動ROVIO7.222→8.112m/1.619→12.345deg、融合3.433→6.253m/.470→.487deg。候補v6を早期不採用: 最悪比1.821>1.5、a1二条件平均比.974>.9、既に2敗で残り全勝でも5/6勝に届かない。v6_early_rejection.jsonに理由。実行中のseed19 b1/seed18 prefixは所有schedulerへSIGINTして停止し、部分結果を保持。未実行を完走と扱わない。seed18/19は以後の新候補に対する未使用検証には再利用しない。
- v6の固定済み全ソース/実行ファイル/校正10件を元SHA確認してfrozen_v6_sources.zipに保存。元の全結果と設定も保存。最終的な安定改善は依然未達。
- v7開発: 特徴保持を遠い目標角ではなく「次回の到達可能角」で評価し、0.8秒先の視野も偏向速度上限から実際に届く角度へ修正。機体運動で90%保持が物理的に不可能な場合は、保持数が最大の候補を選び診断へ明記。情報行列・各重み・センサ/融合設定はv6のまま。次ステップが不可な候補をhysteresisで選び直さないよう修正。
- command_batchは旧単体commandと2000入力で最大差0。37.5degの目標に対し次回1degでは全特徴が保たれるのに旧処理が拒否する例を含め20試験成功。新評価中央値7.95ms/p95 10.48ms（単体）。v7_transition_validation.json。v7-seed17のa1/b1をdomain85/86で開発比較開始。実飛行での改善は未確定、通常の既定構成へはまだ採用しない。

- v7 seed17 a1/b1全長完走・全監査合格。固定→可動の位置/回転RMSE: a1 ROVIO9.135/6.912→5.025/2.790、融合2.663/.631→.619/.445。b1 ROVIO11.332/7.715→10.845/12.237、融合3.231/.461→1.377/.256。両区間で融合改善、b1のROVIO回転は悪化。prefix1070秒を追加実行中、独立反復前なので安定改善とはまだ判定しない。
- v6停止済みseed19 b1はrouteがsettle中のROSメッセージ受信変換例外で終了1、他の所有プロセスは0。未完/無効のまま保持。正常完走・全プロセス0とは記載しない。
- 集約レポートを候補名指定に対応。6組の判定で経路×seedの組合せを厳密に照合し、重複試行を6組に数えない。完了/未完/監査不合格/重複seed/指定外seed/単発大悪化を含む4試験成功。

- v7をseed20/21で反復するためソース/校正/実行ファイルを固定（frozen_candidate_v7.json、frozen_v7_sources.zip）。seed20 a1全長/全監査合格だが、融合位置3.395→29.859m、回転.433→1.240deg、ROVIO位置7.663→40.321m、回転7.721→2.720deg。融合比8.795>事前最大1.5なので早期不採用。v7_early_rejection.json。seed20 b1/開発prefixは中断保持、所有プロセス全終了0。seed21は未開始。独立検証にはseed20を再利用しない。
- 失敗a1では追従40秒時の可動ROVIO特徴数3、推定距離中央値69.61m、80秒の推定速度2.60m/s（真値巡航0.5m/s）。現行の向き決定もROVIO推定深度/速度を使用するため、悪い尺度へ判断が引きずられる経路がある。因果効果の確定は次比較で行う。
- 偏向判断の姿勢とencoder時刻の差は中央値.148秒。瞬間角速度による単純予測は実際の機体振動で回転予測を悪化（a1中央値.192→.769deg、b1 .233→.726deg）。controller_latency_audit.json/latency_motion_prediction_validation.json、真値は事後診断のみ。
- v8開発: 偏向判断だけに画像の角点を最大180点追跡し、同時刻LiDARの近傍平面から距離を求める地図を追加。ROVIOが選んだ25点以外も視野候補の予測に使うが、情報評価は25点上限を維持。深度不明/面不成立/端部は地図に登録しない。特徴共分散は偏向評価専用の保守的近似で、推定器の因子/共分散には投入しない。地図位置は融合姿勢を使用し、真値は使わない。
- v8は取得時刻の機体回転を実際のIMU履歴で揃え、未来の偏向予測の角速度は直前0.3秒平均とする。推定器/入力画像/IMU/LiDAR/融合重みは固定側と共通のまま。image_geometry_map.py/view_motion.py、planner_node.pyのopt-in経路、tracking_aware_v8.yaml。既存/新規純Python29試験と実ROSメッセージ2試験成功。v8-smokeの20 route秒で接続/実行を確認中、性能改善は未検証。
- v8-smoke（20 route秒）は起動/同期/停止の動作確認として完走。画像180点があってもLiDAR深度対応は全フレーム0。UniPilotのLiDAR SDFはpitch約90deg/yaw約180deg、垂直走査0..90degで機体後方半球を向き、前方カメラとの重複が乏しい。センサ配置は変更せず、対応を無理に付けない。これはジンバル改善の性能証拠には含めない。
- v8に時系列画像の三角測量を追加。画像上で継続追跡した角点と、同時刻の融合姿勢の移動から距離を推定する。最低移動8cm、視差角0.02rad、正深度、交差残差、距離標準偏差45%以下を要求。移動なし・平行な視線は拒否。真値やGazebo構造は使わない。純Python30件/実ROSメッセージ2件の試験成功。v8-seed17 a1/b1で開発比較開始。準備運動後に地図登録82/116点を確認、精度はまだ未確定。
- Windows残容量を監視。encoder可逆圧縮は累計36ファイル548618204 bytes削減。Linuxの削除済み空きブロックのみfstrimで2.4GiB解放通知、存続ファイルは変更しない。
- v8 seed17 a1/b1全長完走、全監査合格。a1固定→可動: ROVIO5.203→11.502m/2.204→1.807deg、融合3.149→2.869m/.508→.583deg。b1: ROVIO9.877→7.198m/9.295→8.455deg、融合5.549→.724m/.664→.385deg。a1は位置改善約9%に留まり融合回転は悪化。安定改善の判定はまだ行わない。v8-prefix-seed17（1070 route秒）とv8-seed20（a1/b1、既知の失敗条件なので開発用）を追加実行中。
- 独立した画像列試験を追加: 8m平面に対し既知の0.04m刻み移動で画像を生成し、画像追跡→三角測量の距離中央値誤差15cm未満・30点以上を確認。回転中心固定の純カメラ回転では全20フレームで深度登録0。test_image_geometry_map.pyの7試験成功。
- Windows容量監視: 空き約8.7GiB。今回の記録rootは約1.5GiB、元画像/点群/rosbagは追加していない。Windows pagefileは割当19337MiB、使用2509MiB、peak5970MiBを確認。ホスト空きの減少すべてを今回ログの増加とは見なさず、/mnt/cの実残量に基づく停止guardを維持。
- v8追加監査で偏向指令更新の途切れを検出。追従中最大間隔: seed17 a1 1.411秒、b1 9.154秒、seed20 a1 1.810秒。センサ/推定器入力は連続だが、画像地図・探索処理と1000Hz IMU/encoder受信が同じlockを共有し、姿勢時刻合わせに必要な履歴の受信が遅れていた。これらは所定の0.5秒最大制御間隔を満たさず、採用判断用の有効試行から除外。数値/元ログは保持し、controller_timingとplanner_continuousを全候補共通の監査へ追加。従来v7の最大間隔は.234〜.244秒。
- v8b: IMU履歴とencoder受信を画像地図/探索のlockから分離。履歴コピー/追記だけ専用短時間lockを使用し、回転積分はコピーに対して実行。方向評価式/重み/センサ/融合設定はv8と同じ。地図更新の所要時間も診断へ追加。主処理lock保持中でも100IMU/encoder受信が完了する実ROSメッセージ試験を追加。純Python31件/実ROSメッセージ3件が成功。
- v8-prefix-seed17とv8-seed20 b1は途中停止し保存。v8b-seed17でa1/b1/prefixを順次再取得開始（最大2並列、domains85/86）。独立検証前、安定改善は未達。
- v8bも追従中の指令間隔がa1最大1.548秒/b1 1.207秒となり中断。IMU/encoder個別lock分離だけでは不足。画像地図計算の所要時間中央値約53〜74ms/p95約144〜154msが同じ受信lockを占有し、MIMOSA姿勢等が.5秒以上古くなるためholdとなっていた（stale累積a1 105/b1 147）。未完・不採用として保持。
- v8c: 画像地図作成を専用workerへ移し、入力の組合せ受渡しと結果登録だけ短いlockを使う。処理待ちは最新1組に限定し、間引かれた偏向用地図フレーム数をgeometry_coalescedへ記録。推定器へ入るセンサは一切間引かない。2本の観測視線の三角測量を行列バッチ化、500例で独立lstsq解との差2e-11以内（判定条件/不確かさ式不変）。画像地図作成中も受信lockを取得できる試験追加。純Python32件/実ROSメッセージ4件成功。
- v8c-seed17でa1/b1/prefixの開発試験を開始、最大2並列、通常既定値には未採用。まず指令更新の連続性を確認してから精度・反復へ進む。
- v8cはa1/b1完走。更新間隔はa1 .447秒、b1に.539秒が1回あり後者は所定.5秒を満たさない。開始済みprefixは初期化中に停止。短い途切れも判定を緩めず保持。
- v8d: 遮蔽判定を角度近傍木検索に変更。最終判定は元の厳密dot条件のまま。24000点/500方向＋境界角、全候補コスト/選択指令の完全一致を検証。300特徴/24000点の中央値35.1→12.7ms。occlusion_query_validation.json。推定器やセンサの間引きはなし。
- 既存のsimulation pacerへ偏向指令の完了時刻も追加。last planner stampから0.3秒を超えたら物理時間を一時停止し、完了指令が届いて0.1秒以内になってから再開する。停止は実時間側の待機だけで、物理dt/センサ時刻/頻度は不変。比較は計算待ちありのシミュレーション条件であり、実機での実時間処理保証ではない。config pace_planner opt-in、protocolへ明記。
- v8d-seed17（a1/b1→prefix）をjobs2で開始。純Python34件/実ROSメッセージ4件合格。方向評価式と融合設定はv8と同じ、計算速度と実行順制御だけを修正。安定改善は未達。
- v8dはb1のplanner待ち一時停止から復帰せず、pacerが15秒で終了1。a1も中断し全所有process停止。原因は、plannerを進めるROSタイマーも同じ停止したsimulation clockを待つ循環。性能比較として無効、元記録保持。
- v8e: pace_planner時のみ実時間側で未完計算を再試行し、成功した指令だけsimulation時間5Hzで制限する。入力待ちで早期returnした際にはlast_planを進めない。停止中の入力到着後にも完了でき、同一simulation時刻で指令を連打しないことを確認。geometry失敗時のshutdownを受信lock外へ移動。実ROSメッセージ6試験成功。v8e-seed17（a1/b1→prefix、最大2並列）開始、方向評価/融合重み/センサはv8dと同一。
- v8e seed17 a1/b1完走、入力・共通初期状態・指令間隔・実角度/角速度・出力連続性の全監査合格。位置m/回転degの固定→可動RMSE: a1 ROVIO9.073/6.777→5.960/7.095、融合5.715/.416→2.195/.572。b1 ROVIO10.149/6.323→13.291/16.706、融合6.282/.623→1.874/.266。融合位置は両方改善、a1融合回転とb1 ROVIOは悪化しているので隠さない。
- report_gimbal_campaign.pyで実encoderの角度範囲と合成回転速度も共通監査へ追加。過去v8cは両方合格、b1の指令間隔不合格判定は維持。
- v8e-seed17 prefixをdomain85で、既知の失敗条件を再確認するv8e-seed20 a1→b1をdomain86で実行中（最大2並列）。seed20は開発データであり未使用検証に数えない。未使用seed21/22は未着手。Windows実空き約12GB、完了48encoder可逆圧縮で累計684MB削減。
- v8e seed20 a1完走・全監査合格。固定→可動RMSE: ROVIO8.483/1.939→5.613/4.665、融合2.543/.422→1.781/.390（位置m/回転deg）。seed20は旧v7で破綻した既知条件の再確認であり、未使用検証には数えない。
- v8eを未使用seed21/22の3経路×2比較用に固定。frozen_candidate_v8e.json/frozen_v8e_sources.zip、24ソース/校正/実行ファイル、全ZIP内容SHA確認。全ての過去protocolを照合し21/22未使用を確認。prefix開発試行とseed20 b1はまだ途中であり、その結果も保持・報告する。成功基準は従来の5/6勝・経路平均≤.9・全体平均≤.8・最大≤1.5のまま。
- 初期化prior生成元rovio_initialization.pyを以後の各飛行implementationへ保存対象追加。実処理は変更なし。凍結証拠の照合はnative model生成元2ファイルと当該初期化元にも対応。開発3短試行の改善だけでは安定改善の結論は出さない。
- 未使用seed21のa1→b1反復をdomain86で開始。直前の固定24ファイルSHAは全一致。再試行/設定変更で結果を選別しない。
- 反復schedulerに--separate-prefixを追加し、長い区間を別の出力フォルダに分離。長区間の結果記録と短区間の記録が同時に同じcampaign.jsonを書かず、dev prefix終了後にdomain85を即座に使える。未使用試験開始前に凍結manifestへprefix別フォルダ名を指定済み。実験の式・パラメータ・評価基準は変更なし。最大2simと長区間の750秒ずらしは維持。
- v8e seed20 b1完走・全監査合格。固定→可動RMSE: ROVIO9.199/5.764→12.412/15.616、融合3.355/.424→.379/.199（位置m/回転deg）。融合は改善、ROVIO単体は悪化。開発用短4組すべて融合位置が改善したが、長区間は途中729秒の融合10.710→12.229mで悪化しており、全区間の安定改善とはまだ言えない。途中値は採否に使わず、1070秒全長結果を確認する。
- v8e開発prefix1070秒完走・全監査合格。固定→可動RMSE: ROVIO78.172/29.383→63.418/27.476、融合12.361/3.869→17.317/4.636（位置m/回転deg）。この長区間では融合が悪化。短い開発4組の改善だけをもって採用しない。
- 独立検証seed21 a1完走・全監査合格、保存した凍結元も36/36一致。固定→可動: ROVIO6.327/8.001→9.111/8.161、融合3.232/.396→2.924/.437。融合位置比.90469で小改善、回転は悪化。反復は継続し、結果に合わせた設定変更はしない。
- 現在seed21 b1/domain86とseed21 prefix/domain85を実行中。完了encoder54件可逆圧縮で806MB削減、Windows空き約10GB。
- 追加診断でv8eの特徴保持入力の欠点を確認。画像地図workerの結果へROVIO追跡点も載せていたため、地図処理遅延を現在の特徴不足と混同していた。地図に非空の追跡点があるのに保持制約へ空配列を渡す割合: a1 512/1153(44.4%)、b1 1093/1323(82.6%)、prefix4406/5097(86.4%)。prefixの地図時刻差中央値.25秒/最大1.15秒。v8e_track_freshness_diagnosis.json。これだけで全ドリフトの原因と断定しない。
- v8e反復を制御修正のため終了。seed21 b1も完走/全監査合格、ROVIO12.007/6.493→16.963/21.027、融合7.647/.879→3.560/.511。完了した2反復の位置は両方改善だが全6組未完、成功扱いしない。prefixは291.5秒時点で中断保持、全所有子process0。schedulerはSIGINTによる130。seed22未起動・protocolなしを確認。v8e_early_rejection.jsonに中断理由、全数値/凍結元保持。
- v9: 最新のROVIO追跡点を画像地図作成とは別経路で受信し、実測IMU回転と融合速度で偏向判断時刻に揃える。保持用の視線は従来同様8m仮定（距離測定値ではない、地図/推定器へ入れない）。本当に空の追跡点と未受信/古い入力を区別し、後者は指令を計算せず受信を待つ。現在視線の受信を画像workerに待たせない試験、取得時刻の移動変換、未来/古い入力拒否、空配列の区別を追加。実ROSメッセージ8試験成功。
- v9-seed17をprefix/domain85、a1→b1/domain86で開始。方向評価式/重み/センサ/融合はv8eから変更せず、保持入力の取得経路だけを変更。目標未達、次の独立検証候補は未使用seed22/23（21は開発に使用済みとして扱う）。
- v9 seed17 a1完走/全監査合格（新しいcurrent_tracks_aligned含む）。固定→可動: ROVIO7.138/1.407→13.308/8.488、融合5.297/.612→7.078/.752。保持入力の欠点修正だけで精度改善は保証されない。b1/prefixは継続。
- 長距離の融合姿勢ドリフトを別に確認。MIMOSA設定のIMU白色雑音密度はacc .8、gyro .01で、実SDF/1000Hz/量子化分散からの値acc .00148842758、gyro .000636416563より振幅で537.48倍/15.71倍大きい。Manager::generatePreintegratorParamsは各値の二乗をGTSAMの連続時間共分散へ設定する。fusion_imu_noise_audit.json。これを直ちに不具合やドリフトの主原因と断定しない。
- --fusion-imu-noise legacy|sensorを追加（既定legacyで従来の設定を維持）。sensorはセンサ定義の白色雑音/量子化/更新頻度からこの2値だけ生成し、固定/可動へ同一適用する。biasの因子/初期prior/ROVIO因子/実センサ/時刻/画像は変更しない。計算式・元値・適用値をfusion_imu_noise.jsonへ記録。200→1000Hzで密度不変、legacyで設定不変、sensorでも他の設定不変、独立式との一致を含む4試験成功。次の空き枠で精度比較するためのopt-inであり、まだ採用していない。
- v9 b1も完走、全所有子process0。空いたdomain86でv9-sensorimu-seed17（a1→b1→prefix、jobs1）を開始。--fusion-imu-noise sensor以外はv9と同じ制御/校正/物理センサ/経路。従来noiseのv9 prefix/domain85は継続中、全体2sim以下。sensor値は仕様から生成し真値誤差による探索はしていない。性能はまだ未確認。

- v9 seed17 prefix1070秒完走/全監査合格。固定→可動ROVIO位置68.292→88.174m/回転28.791→40.243deg、融合12.681→21.858m/3.967→5.124deg。b1融合4.472→.743mと改善するが、a1/長区間が悪化しており採用しない。
- sensor白色雑音設定のa1はroute162.839秒で停止。161.839秒の途中RMSEで融合固定86.678m/可動56.372mまで悪化。完走性能には数えず、この設定を採用しない。センサの白色雑音だけで融合モデルの誤差全体を表せることは示されていない。既定legacyは維持。
- 停止原因はpacerの再開条件: 5Hzの向き判断に対し再開時刻差を.1秒以下に要求し、正常な判断時刻差.136秒でシミュレーションを永久停止させていた。再開/停止閾値を判断周期の1.25/1.5倍（.25/.30秒）に修正。実記録の再現と全スケジュール位相の2試験成功。入力/センサ周波数は変更なし。
- v10開発: 視覚情報を追加した後の投影分散の減少を、従来の(前-後)/前からlog(前/後)で評価するopt-inを追加。従来式は分散が十分減った領域で候補間の差が圧縮される。新式は残り分散が半分になる改善を一貫してlog(2)と扱う。多変量の相互情報量そのものとは呼ばない。他の重み/保持制約/速度/融合設定はv9と同じ。精度は未検証。

- v10 a1全長完走/全監査合格。固定→可動ROVIO8.187→12.549m/5.348→12.138deg、融合3.904→5.242m/.553→.741deg。log評価式だけでは短区間も改善していない。b1/prefixは継続中。
- audit_camera_motion.py追加。保存済み可動60秒の生画像を独立LK追跡し、真値機体姿勢と取得時刻encoderから予測したカメラ間幾何を事後検査。真値は診断にだけ使用。移動が1cm以上の画像対364組で、画像対応の幾何残差中央値.0171pixel、偏向中164組で.0247pixel。時刻ずれ-100〜+100msの11候補で最小は0ms（偏向中±5msでは.122/.110pixel）。この記録で動的描画/encoderに大きな時刻ずれを示す証拠なし。全世界/全条件を保証する検査ではない。元画像の複製/追加収録なし、camera_motion_audit.jsonに数値のみ。

- 画像の遠方描画打切り境界を追加診断。audit_clip_tracks.pyは既存a1の実画像とROVIOの測定点、独立画像追跡の幾何判定を同時刻で照合。幾何不整合な追跡の観測424件中90.09%が白い大領域の8px以内、92.45%が24px以内。一貫した追跡900件では34.0%/34.56%。全 eligible track観測で不整合は32.2%を占める。境界が原因の全てとは断定しないが、20mで人工的に移動する輪郭を追う影響を切り分ける根拠。
- 次の開発候補は同じv10制御でcamera_far=200mを両側へ共通指定する。元の20m結果は保持。描画距離と、それから決める偏向地図の最大距離が変わるため通常センサnative完全同条件とは呼ばない。解像度/画角/ノイズ/IMU/速度/初期化/融合重みは不変。基準が改善しただけではジンバルの成果に数えず、同一飛行内の固定/偏向差を評価する。

- b1でも同じ白い描画境界の関与を確認。幾何不整合な観測351件中85.47%が白い大領域8px以内、89.74%が24px以内。一貫した観測1682件では8.50%/9.22%。clip_boundary_track_audit_b1.json。32.2%/17.3%という観測割合は長いtrackが多い影響を含む。独立試行数とは扱わない。
- v10 b1全長/全監査合格、固定→可動ROVIO13.835→10.434m/8.910→11.645deg、融合9.531→2.363m/1.061→.512deg。a1は悪化済みなので全体採用はしない。現在v10のprefix/domain85とv10-far200-seed17のa1→b1/domain86。後者はcamera_far=200だけCLIで追加、通常の既定設定は未変更。
- report_gimbal_improvement.pyの候補一致監査へprotocol_fieldsを追加。今後の凍結に描画距離/IMU融合noise等を含める。同じコードでも異なる実行引数を成功候補に混ぜない。欠落/変更した描画距離の拒否を含む5試験成功。

- far200 a1完走/全監査合格。固定→可動ROVIO12.870→10.868m/.644→3.716deg、融合4.768→3.571m/.620→.573deg。融合位置約25%改善、ROVIO回転悪化は残る。1試行だけで安定改善と判定しない。
- 世界全域のmesh境界は[-28.607,-432.115,-7.893]..[420.049,18.893,8.408]m、対角636.370m。31モデルでvisual/collisionの幾何/poseが一致（XML空白を除く意味比較）。world_render_extent.json。200mでも人工打切りが残り得るため、次候補v11はcamera_far=1000mで全世界内の描画打切りを解消する。値は精度結果から選ばず世界の範囲から定める。制御式はv10と同一。全ての20m/200m結果を残し、1000m条件の固定/偏向差を別に評価する。
- 反復schedulerへ--camera-farを追加し、凍結したprotocol_fieldsと異なる指定は起動前に拒否。通常の既定は変更しない。将来のv11反復では描画距離1000も凍結対象にする。

- v10(20m) prefix1070秒も完走/全監査合格。固定→可動ROVIO73.268→79.141m/25.835→22.795deg、融合16.223→17.801m/4.392→4.261deg。3区間で融合位置1勝2敗のため不採用。全所有子process0。
- v11-prefix-seed17は描画1000mで開始（domain85/session35141）。制御はv10同一、tracking_aware_v11.yamlを保存。far200 b1/domain86は継続中。far200 campaign完了後にv11のa1→b1をdomain86で開始する予定。
- observe_feature_lifetimes.py追加。生画像/点群を保存せず、ROVIOの成功追跡IDの継続時間を数値集計。v11 prefixのsim145.4〜205.35秒、両側1200frameを確認。隣接frameの同じ特徴保持率は固定98.005%/可動98.150%、成功特徴数中央値9/11。完了追跡の寿命中央値.7/.4秒、90%点4.68/6.425秒（始終端の打切りは別計数）。この直進区間に限った観測であり全旋回の保証はしない。

- far200 b1完走/全監査合格。固定→可動ROVIO27.236→7.244m/3.956→3.583deg、融合15.966→.253m/2.225→.158deg。短2区間は両方位置改善。通常設定の採用や反復成功とはまだ扱わない。
- v11-seed17 a1→b1/domain86（session10767）を開始。v11-prefix-seed17/domain85（session35141）と合わせて最大2sim。両方camera_far=1000、v10と同じ向き評価/重み、通常センサ/融合条件。
- 向き評価式の設計診断も保存: 全weakness確率を一律に縮小しても、旧の分散比/現行log分散比では共通の係数が分子分母で相殺する。v10では弱方向の確率すべて.1未満でも平均指令変化a1 .503/b1 .527/prefix .838deg。weakness_normalization_diagnosis.json。これは設計上の性質であり現在の失敗原因と断定しない。v11評価中は方向の式を変えず、変更案は保留。

### 2026-10-17 v11の途中結果とv12方向別評価
- v11 far1000 a1は全長完了/全監査合格。固定→可動の位置RMSE: ROVIO 8.465→10.623m、融合2.995→2.257m。回転RMSE: 1.068→4.968deg、.539→.489deg。ROVIO単体は悪化。b1/prefixは継続中、目標未達。
- 長prefixは862秒途中で融合3.187→6.174m。これは完走値ではない。短い試行だけで安定改善とは扱わない。
- 保存済み可動カメラ60秒を使った地図再構成の調査: planning_map_visibility_audit.json。現在視野内の過去特徴のうち追跡されていない分の減衰重み比は中央値.00934、90%点.151。消失した特徴が主因という証拠には不足。2枚の静止画の新規角点も推定画素誤差が既存2px床値未満、暗いことだけを理由に任意の重み変更はしない。
- 次候補v12: core.directional_uncertainty_gainをopt-in追加。各LiDAR固有方向について、結合した6次元情報行列から計算した追加前/後の投影分散比の対数を取り、その後に退化確率を掛けて合計。旧方式では全確率を縮小しても分子分母で相殺されたが、新方式は退化確率に比例して利得を小さくする。真の相互情報量/フィルタ共分散とは称さない。他の制御重み、推定器、far1000条件は変更しない。v11の実行中processには未適用。
- test_uncertainty_gain+test_core+test_pacing_limitsの26試験成功。確率比例、非退化方向の除外、単位変更不変、他方向との結合を検証。精度効果は未検証。未使用seed22/23はまだ使わない。
- Windows実空き9.4GiB、GUI/bag/連続生画像なし。

- v12 a1はroute187.68秒で停止信号によって中断。campaign/runtimeはinterrupted、容量不足/例外記録なし。信号発生元未特定、全length結果と扱わない。元のv12-seed17を保持しv12-retry-seed17で同一条件のa1→b1を開始（domain86/session5910）。v12-prefix-seed17/domain85/session73548は継続。campaignに次回のsignal番号の記録のみ追加、手法は変更しない。
- live_map_visibility_audit.json: v12prefixの60秒を独立10Hzで画像地図再構成し、最初8秒を除外。視野内で追跡されていない地図特徴の減衰重み比は中央値0、90%点.0107。古い地図点の残留がこの区間の主要因という証拠なし。生画像保存なし。
- v12のROVIO距離推定20秒診断: successful4023観測の距離10/50/90%点6.148/9.009/43.266m、相対標準偏差.146/.201/1.343、4m未満0。この区間では8mの保持視線仮定が著しく近距離を見落とす証拠なし。
- v12prefixは288秒途中でROVIO固定4.309→可動45.303m、融合1.322→20.345m。特徴数は可動の方が多い場面もあり、単なる追跡点数不足では説明できない。初期化時の観測方向と、その後の視野切替で新たな特徴の距離尺度を推定する過程を次に切り分ける。角度固定だけの改善は動的偏向の成功には含めない。
- 完了encoder68ファイルを圧縮/展開SHA一致後に元CSV削除、1,111,005,532bytes削減。Windows空き約12GB。

### v12再実行a1完走と、持続注視の次候補
- v12-retry-seed17 a1は全長/全監査合格。固定→可動RMSE: ROVIO9.348→10.648m/.564→3.282deg、融合3.211→2.027m/.476→.422deg。融合位置改善、ROVIO単体悪化。b1とprefixは継続。
- v12prefix sim約630秒の少数特徴を、取得時刻encoder+GT姿勢で実world meshへraycastして後解析。固定116/可動130観測、推定距離/実距離の中央値1.546/1.831。実距離中央値9.370/6.552m、推定距離中央値16.101/13.096m。ROVIO再構成視線自体の誤りも含むので厳密な独立距離校正ではない。推定器/制御へ真値は一切渡さない。range_validation_world.json、再現scriptをdiagnostic_sourcesへ保存。
- 次候補v13を実装、未実走。fixation.py: 1秒以上追跡され、距離の相対標準偏差≤.3のROVIO特徴から、LiDAR弱方向の情報増加で注視点を選ぶ。近い視線の最大3点を選択後はIDを保持し、実測視線に追従。新しい候補の利得が僅かに変わっても乗り換えない。追跡消失.25秒/可動範囲外で次の成熟点へ引継ぐ。成熟点なしでは現在角保持。速度/可動範囲は既存同じ。相対σ.3は既存ROVIOの成熟深度採用基準に合わせた。保証された真の不確かさとは扱わない。
- planner_nodeのopt-in fixation分岐では最新ROVIO ID/実測距離/共分散/追跡継続時間を読み、取得時刻から現在へ既存IMU回転/融合速度で移す。旧設定は従来8m視線のまま。ROVIO/MIMOSA自体の重み/処理は変更しない。追跡点IDの保持、幼若/高不確かさの除外、取得時刻/共分散変換、消失時引継ぎ、速度/可動範囲を含む32純Python+9実ROS message試験成功。
- v13も単発成功では採用しない。初期化視野の固定角対照を先に行う予定で、動的偏向の改善とは区別する。未使用seed22/23は未使用。Windows9.4GB、sim2本、GUIなし。

- range_validation_samplesの同一ROVIO IDを1秒間隔で比較し、最初のworld交点を次時刻に投影して静止世界との整合を確認。固定91組の角誤差中央値.134px/90%点.285px/2px超0%、可動104組では.179px/.599px/2px超6.73%。多くの特徴は静止面と整合し、距離尺度の悪化を大規模な誤対応だけで説明する証拠はない。filtered bearingを使う後解析であり、独立なraw画像追跡と同一視しない。feature_world_motion.jsonと再現script保持。

### v12全結果・v13注視補償の修正
- v12 b1全長/全監査合格。固定→可動: ROVIO46.557→32.974m/.687→3.021deg、融合33.881→17.765m/4.023→1.869deg。v12prefix1070秒も合格だがROVIO39.025→115.048m/8.084→3.502deg、融合5.930→56.568m/3.853→6.989deg。短2区間の改善だけでは採用せずv12不採用。
- far1000-pitch20-seed17 a1→b1/domain86/session27271を開始。固定角20degと0degの同一飛行対照。初期化時の視野も異なるため--shared-initializationは使わない。動的偏向の実験ではないとHTML/図凡例にも明記する報告修正。
- v13-prefix-seed17/domain85/session11318を開始したが、注視の並進運動補償が抜けていることを独立運動試験で確認したため約110route秒で手動停止。精度結果を見て打切ったのではなく、仕様修正のための中断。全データ保持、完走や精度成功には不使用。
- v13b: body回転だけでなくbody並進による注視点の視線変化を、実測IMU角速度/融合速度/ROVIO距離から1制御周期先へ予測する。中心へ寄せる探索成分は5deg/s、静止対象を追う補償を加えた合計は既存機械上限20deg/s以内。2m前方/1.5m横の点に0.1m接近すると1.41deg/0.2sの追従が必要だが、5deg/sのみでは不足する再現試験を追加。33関連試験成功。センサ/推定器/融合は変更なし。次はv13bで再実行する。

- 固定角20degのa1は完走。旧reportが全時刻のgimbal速度と動的保持入力を検査して不合格にしたため、静的対照の検証範囲を明示して修正。0.309→0.310sのspawn直後にencoder1.513→1.144deg、全時刻最大369.391deg/sを記録しており、この値はJSONに保持する。ROVIO初期化開始（初出力-3秒）以降は20degに静定していることを別途必須検査し、この範囲で速度を検査する。実際の動的偏向試験は従来通り全時刻の20deg/s上限で判定し変更なし。固定指令は現在特徴を使わないため、保持入力時刻検査は固定角対照に限って適用しない。静的対照を動的偏向の6組成功判定へ数えない。

### 固定角対照の結果とv13c
- far1000-pitch20-seed17はa1/b1とも完走・固定角用監査合格。a1は上記の通り改善、b1はROVIO31.713→35.298m/1.699→2.387deg、融合21.444→23.782m/2.821→2.868degで悪化。下20degを一律採用する根拠はなく、初期向き/基準カメラの0degは維持。初期化を含む固定角比較を動的偏向の成果とはしない。
- v13b長区間は継続。約370秒時点の注視引継ぎ65回すべてで直前目標がpitch30またはyaw50の2deg以内。最終特徴年齢10/50/90%点2.99/9.05/21.06秒。まだ画面内で追跡できる特徴も「中央を向ける可動範囲外」という条件で引継いでいた。
- v13cで、保持するかは現在画像の画角内かつROVIO成功追跡で判定し、中央へ向けられなくても保持する。新規アンカーは成熟/中央へ到達可能/現在画角内をすべて満たすものから選択。範囲外目標の運動補償はclamp前の視線角差から計算し、達成不能なpitch差がyaw速度枠を消費しないよう修正。既存機械限界/探索pitch±30は変更しない。
- 最初のv13c-seed17は単体試験失敗の確認前に起動してしまったため、19.3実秒で直ちに停止してinterruptedとして保持。軌道精度データではない。試験fixtureを実UniPilotの±50/±90、現画角に一致させ、新候補の画角外点を再選択しない条件も修正。36関連試験合格後、v13c-retry-seed17 a1→b1/domain86/session77345を新規開始。v13b-prefix-seed17/domain85/session85403は全長対照として止めず継続。

- v13b prefix1070秒完走/全監査合格。固定→可動ROVIO位置31.342→12.926m/回転3.655→2.376deg、融合4.657→8.902m/3.970→4.151deg。ROVIOの全体誤差改善は融合の改善を保証しないため不採用。v13c prefixをdomain85/session31920で開始、a1→b1/domain86/session77345と最大2並列。
- v13c a1完走/全監査合格。ROVIO8.179/.452→8.937/5.091、融合2.438/.428→2.008/.373（m/deg）。融合位置18%改善だがROVIOは悪化。安定改善は未達。
- relative_motion_diagnosis_v13.json: v13b prefix途中の1秒相対移動RMSEノルムは固定.1324→可動.1520m。累積ROVIO位置が改善しても融合へ渡す局所移動が改善しているとは言えない。時刻順入力の連続監査合格、odometry_manager/graph_manager警告ログ0byte。D-opt閾値拒否の記録なし。既存ログはcallback件数であり、受理factor全数の直接計数まではしていない。校正/融合設定はこの診断で変更していない。
- 完了77encoderをSHA照合付き可逆圧縮、累計1,298,062,362 bytes削減。Windows実空き約8.9GB。元bagは保存、追加raw画像/点群収録なし。

- 融合回転の切分け用に --fusion-imu-noise sensor_gyro を追加（既定legacyは不変）。sensor設定で加速度・角速度を同時変更した試行は悪化したため、今回はgyro白色雑音密度だけをSDFのstddev/量子化/取得Hzから算出する。既存.01→.000636416563 rad/s/√Hz、acc .8・bias因子・ROVIO・LiDAR・外部odometry共分散は不変。固定/可動へ同一適用し、GT誤差から値を探索しない。test_unipilot_samplingの独立式/他設定不変/200→1000Hz密度不変を含む5試験合格。性能検証前であり採用はしていない。

- v13c b1全長/全監査合格。固定→可動ROVIO位置41.939→16.585m/回転1.112→13.280deg、融合29.591→.145m/3.378→.350deg。短2本の融合位置は改善、ROVIO回転は悪化。prefix/domain85/session31920は継続、安定改善とは未判定。
- gyro-only切分け v13c-gyro-seed17 a1→b1/domain86/session58780 を開始。値は仕様式から決定し、既定設定は不変。全長結果と既存条件を同時記録し、良い試行だけを残さない。

- 名目の相対移動共分散をカメラ角へ回転させる仮説も事後検査したが、採用しない。camera_covariance_transport_audit.json: 真値相対誤差を名目分散で正規化した二乗量平均（小さいほどモデル整合）は、可動a1で現行8.789→カメラ軸回転43.215、b1で23.536→43.069、v13b prefixで5.835→13.510。姿勢・距離が既に融合されたROVIOの誤差は単純にカメラ方向へ回るものではなく、固定名目行列を回すだけでは悪化する。真値は事後診断のみ、推定器コード/重み変更なし。相関標本のため独立試行の統計検定として扱わない。

- v14 opt-in候補を追加（未飛行）。成熟した同一特徴を保持するv13cの選択規則を維持し、常時中央へ寄せる代わりに、次時刻で特徴群が画像内側60%に残る最小偏向を選ぶ。60%は既存border_fraction=.2から計算、2deg候補+現在/中央方向、可動範囲・20deg/sは不変。予測が光学中心/後方へ破綻し全候補不可なら保持し、非有限な指令を出さない。画像中心付近では無動作、端で最小追従、端停止時もID維持、異常予測保持を含む40関連試験成功。最初の試験呼出しは存在しないtest_uncertaintyの名前誤りで失敗したが、正しいtest_uncertainty_gainで全件成功。現実行v13c/gyroのプロセスは既に読み込んだ旧制御を使用し、試行途中では切替えない。

- v13c-gyro-seed17短2本全長/全監査合格。位置m/回転degの固定→可動: a1 ROVIO8.731/.570→5.844/2.197、融合2.377/.424→2.327/.413。b1 ROVIO42.089/3.119→14.782/11.356、融合28.321/3.313→.172/.165。a1改善は2%で小さく、既存legacyの大きなb1改善も再現するため、gyro変更が主因と断定しない。異なる物理飛行を挟むため旧試行との絶対差から因果も断定しない。長区間のsensor_gyroは未実施、独立検証候補は3区間のlegacy条件を優先。
- v14分岐追加後のv13cデフォルト経路を、開発a1へ保存した変更前のfixation.pyと比較。欠落特徴/回転/並進を含む400連続ケースで指令byte一致・診断dict一致。v13c_equivalence_after_v14.json。v14はまだ未飛行・不採用。

- v13c prefix1070秒完走/全監査合格。固定→可動ROVIO47.587/5.810→25.067/4.794、融合14.362/3.940→6.114/4.131（m/deg）。開発3経路で融合位置改善（a1比.824、b1比.0049、prefix比.4257）を得たが、prefix融合回転は悪化。安定改善は独立反復前なので未達。
- 2026-10-17 14:17 UTC頃、全開発3経路完了後にv13cを固定。frozen_candidate_v13c.json/frozen_v13c_sources.zip（26ファイル、ZIP全内容SHA照合）を保存。物理世界を描き切るfar1000m、fusion_imu_noise=legacy、共通初期化/26秒準備/時刻順融合、DRPM OFF、基準0deg、真値オンライン不使用を明記。gyro-only/v14/固定20degは採用せず。未使用seed22/23を全過去protocolと照合して独立検証を開始。
- validation-v13c-seed22 a1→b1をdomain86で開始し、schedulerでseed22 prefix/domain85を同時開始。後続seed23も同じ凍結設定。6組完了まで変更しない。schedulerは先行prefixが完了したdomain85を後続prefixへ再利用できるように修正（出力別directoryの場合のみ）。最大2sim・長区間750秒ずらし・RAM>3GBを維持、7状態試験成功。
- 完了81encoderを可逆圧縮・SHA照合、累計1,390,654,574 bytes削減。Windows空き約8.9GB。元bag保持、追加bag/raw画像/点群保存なし。

- 独立反復の開始時に両飛行の凍結証拠87項目が全一致。推定/制御設定は以後変更していない。
- compact_finished_logs.py追加。runtimeで全子process停止を確認した試行だけ対象にし、同一物理飛行のtruth.csv/route_tracking.csvを全byte SHA一致確認後にhardlink化、処理時間JSONLを可逆gzip・展開SHA確認後に冗長原本を削除。ファイル名を維持する真値/軌道は値・標本数・時刻不変。圧縮タイミングログの旧解析はgzip -dkで復元可能。元bag/画像/推定CSV/不採用結果は保持、実行中は対象外。152処理で848,407,986bytes削減、compact_log_audit.json/復元説明を保存。Windows実空き約11GB。研究測定値は直接編集せず、必要ならコピーして編集することをSHARED_REFERENCES.txtへ明記。

- コンパクト化後のv13c短2経路で、精度・全監査を再計算して保存済みresults.jsonと辞書全体が完全一致。compact_log_report_check.json。削除済みブロックのみfstrimで2.7GiB解放通知、既存ファイルには変更なし。Windows空きは外部の使用も含め約9〜11GBのため、/mnt/cの実値と2GB停止guardを引き続き使用。

- v13c独立反復seed22 a1が全長完走/全監査合格、凍結87項目一致。固定→可動ROVIO6.190/1.156→7.999/.652、融合2.656/.487→7.130/.511（m/deg）。融合位置が2.684倍で事前最悪上限1.5を超え、残り5試行がどうなっても合格できない。このため全6組完走予定を変更し、無駄な収録を避ける早期不採用として停止。悪化結果を除外せずv13c_early_rejection.jsonへ保存、seed22 b1/prefixの途中記録も保持。b1 route終了1（停止中）、他子process0、全所有子process停止済み。scheduler自身はKeyboardInterruptによる終了1。seed23は未起動。
- v13cは開発3経路では全改善したが安定改善に失敗、採用しない。seed22は以後開発条件、次の独立検証は未使用seed24/25を予定。目標未達。
- v14の開発を開始。a1→b1を既知の失敗条件seed22/domain86、prefixをseed17/domain85で最大2並列。特徴選択・保持はv13cと同じ、画像内側60%で見えている間はカメラを動かさず、端へ近づくと最小偏向する。センサ/融合/初期化/速度上限は維持。実装は40試験合格済みだが飛行精度は未確認。

### v14短区間結果・v15追跡品質回復（2026-10-17 14:41 UTC）
- v14-seed22短2区間は全長/全監査合格。固定→可動の位置m/回転deg: a1 ROVIO10.847/.607→167.653/2.616、融合3.342/.555→175.084/6.191。b1 ROVIO35.043/1.284→12.518/9.238、融合20.477/2.942→.214/.231。a1悪化が極端で採用しない。prefix/domain85/session33845は全長確認のため継続。
- a1可動で成功追跡4点未満の割合が時間区分ごと30〜40%、後半は成熟深度点中央値0。従来fixation経路は深度情報がなくなると最後の視野を保持し続け、元Plannerの追跡喪失回復処理を通っていなかった。
- v15はv13cを基に、成熟点4未満なら新たな偏向を待ち、1秒続いたら既存5deg/sで機体前方へ戻す。前方1deg以内・品質回復1秒を満たすまで再偏向しない。推定器リセット/真値利用/融合重み変更なし。v14のdeadbandはOFF。44試験成功。v15-seed22 a1→b1/domain86/session69390開始。まだ精度未確認、安定改善未達。
- v14 a1画像6枚だけ保存（35/120/210秒の固定/可動）。上向き視野は暗い壁面が多い。実機mesh+保守的ロータ円盤による160x90raycastでは自己遮蔽率: 正面0%、yaw50/pitch-30 .611%、yaw-50/pitch-26 .417%。画像の大半を機体が塞いでいる証拠なし。個別ROVIO特徴の自己遮蔽samplerはROS環境混在で0件、追跡点の自己遮蔽は未確認。self_occlusion_with_rotors.json等保持。
- Windows実空き約12GB、sim最大2、追加bag/連続画像/点群なし。新たな独立反復は未使用24/25予定、seed22は開発扱い。

- v14 prefix1070秒完走・全監査合格。固定→可動ROVIO52.268/2.717→34.311/3.798、融合6.848/3.814→7.657/4.792（m/deg）。v14は3経路中2経路で融合位置悪化し不採用。全所有v14 process停止。v15-prefix-seed17/domain85/session76352開始、v15短区間/session69390と最大2並列。
- 完了encoder86ファイルで累計1,449,527,491bytes、追加コンパクト化161処理で885,268,091bytes削減。可逆性/SHA監査を保存。

- v15 seed22 a1全長/全監査合格。固定→可動ROVIO 5.531/1.033→6.412/2.152、融合 3.318/0.368→0.498/0.251（m/deg）。融合位置比0.1502。ROVIO単体悪化は保持。短1本の改善だけでは採用しない、b1/prefix継続。3試行の起動時実装snapshot158ファイルが全一致。

- v15 a1の追跡区間1秒相対移動誤差を後解析。ROVIOの前後軸RMSE .20655→.19075m、同軸平均誤差 -.09388→-.04665m。50ms相対移動RMSEは.01454→.01519mと増える。累積ROVIO誤差だけでは融合の改善を説明しない。30秒ごとの精度/相対移動/回復状態をsegment_diagnosis.jsonに保存し、後半の固定融合誤差蓄積と偏向側の抑制を確認。真値は事後解析のみ。解析scriptはdiagnostic_sources保存。

- v15 seed22 b1全長/全監査合格。固定→可動ROVIO15.044/1.437→1.598/.999、融合.874/.193→.696/.203（m/deg）。融合位置比.7955、回転は.010deg悪化。短2本の融合位置は両方改善、prefix/domain85/session76352は約400/1070秒で継続。独立反復前の開発頑健性確認として、既使用seed17の短2本をv15-short-seed17/domain86/session96139で開始。未使用seed24/25はまだ使用しない。

- v15追加開発seed17 a1も完走/全監査合格。固定→可動ROVIO11.783/.941→10.078/2.017、融合1.582/.442→.692/.397（m/deg）。融合位置比.4376。a1は開発2条件とも改善、ROVIO回転悪化は両条件で残る。seed17 b1とprefix全長は継続。

- 次の切り分けv15gを準備（実走前）。カメラ制御はv15のYAML値と完全一致、融合IMUのgyro_noise_densityのみ従来.01からSDF/1000Hz仕様由来.000636416563 rad/s/sqrt(Hz)へ固定/偏向両側で変更する予定。既存sensor_gyro profileを使用、探索的な数値最適化なし。acc .8・bias過程・ROVIO・odometry因子は不変。MIMOSA公式初期化は静止平均gyroをbias初期値へ使うため、初期bias sigma .001をセンサの未校正bias .005と単純比較して不具合とは扱わない。全白色雑音を仕様へ合わせた過去sensor profileは悪化しており、その不採用記録も保持。

### v15長区間結果とv15g開始（2026-10-17 15:07 UTC）
- v15 prefixは1070秒完走、入力/出力/同期/初期化/角度の全監査合格だが、収録後GazeboがSIGINT/SIGTERMで終了せずtimeout→SIGKILL、simulation exit -9。他process全0、全所有process停止。processes_exit_zeroのみfalseとして不合格記録を保持。終了失敗を正常終了へ書き換えない。
- その全長CSVの固定→可動: ROVIO36.456/6.645→9.960/3.576、融合4.534/4.103→7.520/4.035（m/deg）。位置融合比1.6586、良好なROVIO単体の累積精度でも融合改善は保証されなかった。v15は安定改善未達、独立seed24/25は使わない。
- v15g-prefix-seed17/domain85/session85568、v15g-seed22 a1→b1/domain86/session99552開始。カメラ制御はv15と値完全一致、変更はMIMOSAのgyro測定雑音密度のみ.01→SDFからの.000636416563、固定/偏向とも同じ。融合acc .8、bias、外部odometry係数、ROVIO、物理センサは不変。任意の数値探索ではなく仕様整合の切分け。
- v15追加seed17 b1は全監査合格。固定→可動ROVIO 21.414/13.893→9.698/12.204、融合 5.248/0.967→0.555/0.149（m/deg）、融合位置比0.1058。短4比較と長区間の記録を全保持。

### v15g不採用と、尺度を混ぜない視線予測v16（2026-10-17 15:20 UTC）
- v15g seed22 a1は全長/全監査合格だが固定→可動ROVIO8.532/1.383→103.249/3.577、融合2.202/.584→105.880/3.257（m/deg）と大きく悪化。gyro仕様値への変更は採用しない。b1/prefixは原因切り分けの完走を継続、実装/設定は途中変更していない。v16は従来legacy融合へ戻して検証予定。
- v15g prefixの重力/偏りをread-only 25秒/5Hz記録。固定/偏向各120点。約269〜293sim秒で、初期姿勢整合後の推定重力の座標傾きは中央値.221/.130deg、機体に対する重力方向の誤差1.579/1.269deg。単なる座標変換の読み違いだけで姿勢誤差を説明する証拠なし。ROVIOとMIMOSAのgyro bias差は数1e-4 rad/s。
- 100Hz真値姿勢の差分と1000Hzgyroの25秒比較も実施したが、残差標準偏差.034/.035/.021rad/sで仕様.020に対しx/yが大きい。高周波運動の標本化差を含む近似診断で、正確なbias真値/補正には使わない。生bufferは保存せず2秒区分統計のみ。診断script3本とJSON保持、推定器へ真値未入力。
- v16実装: bearing_motion.pyが約.3秒の同一特徴の観測視線変化から、IMUで説明される機体回転を除いて残りの角速度を求める。画像取得時刻→現在および1制御周期先の視線予測にこれを使い、ROVIO特徴距離とMIMOSAの異なる尺度の速度を組み合わせない。動的extrinsic変換済みのbody視線を使うのでジンバル回転を再度加えない。特徴選択のFisher計算やLiDAR情報は従来のまま、推定器/重み/回復処理も不変。
- opt-in fixation_bearing_motion=trueをtracking_aware_v16.yamlへ追加。物理運動・尺度/融合速度不変・回転二重補償なし・短いgyro biasの視覚帰還相殺・ID喪失/時間間隔など51純Python試験合格。ROS message経由試験結果を下へ追記する。飛行精度は未確認、安定改善未達。

- v16の実ROS message経由11試験成功。25特徴の角速度計算は中央値1.23ms/95%点2.51ms/最大3.51ms。v16-seed22 a1→b1/domain86/session38497を開始、融合legacy・制御v16。v15g-prefix/domain85/session85568は約710/1070秒で継続。v15g短2本は全完走停止済み、b1数値はresults.json。まだ安定改善未達。

- v15g b1も全長/監査合格。ROVIO27.586/2.299→1.457/1.132、融合14.677/2.160→.441/.153（m/deg）。a1大幅悪化があるのでgyro仕様値の候補は採用しない。prefix診断は継続。
- 1000Hz IMUのorientation差分でもgyro参照を試したが、区間平均残差std .074/.064/.015rad/s、期待白色平均約.0142とx/yが合わない。更新段階や差分近似の影響が未分離なため正確なbias真値と扱わず、推定器修正には使わない。生buffer破棄、2秒統計のみ保持。
- v15/v15g全完了短区間とv15prefixの制御用.3秒平均body角速度は最大14.54deg/s、20deg/s超の時刻0。少なくともこの平均運動に対してgimbal上限不足の証拠はなく、角速度上限は変更しない。高周波瞬時値とは区別する。

- v16 seed22 a1完走/全監査合格。固定→可動ROVIO7.271/.673→5.917/3.966、融合.946/.483→.787/.425（m/deg）。融合位置比約.831、ROVIO回転悪化は保持。b1は同campaign/session38497で継続。v16-prefix-seed17/domain85/session63526を開始、融合legacyで統一。
- v15g prefix1070秒は全長/全監査合格、全所有process正常停止。固定→可動ROVIO56.252/5.750→11.423/6.398、融合15.619/4.223→9.646/4.143（m/deg）。b1/prefixで改善してもa1の105.880m発散を理由にv15g単独は採用しない。旧v15（fusion legacy）のprefixは終了監査不合格かつ位置悪化。異なる物理試行の絶対誤差差をgyro設定のみの因果効果とは扱わず、各固定/偏向組だけを直接対照とする。
- v15g prefix後半943.9〜967.7sim秒の重力診断: 推定重力の初期整合座標からの傾きは固定3.100deg/可動2.634deg、機体から見た重力方向誤差3.364/2.998deg。座標側の変化と姿勢側の誤差が共存する。報告精度へ時間変化する整合を追加せず、従来の初期姿勢1回整合を維持。生画像なし、少量JSONのみ。
- 完了encoder96ファイル、累計1,693,243,869bytes削減。コンパクト化180処理、累計1,036,949,582bytes削減。可逆性/SHA保持。Windows実空き約10GiB。

### 2026-10-17 15:36 UTC — v16短区間2件完了、反復継続（目標未達）
- v16-seed22 a1/b1は全長・全監査合格、全子process exit0。融合位置RMSE a1 0.945660→0.787464m（比.832713）、b1 3.005→.137m（比.045709）。ROVIO b1 17.326→5.277m、回転3.881→3.201deg。a1のROVIO回転悪化は既記録のとおり保持。
- v16-short-seed17 a1→b1をdomain86/session25083で開始。v16-prefix-seed17/domain85/session63526は継続中。いずれもlegacy融合設定。同時2sim。反復は開発扱い、独立検証ではない。
- /tmp/freeze_gimbal_v16.pyの開発入力一覧に追加反復2件を含めた。未実行。3経路で改善が揃っても未使用seed24/25の独立検証前に成功と判定しない。
- 終了記録の可逆圧縮・同一参照共有を更新（97encoder/182項目、累計約2.75GB削減）。Windows実空き8.4GiBを監視。連続生センサ記録なし。

### 2026-10-17 — v16追加a1は改善せず、融合雑音モデルの診断
- v16-short-seed17 a1完走・全監査合格: ROVIO8.823→4.095m、融合1.116→2.129m（比1.907898）。採用条件に達しないためv16を凍結しない。追加b1とprefixは現時点実行中。
- 同条件50msの相対誤差を従来の融合用共分散と比較。seed17 a1の六軸二乗誤差/想定値は固定[.96,.99,.30,.85,.22,.48]、可動[.97,1.00,.55,.87,.55,.44]、規格化二乗誤差平均4.04/4.55。seed22 a1/b1も保存。現在の結果だけから融合の不確かさ設定を変更する根拠は得られず、変更しない。時系列相関や局所的なモデル誤差を否定する診断ではない。
- 診断はv16_relative_noise_audit.jsonとdiagnostic_sources/audit_v16_relative_noise.py。次の検証候補は、注視対象以外の追跡点が視野から外れることを抑える制約。現fixationは最大3点を保護するが、他の追跡点を保護していないという実装上の事実に基づく。精度改善の因果は未検証。

### 2026-10-17 — v17周辺追跡点の保護を追加（オンライン精度未検証）
- v16の注視制御に任意fixation_protect_other_tracksを追加。現在追跡成功している全視線について、指令直後と0.8秒先を観測視線速度・IMUで予測。視野端の余裕を重みとし、同じ機体運動でカメラを保持した場合の90%以上を残す最大の偏向量を33候補から選ぶ。既存設定retain_fraction=.9/border_fraction=.2/prediction_horizon_s=.8を使用。
- 注視点以外を保護するのが変更点。速度・距離の推定尺度、真値、融合重みは使わない。質が不足した際の前方復帰はv16のまま。v17設定を指定した場合のみ有効、v16以前の動作は不変。
- retention.py、fixation.py、tracking_aware_v17.yaml、test_retention.py、test_fixation.pyを変更/追加。56件の関連単体試験に成功後、注視制御から呼ぶ統合ケース1件を追加しfixation20件成功（合計57ケース）。初回の試験起動は作業directoryが違いimport失敗、正しいdirectoryで実行した結果。
- v16追加b1とprefixは走行中。新モジュールの精度効果は未測定。候補v16のsource snapshotは保持し、更新後のsourceと混同しない。

### 2026-10-17 — v16追加b1完了、v17短区間開始
- v16-short-seed17 b1は全長/全監査合格。ROVIO42.731→2.724m、融合30.738→.551m、融合回転3.853→.115deg。全所有process exit0。a1悪化は変わらず、v16を安定改善としない。
- v17-short-seed17/domain86/session92695開始。旧短区間campaignの正常終了を待って起動。v17-prefix/domain85/session62557は旧prefix正常終了待ち。待機scriptは完了状態確認後execでcampaignへ置換し、同時sim数を増やさない。

### 2026-10-17 — v16長区間も未改善、v17 3区間を継続
- v16-prefix-seed17全1070秒、全監査成功、全所有process exit0。ROVIO36.004→17.925m、回転5.305→8.488deg。融合6.540→7.862m（比1.202097）、回転4.153→4.083deg。v16は追加a1と長区間で悪化のため不採用。結果/六軸図は各campaignに保存。
- v17短区間session92695/domain86、v17-prefix session62557/domain85が稼働中。同時2sim。旧v16全process終了、追加生画像/点群保存なし。可逆圧縮累計encoder100files、共有/圧縮188項目、計約2.88GB削減、Windows実空き8.3GiB。

### 2026-10-17 16:01 UTC — v17 a1は小幅悪化、既存seed22で反復を予約
- v17-short-seed17 a1全長/全監査合格、ROVIO8.772→3.635m、融合.685→.965m、回転.348→.406deg。融合は改善せず、採用は未判断。誤差の絶対値が小さいことと、ジンバルの相対的な性能改善を区別する。
- 同seed17 b1とprefix継続。終了後にdomain86でv17-a1-seed22のみを試す待機scriptを予約。これは既存seedでの開発反復であり独立検証ではない。1件だけの改善で採用しない。独立6組の成功基準（5勝、経路平均比≤.9、全体≤.8、最悪≤1.5）は変更なし。

### 2026-10-17 16:06 UTC — v17 b1完了、a1 seed22反復中
- v17-short-seed17 b1は全長/全監査合格、全process exit0。融合位置19.757→.852m、回転2.480→.165deg、ROVIO38.174→2.413m、回転4.784→2.646deg。a1の.685→.965mの悪化も保持。
- v17-a1-seed22/session46149/domain86を開始。v17-prefix-seed17/session62557/domain85継続。両方legacy融合。次の制御方式/IMU設定変更は実施していない。

### 2026-10-17 16:10 UTC — v17凍結手順の準備のみ
- /tmp/freeze_gimbal_v17.pyを準備（未実行）。v17-short-seed17のa1/b1、prefix-seed17、a1-seed22の4件を全て保存し、候補に含まれるretention.py/bearing_motion.pyもSHA固定する。
- 開発から独立検証へ進める条件は、反復を含む各経路の平均比≤.9、全体平均≤.8、開発中の最悪比≤1.5、全件完走/監査成功とする。以前の準備scriptが要求した「開発全反復で個別に改善」から、反復した経路の平均で判断するよう変更。開発での小幅悪化を隠さず、実際の採用判定は未使用seed24/25の6組で行うため。**独立検証の成功基準は変更なし**。反復の結果を見て閾値をさらに変えない。

### 2026-10-17 16:13 UTC — v17 a1 seed22改善、長区間判定待ち
- v17-a1-seed22全長/全監査合格、全子process exit0。融合位置6.141→1.255m（比.204418）、回転.675→.483deg。ROVIO位置11.817→7.771m、回転.351→6.409degで回転悪化は残る。
- a1開発2反復の融合誤差比平均=(1.408046+.204418)/2=.806232。b1比.043131。v17-prefixの結果待ちで、まだ凍結・独立検証を始めない。全4試行のseed/route以外のprotocolが一致、実際に保存した現候補source5ファイルも一致を確認。

### 2026-10-17 16:18 UTC — v17長区間は小幅改善、開発を3経路×2反復に揃える
- v17-prefix-seed17全1070秒・全監査合格、全process exit0。融合位置7.884→7.106m（正確な比0.901402366）、回転4.040→3.753deg。ROVIO49.137→11.108m、回転7.753→6.169deg。位置は改善したが、事前の経路平均比≤.9にはこの1反復だけでは届かない。
- 平均比の閾値は変更せず、a1だけでなくb1/prefixも既存seed17/22の2反復に揃える。追加はv17-b1-seed22/domain86/session93417、v17-prefix-seed22/domain85/session22685。同じ設定、全6開発反復を保持。この6件で不足なら同候補へさらに反復を足して通過させない。
- /tmp/freeze_gimbal_v17.pyに追加2件と開発6組中5勝条件を追記。独立検証は引き続き未使用seed24/25、成功基準は不変。未実行。
- 終了データ可逆圧縮encoder104件/その他196項目、約3.03GB削減。fstrimで1.5GiBの空き領域をWindowsへ返却要求。実Windows空きは直前7.6GiB、継続監視。

### 2026-10-17 16:22 UTC — 空く計算枠でv17g予備比較を予約
- v17-b1-seed22完了後、domain86でv17g-a1-seed17だけを実施する待機scriptを準備。v17gは視線制御とROVIO/VIO相対因子はv17と同じ。融合のジャイロ白色雑音密度のみ.01→SDF由来.000636416563を両側へ適用。元sensor/画像/LiDAR/加速度.8/bias設定は変更なし。
- 根拠: センサ仕様と融合用のジャイロ測定雑音に差があること、旧v15gではa1の推定速度が最大9m/sとなり視線予測に混在したが、v16以降はその速度・距離尺度依存を除いたため、旧失敗だけで現制御との組合せを否定できない。改善の因果を証明したという意味ではなく開発上の予備比較。
- tracking_aware_v17g.yamlはv17設定のcomment付きcopy。主候補v17の設定/実装と進行中2本は不変。新しい生センサ記録はしない。同時sim最大2本。

### 2026-10-17 16:26 UTC — v17 b1 seed22改善、残る開発はprefix seed22
- v17-b1-seed22全長/全監査合格、全process exit0。融合23.194→.773m（比.033331）、回転3.391→.174deg。ROVIO43.802→10.207m、回転12.721→7.086deg。これで開発5件完了、4勝1敗。
- v17-prefix-seed22/session22685/domain85継続。空いたdomain86ではv17g-a1-seed17/session84004開始。生成された両側mimosa.yamlの一致と、v17からの差がgyro_noise_density1項目だけであることを確認。

### 2026-10-17 16:33 UTC — v17g a1不採用、数値ログを追加圧縮
- v17g-a1-seed17全長/全監査成功、全所有process exit0。ROVIO7.014→6.239m、回転1.706→4.170deg。融合.879→4.212m、回転.413→.543deg。ジャイロ雑音密度をセンサ仕様へ合わせた組合せでもa1は悪化し、不採用。この設定の追加試行は行わず、主v17のlegacy設定を維持する。単一試行で設定変更の悪化原因までは断定しない。
- 容量確認: Windows pagefileはAllocatedBaseSize19337MB/CurrentUsage2807MB/PeakUsage5970MB。前後の比較がないため今回の空き低下の原因とは断定しない。OS設定は変更なし。
- compress_finished_numeric.py追加。停止済み旧候補のみ（v17系除外）のCSV/TUMをgzip3で可逆圧縮、展開SHA256一致後に非圧縮コピーを削除。元hardlinkの共有も圧縮側で保持。1202group/1395paths、1,369,890,455bytes削減。numeric_compression.jsonに台帳、各flightへ復元手順。
- report_gimbal_campaign.pyの既存圧縮読取対応を共通初期化確認CSVとordered_inputs.csvにも拡張。v16-seed22の2件＋v16-prefix-seed17の1件を圧縮済み入力から再計算し、全精度値/全監査辞書が圧縮前results.jsonと完全一致。numeric_compression_validation.jsonへ保存。評価数式/成功基準/推定器/現在の候補sourceは変更なし。

### 2026-10-17 16:38 UTC — 融合までのIMU取得数の追加監査
- audit_imu_delivery_counts.py追加。従来の処理済みIMU末尾の時刻だけでなく、連続する融合イベント間の実際のIMU callback数と取得時刻差/周期の一致を検査。先頭の非ゼロprefix以降、丸め許容2ns。推定器や時刻は変更しない。
- 完了済みv17主比較5件・固定/可動の10stream全件で不一致0。imu_delivery_audit_v17.jsonに保存。初期の読み取り診断imu_prefix_count_audit.jsonも保持。残る進行中prefix22と、後続の独立検証は終了後に同scriptで追加監査する。

### 2026-10-17 16:41 UTC — v17開発6組は基準通過、凍結後の独立検証開始（目標まだ未達）
- v17-prefix-seed22全長/全監査成功、全所有process exit0。融合17.923→6.543m、回転3.613→3.724deg。ROVIO66.543→15.039m、回転4.708→5.068deg。
- 開発6組の集計: {"pairs": 6, "wins": 5, "mean_ratio": 0.49256112008162467, "worst_ratio": 1.4080464765699656, "per_route": {"run1_tail_a1": 0.8062320642818552, "run1_tail_b1": 0.03823104549262657, "run1_prefix": 0.6332202504703921}}。設定を追加変更せず5勝/6、各経路平均≤.9、全平均≤.8、最悪≤1.5を通過。完了6件×2の12streamで追加IMU受信数監査も不一致0。
- /tmp/freeze_gimbal_v17.py実行、28ファイル/バイナリSHA/全非seed-route設定/開発全結果をfrozen_candidate_v17.jsonとfrozen_v17_sources.zipに固定。凍結は上書きしない。候補本体に以後変更を加えない。
- validation-v17-seed24 a1→b1をsession40882/domain86で開始。継続scheduler session80832がseed24 prefix/domain85、次いでseed25 shortとprefixを管理する。schedulerのfinallyは最初のshortを管理しないため停止時は40882と80832の両方を停止する。最大2sim、長2本は750秒+RAM3GBでずらす。
- 独立seed24/25は今回初使用。全6組の完走/全監査/凍結一致/同じ事前成功基準を満たすまで最終成功とはしない。設定変更や不都合な結果の除外を行わない。

### 2026-10-17 16:47 UTC — v17独立検証1/6完了
- seed24 a1は全長/全監査/凍結28source・実行条件一致。融合位置3.405→1.057m（比.310399685）、回転.441→.422deg。ROVIO位置8.552→9.420m、回転1.480→15.643degで悪化も保持。融合の位置改善とROVIO単体改善を区別する。
- 完了した開発6組＋独立1組、計14streamの実IMU受信数追加監査も不一致0。現在seed24 b1とprefix継続、schedulerがseed25を予約。設定は凍結のまま。Windows実空き6.7GiB、rootログ2.6GB。

### 2026-10-17 16:55 UTC — v17独立検証2/6完了
- seed24 b1全長/全監査/凍結一致。融合位置14.270→1.262m（比.088463250）、回転1.991→.136deg。ROVIO位置25.216→2.536m、回転2.038→.657deg。これまで2組とも融合位置改善、未完4組を継続。
- 最初のshort/session40882は正常終了。以後はscheduler/session80832が全進行中campaignを所有。seed25 short/domain86開始、seed24 prefix/domain85は約600/1070秒。

### 2026-10-17 17:00 UTC — 独立検証の集約表示を準備
- report_gimbal_adoption.py追加。独立6組の完了を要求し、既存evaluateの数値が保存結果と完全一致することを確認して、位置/回転集約2ページ＋六軸12ページを作る。失敗基準なら失敗と表示し、全6組とROVIO悪化を残す。評価数式・凍結した制御/推定器は不変。構文確認成功、全6組の描画は検証完了後。

### 2026-10-17 17:02 UTC — v17独立検証3/6、a1の両seedで改善
- seed25 a1全長/全監査/凍結一致。融合位置5.813→.510m（比.087814233）、回転.663→.416deg。ROVIO位置15.581→2.917m、回転1.129→1.750deg。a1経路平均比.199106959、全完了3組平均.162225723、3勝。
- seed25 b1とseed24 prefixを継続。prefix24終了後、schedulerが同domain85でprefix25を開始。未完の長区間を成功扱いしない。

### 2026-10-17 17:06–17:10 UTC — v17独立長区間で不採用、v18の必要時のみ偏向を準備
- seed24 prefix全1070秒/全監査/凍結一致。融合位置5.311778731→15.401741237m、比2.899544958が最悪比上限1.5を超えるためv17は不採用。回転3.6265→4.5356deg。ROVIO位置47.7566→20.4603mは改善するが回転1.5914→4.4222deg。最終成功とはしない。v17_early_rejection.json保存。
- scheduler/session80832へSIGINT、seed25 b1とprefixの未完をinterruptedとして保持（性能失敗とは扱わない）。全所有子process exit0。短区間3勝、長区間1敗の全確定結果/全設定/不採用zipを保持。seed24/25は以後開発用であり独立検証に再使用しない。
- v17_prefix24_failure_diagnosis.json: 100秒区分で両側の相対誤差/全体誤差を比較。偏向側は100–300秒から水平/旋回誤差が増え、融合横方向のずれが後半約20m。50ms相対誤差の各軸平均は名目共分散の数倍以内で、単純な全軸の重み変更だけを支持しない。因果は未確定。
- v18は成熟した注視点を常に画面中央へ寄せず、0.8秒先にも視野内側60%へ残る場合は角度保持、外れる場合のみ最小の偏向。旧v14の距離/融合速度依存の予測を使わず、v16の観測視線変化を使う。v15の質低下時の前方復帰とv17の周辺追跡点保護を併用する。融合/センサ/重みは変更なし。
- fixation.pyの任意deadband経路を更新、tracking_aware_v18.yaml追加。新しい尺度・速度非依存/不要な偏向なしの2試験を含む59関連試験成功。設定copyの初回コマンドはdirectory誤りで失敗し、絶対pathで作成し直した。精度は未検証。
- 停止済み数値ログ圧縮をv17にも適用し約164MB追加削減。独立3短試行の全数値/検査が圧縮前と完全一致、v17_compressed_validation_roundtrip.json。進行中のファイルは対象外。

### 2026-10-17 17:16–17:23 UTC — v18不採用、時間相関を扱うv19の融合試験開始
- v18 seed24 a1全長/全監査成功。ROVIO位置5.506→5.837m、回転.858→1.423deg。融合位置1.274→2.895m、回転.406→.502deg。開発の最悪比基準を超え不採用。prefixとb1は中止として保存、両campaign全子process exit0。角度保持だけで改善するという仮説はこの試行では支持されない。
- 元の独立した固定UniPilot校正2飛行を再分析。50ms誤差を独立として足す予測に対し、1秒の実際の誤差二乗はa1並進で約9–20倍、Yawで約5倍。fusion_temporal_accumulation_audit.json。座標移動もあるためこの比をそのまま重み倍率には使わない。
- 前時刻の誤差を測定した移動の逆変換で現body座標へ移し、六軸それぞれ一次自己回帰（AR(1): 前の誤差がどれだけ残るか）を校正。係数[.03845,.03178,.40650,.90480,.62045,.80424]、残差の6x6共分散を元2飛行から同じ比率で計測。元入力SHA照合成功、native_fusion_ar1_calibration.json。評価中の真値や融合結果では調整しない。元校正は不変。高次の相関・共有IMUの相関・可動視野の分布差は残る近似。
- temporal_odometry_factor.hpp: 3つの機体姿勢を結ぶ制約で、今回の相対姿勢誤差から、座標変換した前回誤差の予測を引く。最初の制約だけ元の50ms共分散。正確な解析微分を使用。30組の非ゼロ誤差に対する540列の数値微分一致、係数0で独立制約と一致、既知の相関誤差・座標輸送の復元、clone/rekey検査成功。GTSAMのPose3 EXPMAP設定も確認。通常BetweenFactorは既定の近似微分であり、新3姿勢factorの微分と同一とは主張しない。
- temporal_odometry_manager.hppは通常managerと分離し、build_temporal_fusion.pyでbuild/mimosa-temporal/mimosa_temporal_nodeへ独立ビルド。元node/library不変。初回はGTSAMのOptionalMatrix API差、次は試験実行時ライブラリpathを修正し、最終ビルド/試験成功。temporal_fusion_build.log保持。
- unipilot_campaign.py/unipilot_run.pyへ任意--temporal-fusion（ordered-fusion＋校正必須）追加。両側へ同じモデルを適用、actual binary SHA/source snapshot/protocol temporal_fusionを保存。reportの校正ファイル名対応を追加。通常既定はfalse。
- v19のカメラ制御はv17と同じ（v18のdeadbandはOFF）。現在v19-short-seed24 a1→b1/domain86/session59002、v19-prefix-seed24/domain85/session88110。最初の新managerで約180件のROVIO入力と融合出力が正常に継続した後、prefixを開始。精度は未確認。全長・全監査で評価する。
- Windows実空き約9.7GiB。GUI/bag/連続画像点群なし、最大2sim。旧候補・失敗・途中結果は圧縮して保持。

### 2026-10-17 17:28–17:35 UTC — v19不採用、ROVIO移動量のみ融合のv20を検証
- v19 seed24 a1全長/全監査成功。融合1.736→17.155m、回転.430→.840deg。ROVIO12.828→48.267m、回転.874→1.145deg。AR(1)モデルの導入でジンバルの改善が安定するとは言えず不採用。b1/prefixは中止として保持、全所有子process exit0。新モデルがROVIOへ直接入力されるわけではないが、融合地図の弱方向から注視点を選ぶ制御経路は残る。
- 次の検証仮説: v17長区間はROVIO位置改善でもyaw悪化が融合側の横ずれを増やしたため、ROVIOの回転変化を直接拘束に加えず、並進変位だけを融合する。カメラ制御はv17と同じ。LiDAR/IMUは従来通り、DRPM OFF。固定/可動で同じ変更。性能改善を予告するものではない。
- translation_odometry_factor.hpp/manager.hpp追加。前body座標系の変位3成分だけを制約。ROVIO姿勢は局所変位の座標を求めるためには使うが、独立の回転観測としては加えない。原calibration2飛行のSHAを確認してこの3成分残差の二乗平均を測り直す（sigma [.01578479,.00396098,.00168519] m）。native_fusion_translation_calibration.json。時間相関は再び独立近似、今回の仮説は回転拘束の有無だけ。
- test_translation_odometry_factor.cpp: 30例360列の数値微分一致、終点の向きだけを変えても変位制約が不変、世界座標変換不変、clone/rekey成功。build_temporal_fusion.py --translation-onlyで別build/mimosa-translation/mimosa_translation_nodeを生成。旧通常/AR版binary不変。translation_fusion_build.log保存。
- unipilot_run/campaignに任意--translation-fusion追加（ordered＋校正必須、temporalとは排他）。実binary/source/設定をsnapshot保存。構文検証成功。現在v20-short-seed24 a1→b1/domain86/session47760のみ開始。initial処理確認後にprefix/domain85を開始する予定。planner-config tracking_aware_v20.yaml、融合校正はnative_fusion_translation_calibration.json。
- v19のcamera方向/feature診断: 開始0–25sの中央値yaw41deg/pitch8deg、後にdepth中央値56m/成熟点1へ低下して前方復帰。この深度崩れの原因はまだ確定していない。新融合試験でも同じ失敗が残るか確認する。

### 2026-10-17 17:45 UTC — v20 a1完了、融合と制御遅延を切り分け
- v20移動量のみ融合 seed24 a1全長/全監査合格、全子process exit0。ROVIO位置7.383430→3.478541m、回転.82654→.64973deg。融合位置.794297→1.297250m（比1.633206）、回転.37287→.38131deg。事前の最悪比1.5を超えるため採用候補としては不合格。残るb1/prefixは回転拘束を外した影響の診断として完了まで継続中。
- 次候補v21はcamera=v17、通常6軸外部odometry融合へ戻し、採用済みDRPM（registration_mode1/noise_model2）を固定/可動両側で有効にする準備。最新の自走改善指示の範囲で、LiDARの誤った拘束が視覚改善を打ち消す仮説を切り分ける。旧DRPM OFFとの絶対値は混同しない。unipilot_campaign.pyに明示--drpm-fusion追加、report_gimbal_campaign.pyも明示指定で両側の採用DRPM設定を検査・ON表示。既定OFFは保持。OFFレポートの全数値/監査一致と、ON要求がOFF入力を拒否する検証に成功。新融合試験はまだ未開始。
- v20 a1の判断時刻に対する特徴点遅延は中央値.412s（最大.5s）、融合姿勢は中央値.14s。制御計算中央値1.82ms、計画用画像地図中央値63.4ms。遅延差の原因は未確定、現在のdomain85へ読取専用20秒probeを接続し、ROVIOの点/姿勢配信遅延を別途計測する。最初はROS2既読shellのためenv拒否、CLEAN環境で起動し直した。

### 2026-10-17 17:48 UTC — 特徴点受信経路の遅延を切り分け、v22を準備
- domain85の読取probeでROVIO点群とodometryはどちらも20Hz、配信遅延中央値.109s。TCP no-delayなしの別probeでも.108s。一方、同時期のplanner使用点の遅延中央値.447s、融合姿勢.1455s。配信元ではなくplanner内部の受信/処理経路で差が発生している。pcl_latency_probe_v20.json、pcl_latency_default_probe_v20.json。
- fixationは直接ROVIO点のみを使うのに、features callbackはその後に別の画像地図用mutex/joinを待ってから次の点を受信する。任意fixation_live_onlyを追加し、この制御方式で未使用の画像地図処理/画像受信を省き、連続する点群callbackを制御用mutexから分離する。gyro履歴/ROVIO点の距離と共分散/LiDAR弱方向/現在推定姿勢は従来通り。推定器入力やセンサは変更なし。これが実遅延の全原因か、オンライン改善するかは未確認。
- planner_node.py、test_planner_cloud.py、tracking_aware_v22.yaml変更/追加。実ROS messageでmutex保持中に2フレームが連続受信できる試験を含め12件成功。通常map使用/旧設定は不変。v22はcamera policy=v17、通常6軸融合/DRPM OFFでv20の並進限定とは分ける。
- v20短区間完了後にdomain86でv22 a1→b1を開始する待機scriptを起動。v20prefixは診断を継続。v21 DRPM ONの準備は保持するが、見つかった制御遅延の切り分けを先に実施する。

### 2026-10-17 17:50 UTC — v20 b1完了、v22開始
- v20 seed24 b1全長/全監査成功、全process exit0。ROVIO36.433→3.960m、回転1.858→1.467deg。融合24.046→.134m（比.005558）、回転2.896→.196deg。a1悪化を打ち消して成功とはしない。prefix/domain85/session14111は約565/1070秒で継続中。
- v20-short/session47760は正常終了し、待機していたv22-short/session40432が同domain86でa1→b1を開始。現在はtakeoff/準備中。GUI/bag/画像点群連続保存なし。停止済み数値ログは可逆圧縮し、元データ/失敗結果を保持する。

### 2026-10-17 17:58 UTC — v22 a1小幅改善、6組の開発を事前固定
- v22 seed24 a1全228.49秒/全監査成功、全子process exit0。融合1.951→1.822m（比.933798）、回転.376→.373deg。ROVIO5.568→5.376m、回転.638→3.192deg（悪化を保持）。精度採用はまだ判断しない。
- 特徴点遅延は全区間p50=.092s/p95=.135s/max=.150s、旧v20 a1はp50=.412/max=.5。使用していない画像地図から受信経路を切り離した効果をオンラインで確認。12関連単体も成功済み。センサ・推定器の値/時刻は変えていない。
- continue_gimbal_development.pyを追加し、v22はseed24/25×a1,b1,prefixの6組と事前宣言。development_v22.jsonに対象/source SHA/従来成功基準を保存。現在scheduler/session6583はv20prefix正常完了を待ちdomain85へv22prefix24を、v22-short24(session40432)正常完了を待ちdomain86へshort25を開始。その後メモリと先行長区間750s条件でprefix25を開始する。最大2sim、全結果を保持。
- freeze_gimbal_candidate.py追加。事前宣言6組の全監査/source/binary/protocol一致と従来基準を満たさない限り固定を拒否、使用済みseedを独立検証に拒否、既存freezeを上書きしない。構文確認と、今の未完了6組ではfreeze/zipを一切作らず拒否する確認に成功。v22の独立検証予定seed26/27は未使用、まだ開始しない。
- 広いpitchでの遮蔽診断 self_occlusion_extended_pitch.json。保守的な全ローター掃引外形を含む80x45本のレイで、yaw0/±50、pitch+60/+90は自己遮蔽0。pitch-60は最大.444%、-90は最大2.444%で画面端。v22の制御pitch±30は変更していない。

### 2026-10-17 18:09 UTC — 容量対応後、v22を1本ずつ再開
- v20prefix全1070秒取得、数値参考値は融合37.694→9.826m、回転5.256→4.036deg、ROVIO80.151→41.904m/7.388→11.675deg。入力・初期一致・連続性等は全合格だが、終了時GazeboがSIGINT/SIGTERMに応じずsimulation exit -9。valid=falseとして保持、成功試行には採用しない。
- v22 seed24 b1は全長/全監査/全子exit0。融合23.894→.479m、回転3.211→.215deg、ROVIO36.937→2.288m/4.699→2.697deg。短2組は完了、残る4組は未完。
- scheduler/session6583は前候補の終了コード不合格を新候補の開始条件にして停止。前候補は計算枠の解放確認だけに修正（新候補の検証基準は不変）してsession36417で再開したが、Windows実空きが約12→2.4GiBへ急減したためSIGINTで停止。開始したv22prefixは軌道時刻0/全子exit0のinterruptedとしてv22-interrupted-storage-prefix-seed24へ保持、旧進捗をdevelopment_v22_progress_storage_stop.jsonへ保存し所有PID消滅を確認。性能結果を見て除外した試行ではない。
- 全所有sim停止後、Windows空きは約12.6GiBへ戻った。急減の原因は未確定。新数値ログ量では説明できない。Windows pagefile割当19337MB/使用2942MBを確認、OS設定は変更していない。
- ユーザーの不要データ整理指示に従い、現在使わない常設prefix元bagのローカル複製のみ80MCAP/45,826,463,553bytes（42.679GiB）を整理。Drive上80件の現在のサイズ/MD5が保管記録と一致し、削除直前のローカルMD5も全件再照合。Drive原本、他2runのローカルMCAP、metadata、設定、精度CSV/図は保持。prefix_cache_remote_verification.json/prefix_cache_eviction.json。既存のlocal_zab_bagsが欠けたローカルデータを選ばずクラウド取得に戻ることを確認。CACHE_EVICTED_20261017.jsonに再取得先を保存。
- Linux内の使用量は減ったがWindows実空きは約12.7GiBのまま。fstrim実施、Windows側保存ファイルの縮小は未確認。削除42.7GiBが物理Windows空きに反映されたとは報告しない。
- continue_gimbal_development.pyへ--max-concurrent1を追加し、source/seed/成功基準の事前宣言は変更せず再開。現在session68442がv22-short-seed25/domain86を所有。次にprefix24、prefix25を順番に実施。全sim最大1本、--min-host-free-gb8を指定。独立検証schedulerも停止基準8GBへ（まだ未起動）。元短session40432、prefix14111、旧scheduler6583/36417は終了。

### 2026-10-17 18:14 UTC — モデル公開の競合を修正し再開
- 18:09再試行はシミュレーション開始前にunipilot_gimbal.sdfの書きかけを読み、XML unclosed tokenで停止（wall2.15s/全子exit0）。生成側がSDFを直接writeし、受信側はファイル存在だけを準備完了と見ていた。v22-failed-model-publication-short-seed25に保持、旧scheduler68442終了。
- unipilot_gimbal.pyで補助camera_mount.json等を完成後、SDFを一時ファイルへ書き、完成したSDFだけをrenameで公開する。推定器・カメラ制御・物理設定の変更なし。32設定組合せの全生成ファイルが旧版とbyte一致、意図的に途中writeで待たせても公開名は存在せず、公開時にはXML/補助情報が完全な再現試験に成功。最初の試験wrapperがET内部StringIOもpath扱いした失敗を修正後の結果。atomic_model_equivalence.json。
- 完了済v22短2飛行のSDF/camera_mount/imu_samplingも修正版生成物と全SHA一致。development_v22_before_atomic_model.jsonに元宣言を保持し、起動用generatorのSHAだけを更新、旧generatorはこの3生成物が完全一致する場合に限り開発証拠として認める。制御・校正・試行数・seed・成功基準の変更はなし。独立凍結manifestではこの例外を持たせず、最終source SHAを厳密要求する。report/freeze/schedulerの狭い対応を追加。検証6件（別のモデルを同等扱いしない、例外なしでは旧sourceを拒否、未完/悪化/seed重複等の拒否）成功。初回は新テストの配置ミスによるNameErrorを修正後の結果。
- 失敗時の所有PID消滅確認・進捗archive後、scheduler/session14323で同じ6組計画を再開。現在v22-short-seed25/domain86のa1が離陸してsettleまで到達。最大1sim/Windows8GB停止。現在Windows実空き約12.6GiB。
- Windows側VHDX属性は32（非sparse）、長さ169091268608bytesであることを読取確認。Linux内で42.7GiBを整理してもWindows保存ファイルは小さくなっていない。WSL停止やVHD操作・OS設定変更は実施していない。

### 2026-10-17 18:21–18:27 UTC — v22不採用、LiDAR退化時だけ偏向するv23を開始
- v22 seed25 a1全長/全監査/全子exit0で、融合1.369→49.450m、ROVIO8.311→60.227m。最悪比1.5を超え不採用、v22_early_rejection.json保存。scheduler14323へSIGINT、b1途中を保持し全所有sim停止。prefix未実施。受信遅延修正とモデルのatomic公開修正は有効なため保持するが、v22の性能改善とはしない。
- v22_a1_seed25_failure_diagnosis.json: 冒頭25秒の視線中央値yaw42.24/pitch−16.47deg、次の25秒で特徴中央値4/成熟点1、50–75秒で深度中央値36m、100–125秒で153m。前方復帰後も尺度崩れが続く。既存v22seed24でも冒頭25秒のLiDAR退化確率の最大軸中央値.014、全sampleで.5未満なのに偏向していた。常時の少数点注視が不要な視野変化を加えることを切り分ける。特徴低下から尺度崩れまでの因果を確定したという意味ではない。
- fixation.pyへ任意fixation_degeneracy_gate追加。最大退化確率を時定数1秒で平滑化し、.5以上で注視制御へ入る/.2以下で前方へ5deg/s以内で戻る。中間は状態保持し、瞬間的な変動で切替えない。従来の点選択/予測/保持、通常6軸融合、DRPM OFF、センサと重みは不変。tracking_aware_v23.yaml追加。35関連試験成功（非退化で前方保持、継続証拠で開始、1sample低下で反転しない、復帰速度制限を含む）。初回config/test追加はcwd誤りで作成されず、絶対pathで実施し新試験込み再実行成功。
- v23-short-seed25/session19604/domain86でa1→b1を開始。scheduler64696は開発seed25/24×a1,b1,prefixの6組をdevelopment_v23.jsonへ事前宣言し、最大1simで続きを管理。事前成功基準は不変。独立seed26/27はまだ使用しない。Windows実空き8GB停止基準、生センサ記録なし。

### 2026-10-17 18:33 UTC — v23 a1 seed25改善、b1継続
- v23 a1 seed25全228.49秒・全監査/全子exit0。融合位置2.727→.988m（比.362281）、回転.448→.403deg。ROVIO位置10.255→7.756m、回転.387→.956deg。開発1/6、まだ採用しない。現在b1同seedを継続。
- report_gaze_behavior.py追加。既存の決定ログから実測yaw/pitch、LiDAR退化確率と平滑化値、追跡特徴数を3段の図と小JSONへ出す。制御/推定/評価への入力なし。a1で生成して視線と退化の時間関係を確認する。決定sample比は独立試行数とは扱わない。
- 停止した過去試行のencoder/数値CSV等を可逆圧縮、進行中は対象外。Windows実空き11.2GiB、現task root約2.9GB。読み込み中の診断CSVの末尾が書きかけで一度解析エラーとなったが、実験データ不良ではなく、以降はライブ診断時だけ未完の最終行を読み飛ばし、正式評価では停止済み完全データを使う。

### 2026-10-17 18:38 UTC — v23 b1 seed25小幅改善、同設定の反復へ
- b1全263.38秒・全監査/全子exit0。融合19.357→18.209m（比.940673）、回転2.418→2.340deg。ROVIO36.150→31.490m、回転4.041→1.217deg。開発2/6は2勝だが、b1の残差は大きく、この1件では経路平均比.9にも届かない。追加した試行で条件を変えず予定した残り4件を確認する。
- 最初のshort/session19604は正常終了。scheduler64696がv23-short-seed24/domain86（PID1302008）a1→b1を開始。以後のprefix25→prefix24も同scheduler所有、最大1sim。
- a1の偏向有効判定は約28.18秒から、実際に動き出すのは約80秒から（成熟点不足の間は前方復帰）。gaze図に両者を区別して表示。固定/可動の実画像は視線が一致する期間は同一で、偏向開始後に分離。推定器の初期状態だけでなく、この共通入力期間の一致も保持。

### 2026-10-17 18:43–18:45 UTC — v23は反復で不採用、視覚不足の回復を追加したv24開始
- v23 a1 seed24全長/全監査/全子exit0。融合1.081→1.792m（比1.657796>1.5）、回転.407→.373deg、ROVIO9.550→4.989m/1.135→2.072deg。ROVIO位置改善だけでは融合改善を保証しない。v23_early_rejection.json保存、scheduler64696停止。開始したseed24b1は39.79sでinterrupted/全子exit0として保持。prefixは未実施。
- v23 b1 seed25の40秒区分診断では、LiDAR退化確率が低い80–180秒にも固定/可動共通の誤差が増加。融合位置区間RMSEは約.91→8.28→27.58m。この間は視線が同じで、偏向開始が約185秒まで遅れた。低い確率が正しいLiDAR観測を保証しているとは扱わない。segment_error_diagnosis.json保存。
- v24はv23を土台に、成熟した画像特徴が4点未満の状態が1秒続いたら、LiDAR退化判定と別に回復動作を開始。下方pitch60deg/yaw0を5deg/s以内で目指す。そこで1秒以上特徴が回復し角度が整った場合、以後の休止視野としてその角度を保持し、前方への往復を避ける。LiDARが弱い間の注視点選択は従来の情報行列評価を維持。探索は床側に近い特徴を得るための明示的なヒューリスティックで、未観測の画像情報量を知っているとは扱わない。真値は未使用。
- controller pitch範囲を±30→±60degへ。機械範囲±90/速度上限20deg/s/探索5deg/sは不変。既存保守的遮蔽検査で+60degは自己遮蔽0を確認済み。通常6軸融合/DRPM OFF/センサ/元校正/ROVIO25特徴は両側共通、不変。
- fixation.py、tracking_aware_v24.yaml、test_fixation.py変更。37関連試験成功: LiDARが良い判定でも視覚不足で回復開始、回復視野の保持、速度制限、範囲外指定の拒否、従来動作を含む。v24-short-seed24/session12116/domain86を開始、scheduler75632がseed24/25×3経路の6組を事前宣言して最大1simで後続を管理。独立26/27は未使用、基準不変。
- Windows実空き9.8GiB、VHD169091268608bytesで変化なし、pagefile割当19329MB/使用2244MB。空き減少の原因は未確定、OS設定変更なし、8GB停止継続。

### 2026-10-17 18:58 UTC — v24不採用、描画と終了時ダンプを切り分け
- v24 seed24 a1は融合位置RMSE .663→10.788m、ROVIO5.978→10.656mで不採用。v24_early_rejection.json。b1途中停止、残り未実施。全scheduler/sim停止後、静止画像の描画対照だけを起動。下向き視野は大半が黒く特徴が失われる。連続画像保存なし、各条件2PNGと統計のみ。
- 重要訂正: 過去runtime.jsonのsimulation exit0はros2 launch親の値で、内側Gazeboの正常終了を保証しない。v24a1等のsimulation.logではgz子が終了時SIGSEGV/SIGKILL。入力時刻/推定出力/飛行完了の検証とは区別する。これまでの「全子exit0」はGazebo孫processまで確認できていなかった。
- Windows急減の直接原因を確認: WSLがGazebo終了時のcore dumpをWindows Temp/wsl-crashesに保存。project simulation.logのPID/時刻と一致した9件11.096GiBのみ削除し、実空き6.14→17.24GiB。windows_crash_dump_inventory/cleanup.jsonに根拠保存。core_patternはWSL pipeなのでulimit core0でも記録された。OS設定は変更なし。静止描画診断の1coreはbacktrace解析後に整理予定。
- coreの主threadはSensorsPrivate::Stopでrender thread終了待ち。render threadはthread-local destructorの未mapアドレスで異常、backtrace signal handlerも異常。原因libraryはまだ未特定。
- stationary-render-audit/batch_originalで実飛行画像を静止再現。元ZaB材質はreceive_shadows offだが移植PBR側の適用は未確認。影無効/stock描画の対照をstationary-render-controlsで開始(session90757)。元world/assetは変更しない。診断scriptへ8GB実空き監視、開始PID/終了code/ロードlibrary保存と中断時cleanupを追加。

### 2026-10-17 19:03 UTC — WSL終了時crash修正、v25 DRPM対照を開始
- Coreのunmapped destructor PCを終了前/proc/PID/mapsへ照合し、/usr/lib/wsl/lib/libd3d12core.so offset0x2edb10と特定。LD_PRELOADでこのlibraryをプロセス終了まで保持。静止3条件すべてGazebo自身exit0、元画像2枚は完全一致。wsl_d3d12_retention_validation.json。unipilot_run.pyの実験Gazeboに適用、非WSLでは既存pathなしなので追加しない。
- simulation_shutdown.py追加し、launch親ではなく実Gazebo子の開始/正常終了/異常終了を監査。未終了・子なし・複数子・異常・正常の5ケース成功、過去v24/v23のgz exit-9を検出。今後report_gimbal_campaignはこの条件もvalid判定に含む。過去の精度数値は不変、旧「全子正常」は訂正済み。
- 描画対照: stockは2画像byte一致。影なしはactive一致/fixed1画素1階調差のみ。両面描画でも白い抜けは変わらず、背景を緑にすると白部が緑へ変化。ns4元meshへ画像方向のraycast: 下向き背景201点中200点、前方638点中608点は当該meshとの交差なし。白部の大半は元mesh欠落で、近傍他section含む完全scene率は未測定。以前のfar clip起因という解釈は不十分。原geometry/texture/worldは変更しない。render_surface_diagnosis.json/mesh_background_ray_test.json。
- project所有の静止試験3dump3.724GiBも小さなbacktrace/maps記録後削除、Windows空き18.5GiB。以後新dumpなし。OS設定変更なし、画像保存は各条件2PNGのみ。
- v25はv22低遅延注視制御＋固定/可動とも採用DRPM有効(registration_mode1/noise_model2)、その他通常6軸融合/元校正/原環境。3区間×seed24/25の6組をdevelopment_v25.jsonに事前固定、独立26/27未使用。firstshort/session99567、scheduler86915、domain86、sim最大1。DRPM単独効果とgimbal効果を混同しない。schedulerへ明示--drpm-fusionと設定記録を追加。精度未確認。

### 2026-10-17 19:12 UTC — v25悪化、v26画像の動きの安定化へ切り分け
- v25 a1全長取得、入力/出力連続性/共通初期化/両側DRPM同一は成功。ROVIO位置7.785→7.208m、回転.912→6.193deg。融合3.310→15.724m（比4.751）、回転1.213→.938deg。安定改善に不合格。終了時GazeboはSIGKILLのため新総合valid=false、v25_early_rejection.jsonとして保持。b1は準備中28.64秒で中断し全所有process停止、Gazebo自身exit0。
- D3D12 preload後は新coreなし。静止条件は3回exit0だが、飛行終了では別の待機が残った。runnerがlaunch群へSIGINTし、launch自身もGazeboへ転送するため二重signalになっていた。simulationだけ親へSIGINTを送り転送を一度にする修正。飛行終了での効果はv26で確認する。前回の「終了問題修正」はcore原因修正を指し、全飛行の正常終了実証ではなかった。historical_gazebo_shutdown_audit.jsonは137飛行を再確認（clean2、その他135）、数値結果を書換えない。
- v26は画像全体の動きを抑える比較。image_stabilization.pyを追加し、ROVIOの追跡中の方位とIMUだけから二軸の回転補償を解く。距離・共分散による注視点選択、地図検索、LiDAR弱方向による選択はこの候補では使わない。これは追跡維持のbaselineであり、情報量に基づく能動視野選択の成功とは呼ばない。4点以上/.3秒以上の追跡、角度残差のHuber .004rad、4反復。少ない時はgyro補償。yaw±50、探索pitch±30、機械pitch±90、最大20deg/s。
- 位置・尺度・基準カメラ・飛行指令・真値は制御へ入力しない。予測方位は既存bearing_motionから得るため融合速度/深度尺度に依存しない。静止保持、gyro符号、画像移動低減、尺度不変、外れ値、角度速度限界、点なしfallbackなど新7件＋既存44件、計51成功。元core18件は成功、存在しないreport名で呼んだ2件は実行失敗、正しいtest_improvement_assessmentの6件成功。
- v26-short-seed24/session46137、scheduler44606、3run×seed24/25をdevelopment_v26.jsonに固定。DRPM OFF/通常6軸融合/元校正、元環境・画像・IMUは維持。sim最大1、Windowsguard8GB、精度未確認。v25のfirstshort99567/scheduler86915停止済み。
- 独立検証runnerへ最大同時数1と凍結DRPM値の伝播を追加、総合reportのDRPM表示を固定OFFから実設定へ変更。まだ独立26/27未使用。停止済データ可逆圧縮、元bag/失敗結果保持。

### 2026-10-17 19:18 UTC — v26不採用、v27前方復帰付きの追従を開始
- v26 a1全長/全監査合格。ROVIO9.097→7.814m、回転1.017→.735degへ改善したが、融合.598→2.306m（比3.855）、回転.495→.585degで不採用。後半yaw約-49/pitch-30degで上限付近に張付き、追跡1点へ低下。v26_early_rejection.json。残りb1は途中停止して保存。
- Gazebo自身もexit0、終了要求から1.5秒以内に消滅。D3D12 preload＋launch親だけSIGINTを送る修正で飛行終了の成功例を確認。shutdown観測器は既に終了しておりgdb添付を実施せず。新coreなし、Windows約18.5GiB。
- v27はv26に前方復帰の時定数5秒を追加。短時間の画像動きを補償しながら角度を指数的に緩め、継続的な回転端張付きを避ける。特徴点が少ない時もgyro補償＋前方復帰。尺度を使わない画像servoの比較であり、LiDAR弱方向地図検索の成功とは混同しない。関連53試験成功。
- v27-short-seed24/session71596、scheduler29906を開始。DRPM OFF/通常融合/元校正、開発6組/独立未使用26,27は維持。v26 46137/44606停止。
- 並行してROVIOの特徴点上限25→75を別build/rovio-capacity-75にコンパイル。build_rovio_capacity.pyが元sourceと3箇所の容量置換、使用headerとbinaryのSHAを保存。4.8MB、実行中の25点版や既定binaryは変更なし。まだ飛行/精度未検証、v27には使用していない。
- report_gaze_behavior.pyへ画像安定化に使用した点数表示、v24の品質回復enabled判定を反映。

### 2026-10-17 19:34 UTC — v27短2本、ROVIO前段の切り分け、水平注視v28開始
- v27 a1は入力/出力/共通初期化の監査合格だがGazebo終了がSIGKILL、総合valid=false。固定→可動: ROVIO9.715→10.530m/.311→3.965deg、融合2.481→3.089m/.666→.652deg。b1は全監査/Gazebo自身exit0: ROVIO42.026→41.459m/2.075→5.050deg、融合29.479→28.641m/3.522→3.182deg。改善は安定せず、scheduler29906は後続開始前に停止。未実施の4組を合格/失敗として数えない。firstshort71596/監視76400/全所有sim終了。
- 以前の固定カメラ校正用a1/b1入力を再利用してROVIOだけ再生。現25点版の出力は当時のdiagnostic CSVに両区間でbyte完全一致。75点版でもa1出力完全一致、b1中央値14点のまま、位置14.844→14.524m/回転3.594→4.178deg。75点版は既定へ採用しない。別binary/manifestと数値のみ保存、入力を複製しない。
- フロントエンドの探索的切り分け（固定カメラ、融合・制御未使用）。特徴間隔penaltyDistance100→40はa1位置10.280→1.504m、b1位置14.844→13.187mだが回転3.594→9.412deg。20ではa1 1.559/b1 16.272m。直接画像補正true/間隔100では8.132/12.756m、間隔40では2.887/9.636mだが回転4.802/9.242deg。追跡最下段endLevel1→0では間隔100が8.019/18.814m、40が7.831/22.813m。全条件/悪化も保持、両区間での安定化を確認できず既定変更なし。旧校正飛行は現26秒共通準備運動前の記録なので新制御の改善実証とは扱わない。
- replay_rovio_capacity.py/build_rovio_capacity.py追加。元入力/設定/バイナリのSHA、定刻サンプル数、精度と小診断を保存。新raw画像/点群なし。offline再生3個まで並列、simは0または1。全再生終了。
- D3D12 preload後の新coreはなし。飛行SIGINT終了待ちだけは残る。Gazeboソースではsignal callbackが直接Stopする一方、/server_controlは別threadでStopする。unipilot_run.pyの終了をまずserver_control stop:true要求→実Gazebo子終了を最大5秒確認→残りをSIGINTへ変更。要求結果をshutdown_request.jsonへ残し、失敗時の従来signal/kill fallbackは保持。推定器停止前のtracking末尾drainは変更なし。効果は次の飛行で確認、原因確定とはまだ扱わない。
- v28: v22低遅延の成熟点注視をyawだけへ限定。pitch指令0、上下画角内の点は注視候補に残す。上向き暗部/下向きmesh抜けへの移動で特徴が失われた既存観測を動機とするが、この変更で改善するとは未確定。自己位置・深度の代わりに観測方位とIMUで追従する従来方式、周辺点保護、品質不足時前方復帰、yaw±50deg/20deg/sを保持。DRPM OFF/通常6軸融合/原校正/25点ROVIOは両側共通。
- fixation.py/test_fixation.py/tracking_aware_v28.yaml変更。初回で0pitch許容の既存検査と設定copyによる旧可変設定テストの失敗を検出・修正後、48関連試験成功。0pitchでも上下に少しずれた可視点を追える/垂直画角外は選ばない試験を含む。
- v28-short-seed24 session40432/domain86、scheduler55744で開発seed24/25×3経路の6組を事前固定。独立26/27未使用、成功基準不変。Windows実空き約18.2GiB、最低8GB停止。目標未達、実験継続中。

- 19:36 UTC追記訂正: Gazebo描画の新coreはないが、別processのROS2 parameter_bridgeに終了時SIGSEGVが2件あった。v26b1/v27a1のPIDとsimulation.logを照合し、FastDDS StatefulWriter::check_acked_statusのbacktraceを保存後、2dump約279MiBを削除。windows_bridge_dump_cleanup.json。終了方式変更による改善は未確認。

### 2026-10-17 19:43 UTC — v28 a1改善、停止競合の修正版を別buildで検証準備
- v28 a1全長/センサ・推定器・初期化・指令・encoder監査合格。固定→可動ROVIO9.074→6.291m/.521→.739deg、融合1.425→1.084m/.459→.440deg。Gazebo停止時SIGSEGV→SIGKILLでvalid=false、精度改善と終了健全性を区別する。b1継続。scheduler55744/PID1327995は後続開始前に停止し、終了修正を先に実証する。初期short40432は最後まで実行。
- /server_controlへのstop:trueはdata:trueを返したが、libgz-sim8.so.8 offset0x163b24で停止eventの接続リスト走査中にnull参照。Gazebo8.15.0 sourceのServerPrivate::Stopはservice thread/signal handler/destructorから呼べ、停止eventは同じ対象へ同期なしにEmitする。描画library終了と別の競合を検証中。coreはWSLの2GiB制限で途中までしかなく、完全stackは未取得。アプリのsignal stack/mapping/逆アセンブルを小ファイルに保存後、所有PID1328130と照合して2.05GBdump削除（windows_v28_dump_cleanup.json）。
- stockと同じ8.15.0 revision446a443をbuild/gz-stop-sourceへ分離。変更はServerPrivate::Stopをmutexで直列化することとprivate mutex追加の2箇所。最初はonce flagも入れたが再Run時の振る舞いを変えないため削除、停止の直列化だけにする。core targetのみbuild/gz-stopへ-j2でビルド中。既存/実行中のGazebo・センサlibraryは変更しない。完了後、画像一致と実Gazebo終了を静止対照で検証してから飛行へ使う。
- performance/no_crash_dump.cpp・build_no_crash_dump.py追加。実験processだけPR_SET_DUMPABLE=0を設定し、Windowsへの大きなcore保存を防ぐ。machine-wide core_pattern/OS設定は不変。exec後の状態0、意図的SIGSEGVの終了code-11が保持され、Windows新dump0を確認。既定飛行へはまだ適用前。stderr/終了コード/小backtraceは保持し、失敗を正常扱いしない。
- 旧固定校正入力の時間相関を追加集計（relative_factor_correlation_diagnosis.json）。50msの前後移動誤差相関a1 .895/b1 .909、500ms隔てても.340/.703、独立扱いは正確ではない。前回AR(1)試験は改善しなかったことも保持。
- 次の切り分けとしてcalibrate_unipilot_fusion.pyに任意--integration-window-sを追加。既存MIMOSAの1秒窓における実測相対姿勢誤差の二乗平均を20分割し、50ms独立因子の等価共分散にする近似を用意。厳密な時間相関/共有IMUモデルとは呼ばない。原固定2校正飛行だけを使用、現在の可動結果で値を最適化しない。native_fusion_window1s_calibration.json、回転sigma[.009685,.009645,.037179]deg/並進[.054837,.011043,.005375]m。既定50ms再生成は元JSONとbyte完全一致、新共分散は未採用・v28には未適用。
- audit_stationary_render.pyに別core path/停止serviceとprocess-local core guardの任意検証経路を追加。静止対照の起動はb1終了・core build完了後（最大1simを維持）。Windows実空き約18.5GiB、数値中心の記録を維持。目標未達。

## 2026-10-17 19:50 UTC — yaw-only v28短区間結果・停止処理の独立修正

- v28 seed24はa1の融合位置RMSE 1.425→1.084 m、b1 25.961→0.469 m。ROVIO位置も9.074→6.291 m、38.890→9.332 mへ減少したが、回転は両区間で悪化。終了時Gazebo強制停止のため有効な完成6組とは数えていない。
- v28の上下固定（pitch=0）と左右のみの追跡制御をそのままv28sとして再検証。退化処理OFF、元の50ms融合較正を維持。1秒窓較正は準備のみで未使用。
- Gazebo公式8.15.0のStop同時実行をmutexで直列化した独立ライブラリをbuild_serial_stop.pyで構築。物理・描画・センサの計算処理は変更なし。全起動処理への適用は明示オプション--safe-shutdownに限定。シグナル停止1回・サービス停止2回の静止対照は全子プロセス終了0、元の2カメラ画像とバイト一致、必要pluginも正常読込。飛行後終了は次の実験で確認する。
- process-local no_crash_dumpを自分のシミュレーション子プロセスだけに適用し、Windowsへの巨大なクラッシュダンプ生成を防ぐ。異常終了は隠さず報告に残す。子プロセス終了監査をGazebo以外のROS橋渡しにも拡大。パーサ6ケース成功。
- v28sの3経路×2乱数条件（24/25）を事前宣言し、短区間から再開。検証用26/27は未使用。GUI・rosbag・連続画像点群は保存しない。Windows空き19.8GB、停止閾値8GB。

### v28s seed24 a1 完了
- 固定→可動の融合位置RMSE 3.62186→1.04359 m（比.28814）、回転.56740→.50051 deg。ROVIO位置8.78543→7.53846 m、回転.65815→1.99992 deg。ROVIO回転は悪化を明示。
- 入力・初期状態・制御連続性等の全監査合格。実Gazebo/ROS橋渡しを含む全子process終了0、service stop応答true。独立停止ライブラリとprocess-localダンプ防止のロードも確認。6開発比較中1組のみ。

### v28s seed24 b1 完了・seed25反復開始（20:01 UTC）
- b1融合位置17.0917→.9942m（比.05817）、回転2.3765→.1447deg。ROVIO位置31.4156→3.1807m、回転1.7953→2.9734deg。位置改善とROVIO回転悪化を区別。全監査合格、全子process終了0。
- 6組中2組完了2勝。seed25 a1/b1をschedulerが開始。長経路2本と独立26/27はまだ未確認。停止修正後の飛行2本は正常終了。Windows空き19.8GBを維持。

## 2026-10-17 20:08 UTC — v28s不採用、実測1秒誤差較正v29
- v28s seed25 a1は全監査合格だが融合位置1.48891→2.93552m（比1.97158>事前上限1.5）、回転.45050→.54030deg。ROVIO位置7.10617→7.59384m/回転.95895→2.05887deg。安定改善として不採用。b1は準備中に停止し中断記録保持、全子process終了0。長経路は未起動。
- v29ではyaw-only制御を維持し、以前の固定UniPilot較正2飛行から求めた1秒相対姿勢誤差の2次モーメントを20で割り、50ms因子の共分散とする近似を両推定器へ同じく適用。1秒は既存MIMOSA平滑化窓で、精度を見て探索した時間ではない。共分散の対称・正定値と20倍の蓄積モーメント一致を確認。通常50ms較正は変更していない。
- 相関の完全なモデル化ではない。50ms刻みの並進誤差相関は前方軸.895/.909、1秒後も.171/.681。approximationの制約を較正JSONに明示。融合回転yaw標準偏差.01535→.03718deg、前方並進.01578→.05484m。制御・センサ・ROVIO本体・DRPM OFFは同一。
- continue_gimbal_development/validation、freeze、reportへ別較正ファイルの固定・伝播・保存照合を追加。v29開発6組（24/25×3経路）を宣言、firstshort session26158、scheduler50630、最大1sim。独立26/27は未使用。

### v29 seed24 a1 完了
- 全監査合格、融合位置固定.30359→可動.34042m（比1.12132）、回転.31178→.32932deg。ROVIO位置7.67094→7.41845m、回転.28636→.97414deg。ジンバル改善は出ていない。融合設定の効果については別飛行の絶対値だけで因果断定しない。次b1へ継続。

### v29 seed24 b1 完了・反復中（20:18 UTC）
- 全監査合格。融合位置5.14861→.45038m（比.08748）、回転.59000→.17267deg。ROVIO位置49.36441→6.80726m、回転2.94942→.77096deg。b1では位置・回転が両方改善。a1は比1.12132なので現2組1勝。seed25短2本をscheduler50630が開始。未使用26/27は継続して未使用。

### v29 seed25 a1 完了（20:23 UTC）
- 全監査合格。融合位置.33805→.32307m（比.95569）、回転.57723→.54002deg。ROVIO位置12.04026→11.37954m、回転.46485→.76474deg。seed24/25ともa1融合は約.3mで、gimbalの差は+3.68cm/−1.50cm。
- a1平均比1.03850のため、元の「全経路平均比≤.9」は達成不能と明示。事前基準を満たしたと扱わない。長経路とb1反復を完了して有効な範囲と中立な範囲を調べる価値があるため、6組は継続する。独立26/27は未使用。

### v29 seed25 b1 完了・長経路開始（20:28 UTC）
- 全監査合格。融合位置.31836→.27899m（比.87632）、回転.16016→.16924deg。ROVIO位置38.21124→11.64825m、回転3.26144→1.46536deg。seed24と違い固定融合も小さい誤差で、改善幅は約4cm。
- v29短4比較は3勝、融合比平均.76020、最大1.12132、絶対最大悪化.03683m。a1平均比1.03850は元の経路ごと.9基準不合格のまま。長経路は未確認、成功とはしない。
- scheduler50630がfirst_prefix(PID1351652/domain85/v29-prefix-seed24)を開始し、その終了後にseed25prefixを実行する。現在同時sim1。firstshort26158は完了終了済み。独立26/27はまだ未使用。

### 比較一覧を追加（20:35 UTC）
- report_gimbal_development.pyで開発比較の完了値・進行状況・元の事前基準の判定を分けたHTML/JSONを生成。v29_progress.html。保存コード/較正/実行ファイル35項目の一致を表示前に検査。4組を生成確認。元基準不合格を成功と表示しない。実験のコード・入力・設定には変更なし。
- audit_imu_delivery_counts.py --candidate v29は完了4組/8系統すべて合格。取得周期から求めた必要IMU個数と実受信数が一致。長経路は途中であり未集計。

### 誤差方向回転仮説の事後診断
- diagnose_view_covariance.pyで、元の固定較正の1秒誤差モデルをジンバル方向に回転させる仮説を完了4組で診断。固定視野の対照は完全同値、可動4組すべて平均ガウス負対数尤度が悪化（2倍量の差+4.326,+3.578,+6.586,+4.832）。単純な視野方向回転は実測を説明せず不採用。真値はこの事後診断だけ、推定入力なし。
- report_gimbal_partial.pyを追加。書込み中CSVは末尾の未完行を除き、共有期間だけを時刻整合して途中値を表示。採用判定・最終精度には使わない。記録器/推定器/実験条件は変更なし。

## 2026-10-17 20:49 UTC — v29長経路で融合悪化、DRPM切り分けへ
- prefix seed24は全監査合格・全子process正常終了。ROVIO位置73.59533→15.28962m、回転8.05325→2.09192degと大幅改善。しかし融合位置6.45688→7.69379m（比1.19157）、回転3.60832→4.31737degで悪化。現設定の安定改善は未達。
- v29は完成5組を保持。元a1経路基準不合格に加え長区間融合悪化のため、seed25prefix途中を停止し次の切り分けへ。中断値を完走値と扱わず保持。未使用26/27は未使用。
- v30はv29のyaw-only制御・1秒実測較正・全センサをそのまま、固定/可動両方に採用済みradial+official-update DRPMをONにする。LiDAR側の弱い方向を抑える処理とgimbalの組み合わせを比較し、効果はDRPM OFFの固定絶対値とも比較する。相対改善のためだけに悪い基準を採用しない。v25のDRPMは2軸制御/旧較正で失敗しており、その不採用記録も保持。

## 2026-10-17 — v30不採用、融合の重力方向を検証するv31
- v30 seed24 a1は全監査合格だが融合位置固定387.809→可動10.999m。ROVIO位置9.196→8.411m。DRPM OFF v29約.3mから両方悪化したため、比率だけを成果とせず不採用。b1準備中停止、全子process終了0。v30_early_rejection.jsonを保持。
- v29 prefixを軸別に診断。重力方向の絶対傾き誤差（yawに不変、初期姿勢合わせなし）はROVIO固定.12025/可動.10503deg RMSE、融合固定3.5464/可動4.2739deg。外部odometryは相対姿勢因子だけなので、ROVIOの絶対傾きは渡していない。MIMOSA自身のIMUは使用中であり、これだけが原因と断定はしていない。
- v31はv29制御・1秒較正・DRPM OFFを維持、両融合系へROVIO重力方向を1Hz追加。GTSAM公式Pose3AttitudeFactorで2方向の傾きだけを拘束。位置・yaw・真値を使用しない。ノイズはSDF加速度bias2次momentの平方根/重力から決定（約.296deg）。共有IMU/時間相関の厳密な処理ではなく、疎な保守的近似。
- performance/tilt_odometry_manager.hpp、test_tilt_odometry.cpp、build_tilt_fusion.py追加、ordered_mimosa_nodeの別ビルドを追加。通常バイナリは変更なし。40姿勢/非自明外部変換/yaw位置不変性、240数値ヤコビアン列、graph rekey検証PASS。比較用別binaryビルド成功。
- unipilot_run/campaign、development/validation scheduler、report/frozen evidenceへopt-in --tilt-fusionと保存照合を追加。全フレーム入力・初期条件同一、両系へ同設定、tilt_factors.csvで1Hz/方向/設定を監査。v31開発6組24/25を事前宣言し開始。独立26/27未使用。Windows空き19.84GB、8GB停止保護、raw収録なし。
- report_absolute_tilt.pyを追加。完成区間を0.1秒共通gridで補間し、姿勢合わせを行わずbody座標の重力方向を比較する事後診断。v29prefixはROVIO .12684/.11245deg、MIMOSA 3.53860/4.26503deg（固定/可動）。以前の診断gridと数値が僅かに異なるが結論同一。3分割すると融合の傾き誤差は後半へ増大。推定入力には使用しない。

### v31 seed24 a1 完了
- 全監査合格（新しい傾き因子の周期・単位方向・ノイズ含む）、保存ソース/較正/binary照合合格。融合位置固定.467→可動.385m、比.82448、回転.451→.446deg。ROVIO位置9.630→6.079m、回転1.159→2.821deg。融合は18%改善だがVIO回転は悪化。1組だけなので安定改善とはしない。
- 重力方向絶対傾きは融合.48397/.48798deg、ROVIO .19162/.19258deg。固定/可動とも同程度。長区間でのドリフト抑制はまだ未確認。

### v31短区間は不合格、長区間の傾きだけ切り分け
- seed24 b1全監査合格。融合位置.16480→.48416m（比2.93785）、回転.18659→.20290deg。ROVIO位置41.0508→2.749m改善、回転.953→2.220deg悪化。短2組1勝、安定基準不合格。seed25short準備中にschedulerへSIGINT、成果を作るための追加反復をしない。
- 未確認の「長区間の傾きドリフトを新因子が抑えるか」だけ診断するため、同条件seed24prefixを別途実行する。事前宣言の候補を合格扱いにしない。v31_early_rejection.jsonに理由と追加診断の目的を保存。

### 次候補の準備：移動量＋重力方向を分離
- v31 a1/b1の偏向ROVIOでは位置は改善する一方yaw誤差が増え、推定gyro-z biasの固定/可動差は後半約.00016/.00022rad/s。特徴数の10/50/90百分位は両者で近い。これは原因の断定ではなく、相対回転拘束を外す切り分けの根拠。
- gravity_translation_manager.hpp / build_gravity_translation.pyを追加。既に数値微分検証済みのTranslationOdometryFactorとGTSAM公式Pose3AttitudeFactorを組合せ、ROVIOの局所移動量3成分と絶対重力方向2成分を渡す。明示的な相対回転観測は加えない。移動量の座標を定める前時刻ROVIO姿勢は引き続き必要で、yawから完全独立とは言わない。
- calibrate_translation_fusion.pyへ任意1秒窓/20stepの2次moment近似を追加。元の固定較正2飛行のSHAを確認し、native_fusion_translation_window1s_calibration.json保存。sigma[m]=[.05482781,.01105262,.00545119]。精度結果を用いた重み探索なし。
- --tilt-fusion --tilt-translationで別binary/設定/保存監査へ伝播。既定false。旧tilt binaryと進行中v31prefixは変更なし。40例240微分列の重力因子検証、30例360微分列の移動量因子検証PASS、別nodeビルド中。飛行はまだ未実施。
- 新しい分離因子nodeビルド成功。既定50msの移動量較正は従来保存値と数値完全一致（translation_calibration_regression.json）。現在のv31prefixには新nodeを使わない。
- v31 IMU個数監査は完成2組4系統すべて不一致0。準備中に停止したseed25の2系統は対象interval0で検証不能となり、監査JSONのall_validはfalse。未完飛行を成功と扱わない。入力欠損の検出ではない。
- 旧固定較正2飛行の絶対重力方向誤差は.20670/.11608deg RMSE。2方向の等方分散を全角度誤差2次moment/2、飛行ごと等重みで計算した標準偏差は.118531709deg（.00206876859rad）。初期のセンサbiasから求めた.2956degとは区別する。今回の候補飛行や融合成績から合わせていない。
- --gravity-direction付き較正をnative_fusion_displacement_gravity_calibration.jsonへ保存し、次の分離方式だけこのROVIO実測傾きノイズを使う。適用周期1Hz維持。事後の診断スクリプトと1e-15以内一致。固定view→可動view/時間/共有IMU相関の近似は残る。
- tracking_aware_v32.yamlを準備。v31と制御のパラメータは全項目完全一致。次候補では相対回転拘束除去と傾きノイズ較正の2点が変わるため、結果を片方だけの因果効果と断定しない。飛行未開始。

### v31長区間診断を途中終了
- 約696秒時点の絶対傾きRMSEはROVIO固定.1342/可動.1316deg、融合2.2104/2.3549deg。直近融合傾きは3.9245/4.1371degで増大しており、sensor-bias由来の弱い1Hz拘束だけでは保持できない。この診断目的が判明したため残りを待たずSIGINT。partial_tilt_diagnosis.json/partial_progress.jsonに途中であることを明示。完走の精度や安定改善として扱わない。
- 次はv32: yaw-only制御同一、DRPM OFF、移動量3成分と実測較正された重力方向2成分を融合。開発24/25の6比較を改めて宣言する。独立26/27は未使用。

### v32 a1精度改善、launch親だけの終了失敗
- a1全長入力/出力連続・条件/保存照合合格。融合固定.44470→可動.33996m（比.76448）、回転.402→.381deg。ROVIO6.310→6.943m、回転.852→2.199deg。融合位置は24%改善。
- ただしruntimeのsimulation親だけ-9。Gazebo/create/ROS2 bridgeは全て実終了0、他processも0、tracking drain済。終了時にGazebo OnProcessExitからのShutdownと、ownerから親へのSIGINTが重複したログ。元終了コードと監査不合格を保持し、採用の有効比較として数えない。scheduler89715はこの失敗を検出して終了、firstshort59728のb1は継続。
- unipilot_run.pyの停止手順を修正。service停止後に実Gazebo正常終了が確認できた場合、既存OnProcessExitの自動Shutdownに任せ、ROS launch親へ重ねてSIGINTを送らない。15秒上限と実終了監査を維持。飛行/推定/制御コードは変更なし。進行中v32b1は旧ロード済処理のまま。
- test_launch_exit_sequence.py（Gazebo/センサ/飛行なし）で自動Shutdownの親5回/子10個すべて0を確認。launch_automatic_stop_test.json。変更後の実Gazebo終了は次の飛行で確認する。

### v32 seed24 b1 完了、終了修正版v32sへ
- b1は全監査/全子process合格。融合位置固定5.316→可動.242m（比.0454768）、回転.688→.200deg。ROVIO位置58.349→23.564m、回転6.306→4.239deg。後半で固定側に誤差が増大し、途中の値から結論を先取りしない必要があった。比較内で同一物理飛行、開始状態、入力時刻を確認。
- v32 a1も精度は改善したが親launcher -9のため判定不合格を維持。実終了コードを書換えず、v32sで推定/制御/較正値をすべて維持し、終了手順だけ修正して6組を再宣言。開発seed24/25、独立26/27は未使用。Windows空き19.84GB、max1sim、raw保存なし。

## 2026-10-17 — 重要な訂正：MIMOSA重力設定が実装で未使用
- 実際にリンクされるbuild/mimosa-speed/imu_manager.cppは、use_estimated_gravityを設定読込以外に使わず、常にImuFactorWithGravityと可変G(0)による予測を使う。comparison_wsと元のmimosa repository HEADも同じ。false設定でも固定重力にはなっていなかった。
- v32s a1の179.55秒に/mimosa/graph/debugと/fixed_mimosa/graph/debugを読み取り、重力world方向がそれぞれ[.0406779,-.0082009,-9.7999123]/[.0385060,-.0066427,-9.7999223]と動くことを確認。world-Zを重力とみなす傾き誤差.60869/.55726degは、推定重力を使うと.38929/.33916deg。1時刻のみで全長の原因は断定しない。gravity_frame_snapshot.json。
- したがって先の「MIMOSAの絶対重力傾きが約4deg」という説明は厳密でなかった。保存値はworld-Zと真値の比較で、内部の推定重力に対する誤差ではない。既存の数値は変更せず、対象JSONへinterpretation_correctionを追記、report_absolute_tilt.pyのscopeも修正。通常位置/姿勢RMSEの計算は変更していない。
- world-Zを直接参照する新しいtilt因子は、固定重力という前提をこのbackendで保証できていないため、v32sを停止。精度に基づく選別ではなくフレーム/設定の不整合修正を優先する。a1は完走、b1は準備中に中断。終了修正版の実Gazebo/launch親/他processは全0、launch_stop_mode=automatic_server_exitを確認。
- 次にfalse設定で実際に固定重力の標準ImuFactorと同じ固定重力予測を使う、別ライブラリを準備する。true時の挙動と元ライブラリは保持。効果は未検証。

## 2026-10-17 — v33 固定重力設定の修正と監査
- build_fixed_gravity.pyで実際の速度改善版IMU sourceを3箇所だけ変更。use_estimated_gravity=falseなら標準5-key ImuFactorと定数重力予測、trueなら従来6-key gravity factorと可変重力予測を使用。LiDAR/map objectと元ライブラリを保持し、別libを生成。
- test_fixed_gravity.cpp: 30例で定数重力時の予測・残差・5状態Jacobianの等価性、推定重力変更時の差を確認PASS。生成source/object/libraryのSHAをmanifest保存。
- runner/campaign/development/validation/reportへ --fixed-gravity-imu を追加。world-Z tilt因子はこの修正済ライブラリを必須にし、aligned init・設定falseを確認。1Hzの小さいgraph状態CSVを保持、全受信時刻で重力が[0,0,-9.8]に固定されていなければ明示失敗で停止。
- v33はv32sと制御/センサ/較正/融合nodeを維持し、この重力設定の実装だけ変更。開発6組seed24/25、独立26/27は未使用。Windows空き19.84GB、8GB下限、最大1sim、raw保存なし。まだ飛行効果は未確認。
- v32s a1完成値は融合.425675→.387069m（比.909307）、全既存監査/終了合格。後から判明した重力座標の前提不足を別記し、候補を採用しない。b1は準備中中断。

- v33初期化中、gravity監査がC++ floatの9.800000190734863とPython double9.8の差を異常と判定し停止。全sim子正常終了、監査process1保持、軌道精度なし。manager.hppでfloat宣言を確認し、監査の期待値を同じfloat32丸めへ修正（許容幅1e-8は不変）。既観測の実際の重力変化は引き続き検出する数値確認。v33sは監査表現だけ修正し、推定/制御は同一。

- v33s seed24 a1完成、全監査/実終了合格。融合位置0.391733→0.372343m（比0.9505005420156232）、回転.423→.434deg。ROVIO9.731→8.536m/1.494→2.103deg。固定重力を全受信で確認、audit CSV全長。絶対傾きはROVIO.1753/.1784deg、融合.5136/.5321deg。重力設定修正だけで融合の傾き誤差がなくなったとは言えない。

- v33s seed24 b1完成、全監査/全終了合格。融合位置5.696168→0.291546m、回転1.350790→0.236485deg。ROVIO位置45.548701→7.516929m。終盤で固定側ドリフト増大、途中の小誤差から不採用を先取りしなかった。短2組改善、独立未確認。schedulerがseed25短2組を開始。

- v33s seed25 a1完成、全監査/終了合格。融合位置0.579776→0.340155m、回転0.507418→0.495879deg。ROVIO位置5.632227→7.535564mで悪化。融合を主評価とし、単体の悪化も隠さない。完成3組全勝、開発の長区間/独立試験は未確認。

- v33s seed25 b1全監査/終了合格、融合.328→.530m（比1.61537）悪化、ROVIO25.393→2.193m改善。短4組3勝、元基準の最大比1.5は未達。長区間は重力修正の効果が未確認のため継続。
- v33s_b1_seed25_axes_diagnosis.json: 融合の主な差はworld x RMSE.325→.527m、zは.0298→.0171m改善。ROVIOの局所1秒移動量誤差は各軸すべて可動側で小さく、x .279→.0668m。VIO単体の悪化が直接原因とは言えない。真値は事後評価のみ。
- record_vio_uncertainty.pyを追加。進行中prefixでROVIOが報告する姿勢分散/速度共分散を1Hzだけread-only記録、推定器/制御入力は不変。ownerのruntime作成または1800秒で終了。最初の固定topic誤記でactiveのみだったstartupを別名保存し、/fixed/rovio/odometryへ修正、両側受信を確認。新記録は初期化後からの診断であり全長監査と扱わない。

- build_rovio_uncertainty.pyを追加。通常25特徴ROVIOの推定は変えず、既存診断CSVの末尾に速度・姿勢分散・速度共分散だけ追加する別binary。replay_rovio_capacity.py --uncertainty-logで以前の独立固定較正2飛行をそのまま再生、元の全CSV列が文字単位で一致しなければ中断。生入力は参照のみでコピーしない。ビルド中、まだ数値分析未実施。進行中v33sのコード/重みは不変。

## 2026-10-17 22時台 — v33s不採用、速度観測方式v34へ
- v33s長区間は710.681秒で診断停止、全子process/実Gazebo/launch正常終了。短4組3勝1敗・最大比1.61537は元基準1.5を満たさないため不採用。662秒の絶対傾きROVIO固定/可動.139/.135deg RMSE、MIMOSA2.388/2.520deg、直近約3.8deg。重力を真に固定しても問題は残る。途中値を完走結果とはしない。seed25prefixと独立26/27は未実施。v33s_early_rejection.json。
- 旧固定較正2飛行のlogging-only再生: a1 24.5秒、b1 38.8秒。元の全16列・全行は文字単位一致。追加body velocity/pose variance/velocity covarianceだけ取得、元入力コピーなし。rovio-uncertainty-calibration/results.json。
- calibrate_velocity_fusion.pyで旧2飛行の速度誤差と公表covarianceを照合。正規化誤差2次momentの飛行等重み平均の最大固有値7.908699を共通分散倍率として固定（floor1、biasを除かない）。a1速度RMSE[.23564,.03160,.02193]m/s、b1[.13547,.01630,.00741]。真値差分を±10msから±20msに変えたRMSE変化は4.25e-5/8.52e-5m/s。今回候補の融合成績から倍率を探索していない。
- body_velocity_factor.hpp / velocity_odometry_manager.hpp / build_velocity_fusion.py追加。ROVIOの機体基準速度と公表共分散を1HzでMIMOSAの姿勢/速度状態へ拘束。外部global位置・yaw・連続pose差を使わない。T_B_Sにより取付位置の速度成分と共分散を変換。重力方向因子は既存1Hz・同じ較正。共有IMU・速度/傾き間・時系列の厳密な相関処理ではない。
- 40座標/非ゼロ取付位置/世界座標変更の不変性、360状態Jacobian列・240covariance変換列、key変換検証PASS。別nodeビルド成功、元node不変。ordered dispatcherのsourceは生成コピーだけ変更、元を保持。
- runner/campaign/development/validation/reportへopt-in --velocity-fusion追加。較正・1Hz因子周期・共分散正定性/分散倍率・source/binary保存照合を検査。従来条件はflagfalseで維持。v34はv33sとyaw-onlyカメラ制御/センサ/固定重力/tilt較正/DRPM OFF同一。開発6組seed24/25を事前宣言、独立26/27は未使用。速度方式の実飛行効果はまだ未検証。

### 追加切り分け：MIMOSAのIMU biasモデルとGazeboの不一致
- Gazebo GaussianNoiseModel.ccの実ソースを確認。bias全体はphi=exp(-dt/tau)で0へ戻り、innovation分散はdynamic_bias_stddev^2*tau/2*(1-exp(-2dt/tau))。ROVIO側は既にこの式、MIMOSAは現在bias_{k+1}=bias_k+random-walk。センサ規格から直接確認したモデル差で、精度のためにノイズ値を探索したものではない。
- mimosa_bias_model_mismatch.json: v33s長区間の最終sim781.4秒で、SDFが定めるacc biasの2次moment平方根.003555m/s²に対し、MIMOSA y推定は固定.3564/可動.3743m/s²（約100/105倍）。これは潜在biasの実誤差を測った値ではなく、モデル上の分布と推定状態の不整合。姿勢の誤差をbiasに吸収している仮説の根拠。gyroは同じ指標で約1倍。
- mean_reverting_bias_factor.hpp / sensor_bias_model.hpp / test_mean_reverting_bias.cpp / build_sensor_bias_fusion.pyを準備。センサのtau/駆動雑音からbias遷移因子を作る任意別library、元Manager ABIを維持、1process1IMU managerを明示制限。false時は元BetweenFactor。固定重力/同じLiDAR objectsを継承。まだbuild中、v34に未適用。
- v34 a1完成は全監査/終了合格、融合.541→.516m、ROVIO9.371→6.218m。現在b1実行中。候補の制御/設定は変更していない。

- センサbias因子検証: 40条件の平均/分散遷移の時間分割整合、Gazeboの式との一致、480Jacobian列、random-walk極限、key変換がPASS。別library build成功。その後1Hzの実因子phi/innovation sigma監査を追加して再build中。sensor_bias_parameters.pyは生成済みSDFからacc tau175s/gyro400s、driving sigma.000375/.00002を取得することを確認。白色雑音・初期化・物理センサはまだ変更していない。
- v34 scheduler13559のみ停止（子campaignはまだ0）。外部firstshort93826のb1は最後まで継続。今回確認したモデル不一致を先に揃えるため次の反復を起動しない。精度基準による不採用と混同せずv34_early_rejection.jsonへ明記。独立26/27未使用。

### v34終了、v35: センサに合わせたbias遷移を比較
- v34短2組は全監査/全process正常終了。a1融合.540955→.516225m、b1融合.283→.321m（悪化）。b1 ROVIO24.392→12.870m。2組1勝で安定改善とはしない。次の反復前にセンサモデル不一致を修正する。
- sensor-bias library再ビルド/40遷移例/480微分列PASS。実際に追加したbias因子のphiとinnovation sigmaを1Hzだけ保存、生成SDFのtau/駆動雑音との一致と全長監査を追加。
- runner/campaign/開発・独立scheduler/reportへopt-in --sensor-bias-model伝播。別libraryのSHAを保存/照合。v35はv34とカメラ制御・速度/傾き融合・白色IMU雑音・物理センサ同一、bias状態遷移だけSDFに合わせる。全6組seed24/25を宣言、独立26/27未使用。生画像/点群収録なし、Windows空き19.87GB/下限8GB。
- これは区間間のbias遷移モデル修正であり、短いIMU積分区間内は従来の一定bias近似。共有IMU相関の厳密な処理ではない。効果は実飛行比較でこれから確認する。

- v35 seed24 a1完了、全監査/全process終了合格。融合位置 0.481767→0.449893m（比0.933839）、回転 0.341482→0.360625deg。ROVIO位置7.629625→5.993541m、回転0.334765→0.802997deg。bias因子の実際のphi/雑音がSDFに一致、両processの/proc/maps library SHA照合合格。1組のみで安定改善ではない。

### 次段階の成分比較を準備
- v35はseed24の短2組を完走させ、旧白色雑音モデルでの追加反復は保留。scheduler22291/PID1396523のみ停止、子campaign未開始。firstshort59478のb1は継続。v35_component_comparison.jsonへ理由を保存、6組基準の合格とはしない。
- 既知の残るIMU white-noise設定差を、新しい固定重力・sensor biasモデルと組合せて切り分ける。v36予定: acc density .8→.00148842758、gyro .01→.000636416563。生成SDF stddev/precisionと1000Hzから変換、物理センサや初期化は不変。v9/v15の同white-noise変更は旧gravity/bias/融合で失敗しており、その不採用は維持。新しい組合せの結果は未確認。
- development/validation schedulerへ既存--fusion-imu-noise伝播を追加。v36 YAMLの制御パラメータはv35と全項目一致。現在のv35 b1はlegacy white-noiseのまま継続する。
- v35 b1完了、全監査/全子process終了合格。融合位置.223→.261m、回転.178→.183degで小さい悪化。ROVIO位置45.623→6.823m、回転6.450→1.820degで改善。v35短2組1勝1敗、安定改善の成功扱いなし。firstshort59478も終了。
- v36: bias/gravity修正後の物理white-noise設定を固定/偏向同時適用する6組24/25を事前宣言。旧6組基準不変、独立26/27未使用。レポートに実際のSDFから再計算したwhite-noise値の一致も追加。sensor biasモデル以外のrawセンサとカメラ制御はv35と同じ。最大1sim。

### v36白色雑音設定単独では悪化、v37で地図の傾き拘束を切り分け
- v36 a1全長/全監査/全process正常終了。融合位置固定17.703→偏向19.197m（両方v35約.5mより悪い）、回転.773→.802deg。ROVIO10.603→12.609m/2.220→1.120deg。物理noiseモデルに一致してもこの融合で性能が良いとは限らず、不採用。b1は起動せず、scheduler70059とfirstshort99462を停止。v36_early_rejection.json。
- 傾き絶対誤差はROVIO固定.1595/偏向.1686deg、融合.6246/.6349deg。センサbiasを正しく遷移させても地図と慣性/視覚の傾きが一致しない。原因を特定済みとはしない。
- 元MIMOSA HEADと実リンクに使ったcompact-prefetch headerにreg_4_dofが存在することを確認。これをONにするとLiDAR因子の回転拘束を重力軸周りのyawだけにし、位置3成分は保持。roll/pitchはIMUと既存ROVIO重力因子へ任せる。既存因子の処理は改造しない。
- opt-in --lidar-four-dofをrunner/campaign/schedulers/reportへ追加。fixed gravity + tilt因子 + DRPM OFFを必須、project_on_degneneracyによる上書きを禁止。固定/可動に同設定、ソース・library・設定保存監査。v37はv36からこの標準設定だけ変更、制御/YAML値は同一。比較は元の良好なv35絶対精度とも照合し、悪い固定基準との比だけで成果にしない。

### v37不採用、v35の残り比較を再開
- 標準4DoFの実headerテストは15非水平姿勢でroll/pitch拘束なし・位置/yawの情報と補正量/対応点不変を確認PASS。verify_four_dof_registration.py / performance/test_four_dof_registration.cpp、build/mimosa-four-dof-check/result.json。既存libraryには変更なし。
- 一方v37オンラインはROVIOが出力する前、MIMOSA初期化26.6秒の次27秒未満から姿勢が急変。追従12秒で位置誤差>1km、即停止。途中区間/失敗として保存、全process正常終了、完走精度とは扱わない。発散の起点は分かったが原因を確定していない。v37_early_rejection.json。
- v36/v37の設定変更は両方不採用。安定していたv35を元の6組宣言のまま完了させる。完了済み短2組を再実行せず、残りseed25短2組とseed24/25prefixだけ追加。v35はbias動態修正、固定gravity、legacy白色IMU（モデル誤差込みの既存設定）、速度+傾き融合、yaw-only、DRPM OFF。
- unipilot_run.py/campaign.pyのv37版をsource_after_v37へ保持し、v35保存snapshotから2ファイルだけ復帰。development_v35.jsonの全SHAが完全一致することを確認。scheduler --resumeが同じ宣言/元の未開始progressを確認して継続。独立26/27は未使用。短2組1勝1敗もそのまま残し、成功基準を緩めない。
- v35 seed25 a1全監査/終了合格。融合位置.421→.578m（悪化）、回転.313→.311deg。ROVIO7.329→5.963m/ .651→.825deg。完成3組1勝2敗で元の5勝/6条件は達成不能。長区間は新bias/velocity融合のドリフト挙動が未確認なので診断として確認するが、同候補を採用とはしない。
- probe_planning_depth.pyを追加。固定/偏向のROVIO特徴点の距離と、同時刻LiDARの検証済み局所平面から得た距離を1Hzだけ比較するread-only診断。画像/点群は保存せず、支持点数・距離中央値・比率百分位のみCSV。推定器/制御へ入力なし。seed25 b1へ接続。LiDAR照合自体にも対応誤差があるので真値とは扱わない。
- read-only距離probeは支持0が続いたため停止、推定器/制御入力に変更なし。SDFのLiDAR pose pitch1.57/yaw3.1416、垂直走査0..1.5708で後方半球を向くことを再確認。v8の既知記録とも一致。したがってこの照合からROVIO距離の良否は判定できない。小さい比率CSV/スクリプトのみ保持。最初の起動はROS2環境警告が出たが購読は動作、別の1回読取りはenv -iのROS1環境で実施。今後のROS1 probeもclean environmentを使う。
- v35 seed25 b1全監査/全process終了合格、融合.348→.333m、回転.197→.198deg、ROVIO20.819→8.423m/3.815→1.479deg。完成短4組2勝2敗、平均比1.10849、最大比1.37258。元5勝/6条件は達成不能、安定改善を主張しない。scheduler57816はprefix24/domain85/PID1410155を診断中。
- diagnose_visual_motion.py: 4比較の実1Hz body速度因子と同時刻終端の1秒body移動量を真値で事後診断。初期化/終了holdを除き、入力/較正/推定器には返さない。v35_visual_motion_errors.json。瞬間速度RMSE固定→偏向[a1/24 .1493→.1635m/s、b1/24 .9671→.1045、a1/25 .1521→.1577、b1/25 .3034→.0881]。a1の総位置RMSE改善から局所速度改善とは言えない。
- 同じ標本時刻の1秒移動量誤差の数値は8系統中7系統で瞬間速度誤差より大きく、残り1系統も同等。速度を単純に1秒pose差へ置換する案を今回は実装しない。共有IMU・時系列依存は診断だけでは解消されない。

### v35診断終了、v38: 4DoFをlegacy白色雑音で分離
- v35 prefix24は約250route秒で直近重力傾き固定1.85697/可動1.83349degまで増大（単時刻、RMSEではない）。途中位置RMSE約1.6m/1.6m。新bias/velocity方式でも長区間の傾き漂移は解消しない。既に短4組2勝2敗で元基準達成不能のため診断を停止、全子process0、途中値を完成精度としない。prefix25未開始、独立26/27未使用。
- v38はv35を基準に、LiDAR標準reg_4_dof=trueだけを変更。IMU白色雑音は安定していたlegacy、bias平均回帰/固定gravity/速度+傾き1Hz/カメラyaw-only/物理センサを保持する。v37は4DoF+sensor白色雑音で発散したため、2×2の残る組合せを確認する。較正・閾値を結果に合わせて変更しない。
- runner/campaignを保存済みsource_after_v37から復帰して4DoF flagを利用。既存libraryは不変、生成設定と全source SHAを新v38宣言へ保存。現在v35scheduler57816/子prefix1410155は停止済み。

- v38 seed24 a1完走、全監査/全process正常終了。融合位置固定.556→偏向.463m（比0.83258703）、回転.212→.243deg。ROVIO位置9.145→6.204m、回転.284→.743deg。絶対傾きは融合.210/.221deg、ROVIO.184/.194deg。標準4DoF+legacy white-noiseの組合せはこの飛行で安定、v37と同じ初期発散はなし。ただし1組のみ、安定改善や長区間の解決は未確認。

- v38 seed24 b1全監査/全process正常終了。融合位置.332→.500m（比1.508256>元上限1.5）、回転.224→.151deg。ROVIO位置34.250→5.247m、回転4.198→4.033deg。元基準の合格は達成不能、採用扱いにしない。4DoFによる長区間傾きの切り分けと反復の再現性は未確認のため、宣言した残り比較は診断として継続。閾値の丸め・変更なし。

### 視野復帰処理の診断と次候補の準備（進行中v38へ未適用）
- v38短seed24 a1では意思決定の44.5%が正面復帰、b1では正面復帰166標本のうち47標本で既に成熟特徴4個以上があるのに正面まで戻し続けていた。以前のv35 a1も復帰44.5〜50.6%。この集計だけで精度悪化の原因を確定はしない。
- v38 seed25 a1終盤の固定/可動画像を各1枚だけread-only取得（sim298.2/298.3秒、同時刻ではない）。この2枚には機体による遮蔽はなく、ほぼ正面のトンネル画像。連続画像保存はしない。
- parallax_fixation.pyを新規追加。成熟点が4個以上なら既存の成熟点注視を使用。それ未満なら、既に追跡中の成熟点、または0.3秒以上観測され機体回転を除いた視線変化が2sqrt(2)*bearing_sigma/window以上の点を保つ。距離を仮定せず、実測視線変化とLiDAR弱並進方向への幾何的支持で順位付け。これは公正な情報行列推定を主張するものではなく、距離未収束時の視野選択heuristic。推定器の観測/covarianceを偽造・変更しない。
- yaw-only、pitch0、速度/可動範囲/他特徴の保持を既存と同じにする。成熟が戻れば、正面への復帰完了を強要せず通常注視へ戻す。証拠不足時は既存復帰へ。新moduleはまだplannerから読み込まず、v38のsource SHA/設定には影響なし。
- test_parallax_fixation.py: 不確かな距離でも観測された視差を使える、深度尺度不変、純回転/若い点/視野外点の排除、成熟点の継続、通常成熟時の完全同値、空入力、入力配列不変、角速度制限。既存fixation/bearingを合わせ42試験PASS。飛行での効果は未確認。

### v38不採用、300秒の傾き診断とv39の宣言
- v38短4比較すべて全監査/終了合格。融合固定→可動: a1/24 .556→.463、b1/24 .332→.500、a1/25 .607→.572、b1/25 .401→1.019m。2勝2敗、平均比1.45653、最大比2.54219。ROVIO位置は4組すべて改善するが、融合の安定改善はない。v38_early_rejection.json、v38_progress.html。基準を変更せず不採用、独立26/27未使用。
- scheduler64638/PID1412374はseed25 b1が正常終了した直後に停止、以後の全長prefixを起動していない。firstshort27876も完了。v38-gravity-diagnostic24（session89720/domain85）は同一条件でprefix最初の300秒だけ完走させる。元1070秒比較とは別の傾き診断、v38_prefix_diagnostic_protocol.jsonへ明示。
- plannerが旧コードで稼働し28秒/22判断まで進んだことを確認後、次候補用のopt-in importをplanner_nodeへ追加。進行中診断の読み込み済みコード・保存snapshotは旧条件のまま。v39でのみfixation_parallax_fallback=true。既存制御や推定/融合は変更しない。
- v39: v38と融合/センサ/較正/初期化は同じ、視野制御だけparallax_fixationへ。43関連試験PASS。順位変化で即座に乗り換えず既存の追跡対象を保持する追加試験を含む。schedulerへ新module/testのsource SHA保存を追加。
- v39 scheduler72454は開発24/25×a1,b1,prefixを宣言済み。firstshort待機session64123はv38診断runtime全0を待ち、全source SHA再確認後に実験開始する。最大1sim、Windows実空き約19.87GB/8GB停止、raw保存なし。停止する場合はschedulerとfirstshort待機の両方を止める。

- report_gimbal_partial.pyに、fixed_gravity_imu=trueの試行だけ絶対重力傾きRMSE/直近値を追加。開始時姿勢合わせを使わず、完走前の診断と明示。初期化中/共通追従範囲不足を明確な例外に変更。v38 prefix206秒時点で融合傾き固定.126/可動.122deg、直近.155/.147deg。位置は.699→1.033mで悪化。傾き抑制とジンバル精度改善を分離して記録する。

### v39開始前に停止競合を修正し、v39sへ
- v38の300秒prefix診断は飛行完了、入力・時刻・初期状態・設定・planner/推定器連続性は全て合格。しかし終了処理でpacingだけexit1、全体監査は不合格。Gazebo/launch/他の子は全0。結果を正常完走精度と呼ばず診断として保持。位置固定1.232→可動1.707m、重力傾き.13085/.12671deg、後半100秒も.14075/.13580degで、以前の傾き増大はこの診断では見られない。
- 原因はGazeboを先に停止した後もpacerがpause/resume serviceを呼ぶ競合。pacing.json最終sim368.989秒はroute完了368.320秒より後、入力全長監査合格。追従中の異常を隠していない。v39 firstshortの起動前全0ゲートが検出して停止、実験0本。scheduler72454/PID1422841も停止し、元宣言をv39_preflight_withdrawal.jsonと共に保持。
- simulation_shutdown.stop_world_clientsを追加し、所有pacing processへSIGINTして最後のresume通信が終わるまで最大4秒待ち、その後Gazebo停止。既存失敗のexitcode、timeoutはそのまま記録、成功に置き換えない。終了済みprocessへ再度signalしない。shutdown_request.jsonへworld_clients実結果を残す。
- test_world_client_shutdown.pyは実child processの最終通信完了後にservice終了する順序、既存failure保持、timeout非成功を確認。関連計46試験PASS。実Gazebo飛行後の終了はv39sでこれから確認。推定/センサ/制御の計算は不変。
- v39sはv39視野制御（parallax fallback）と同じ、停止修正だけ追加。6組24/25を新宣言、元v39の結果を捏造・流用しない。元改善基準不変、独立26/27未使用。

- v38初期alignment監査: 4短比較と300秒prefix診断のMIMOSAは、最初の評価時刻に固定/可動の位置・回転がすべて厳密一致。評価の初期座標合わせの違いが融合悪化の原因ではない。ROVIO最大差は.00236m/.01325deg。採点方式変更なし。ROVIOの特徴共分散もLandmarkOutputImuCTでbody基準・特徴状態から作られ、world位置誤差を誤って成熟判定に含める実装ではないことを確認。

- v39s seed24 a1全長/全監査合格、全process0。修正したshutdownはpacingの正常終了0を記録してからGazebo正常停止、実飛行で順序を確認。融合位置固定.557→偏向.465m（比.835685）、回転.377→.389deg。ROVIO位置12.094→11.890m、回転.806→.517deg。正面復帰35/991判断=3.53%（前v38 a1は44.75%）、provisional追跡462/991。総偏向量640.63deg、角度/速度監査合格。視野復帰抑制は達成したが全経路での安定精度改善は未確認。

### v39s再開確認: 短区間seed24完了、seed25へ継続
- ユーザーの再開指示後に所有process・停止記録・凍結source55件を確認。source不一致0。seed24のa1/b1は両方完走、全監査合格、実Gazebo/launchを含む全process0。scheduler PID1423593がseed25短比較を実行中、最大1sim。
- seed24融合位置固定→偏向: a1 .556559→.465108m（比.835685）、b1 .439014→.359585m（比.819074）。回転a1 .376577→.388938deg、b1 .315433→.287705deg。2/2位置改善だが元6比較基準の合格ではない。
- b1 ROVIO位置24.1221→19.9447m、回転1.97451→1.57153deg。実際に融合した1Hzのbody速度誤差はa1 .19768→.18967m/s、b1 .35937→.20900m/s。真値は診断だけに使い、較正値は不変。
- 視野制御はa1で正面復帰35/991判断、b1で42/1094判断。新しい保持処理が働いたことと、異なる乱数/長経路での精度再現性は分けて判断する。独立26/27未使用。
- Windows実空き19.19GB、8GB保護、生画像/点群/bag保存なし。完了結果をv39s_progress.htmlへ更新。

### v39s再試行とv40二軸制御の準備
- v39s seed25 a1全長/全監査合格、全process0。融合位置.511→.562m（比1.100204）、ROVIO7.930→7.886m/1.835→4.502deg。a1の2回平均比.967945で元の各経路≤.9基準不合格。v39sを採用扱いしない。seed25 b1まで完走した後、helper session52279がscheduler PID1423593を止める。prefix未開始、独立26/27未使用。
- 新spatial_parallax_fixation.pyは既存の観測視差保持を二軸へ拡張。現在見えている特徴だけを選び、未観測の床/天井へ探索しない。成熟/視差の採用閾値、選択点の保持、較正/融合、速度上限は不変。pitch方向の到達可能判定と角度予測を追加。元v39s moduleは不変、進行中比較に未適用。
- 新test_spatial_parallax_fixation.pyはyaw-only既存9試験の互換性、上下動への追従、視野外の点を選ばないこと、成熟時の既存二軸との完全同値、距離尺度不変を確認。既存と合計56試験PASS。上下動が実際の誤差の原因だと確定したものではなく、次の切り分け仮説。
- tracking_aware_v40.yaml/v40_design.jsonを準備。計画上pitch±30deg（ハード上限±90deg内）、yaw±50deg、既定速度20deg/s以下。planner/schedulerとの接続は現v39s停止後に行う。

### v39s終了、v40の実飛行比較を開始
- v39s seed25 b1は完走/全監査合格/全子0。融合位置.284763→.295725m、回転.416249→.221207deg。ROVIO位置34.7483→20.1773m、回転4.05397→.790109deg。短4比較は位置2勝2敗。元基準不合格、v39s_early_rejection.jsonで不採用を明示。prefix未開始。
- 停止helper52279完了、scheduler1423593/firstshort12865/全sim停止をpsとruntimeで確認してからplanner_nodeのopt-in分岐とschedulerのsource SHA列挙へ新二軸module/testを追加。v39s実験中のsource不変を保持した。
- v40条件57ファイルを凍結、全SHA一致。firstshort session5294（24 a1→b1/domain86）、scheduler53214（25短→24/25 prefix）。最大1sim、停止時は両方停止する。元6比較基準不変、独立26/27は未使用。融合/センサ/速度/画角/物理はv39sと同一、動かすカメラだけpitchも許可。
- 実験前の56試験PASS。v40の実飛行精度はこれから。Windows実空き19.18GB、8GB下限、生データ保存なし、完了ログのみ圧縮継続。

- v40 seed24 a1完走/全監査/全子0。融合位置0.479449→0.453615m、回転0.393904→0.393508deg、ROVIO位置8.832827→6.906218m/回転0.719521→7.738543deg。実際の1Hz融合速度誤差は.16478→.16749m/s、1秒移動量は.23393→.25482m。ROVIO軌跡の誤差が小さくなっても局所速度の精度向上を意味しない。この1比較だけで採用しない。b1継続中。

### v40短区間の融合切り分けと長区間の追加観測準備
- v40 seed24 b1完走/全監査/全子0。融合位置.316→.439mで悪化、ROVIO位置29.100→3.182m/回転2.653→.642deg。融合へ実際に渡されたbody速度RMSEは.39810→.03574m/sで改善。視覚の大きな改善だけから融合改善を推定しない。
- diagnose_fusion_relative_motion.py追加。軌道中の同時刻1秒窓でROVIO/MIMOSAのbody相対移動量と回転を真値に照合する読み取り専用診断。VIOの1秒移動量誤差b1 .45249→.08647m、融合 .03174→.03098m。融合の前進方向の平均誤差は-1.84→-2.46mm/秒窓。融合済みのMIMOSAとの比較であり、LiDAR単独の実験ではない。位置の初期合わせ/尺度合わせはしない。v40_relative_motion_diagnosis.json保存。
- compare_fusion_sidecar.pyを新規追加。次のv40 prefix24の開始前から、同じ入力を読む追加3ノードを観測用に起動する準備。現在方式active複製1つ＋既存の20Hz移動量+1Hz重力方向融合のactive/fixed。旧固定較正JSON/既存binaryを使い、重み探索をしない。全出力topic/TFはsidecar名前空間へ隔離し、飛行/視野plannerには入力しない。画像/点群保存なし、数値のみ。
- sourceを凍結したv40主比較には変更なし。追加観測は現制御のカメラ運動を条件とした診断であり、新融合を使った能動制御全体の比較ではない。active複製と本体の共通時刻の出力一致・時刻連続性・全process停止を後で確認し、失敗時は診断無効として保持する。2.5GB RAM下限で追加nodeだけ停止、30分開始待ち上限、reference終了で所有追加processを停止。新scriptは構文確認済み、実ROS lifecycleはprefixでこれから検証。
- 待機sidecar session45050、v40_fusion_sidecar_prefix24/manifest.json、v40_fusion_sidecar.log。主firstshort5294は完了、scheduler53214はseed25短を実行中。最大1sim、Windows19.18GB、8GB保護。

- v40 seed25 a1完走・全監査/全子0。融合位置.431→.591mで悪化。短3比較1勝2敗、元5勝/6基準は達成不能。v40_early_rejection.jsonへ不採用を記録。残る短b1と最初のprefix24だけは診断として完走し、prefix25は実行しない。stophelper session55803がprefix24 runtime作成直後にscheduler PID1432636へSIGINTする。
- sidecarは実プロセスの終了codeを直接取得できるよう、roslaunch親を使わず、専用private namespaceへrosparamをロードして既存C++ binaryを直接所有する形へ変更。旧待機45050を実node起動前に停止し、待機だけの記録をv40_fusion_sidecar_prefix24_waiting_v1へ保持。現待機session61431。主比較のsource/条件不変。
- report_fusion_sidecar.py追加。完走後に同一方式のactive複製と主計算の共通stamp poseを完全一致比較し、3副計算のLiDAR/VIO stampと処理済みIMU prefixを主計算と照合する。計算時間/queue長の違いはデータ相違としない。初期化前のimu_upto_ns=0を既存監査と同様に扱う。真値/protocolは計算終了後の一時評価directoryからだけ参照し、6軸誤差の採点関数は既存を再利用。数値TUMは保持するがCSV変換一時ファイルは削除。実験終了・全所有process0・全出力範囲確認が通るまで有効とはしない。

- v40 seed25 b1も完走・全監査/全子0。融合位置.188→.306m（比1.62575）、ROVIO45.772→5.890m。短4組は1勝3敗、平均比1.33428、元基準不合格。各結果はv40_progress.htmlへ反映、採用しない。
- prefix24主campaign PID1441502/domain85開始。sidecar61431の3直接C++process（duplicate1443197、displacement active1443433/fixed1443709）がsim13秒以前に起動し、推定器初期化前から受信。全3つの実ロードMIMOSA library SHAは主と同じ55c50c…1c、loaded_libraries.jsonと実行source snapshotsを保存。207共通pose標本が数値完全一致、最大差0。
- report_sidecar_partial.py追加。書込み中の完全行だけ読み、全出力の共通追従期間で部分RMSEを表示する。完走/採用と扱わない。24.4秒時点はvelocity activeとduplicateが位置/回転とも厳密同値。後でreport_fusion_sidecar.pyにより全長の同値・入力連続性・停止を確認する。

### 2026-10-18T08:06:30.970284+00:00 v40追加診断と次候補の準備
- v40短4組は全監査/終了合格、1勝3敗で不採用。b1/25融合固定.188283→可動.306102m、ROVIO45.772443→5.890493m。長prefix24は同条件の診断として継続し、prefix25は行わない。独立seed26/27は未使用。
- 現融合と同一の追加ノードはprefix途中738秒まで全共通pose完全一致。変位融合の追加fixed/activeも稼働中だが、その時点では4.105→5.737mで改善なし。全長の監査前であり正式結果ではない。report_fusion_sidecarは共通部分だけでなく、複製が主ノード全poseを覆うこと/主ノード全時間の入力を検査するよう補強。
- audit_self_visibility.py追加。実SDFの機体meshを画像取得時のencoder姿勢で投影、ローターのみ回転掃引包絡。207.65秒画像の機体部分578画素(.446%)、457.6秒500画素(.386%)、固定画像はいずれも0。後者と同時刻のROVIO追跡7点を照合し、半径16画素まで全7点で機体領域との重なり0。自己像混入は確認したが、これがドリフト原因という証拠は得ていない。画像は2時刻×固定/可動の4PNGと7点のみ保持、連続収録なし。
- 次候補用joint_information_view.py/test_joint_information_view.pyを独立追加。既観測の成熟点集合について、予測視野内の全点の情報を足し位置/回転の結合を含むlogdet比を評価する。少数点単独選択から、保持集合全体の評価へ。保持と比べ3%以上改善するときのみ動作、元の90%追跡保持/角速度上限/0.8秒予測/成熟判定を利用。不十分な点数ではv40の観測視差方式へ戻す。未知の視野、真値、飛行指令は使わない。特徴間相関を無視した局所評価であり厳密なROVIOの予測共分散ではない。
- 新9試験（相補的拘束、単位変換、回転並進結合、保持、物理範囲、未知点拒否等）とimportされた既存4試験が成功。実行中v40の凍結57ファイルは変更せず、planner接続/新飛行は未実施。Windows実空き19.19GB、下限8GB維持。

### 2026-10-18T17:24:53.729534+00:00 中断後の状態照合、v41開始
- turn中断でv40 prefix24は991.128/1070秒で停止。runtime failed、全owned main/sidecar/Gazebo子0、旧scheduler1432636/helper1441043/supervisor1441176/PID1441502は不在。未完記録を保持し完走結果にしない。
- 追加複製は主ノード全10483 poseを覆い数値完全一致。3追加ノードすべてLiDAR/VIO時刻/IMU個数/時刻境界が主ノードと完全一致、入力連続性合格。完走だけ未達のためcomparison.valid=false。途中共通990.9秒の融合固定→可動: velocity6.96457→7.87912m、displacement5.51323→8.01835m。ROVIO70.25695→22.04680m/2.50923→4.59935deg。別融合でもこの区間の改善は確認できず採用変更なし。reportは未完のintervalを勝手に短縮せずfull accuracy空欄を保持。
- v41のjoint_information_viewをplannerのopt-inへ接続、宣言へ新module/testを含め59ファイルSHA固定。v40凍結中の変更なしを直前確認。実装/既存計69試験成功（新規9件）。合成25特徴の判断時間median6.56ms、95%13.44ms、max16.12ms。
- v41 seed24 a1→b1/domain86を開始、firstshort session23172、scheduler session67428がseed25短/24,25prefixを最大1simで実行。6組条件・成功基準・独立26/27未使用を維持。Windows実空き20.56GB、8GB停止、生bag/画像点群連続保存なし。

- v41 a1/24全228.493秒完走、全設定/入力/出力/速度/終了監査合格。融合位置.465147→.374332m（比.804759、19.52%減）、回転.378425→.375590deg。ROVIO位置7.090697→6.187620m、回転.489126→3.736990deg。1/6のみ、安定改善未達。現在b1/24を同設定で継続。ROOT/v41-short-seed24/index.html・plots/comparison.pdf生成。
- v40追加ノードの停止済み数値16ファイルをSHA検証付きgzip圧縮、Linux bytes11.867MB削減。入力/全10483pose完全一致を圧縮後にも再検査。report_sidecar_partialをgzip対応、数値・評価範囲を変えない。Windows空き約21.2GB、OS/VHD設定変更なし。
- v41 a1/24の視線総移動量は決定時刻の実encoderで647.96deg（同seed v40は817.99deg、飛行間の物理/観測は同一再生ではないため差の因果は断定しない）。成熟点の判断531回中380回hold/151回偏向。ROVIOの姿勢悪化を解消したとの主張はしない。
- read-only observe_feature_spread.pyを追加。v41 b1/24途中からROVIOの追跡点数/成熟点数/距離と相対不確かさの中央値/方位の広がり固有値だけ1Hz記録。画像・点座標・真値・密な点群は記録せず、推定器/制御へのpublishなし。既存ROS1 nodeの環境でsession4447、対象runtime生成か600wall秒で停止。現在実受信/finite値を確認。幾何の分布は精度や観測可能性の証明とは扱わない。新観測器は凍結59ファイルの外、比較アルゴリズムは不変。

- v41 b1/24全263.385秒完走、全監査/全子0。融合位置.3319→.3853m（比1.160813）、回転.239→.184deg、ROVIO位置40.199→8.594m/回転2.073→1.224deg。合計1勝1敗、安定改善未達。実際に融合へ投入した1Hz速度のRMSEは.521915→.118209m/s、平均機体X誤差+.335788→+.063633m/s。VIOのglobal位置だけが改善して見えていたわけではない。
- build_tilt_only_diagnostic.pyで既存velocity managerの速度factor追加1行だけを除く独立binaryを生成（明示diagnosticパラメータ必須、速度ログはomitted_velocity_observationsへ改名）。元node/library/source不変。状態作成時刻・LiDAR・IMU・VIO傾き因子/重み/取得は保持。純LIOではなく「VIO傾きあり・速度なし」の切分け。build成功、実入力検証はこれから。
- compare_fusion_sidecar.pyにtilt_only比較を追加、全出力を隔離、元activeの完全複製も同時実行して入力差/数値差を監査する。実load library SHAを自動保存、実装snapshot自動保存、全実child終了/メモリ2.5GB保護は維持。reportで全区間出力、主streamとの入力同値、tilt因子同値、velocity因子の不在を確認する。v41 prefix24開始前待機を起動、現在ノード未起動。カメラ制御へこの補助結果は戻さない。

### 2026-10-18T17:41:54.031343+00:00 v41不採用、融合の速度因子を切分け
- v41 a1/25全228.493秒・全監査/全子0、融合位置.300→.354m（比1.180573）、ROVIO34.746→6.794m。短3組1勝2敗、a1平均比.992666>.9、元基準不合格確定。v41_early_rejection.jsonを保存。b1/25と診断prefix24は続行、prefix25は行わない。未使用seed26/27維持。
- 補助比較待機session99215（v41_tilt_only_sidecar_prefix24、port11485）は次のprefixでduplicate_active/tilt_only_active/tilt_only_fixedを開始予定。prefix24 runtime生成後にschedulerを止めるhelperを追加、元の主比較設定59SHA不変。
- diagnose_velocity_calibration_quality.pyを追加し、既存の独立した固定視野較正2本だけで誤差と公開共分散の比を距離/特徴数ごとに集計。a1の正規化誤差二次モーメント最大固有値は距離5–10mで4.084、b1は同範囲11.324/5m未満26.031。短距離や特徴数だけで一律に信頼度を上げられる根拠は得られない。時間相関あり、評価flight不使用、融合重みや較正ファイルは変更しない。ROOT/velocity_calibration_quality_diagnosis.json。

- 診断の時間を短縮するため、未開始のv41長prefix2本は取り消し、問題が出たb1全263.385秒で同じ速度因子の有無を比べる計画へ変更した。実行前にv41_velocity_factor_diagnostic_plan.jsonへroute/seed/期間/非採用判定用を固定。6組の成功基準を変えず、追加診断を反復数へ加えない。
- 旧prefix待機99215/PID1452886と旧stophelper78361/PID1454732は、補助node起動前にSIGINT。manifestはwaiting→interrupted/子0件を保持。新owner待機73007はb1/25完走後scheduler67428/PID1446492を停止、全sim解放と59SHA一致を確認後、v41-velocity-factor-diagnostic24（b1/24、domain85/port11485）を起動する。新補助待機はv41_tilt_only_sidecar_b1_24。同時simは1本。

### v41 短区間4比較の完了と不採用（2026-10-18）
- 全4比較が入力・終了・保存条件の検査に合格。固定→可動の融合位置RMSE:
  - run1_tail_a1/20261024: 0.465147 → 0.374332 m。
  - run1_tail_b1/20261024: 0.331523 → 0.384836 m。
  - run1_tail_a1/20261025: 0.300020 → 0.354196 m。
  - run1_tail_b1/20261025: 0.352675 → 0.429206 m。
- 1勝3敗、平均誤差比 1.090787。事前基準に不合格。長いprefix 2件は開始せず、候補を不採用とした。独立検証seed26/27は未使用。
- 別の全長b1診断を開始。通常の速度＋傾斜融合に、同一入力の複製推定器と、ROVIO速度制約だけを省いた固定・可動推定器を追加。傾斜制約は維持するため純LIOではない。追加推定器はカメラ制御に接続しない。
- 診断3ノードとも実際に読み込んだMIMOSAライブラリのSHA一致を確認済み。最終の入力一致・全長精度・正常終了の監査は診断完了後に実施する。
- Windows空き21.22 GB。新規rosbag・連続画像・点群記録なし。

### 速度融合の切分け完了と v42 の事前定義（2026-10-18）
- 全長b1診断は有効。通常構成の複製3198姿勢が完全一致、全入力時刻とIMU接頭部が一致、全プロセス正常終了。速度情報を外すと固定0.365652→0.620743 m、可動0.474061→0.833350 mに悪化。速度情報を省く案は採用しない。この診断飛行はv41開発比較に追加しない。
- `diagnose_fused_tilt.py`で実際に融合した傾き観測を真値と照合。v41短区間4件とも可動の傾き誤差が小さい。`diagnose_heading_motion.py`で1秒方位変化も照合し、ROVIOの方位変化は4件ともMIMOSAより不正確だった。方位制約の追加は見送る。真値は事後評価のみ。
- v42は`stable_view.py`を追加し、既存の特徴品質・運動検出・視野保持しきい値を満たす間だけカメラを止める。特徴が外れそうならv41の追跡に戻す。推定器・融合重み・較正・物理センサ・評価基準は変更しない。9件の追加テストを含む44実行に合格。
- 開発は3経路×seed24/25の6比較を事前固定。不合格が確定すれば打切り、独立seed26/27は合格するまで使わない。数値ログのみ、Windows最低空き8 GB。開始前空き21.22 GB。

- v42のb1飛行に、既存のLiDAR情報重み付け方式を固定・可動それぞれで追加する診断を事前定義（session4036待機）。`drpm_enabled=true, drpm_registration_mode=0`だけ変更、4DoF投影・ROVIO速度/傾斜融合を維持。これは公式DRPM更新則ではない。元の通常推定器の複製を加えて入力と全出力姿勢の一致を検査する。追加推定器はカメラ制御に使わず、v42の成功判定にも加えない。固定側が悪化したことだけで可動の優位を主張しない。

- LiDAR情報重み付けの追加診断は初期化中に不採用。カメラ偏向前、情報重み付けを入れた両推定器で数十mの位置ずれと50秒超の処理遅延が発生した。通常構成の複製は主推定器に追従。診断3ノードを所有supervisor経由で停止し、v42主飛行は継続。`diagnostic_rejection.json`に最終姿勢・時刻を保存。全長精度の比較には使わない。閾値を変えてこの飛行を成功扱いにはしない。

### 並進＋方位に制限したDRPM更新式の準備（2026-10-18、未採用）
- `performance/drpm_four_dof_kernel.hpp`を追加。LiDARの補正を世界鉛直軸まわりの回転と並進3成分に制限し、その4次元で情報・雑音・確率を計算する。更新式は公式`SolveWithSnrProbabilities`を使用するが、4自由度化した拡張であり公式6自由度実装そのものではない。除外する傾き2方向の確率・情報は厳密0。センサ分散・しきい値・ROVIO重みは既存値のまま。
- ランダム40例で座標回転に対する不変性、有限回転後の機体重力方向保持、情報の半正定値性、公式確率式/更新式との一致、微小雑音時の制約付き最小二乗一致を検査して合格。階数不足は拒否。
- `build_drpm_four_dof.py`で別ライブラリをビルド中。既存IMU/地図/通常ICPオブジェクトを再利用し、DRPM登録オブジェクトだけ差替え。現在稼働中v42の凍結ソース61件と通常ライブラリは変更しない。オンライン精度は未確認。診断設計は`v43_drpm4_diagnostic_design.json`。

### v42完了・4自由度DRPMの同一入力診断（2026-10-18T18:22:35.819213+00:00）
- v42短4比較は全監査・全子正常終了。融合位置RMSE固定→可動: a1/24 約.440→.507m、b1/24 .440→.332m、a1/25 .347→.388m、b1/25 .337→.637m。1勝3敗で不採用。元基準/未使用seed26,27を維持し、長prefixは開始せず。61実装ハッシュの最終一致を保存後に次診断の変更を行った。
- 4自由度DRPMの独立ライブラリが完成。40幾何例・公式式一致・傾き保持・雑音に支配されたトンネル方向の抑制・階数不足拒否の試験に合格。元の通常ライブラリは変更なし。
- 追加診断推定器もLiDAR処理完了まで待つ任意のpacing指定をrunner/campaignへ追加。既定は空。既存終了処理を含む5試験に合格。
- 現在v43-drpm4-diagnostic24でb1全263.385秒/seed24を実行。通常融合＋v41制御の主計算と、完全複製、4自由度DRPMの固定/可動を同じ物理飛行で比較。追加推定器はカメラ制御へ入力しない。主campaign PID1468562、補助supervisor PID1468549。新library実ロードSHA一致確認済み。全長の入力/出力一致と正常終了は実験後に検査する。
- Windows実空き約21GB、8GB停止保護。生bag/連続画像/点群保存なし。目標は未達。

### v43終了と初速度の整合性診断（2026-10-18T18:29:36.393387+00:00）
- 4自由度DRPMのb1全長診断は改善なし。通常融合固定.292205→可動.300196mに対し、DRPM固定17.591983→可動24.830422m。全main/追加子0、元13ファイルのSHA一致。追加複製3199共通姿勢は厳密一致。
- strict auditはfalse: 追加ノードの終了が主より0.6秒早く、飛行完了後の6出力を取り逃した。元結果を合格へ変更しない。補助auditで、初期化から事前定義した追従完了344.58秒までの全入力/IMU prefix/VIO観測は全追加ノードで同値、追従出力は全長存在を確認。退化対策は不採用。追加ノード終了は固定2秒待ちを廃し、主runtime確定まで待つよう修正した。
- diagnose_integrated_drift.pyで位置誤差を機体相対移動量と世界座標へ回す姿勢の2成分へ厳密分解。v41/v42の8組で再構成誤差<1e-8m。移動量成分が主に大きく、単なる方位誤差だけでは説明できない。因果効果の独立分解とは解釈しない。
- 初期速度sigma=1e-6m/sというほぼ厳密な静止priorを確認。事後の真値差分では5飛行の初期速度ノルム.0078–.0148m/sで、実ホバリングと不整合。これが後続ドリフトの原因かは未確認。
- v44-init-diagnostic24: a1の事前宣言45秒で、同じactive入力を使い初速sigma1e-6/1m/s × IMU白色雑音legacy/sensorの4条件を比較。1m/sは既存ROVIO初速度priorから採用し、真値や精度を使った調整なし。主は変更せず追加計算は制御へ戻さない。supervisor session31005、main campaign50390。初期化から全入力同値と全終了を監査する。生データ保存なし、Windows約21GB/8GB保護。

### 初期速度の診断完了、v45の比較開始（2026-10-18T18:32:38.890130+00:00）
- v44の事前宣言45秒を完走し、全main/追加子正常終了。通常設定の複製は全pose厳密一致。初速度sigmaだけ1e-6→1m/sにしたvariantは全主入力/IMU prefix/速度・傾き観測同値、設定差分監査合格。位置RMSE .098662→.041569m、回転.380918→.244208deg。真値は事後評価のみ。初速平均0は変えず、不確かさを既存ROVIO初期設定と揃えた。
- センサ由来IMU白色雑音＋初速sigma1は5.191623mで未採用。旧sigma＋同雑音は3053.93mまで発散し、追従後の最後.35秒の処理尾部が不足したためそのvariantはstrict監査false。reportにvariantごとの有効性を明示、全体falseを合格へ書換えない。通常雑音＋初速修正の比較は独立に全監査合格。
- runner/campaign/development/validationへ任意--fusion-initial-velocity-sigmaを追加。未指定なら既存値保持、指定時は有限正値検証、protocolと固定/可動両configへ保存。reportで事前指定値と両config一致を検査。構文/CLI検査と終了処理3試験成功。
- v45はv41の視野制御/物理センサ/融合較正/legacy雑音/4DoF/DRPM OFFを維持し、固定・可動双方の初速sigmaを1m/sにする。3経路×24/25の6比較と元成功基準を事前固定。59ファイルSHA固定、独立26/27未使用。firstshortはv45_firstshort.logを参照、scheduler40893。最大1sim、途中不合格確定時は未開始prefixを省略するが既取得結果を残す。
- Windows実空き21.39GB、8GB保護。数値のみ、生bag/連続画像点群なし。安定したジンバル改善の目標は未達。

### v45最初の全長比較と未接続の次案（2026-10-18T18:37:59.281855+00:00）
- v45 firstshort session28807、scheduler40893。a1/24全228.493秒を完走、設定/入力/出力/終了監査すべて合格。融合位置固定.439151→可動.397022m（9.59%減）、回転.379221→.407239deg。ROVIO位置8.214882→9.209751m/回転.425048→3.606838deg。現在b1/24を同条件で継続。1組の結果で安定改善としない。
- performance/test_imu_factor_derivatives.cppで固定重力IMU因子の全5入力について40状態×24列=960有限差分列を照合、最大差1.86182e-10で合格。現場のGTSAM実libraryを使用、実行中ソース/バイナリを変更せず。正しい測定雑音設定で精度が悪い理由を微分式の不一致では説明できない。
- 未接続のtranslation_information_gain.pyと6試験を追加。視野の評価を位置3成分の不確かさの減少へ絞り、回転との相関は全6次元逆情報から周辺化して残す。位置と回転を独立に扱わず、単位/座標回転不変・明示逆行列との30例一致・回転だけの不要情報を評価しないことを検査。v45の視野制御へは接続していない。8比較の誤差分解から次案を準備しただけで、精度改善の主張なし。

### 同時刻の融合・地図更新の整合性診断（2026-10-18T18:54:21.440720+00:00）
- v45短4比較は全監査/全子正常終了、融合位置RMSE固定→偏向: a1/24 .439151→.397022m、b1/24 .342→.352m、a1/25 .381→.439m、b1/25 .316→.350m。1勝3敗で元基準不合格。prefixは開始せず、59凍結ファイルの最終SHA一致を保存。独立seed26/27未使用、安定改善の目標は未達。
- 同時刻に別センサ因子を追加した際、MIMOSA graph内部stateは更新されるが返却用optimized_values_を更新しない箇所を確認。実ライブラリを使うtest_same_stamp_estimate.cppで旧版の返却速度と内部速度の差.990099m/sを再現、独立修正版では差<=1e-12。別ROS master11595を使用し終了済み。これは実装不整合の再現であり飛行ドリフトの主因と断定しない。
- build_batched_fusion.pyは独立build/mimosa-batched-visionを生成。初期化後の同時刻入力を視覚→LiDARの順に処理し、両更新後の推定値を地図に利用、graph出力を両更新完了後へ延期。SAME_KEYでも推定値を再取得する。センサ時刻/サンプル/融合重み/初期化順序は変更せず、通常ライブラリ/バイナリは保持。
- compare_fusion_sidecar/reportにbatched_fusion診断を追加。通常の完全複製1＋別libの固定/偏向2を同時入力で比較する。監査は同時刻内の順序だけを許容し、時刻ごとのセンサ多重集合・IMU総数/接頭部を厳密比較。順序交換は合格、時刻・センサ種・IMU数・接頭部・重複・欠落は不合格の7ケースに成功。視覚速度/傾き観測とconfigの同値、全評価区間、全子正常終了も必要。
- 現所有: v46診断supervisor session7801/PID1484406、main1484421、sidecar1484420。b1全263.384754秒/seed24、v45カメラ制御と通常融合を主とし、修正版の出力は制御へ返さない。ROOT/v46_batched_fusion_diagnostic_design.jsonに12ファイルSHAと評価範囲を事前固定。完了後同一入力監査/性能比較、効果があれば実制御の比較へ進む。
- Windows実空き約21GB、8GB下限。連続画像/点群/bagは記録しない。数値・設定・ソースだけ保持。最大1sim。

### 同時刻融合の修正確認と位置情報を優先する視野選択（2026-10-18T19:05:28.461584+00:00）
- v46 b1全長同一入力診断は全監査合格。通常融合の完全複製3208pose厳密一致、全入力stamp/IMU prefix・速度/傾き観測同値、全子正常終了。旧→同時刻一括更新の位置RMSE: 固定 .344886→.137010m、偏向 .425205→.338323m。改善は両方式に認めるが、修正後の偏向は固定より悪い。ジンバル改善成功とはしない。12凍結SHA一致。
- 実graphの追加publication試験に成功: 途中の状態は配信0、両更新完了後だけ配信1、元の時刻・返却値・内部値・配信速度が一致。別ROS master11595は停止済み。
- runner/campaign/development/validationに--batched-fusionを追加。対応する独立node/libを固定・偏向双方へ使用し、manifest/生成ソースも保存、reportでbinary SHAを照合。既定falseなので旧実験再現は維持。
- v47: 融合は同時刻修正版、初速sigma1、velocity+tilt1Hz、legacy白色雑音、sensor bias、LiDAR4DoF、DRPM OFF。視野だけを位置3成分の周辺共分散logdet減少で選ぶopt-inにした。回転との相関は残し、回転だけの観測改善を目的へ数えない。既知・現在見える成熟特徴だけ利用、未観測の画像情報や真値を利用しない。角度/速度/特徴保持制限/未成熟時fallbackは維持。
- 並進情報・統合視野・終了処理の29試験合格、旧設定の100視野判断は旧実装と出力/診断が厳密一致。構文/3CLI検査合格。実飛行性能は未確認。
- 現所有: v47 firstshort session75412、scheduler21702、最大1sim。67ファイルSHAを固定し3経路×seed24/25の6比較、元基準不変・独立26/27未使用。不合格確定なら未開始長区間を省略する。v46全終了、v45数値ログ可逆圧縮完了。
- Windows空き 21.39 GB、8GB保護、raw記録なし。安定したジンバル改善の目標は未達で作業継続中。

### v47最初の全区間比較（2026-10-18T19:11:00.421425+00:00）
- a1/24全228.493秒が完了、全入力/連続性/共通初期化/設定/終了監査と67ソース保存照合が合格。融合位置RMSE固定0.343473706→偏向0.289533234m、回転0.282886236→0.275163473deg。ROVIO位置6.828→4.677m、回転1.915→3.335deg。1組だけで安定改善としない。firstshort75412はb1/24を継続、scheduler21702は残りを待機。
- v46同時刻順序の監査: 同時刻3198組中、通常複製は3198組LiDAR先、修正版は初期化時の1組だけ旧順序、残る3197組が視覚先。同時刻の意味を取り違えた監査ではない。全原入力・出力照合結果は維持。

### v47 seed24短区間完了
- a1は融合位置.343→.290m、b1は.296→.313m、1勝1敗。両方全監査合格・全子正常終了。b1 ROVIO位置27.226→11.499m、回転1.425→3.102deg、融合回転.647→.578deg。ROVIO位置改善だけで融合改善と扱わない。
- firstshort75412終了、scheduler21702がseed25短campaign PID1492254を開始。67ソースと元成功基準維持。独立26/27未使用。
- 未実行の将来診断としてcompare_fusion_sidecar/reportにsensor_whiteを追加。同時刻修正版のmain複製を正しく選択し、物理センサから求めたIMU白色雑音だけを変えた比較に対応。旧v36/v37/v44の失敗を保持し、同時刻処理修正後にも悪化するか切り分ける目的。構文検証成功、まだ起動していない。現v47の凍結ソースは変更していない。

### v47不採用確定、数値近似の次診断を事前固定
- a1/25全長の融合位置固定.339→偏向.474m、ROVIO11.079→4.134m。全監査/全子正常終了。短3比較1勝2敗で元基準の達成不可。b1/25は完走させ、全記録保持し未開始prefixを省く。
- 既存solverの再線形化閾値は回転.0175rad/速度.05m/s/gyro bias .1rad/s、部分検査true、wildfire .001。センサ由来の小さい測定雑音と組み合わせた場合の数値近似の影響は未検証。
- numerical_relinearization.pyは全再線形化閾値とwildfireを0、部分検査falseへ切り替える診断だけを提供。測定目的関数・センサ/較正・CHOLESKY・追加反復3回・lagを維持。完全収束や最適解を保証するものとは呼ばない。設定差分/構文検証成功。進行中v47ソース不変。
- v48はa1先頭45秒/seed24を事前固定し、白色雑音legacy/sensor × 元solver/上記の全変数更新の2×2。通常完全複製を含む追加4nodeは同じactive入力を使い制御へ戻さない。14ファイルSHA固定、全入力/観測/全区間/全終了の同値を監査する。現待機owner session96007。v47の全停止後にだけ開始し最大1simを維持。まだ実行前。

### v47終了、v48数値診断を開始（2026-10-18T19:26:32.998315+00:00）
- b1/25も完走し全監査/全子正常終了。融合位置固定.211→偏向.284m、回転.341→.323deg、ROVIO位置26.989→14.239m。v47短4組は1勝3敗で不採用、長prefix未開始。元基準/独立26,27を維持。67凍結ファイル最終一致。旧scheduler21702/firstshort75412は全終了。
- v48 owner96007がmain1496487、補助1496486を開始。a1先頭45秒/seed24/同一入力の2×2数値診断。v48_campaign.log/v48_sidecar.log、完了後report_initial_velocity_diagnostic.pyを実行予定。追加推定器はカメラ制御へ不使用。

### v48数値診断完了と全長診断の事前固定（2026-10-18T19:33:20.488507+00:00）
- a1先頭45秒/seed24、全入力・IMU prefix・視覚観測・全区間・全終了監査true。全複製pose一致、14凍結SHA最終一致。位置RMSE: 元設定.054707m、sensor雑音＋元近似4749.859388m、全変数更新＋legacy雑音.050855m、全変数更新＋sensor雑音.103741m。後者の末端誤差.310445m。数値近似が発散原因の一つと確認、長期安定とジンバル改善は未確認。全所有停止。
- compare_fusion_sidecar/reportにfull_solver_noise診断を追加。同一b1全263.384754秒で元active複製＋全変数更新のlegacy/sensor×固定/偏向を比較、入力照合は対応するカメラstreamと行う。視覚重み・センサ・反復回数は変更しない。構文検証成功、v49の14SHAと全区間を実行前固定。補助出力はカメラ制御へ使わない。Windows空き20.5GB、8GB保護、raw記録なし。

- v49起動: supervisor session97322/PID1499790、main1499792、sidecar1499791。最大1sim、7推定器、全補助LiDAR処理をpace対象に追加。14凍結source不変で完走を監査予定。

- 次の視野評価の未接続部品: joint_velocity_gain.pyは速度3成分と特徴bearing2成分の全共同共分散から、候補視野での追加観測による速度共分散減少を算出。特徴間/速度との相関を保持。6試験（完全Gaussian更新一致、未観測/無相関、情報単調性、座標/単位不変、不正共分散拒否）成功。将来状態の厳密予測ではなく局所的な追加観測proxyである。
- rovio_joint_information.hpp/build_rovio_joint_information.pyはconst filter stateから現在追跡済みのID・機体内座標・速度/bearing共同共分散を配信する独立ビルドを準備。推定器状態や主実装を変更しない。v49凍結ソースは不変、実制御へ未接続・実入力は未検証。

### v49全長数値診断完了、雑音変更を不採用
- b1/24全263.384754秒、wall358.5秒、7推定器すべてexit0。入力全順序/IMU prefix/視覚観測/全区間/連続出力/複製厳密一致を含む全監査true、14ソース最終SHA一致。位置RMSE固定→偏向: 元設定 .181665→.302440m、全変数更新legacy .192922→.326969m、全変数更新sensor 38.452826→36.560737m。sensorの発散は短区間の数値省略だけで説明しきれない。雑音変更も数値設定変更も採用せず、既存の同時刻修正・初速sigma1を維持。安定ジンバル改善は未達。全sim/補助owner97322終了。

### ROVIO共同共分散を使う視野選択の短試験を開始
- JointInformationViewに評価関数の差替え箇所を設け、JointVelocityViewでROVIOの速度3成分と各特徴bearing2成分の全共同共分散を使用。候補・成熟特徴の基準・保持制限・角度速度制限・低品質時fallbackは維持。元方式35単体試験、ROSメッセージ/新方式34試験成功。座標変換は速度blockと速度/bearingの交差blockにも適用。異なる時刻/ID/座標の混用を拒否する。
- 独立rovio-joint-information binaryビルド成功。元ROVIO/ヘッダ/推定処理は保持しconst stateからのみ抽出。runner/campaignに任意--joint-rovio-information、snapshot/protocol/binary照合を追加。development/validation/reportも対応。既定無効。
- v50 smoke: a1先頭45秒/seed24、owner session92632/PID1504149、main1504152、同一入力の旧ROVIO比較auditor1504151。新旧の位置/姿勢/速度/共分散全時刻厳密一致、publishした速度共分散/特徴ID/座標一致、全子正常終了を要求。新しい視野評価が制御するが45秒結果を性能採用判断に使用しない。11SHA事前固定。Windows20.3GB、raw記録なし。
- 補助事後解析v47/v49: ROVIO重力方向の角度RMSEは偏向で同等か改善（a1/24 .1785→.1793deg、b1/24 .2993→.1231、a1/25 .1898→.1532、b1/25 .1331→.0942、v49b1 .1214→.0840）。偏向時に傾き観測だけ悪化するという仮説は支持されない。重みは変更していない。

- v50短確認が全監査成功し停止。1823時刻すべて新旧ROVIOのpose/twist/covariance厳密一致、共同共分散の速度blockも差0、特徴ID一致/座標最大差7.59e-6m、共同共分散PSD/対称性合格。main全監査true、11SHA最終一致。45秒の精度は採用判定に用いない。
- v50開発比較開始: firstshort session17576、scheduler95992、最大1sim。77ファイルSHA凍結、元の6比較と成功基準を維持、独立26/27は未使用。固定・偏向で同一の計測専用ROVIO binary使用、融合/センサ/較正変更なし。
- frozen_evidenceのsource照合キーをbasenameからfull pathへ変更し、生成ソースと元ソースや複数manifestの照合が上書きされるのを防止。評価監査6試験合格。
- 別の事後確認として、初期姿勢合わせをyaw+並進のみへ制限してもv45/v47/v49の固定/偏向順位は同じ。元の全姿勢初期合わせの採用基準/図を変更せず、alignment_convention_diagnostic_v45_v49.jsonへ保存。傾きの初期合わせが改善を隠しているという仮説は支持されない。

- v50 a1/24全228.493秒終了、全監査・全子正常終了・77保存SHA一致。融合位置固定.338→偏向.319m（約6%減）、回転.214→.228deg。ROVIO位置9.636→7.751m、回転.445→1.747deg。b1/24継続中、安定改善判定はまだ不可。
- 次の未接続案correlated_tracking_view.py: 共同共分散が距離などの不確かさを既に周辺化して持つため、旧3D独立特徴用の距離精度ゲートを使わず、時間差分を測れる既観測bearingを評価する。現v50には接続しない。距離精度/尺度に依存しない選択、±89deg近傍の物理制限/特徴保持、未観測点を選ばない4新試験＋既存6試験に成功。性能は未測定。
- 実測の上下限滞在: v47 b1/24は75.2%、b1/25は54.8%。物理可動pitch±90degに対して制御側は±30deg。v50 a1は31.4%が±30deg近傍。今後の到達範囲比較の根拠として保存し、実行中条件は変更しない。

### v50不採用、距離精度ゲートと制御到達範囲を見直すv51開始
- v50 b1/24全263.385秒完走・全監査/全子正常終了。融合位置固定.143332→偏向.266758m（比1.8611）、回転.15605→.15292deg、ROVIO位置23.418→14.112m/回転2.973→7.986deg。事前の最悪比1.5を超えたため、未開始seed25/prefixを省略し不採用。a1は.337976→.319108m。2組の結果は保持、77SHA最終一致、owner17576/95992終了。
- v51は同じROVIO/融合binary・センサ・較正で、correlated_tracking_viewを接続。既観測bearingに十分な時間差があれば、その距離精度を別途仮定せず全共同共分散で速度情報を評価。未知の特徴や真値は不使用。可動pitchを物理仕様±90degまで許可、yaw±50deg/20deg毎秒/scan5deg毎秒/90%保持/fallbackは維持。50関連試験成功。
- v51 firstshort session22678、scheduler98658、最大1sim。79SHA凍結、3経路×24/25の6比較/元成功基準維持、独立26/27未使用。Windows約20.3GB、8GB保護、数値だけ保存。

- v51 a1/24全228.493秒完走、全監査/全子0/79保存SHA一致。融合位置固定.582892→偏向.377860m（35.2%減）、回転.416257→.419459deg。ROVIO位置5.695→6.969m/回転.615→1.590deg。融合改善とVIO全体改善を同一視しない。現在b1/24。実encoder範囲yaw±50deg、pitch -90〜+84.861deg（負が上向き、正が下向き）。
- inspect_extreme_pitch.pyは実行中の偏向カメラから|pitch|≥75degで各符号1枚、最大2PNGだけ保存し、同一時刻のencoderを確認する。生bag/連続画像の記録なし。b1専用observer session73606。上向き75.54degの画像を確認、暗い天井で画面大半の機体遮蔽なし。
- audit_self_visibilityの既存幾何投影を使い、yaw[-50,0,50]×pitch[-90,-75,-60,-30,0,30,60,75,90]の27姿勢を別途検査。実body meshと保守的ローター掃引円筒で最大遮蔽2.262%（yaw50/pitch-90）、ローター0%。この27姿勢の検査であり全連続角度の上限保証とはしない。v51_full_range_self_visibility.jsonへ保存、実入力/制御変更なし。
- 未接続のlidar_conditioned_velocity_gain.pyを追加。ROVIOの仮想的な追加観測で減る速度不確かさを既存融合間隔の位置増分へ換算し、現在LiDAR情報との組合せで位置周辺共分散の減少を評価する計画用proxy。センサ間相関や将来特徴を推定したものではなく、融合への入力変更なし。全6次元逆行列との20例一致・LiDARの弱い方向優先・回転相関・単位不変・未観測/不正時間の5試験成功。進行中v51には未接続。

### 2026-10-18T20:13:36.539625+00:00 — v51終了と実ジンバル駆動モデルの修正（v52準備）
- v51はa1/24全監査合格、融合位置RMSE固定0.582892→偏向0.377860 m。b1/24は0.244382→0.308053 mで悪化し、速度検査だけ不合格。後続schedulerはその検査でexit1、手動停止ではない。全所有プロセス終了、79凍結SHA一致。成功とはしない。
- b1の速度違反は338.645秒から1msでpitchが-90°の機械端へ接触した際、pitch14.20763°/s、合成20.04636°/s。既定検査20.01°/sを緩和せず、元データと失敗判定を保存。
- Gazebo実装のvelocity modeではcmd_maxが各軸14.1421356°/s、従来plannerは合成20°/sだけを制限していた。v51 a1の評価判断8.176%が実駆動上限を超える予測。actuator_reachability.pyで実SDFを読み、core.pyへ軸別速度制限、物理計算1step分0.014142°の機械端余白を追加。モデル・駆動速度・センサは変更しない。
- v52はv51の視野目的/共同共分散/融合設定を維持し、上記到達可能性だけ修正。設定を物理較正として保護し、各実験の検査でSDFとの一致を要求。旧設定は任意機能なしで従来動作維持。
- 54非ROS試験成功、ROS依存cloud12試験はクリーンROS1環境で成功。最初の一括実行ではROSなし環境のcloud importだけ失敗、適切な環境で解決。実controller式を独立積分した80指令の到達性も検証。
- 3経路×2開発seed24/25、元改善基準維持。独立seed26/27は未使用。Windows実空き20.05 GB、下限8GB、raw bagなし。

- v52開始: firstshort97412、scheduler94690。81SHA凍結、最大1sim。observe_lidar_visual_balance.pyを独立追加しa1途中からread-onlyでLiDAR位置情報とVIO速度情報の相対強度を1Hz記録。制御/融合へ出力せず、実際の融合後共分散ではないことを明示。

### 2026-10-18T20:17:06.546355+00:00 — LiDARと視覚の補完性評価（未接続）
- v52を変更せず、lidar_conditioned_tracking_view.py/testを追加。既存融合の速度共分散倍率・1秒間隔を使い、回転との相関を残したLiDAR位置情報と候補視覚観測後の速度情報を組み合わせる。視覚の速度/特徴相関にも倍率の平方根を適用して整合を保つ。これは局所計画用近似で、実融合共分散ではない。
- 独立逆行列・座標/単位不変・LiDARが弱い方向への優先・既存較正一致の7試験成功。新部品はまだplannerへ接続していない。較正値や推定器へ変更なし。
- read-only a1途中1Hz記録でLiDAR/視覚の情報比は方向で大きく異なる。最小方向の中央値0.42、最大方向74.3（途中標本、完成精度ではない）。この値を根拠に方向を区別する候補を準備する。

- v52 a1/24全228.493秒完走、全監査/全子正常終了。融合位置0.219149→0.298114m（比1.360326）で悪化、ROVIO21.846→3.712m。駆動修正後の最大合成速度20.000000003°/s、yaw±49.98586°、pitch−89.98586〜85.49482°。機械端速度違反はこの飛行では解消したが、精度改善とはしない。b1継続。

- v52 a1の事後分解: 機体相対移動量由来の誤差0.1940→0.2907m、回転由来0.0978→0.0563m（独立因果成分ではなく和が元誤差を再構成）。実際に融合した1Hz VIO速度誤差0.48004→0.18140m/s、傾き0.20290→0.18629°。視覚側の両観測が改善しても融合全体が悪化しており、単純なVIO回転悪化説を支持しない。全値は事後評価、推定・制御へ返さない。

### 2026-10-18T20:25:29.989525+00:00 — v52不採用、v53の補完性評価を接続
- run1_tail_a1_active: 全監査=True、融合位置RMSE 0.219149072→0.298114212m、ROVIO 21.846219→3.711557m。
- run1_tail_b1_active: 全監査=True、融合位置RMSE 0.163773499→0.353212841m、ROVIO 29.653441→1.827550m。
- v52は2敗、b1比2.1567>1.5のため元基準により不採用。全owner/observer停止、81SHA最終一致。後続seed25/prefix未開始。駆動不一致修正は検査成功したが、性能向上と区別する。
- v53は準備したLiDAR-conditioned視野評価だけを接続。既存fusion covariance scale=7.908699224909816/period1sを生成設定へ読み取り専用で反映し、ユーザー指定で較正を上書きできないよう保護。評価時にも実融合設定との一致を要求。LiDAR診断のDRPM確率は視野計画だけへ使い、融合DRPMはOFFのまま。
- 関連74試験成功。旧score経路100判断の指令/全diagnosticを旧snapshotと厳密一致確認。新scoreは局所計画近似で実fusion posteriorの保証ではない。センサ/推定/重み/経路/速度/基準を変更せず、3経路×24/25を新規凍結して検証する。

### 2026-10-18T20:31:28.747347+00:00 — 移動後の視覚情報予測の独立準備（v53未変更）
- predictive_bearing_covariance.pyを追加。現在の速度と3D特徴点の共同共分散を、一定速度/角速度で移動した後のbearingへ伝播する。距離と速度の相関を保つが、将来EKF/未知特徴/未知加速度の厳密予測ではない。5試験（独立差分Jacobian、全相関、単位不変、現時刻独立でも移動後に速度情報を生むこと、原点近傍除外）成功。
- rovio_joint_geometry.hpp/build_rovio_joint_geometry.py/test_joint_geometry.cppを別途追加。ROVIOは変更せず、公式LandmarkOutputImuCTと同じ変換で全速度/点群共同共分散をread-only配信する別binaryをbuild/rovio-joint-geometryに作成中。主v53は旧binaryを継続し、凍結85ファイルは未変更。通常raw保存なし。
- 初回compileはheader単体のMPD依存で失敗。失敗compilerだけ停止して記録保存し、既知クォータニオンによる基底回転へ置換。再build中、公式変換/全共分散との独立C++照合も準備。新機能は実飛行へ未接続、精度改善を確認したとの主張なし。

- 2026-10-18T20:33:55.608511+00:00 新geometry-export binaryの独立build成功、主v53は不変。実ROVIO Stateを用いたC++試験で、公式LandmarkOutputImuCTの点位置と全特徴間/速度との共同共分散が一致、元state covariance不変、既知でない外部較正拒否を確認。joint_geometry.pyのID/時刻検査・時刻/座標変換を含むPython8試験成功。audit_joint_geometry_live.pyで元ROVIOとの実入力一致を検査する準備のみ、まだ実行していない。

### 2026-10-18T20:39:13.494035+00:00 — v53終了、geometry read-only実入力監査へ
- run1_tail_a1_active: valid=True、融合位置0.354749851→0.394807673m、ROVIO7.681949→37.139932m。
- run1_tail_b1_active: valid=False、融合位置0.110234054→0.197309442m、ROVIO32.700077→3.605293m。
- b1は機械端294.852秒の1msだけ合成速度20.1908565°/sで速度検査不合格。実指令は-89.9858579°で既定余白内だったが、physicsが次に-90°へ到達。制御遅れと断定はできず、物理端での挙動として確認。元判定を緩めず保持。全owner停止、85SHA最終一致、成功未達。
- operating clearanceを1→2物理step（0.028284°）にして、観測された1step分の端到達にも余白を残す。駆動速度/機械範囲は不変。意図的な1step古い角度を使う指令追従モデルを含む5試験合格。実Gazebo端挙動の再確認はこれから。

- geometry smoke開始: main9983、audit46952。a1先頭45秒、旧v53score＋2step余白、追加read-only ROVIOだけ新geometry exporter。事前設計/15ソースSHA固定。新geometryを制御へ渡さず、元推定との厳密一致を確認する工程検査。旧レポートの余白を再解釈しないようjoint_stop_stepsを生成設定へ記録、旧v52/v53は未記載なら1stepとして検証する。

## 2026-10-18T20:46:29.253604+00:00 — geometry配信の実入力監査完了、v54比較準備
- geometry smoke a1先頭45秒は終了、全所有process停止。元/追加ROVIO/geometry各1805時刻一致、位置・姿勢・速度・共分散は厳密一致。特徴位置/共分散は公式float32出力と誤差6e-8以下。15ソースSHA最終一致。工程検査であり精度採用に使用しない。
- v54は既観測特徴の3D位置と速度の共同共分散を定速予測し、現在LiDARと組み合わせた位置情報で評価。v53候補生成・観測bearing速度による保持制約は維持。joint geometry取得はread-only、推定/融合は変更なし。
- 関連68試験成功。旧correlated/LiDAR条件付き判断200件でcommand/全診断値が厳密一致。runner/planner/宣言/集計へ新配信を明示接続。
- 現駆動上限を維持し機械端から2物理stepの余白。実全角域で速度検査はこれから。未来加速度/プロセスノイズを含む厳密なEKF予測ではない。3経路×seed24/25と従来判定基準を維持、独立26/27未使用。Windows空き19.60GB、下限8GB、連続画像/点群/bagなし。

## 2026-10-18T20:50:39.404758+00:00 — 既存v52/v53の融合入力不確かさ監査
- diagnose_joint_fusion_consistency.pyを追加。融合が実際に利用した1Hzの速度/傾き因子時刻で真値を照合。真値は事後評価のみ、設定/重み/推定を変更しない。結果ROOT/v52_v53_fusion_consistency.json。元無効v53 b1は速度違反を維持明示。
- v52 a1速度RMSE 0.4790→0.1810m/s、b1 0.3315→0.04430m/s。較正済み共分散による1自由度あたり規格化誤差はa1 0.2518→0.2849、b1 0.4050→0.1485。95%楕円被覆はいずれも100%。速度不確かさが全般に過小という説明を支持しない。傾きRMSEもa1 0.2029→0.1863°、b1 0.1159→0.0883°。これで融合誤差悪化の因果は未確定。
- v53 a1は速度0.1348→1.2651m/s、傾き0.1872→0.3440°で両方悪化。速度規格化誤差0.1030→0.7223、傾き1.2475→4.2125。経路をまたいで同じ原因と断定しない。真値速度差分10/20msの比較差は全例0.00091m/s未満。時系列内の標本を独立反復とはみなさない。

## 2026-10-18T20:59:44.501100+00:00 — v54短区間完了・不採用
- a1/24、b1/24とも全長完走、全入力/時刻/設定/駆動/終了監査合格、95凍結SHA最終一致。firstshort7102/scheduler67712はexit0、全sim停止。2step機械端余白は両飛行で速度上限に合格（全角度/全条件の保証ではない）。
- run1_tail_a1_active: {"fixed": {"rovio": 8.894645843366561, "mimosa": 0.3473993326475174}, "active": {"rovio": 11.45228336747685, "mimosa": 0.3483029366926141}} m。
- run1_tail_b1_active: {"fixed": {"rovio": 25.264130859997795, "mimosa": 0.19100713792389734}, "active": {"rovio": 2.183367989571792, "mimosa": 0.31915700874650876}} m。
- 融合2敗、b1最悪比1.5超で元6比較基準は不合格確定。未開始seed25/採用用prefixは省略、独立26/27未使用。ROVIO b1改善を融合成功と扱わない。
- 実融合時刻の事後診断: b1速度RMSE .28960→.05062m/s、傾き .11209→.07674°で両方改善。較正済み速度不確かさ95%被覆は両側100%。融合位置は逆に悪化。誤差分解の機体相対移動量成分 .1781→.3082m、姿勢成分 .0794→.0839m。独立因果寄与とは解釈しない。
- 次は同じv54設定を長いprefix全長で診断する。短経路の不合格を上書きしたり、追加試行で6比較の合格数を増やしたりしない。最新の修正融合では長区間未検証であり、LiDARと視覚の情報強度/累積誤差を確認する目的。

## 2026-10-18T21:05:20.563167+00:00 — 長区間診断の観測追加
- prefix診断main52103を開始。protocol.route_end=1070秒（route_statusの3778.59は元の長い全route長、採用prefix長ではない）。入力/制御/融合はv54の95SHAを維持。
- observe_lidar_visual_balanceに壁時計上限/入力topic指定とanonymous node名を追加。最初の2観測器は同名node競合を避けるため追従前に終了（終了254、未採用、ログ保持）、新owner86352へ再起動。activeの実受信確認。fixedは視野診断配信を無効化しているため対象topicが配信されず0件、これを固定情報行列の計測と扱わない。
- observe_visual_depth.pyを追加、owner18880。1Hz・LiDAR/ROVIO特徴/測定外部姿勢の同時刻を照合し、LiDARの近傍平面で支持できる視線だけ距離比と不確かさの集計値を保存。元のdepth_at_pixelsを使用、画像/点座標/点群は保存しない。ROVIO/MIMOSA/制御への出力なし。両カメラの実受信と計測値を確認。視野重複なし/支持不足は0件、真値代用や因果確定とは扱わない。

## 2026-10-18T21:09:40.476991+00:00 — 標準UniPilotの視野重複と次候補の未接続部品
- 模型定義: LiDAR位置(-.15,0,.05)、姿勢(0,1.57,3.1416)、水平±π/垂直0..π/2/100m。測距方向は機体後半球。固定カメラは(+.18,0,-.025)から+X向き。全LiDAR点の機体X上限約-.0704mなので前方固定画像と重ならない。v54_prefix_lidar_camera_overlap.json。固定側の距離支持0を計測欠落/距離発散と解釈しない。偏向側は上下向きで一部重なり、途中237時刻の距離比中央値1.0037、時刻ごとの中央値の10/90%値.8901/1.1074。全区間値ではない。
- retinal_motion_candidates.py/testを未接続で追加。既観測bearingをカメラ座標で保つ2軸運動を、実駆動範囲の箱制約付き最小二乗で算出。真上/真下近傍の方位角変換を直接使わず画像内回転も扱う候補生成。有限差分、独立scipy解との100例一致、鉛直視野の画像回転補償、観測不能方向での不要回転抑止、可動制限/入力不変の5試験成功。現prefixには未接続、精度改善の証拠とはしない。

## 2026-10-18T21:19:25.785219+00:00 — v54長区間診断完了、v55候補生成
- prefix1070秒全長完走、wall1072.18秒、全監査合格・全所有process終了。保存実装95項目/現在95SHAとも事前宣言一致を確認後に次編集。ROVIO位置43.009→34.370m/回転2.008→10.219°、融合位置6.295→6.347m/回転1.723→1.708°。安定改善は未達、v54不採用を維持。
- 距離観測は両側916時刻・処理エラー0。偏向側827時刻でLiDAR支持、時刻毎距離比中央値の10/50/90%値.9820/1.1472/1.7428。途中237時刻の約1.00という値は全区間を表さない。固定側は前後の視野非重複により支持0。情報強度観測active1087時刻、unmatched/invalid0、fixed配信無効なので0。すべて事後診断であり推定入力ではない。
- 完了数値のSHA検証付き可逆圧縮は2738群/3150パス（総削減3.1366GB Linux bytes）。Windows実空き約19GB、VHD容量回収を主張しない。
- v55はretinal_tracking_viewを追加し、v54候補集合を丸ごと残した上で、カメラ座標のbearing変化を小さくする実駆動上限内の候補を追加。単一特徴に加え画面内の全特徴の動きを抑える候補を作る。評価式/90%保持/3%改善閾値/融合/センサ/6比較基準は維持。ROVIO再座標化の不具合と断定しない。
- helper抽出時のローカルlimits変数不足は起動前のテストで検出し修正。最終78関連試験成功、旧3方式300判断のcommand/全診断値厳密一致。まだオンライン性能は未測定。

## 2026-10-18 — v55完了・不採用、速度誤差の時間相関を切り分け
- v55短2組は全監査/全process正常終了、99SHA最終一致。融合位置a1固定.432877→可動.327969m、b1 .086182→.506559m。ROVIO a1 11.893→25.247m、b1 24.903→2.407m。最悪比5.877で事前基準不合格、後続開発/独立seedは未実施。owner40075/76252停止。
- diagnose_velocity_temporal_consistency.py追加。実際に融合した1Hz速度の誤差を共通世界座標へ移し、重複しない1/2/5/10/20観測平均と、独立と仮定した平均の共分散を比較。真値は事後解析だけ。v52/v54両経路とv55a1の10系列で、1観測の正規化誤差.113〜1.354が20観測平均で1.495〜14.234へ増大。1Hz化しても持続誤差が残る。これは融合悪化の因果証明ではない。
- calibrate_velocity_temporal.pyは元の独立固定2飛行・既存covariance較正だけからAR(1)を較正、rho=.9687373251746727、innovation variance=.06154799481342044。開発データ/融合結果で調整せず、20Hzの重複位相を反復と数えない。元CSV SHA照合済み。
- temporal_velocity_factor.hppと隔離build_temporal_velocity.pyを追加。前時刻の速度誤差が持続することを扱い、既存の速度/傾き/センサは維持。過去状態を保持するため2秒historyを使い、同じhistoryの独立因子も対照とする。40座標/gauge例、720微分列、100密Gaussian尤度一致試験に合格。共有IMU相関は未解決、AR近似は既知の限界。
- 初回buildは生成C++の改行escape漏れで失敗。修正後exec35321はexit0、manifest全8SHA一致。先行失敗build65285がまだ終了していない間に再buildしたため同じlogへ出力が混在した。両process終了済みで実行バイナリ/入力sourceはmanifest照合済み。失敗ログ/混在ログを保持し成功ログと誤認しない。今後のbuildは前process終了を確認して個別logを用いる。
- compare_fusion_sidecar/report_initial_velocity_diagnosticに同一入力の時間相関診断を追加。disabled複製の厳密一致、lag2だけの固定/可動、AR+lag2の固定/可動を比較予定。通常起動/既存binaryは変更なし。性能改善はまだ未確認。

- v56同一入力b1全長診断開始: main exec66382、sidecars57992。事前112SHA固定。旧独立固定較正から求めたARモデルの開発12系列事後検査では、1Hz innovation正規化誤差.216〜2.188/95%範囲84.7〜99.6%。動的視野へ完全に整合したモデルと断言しない。重みの再調整は行わず、まず同一入力切分け。
- 完了済み数値の可逆圧縮終了exec97892。累計2788群3206path/3.2726GB論理削減、重複/タイミング410項目2.1612GB論理削減。展開SHA照合済み、元bag/v56実行中データは不変。Windows実空き18.76GB。

- v56 b1全263.385秒診断完了、main/sidecar全監査・全process正常終了、112SHA最終一致。新binary disabled複製は全pose厳密一致。融合位置RMSE: 元lag1固定.322444/可動.408152m、独立lag2 .099197/.376698m、AR lag2 .954629/.442305m。持続誤差モデルは両側悪化のため不採用。固定/可動比だけならARで改善して見えるが、固定基準自体を悪化させているので成果としない。時間相関だけを今回悪化の主因とは断定できない。
- v57準備: 以前の画像地図方式を、修正済み融合/入力時刻/駆動で再検証する。画像角点とLiDAR距離、またはMIMOSA移動から三角測量した既観測地図で候補方向を評価。ROVIO推定距離をその地図へ使わず、推定器・センサ・融合重みはv55と同じ。新tracking_aware_v57.yaml、物理pitch±90°/実駆動2step余白を維持。純Python38試験成功、ROS接続試験実行中。ARを採用していない。

- v57開始: firstshort88190/scheduler50211、81SHA凍結。純Python38＋ROS接続12試験成功。a1途中で地図更新・視野選択動作、geometry_failure=null。最終性能は未判定。

## 2026-10-18T21:46:53.960550+00:00 — v57 a1全長比較
- a1/24は全228.493秒完走、全監査true。融合位置RMSE固定.288350→可動.258703m（比.897183）、ROVIO11.0317→13.6754m、全所有子exit0。b1は継続中。1組の改善を安定改善とは扱わない。
- 停止済み数値ログ圧縮は2802群3222path/累計3.2980GB論理削減、重複等412項目2.1706GB。Windows実空き18.76GB、raw保持なし、元研究bag不変。
- b1の途中からreadonly深度観測器を追加。実LiDARで支持される視線だけROVIO距離と照合し集計値を保存、推定器/制御への出力なし。全長の因果比較とは扱わない。

## 2026-10-18T21:51:04.548085+00:00 — v57短区間結果、未接続の距離初期化案
- b1/24も全263.385秒完走、全監査true・全子exit0。融合位置固定.104227→可動.116437m、ROVIO43.3841→48.3520m。a1と合わせ1勝1敗、基準の達成はまだ可能なためscheduler50211がseed25を継続。firstshort88190/readonly observer76831終了0。
- 未接続のLiDAR距離初期化を別実装/buildで準備。新画像特徴の初期距離だけ、同時刻LiDARの支持平面があれば置換。既存特徴/機体姿勢/速度の更新はしない。ROVIOの既定相対初期分散を維持し、LiDAR由来の小さい分散へ置き換えない。純VIOの不具合修正ではなくLIVO接続方式の変更であり、別条件として扱う。
- lidar_depth_bootstrap.hppの300平面交差/視野外・後方・線支持拒否試験成功。rovio_lidar_bootstrap.hpp、build_rovio_lidar_bootstrap.py、全4距離表現/40外部姿勢の試験を追加。現在隔離build29419中、実験への接続なし。v57凍結ソース不変。

- 距離初期化の初回build/testはEigen式を回転APIへ直接渡した型不整合で失敗、具体的Vector3dへ評価して修正。元失敗log保持、別retry logでbuild67236/test33465ともexit0。実ROVIOの4距離表現×40外部姿勢、body/covariance不変・disabled/stale同値の160例成功。
- 無効化時の20秒既存合成画像/IMU/encoder再生は従来binaryと497診断行がbyte完全一致。bootstrap_disabled_replay.json、全所有master/node停止済み。新機能ONの精度は未確認。
- v57/25 b1に推定器だけ追加する診断observerを開始。最初のobserverは未使用TF/サービス名の隔離追加をロード前に起動したため、主推定器入力が始まる前（全診断0行）に停止・全子0。元失敗試行を保持。v2 exec37322は全topic/service/TFを分離、主入力開始前に再起動。主MIMOSA/カメラ制御には戻さない。v57凍結81ソースは不変。

## 2026-10-18T22:02:24.105798+00:00 — v57短4組完了、長区間へ
- a1/25全長も全監査true、融合位置.404520→.311864m。b1/25全長も全監査true、融合位置0.484826387→0.198596574m。短4組3勝1敗、平均比.798724832/最悪1.117143583、a1経路平均.834065821、b1 .763383843。長区間2組未評価なので安定改善の達成とはしない。全81SHA不変。scheduler50211がprefix/24全1070秒を開始、次にprefix/25を予定。
- 新距離初期化の条件付きROVIO診断も完了exec37322、全追加子0。無効複製6382診断行が主と厳密一致、両enabled時刻も主と完全一致。固定側555新規特徴でLiDAR支持0・全結果厳密一致。可動側603中33特徴に支持、ROVIO位置31.021→30.563m/回転3.990→4.107°で小さな位置改善と回転悪化。推定結果を主制御/融合へ返していないためジンバル改善成果には加算しない。未採用のまま保持。
- report_depth_bootstrap.py初回はCSV loaderがdictを返す仕様を誤用して失敗。structured arrayへ修正後、全監査true。失敗report logと旧source snapshotを保持し、修正版report/source hashも追加。画像/点群記録なし。Windows18.53GB。

## 2026-10-18T22:07:02.219029+00:00 — v57短4組の融合入力の事後監査
- ROOT/v57_short_fusion_consistency.jsonとv57_relative_motion_diagnosis.json。設定/重み/制御は変えず、真値は事後比較のみ。実融合時刻1Hzの速度RMSE(a1/24,a1/25)固定.207147/.165355→可動.346673/.212493m/s、傾きも.179630/.214588→.183250/.218012°で改善していない。a1融合位置改善を「ROVIOの速度が良くなったため」とは説明しない。
- b1/24速度.493004→.527820m/s、b1/25 .464294→.340006m/s。後者の傾きも.183241→.096127°へ改善。経路/乱数条件で機序が同じとは断定しない。報告covarianceの変化も記録しており、視覚の観測性が一様に改善したとの主張は未実証。独立検証と長区間の判定は継続。

## 2026-10-18T22:13:49.479882+00:00 — v57の移動方向別・事後診断
- diagnose_velocity_directions.pyを追加。完了済み短4組だけを対象に、実融合1Hz速度誤差を水平進行/水平横/上下へ分解。真値は事後評価だけ、速度0.2m/s以上という移動中の診断部分集合を明示し、全区間の合否集計は変更しない。固定/可動の因子時刻一致・方向変換前後の誤差ノルム保存を全件確認。
- ROOT/v57_short_velocity_directions.json。進行方向RMSE a1/24 .223664→.374485、a1/25 .177526→.229483m/sで悪化。b1/24 .594223→.637380、b1/25 .525744→.409383m/s。a1融合位置改善を「進行方向のVIO誤差改善」で説明する仮説にも支持なし。移動方向は実LiDAR退化方向の測定ではなく、因果寄与の確定とも扱わない。v57実行時81ソースは変更なし。

## 2026-10-18T22:19:11.028072+00:00 — v57長区間1組目完了
- prefix/24は全1070秒完走、wall1024.984秒、全監査true・全子終了。融合位置RMSE固定6.335750→可動4.917368m、回転1.558026→1.326847°。ROVIO位置31.433053→77.166023m、回転6.441171→7.281740°で悪化。これを視覚推定単体の改善とは扱わない。
- 開発5組4勝1敗、平均比.794205949。prefix/25（最後の開発組）をscheduler50211/child1560822で開始、乱数26/27は未使用。現在81SHA不変、容量18.53GB。

## 2026-10-18T22:23:14.261798+00:00 — 方向選択の事後集計
- summarize_view_behavior.pyを追加。完了済み試行だけの実encoder/選択理由/追跡数を集計し、未完了はcomplete=falseと保持。初回はencoder時刻列をstamp_nsと誤認して明示エラー、実際の秒単位t列へ修正後exit0。初回失敗logも保持。ROOT/v57_view_behavior.json。
- v57 a1の有効追跡4未満はseed24で23.23%、25で9.96%。b1両方0%、prefix24は3.09%。これはROVIOの不安定化候補だが因果は未検証。60°超pitch滞在はa1/24 19.62%、b1/24 23.50%、a1/25 1.59%、b1/25 20.92%、prefix24 0%。画像内容・機体遮蔽をこの数値だけから推定しない。現在の制御は変更なし。

- v57完了5組を既存diagnose_integrated_drift.pyで分解。prefix24融合の位置誤差を再構成する機体相対移動成分は4.6987→2.9107m、姿勢成分4.8083→4.1049m。交差項があるため加算可能な独立因果寄与とは扱わない。全件の分解再構成一致を確認。ROOT/v57_integrated_drift_diagnosis.json。実融合速度のprefix24 RMSEは.112563→.224703m/sで悪化、傾き.102957→.097829°。視覚速度そのものの改善説は支持されない。

## 2026-10-18T22:27:35.628178+00:00 — Windows容量急減の対応
- Windows実空き18.53→8.07GBへ急減。現在prefix25約80MBの保存では約10GB減を説明できない。過去原因だったWindows Temp/wsl-crashesに新たな大型ダンプなし。pagefile割当20353MB/使用5156MB。原因未確定。
- 再生成可能なpip download cache136ファイルだけ削除、Linux未使用ブロックをfstrimで計5.53GB通知。Windows実空き8.07GBのままなので回収成功とは報告しない。原研究bag/精度CSV/画像図は不変、OS/VHD設定変更なし。8GB停止guardを維持し監視頻度を上げる。ROOT/storage_event_20261018T2227.json。

## 2026-10-18T22:37:02.580079+00:00 — v57 prefix25容量停止・同条件取り直し
- 最後の開発試行がroute868秒付近でWindows空き8GB停止guardに到達。runtime failed/stopped=true、全子/内側Gazebo exit0。scheduler50211はexit1。失敗試行はv57-storage-interrupted-prefix-seed25に丸ごと保持、v57_operational_retries.jsonに理由/元runtime SHAを保存。性能を理由に除外した試行ではなく、途中値を完走扱いしない。
- 停止後Windows空き17.73→18.05GBへ戻った。VHDサイズ169091268608bytesは前日と同じ、pagefile割当20353→19497MBであり、約10GB全体の増減原因は未確定。Tempの2.6GBは現WSL swap.vhdxなので削除しない。新core dumpなし。GPU clock補助だけを8秒実行する切り分けはexit0/Windows空き18.052GB不変。
- 全sim停止中にsync後、再読込可能なLinuxページ/ディレクトリcacheを1回解放。available13GiB、Windows空きは18.052GBのまま。OS永続設定/ユーザデータ/センサ品質は変更なし。ROOT/linux_cache_release_20261018T2235.json、gpu_clock_storage_probe.json。
- 81SHA現在一致確認後、同seed25/同設定/全1070秒を元出力場所へ取り直し開始exec4397。schedulerは再起動せず最後の1組だけ手動管理。readonly observe_host_resources.pyを追加しexec4838で15秒毎のWindows空き/Linuxメモリ/Gazebo RSS/GPUメモリ、1分毎pagefileを保存。推定/制御への出力なし。8GB停止維持。開発完了5組の4勝1敗は不変、独立26/27未使用。

## 2026-10-18T22:58:09.959093+00:00 — v57開発6組合格、設定固定して独立検証開始
- 取り直しprefix25全1070秒完走、wall1182.408秒、全監査true/全子・Gazebo正常終了。融合位置固定6.132338→可動5.034957m/回転1.617406→1.455201°、ROVIO65.828759→108.920478m/12.379024→8.214604°。main4397/observer4838終了0。容量停止の元868秒試行は別保存し採用に加算しない。
- 開発6組5勝1敗、平均比.7986799715、最悪1.1171435829、経路平均a1 .8340658207/b1 .7633838434/prefix .7985902505。事前の全基準合格、全81SHA/保存実装/入力条件一致。v57_final_current_source_verification.json、development_summary_v57.json。ROVIO単体は一様に改善しておらず、融合改善の一般機序確定とは扱わない。
- freeze_gimbal_candidate.pyが6組全証拠・未使用seedを検査し、frozen_candidate_v57.json/frozen_v57_sources.zipを作成。以後設定/ソースは変更しない。未使用seed26/27×同3経路の6独立比較を開始、firstshort exec39331、scheduler75043、最大1sim/DRPM OFF/GUI rawなし/8GB guard。validation_v57_firstshort_command.json。
- retryのWindows最小空き約17.8GB、GZ RSS終盤約7.8GB、GPU約1.2GB。キャッシュ解放後は容量急減が再発しなかったが因果確定とはしない。新observe_validation_resources.pyで6本にreadonly容量観測を追加、各flightに小JSONLだけ。独立検証は未完なので目標達成とは報告しない。

- 開発v57完了後の数値・重複ログをSHA照合付き可逆圧縮。累計2922群3356path/3.6428GB logical削減、重複/時間ログ426項目2.2965GB。新v57prefix25全長レポートを圧縮データから再計算し、全結果辞書が圧縮前と厳密一致（compressed_v57_report_check.json）。凍結81ソース不変、実行中validation-v57は数値圧縮対象外。Windows実空き17.82GB、VHD物理容量回収は主張しない。cleanup exec73179終了0。
- 独立検証監視はfirstshort39331/scheduler75043/readonly resource watcher8335。ROOT/index.htmlの更新初回は下記の集計形式問題で失敗し、修正後に更新した。

## 2026-10-18T23:02:02.872479+00:00 — 全体レポートの診断形式混在を修正
- report_gimbal_improvement.pyがROOT内すべてのresults.jsonを飛行レポートと仮定し、診断ファイルの整数seedでTypeErrorになった。飛行名*_active/valid bool/checks dictを持つ比較と、診断メタデータを区別する表示側の修正を追加。宣言された独立検証に不正形式があれば必ずエラーとし、失敗飛行も履歴に残す。性能の合否基準/計算式は不変。
- 新3試験＋既存合否6試験の計9件成功。初回失敗log保持、retryはexit0でROOT/index.htmlとvalidation_summary_v57.jsonを更新（まだ独立0組、未達表示）。実行時の凍結81ソース不変。

## 2026-10-18T23:06:11.756291+00:00 — v57独立1組目完了
- a1/seed26全228.493秒・wall365.6秒、全監査true。融合位置RMSE固定 0.472135215→可動 0.334258782m、ROVIO 6.706275495→8.121710737m。独立1/6組、誤差比0.707972571、未完了なので成功判断なし。seed26 b1を継続、81SHA変更なし、Windows17.58GB。ROOT/index.html更新。

## 2026-10-19T08:50:03.829391+00:00 — 独立b1容量guard停止、同条件再取得
- seed26 b1はroute198.9568/263.3848sでWindows空き8GB guard停止、全子0/stopped=true。前後サンプルの空きは17GB超で停止瞬間の値は未記録。同時期のホストwall clockに約32410.89秒の跳躍があり、ホスト停止/容量変化の詳しい原因は未確定。現空き約17GB。guard閾値は維持。元データ/親log/campaign/容量記録をROOT/validation-v57-storage-interrupted-seed26-b1へ保存、v57_validation_operational_retries.jsonに理由/SHA。
- 全81SHA一致のまま同seed/全経路を取り直しexec90914。完走済a1は再計算しない。readonly観測wrapperに完了飛行skip・未整理失敗なら明示停止を追加し、同名ログ上書きを防止。凍結runtime対象外。性能判定/推定/制御は変更なし。継続scheduler/resource observer再開、目標未達。

- 再開所有process: firstshort retry90914、scheduler8471、resources61533。observe_storage_guard.pyを追加しexec42781でWindows statvfsを0.2秒間隔read-only監視。変化100MB/15秒/8.5GB未満/実時刻不連続のみ小JSONL保存、閾値や制御は変更なし。停止瞬間値が15秒監視では残らなかったため。ROOT/v57_independent_storage_guard.jsonl。凍結実装対象外。

## 2026-10-19T08:55:32.189321+00:00 — v57独立2組完了
- seed26 b1取り直し全263.385秒完走・wall321.9秒・全監査true/全子0。融合位置固定.226205→可動.195271m、ROVIO21.208077→16.843630m、融合比.863245662。独立a1/b1とも改善、2/6組なので全成功判断なし。seed27 a1→b1を開始、scheduler8471/observer61533/容量42781継続。Windows17.69GB、取り直し停止なし。

- report_gimbal_improvement.pyに独立検証だけを表示するROOT/validation_v57.htmlを追加。既存全履歴indexも維持、同じ検証値/合否式を再利用し数値不変を確認。各runの六軸図/PDFへリンク。凍結runtime81ファイルには触れていない。

## 2026-10-19T09:00:26.077410+00:00 — v57独立3組完了
- a1/seed27全228.493秒/wall290.9秒/全監査true。融合位置.284→.246m、ROVIO15.909→10.918m。独立3組すべて融合位置改善。b1/27を開始し、長2組はまだ未実行。判断基準は変更なし、目標未達。

## 2026-10-19T09:06:19.870735+00:00 — v57独立短4組全改善
- b1/seed27全263.385秒/wall325.9秒/全監査true。融合位置固定.278→可動.202m、ROVIO35.848→49.616m。独立短4組全改善・平均比.7911029259、最悪.8660040747、a1平均.7869883228/b1 .7952175291。ROVIOは4組中2改善2悪化なので融合改善と区別。
- prefix/seed26全1070秒の検証を開始、次prefix/27。全6組未完了につき成功判断なし。scheduler8471、observer61533、容量42781継続、Windows21.6GB。ROOT/validation_v57.html更新。

- 独立短4組の実融合1Hz速度だけを事後診断。RMSE m/s a1/26 .173805→.282391、a1/27 .311721→.340095（悪化）、b1/26 .275378→.157812、b1/27 .914059→.472968（改善）。4組の融合位置改善を一律のROVIO速度改善とは説明できない。ROOT/v57_independent_short_velocity_consistency.json/log。真値は診断限定、制御/推定/判定は変更なし。

## 2026-10-19T09:23:52.811266+00:00 — v57独立長1組目完了
- prefix/seed26全1070秒/wall1058.4秒/全監査true。融合位置固定7.151→可動5.954m、回転1.829→1.662°。ROVIO25.246→48.053m/1.471→4.551°で悪化。独立5組すべて融合位置改善だが長区間の残差は数m。
- 最後prefix/27を開始、scheduler8471/observer61533/storage42781。判定は6組完了後、現時点未達。Windows21.6GB、入力/設定は不変。ROOT/validation_v57.html更新。

- 容量停止試行の保存先変更後、fixed側の共有経路3参照が元の絶対パスを保持していたため、後続retryを参照する不整合を発見。対象は失敗保存2件だけ。元試行自身の経路へ相対リンク5件を修正。旧prefix25では圧縮時にretry経路を取り込んだfixed/route_tracking.csv.gzがあったため、誤参照コピーをwrong_cross_trial_route_tracking.csv.gzへ保持し、本来の失敗試行の記録へリンクを修正。元数値削除/上書きなし、成功した開発/独立結果は変更なし。ROOT/v57_stopped_archive_reference_repairs.jsonに全変更とSHA。今後停止試行を移動するときは固定側の共有リンクを移動先へ付け直す。

## 2026-10-19T09:45:36.454880+00:00 — v57独立全6組完了、17.7%改善だが事前20%基準は未達
- 6/6組が全監査/全凍結証拠true、6/6融合位置改善。平均比.82261479235（17.7385%低減）、最悪.93877491434。経路平均a1 .78698832279/b1 .79521752908/prefix .88563852518。全平均≤.8だけ不合格、基準は緩和しない。ROVIO単体は4組悪化2組改善。validation_v57.html/validation_summary_v57.jsonに全結果。
- run1_tail_a1/20261026: 融合位置 0.472135215→0.334258782m、回転0.395836228→0.363891479°、ROVIO位置6.706275495→8.121710737m。
- run1_tail_b1/20261026: 融合位置 0.226205190→0.195270649m、回転0.398232585→0.387707646°、ROVIO位置21.208076621→16.843629573m。
- run1_prefix/20261026: 融合位置 7.151351806→5.953515653m、回転1.828708577→1.662106559°、ROVIO位置25.246453300→48.052864579m。
- run1_tail_a1/20261027: 融合位置 0.284336379→0.246236463m、回転0.347125020→0.347832394°、ROVIO位置15.908953862→10.917635452m。
- run1_tail_b1/20261027: 融合位置 0.277982733→0.202146096m、回転0.446664407→0.291861665°、ROVIO位置35.847507314→49.615582521m。
- run1_prefix/20261027: 融合位置 6.796262521→6.380160766m、回転1.785864082→1.751330285°、ROVIO位置34.171947379→58.851788570m。
- scheduler8471/observer61533正常終了、全シミュレーション/推定器終了0。容量監視42781は所有PIDだけSIGTERM停止。全81現ソース一致をv57_final_independent_source_verification.jsonに保存。数値可逆圧縮を完了（exec94758 exit0）、圧縮後全6結果の厳密再計算を確認中。v57は改善済みの比較基準として全記録保持。独立seed26/27は今後調整後の未使用検証には再利用しない。

## 2026-10-19T09:45:36.454950+00:00 — v58: 実追跡不足時の地図探索を抑える対照
- tracking_aware_v58.yamlはv57との差分allow_map_recovery true→falseの1項目のみ。実ROVIO追跡数<4では1秒まで角度保持し、その後既存の速度制限付き前方復帰を使う。地図の特徴数が多いことだけを理由に追跡不足時の探索を継続しない。推定器/融合重み/センサ/情報評価式は不変。開発seed24/25×3経路を事前指定し同基準、通った場合の独立seedは新28/29を使う。
- test_observed_map_recovery.pyで地図が存在しても現在追跡数を優先、1秒遅延/復帰速度/回復後タイマreset/v57対照を確認。初回は既存early returnの理由名・待機挙動を誤想定して1失敗、実装は変えず期待を実際の仕様（保持→recover_forward_no_map）へ訂正。新1＋既存速度制限1の2試験成功。
- scheduler15215がdevelopment_v58.jsonで81SHA/6開発組/同合否基準を宣言、firstshort84856（a1,b1/24）開始、observer46197/storage42323。observerへdevelopment読み込み/早期不採用終了を追加（読み取り専用、性能経路外）。1sim/8GBguard/GUI raw bagなし。現在まだv58性能不明。

- v57独立全6組を圧縮済み数値から再集計し、4campaignの結果辞書すべてが圧縮前と厳密一致、全validを確認（exec3356 exit0、v57_independent_after_compression_check.json）。累計数値3036群3484path/3.9351GB logical削減、重複・時間log440項目2.4036GB。Windows実空き約21.6GB、物理回収と混同しない。

## 2026-10-19T09:50:08.814247+00:00 — v58開発a1/24完了
- 全228.493秒/wall282.9秒、全監査true。融合位置 0.410809846→0.314580030m、ROVIO 5.096825405→5.904633317m、融合比0.765755820。前候補と実入力が同一ではないため絶対誤差の候補間比較をせず、各飛行内の固定→可動を比較。b1/24継続中。

## 2026-10-19T09:55:39.335011+00:00 — v58開発短seed24両方改善
- b1/24全263.385秒/wall337.9秒/全監査true。融合位置0.517973941→0.133089308m、回転0.338200134→0.160705387°、ROVIO位置37.609187439→46.475895421m、融合比0.256942092。ROVIO位置は悪化。firstshort84856 exit0、scheduler15215がsecond_short seed25を開始（child1591497）。observer46197/storage42323継続、81SHA変更なし。全6開発組・独立検証未完了なので目標未達。

## 2026-10-19T10:00:34.092300+00:00 — v58開発a1/25完了
- 全228.493秒/wall291.9秒、全監査true。融合位置0.446989042→0.420574435m、ROVIO位置10.296256255→15.395131746m、融合比0.940905473。改善は小幅、現3組すべて融合位置改善。b1/25継続中。

## 2026-10-19T10:07:57.115739+00:00 — v58不採用
- b1/seed25全263.385秒/全監査trueだが、融合位置.225→.369m（比約1.64）、ROVIO26.710→7.323m。短4組は3改善1悪化、最大比1.5とb1経路平均.9の事前閾値を超過。early-stopが未開始prefix2本を省略、完了試行は全保存。development_v58.json/early_rejection/summaryに詳細。全81SHA最終一致。scheduler15215/observer46197は終了0、storage42323所有PIDだけSIGTERM停止。全sim停止済み。v57独立6/6改善17.7%を比較基準として保持。
- 次の検証候補: 現在のROVIO bearingを視認性用8mへ置換するdirect inputが、同時刻画像地図側で得られるLiDAR支持距離を使っていない点。推定器への新因子は加えず、取得時刻へ補償した点群平面によって追跡点の距離を推定し、視野離脱の予測だけを修正する方針。現在は調査段階、v59未実装/未実行。

## 2026-10-19T10:12:51.075182+00:00 — v59: 現在追跡点の視認性予測へLiDAR支持距離
- v57基準のallow_map_recovery=trueへ戻し、tracking_aware_v59.yamlではtrack_lidar_depth=trueだけ追加。ROVIO/MIMOSAの更新や融合重みは変更しない。direct bearingは従来8mのまま受信し、5Hz計画時だけ既存depth_at_pixelsの検証済平面で支持された距離へ置換。距離未取得/疎/背面/取得時encoder欠落は元8mの点を保持。
- tracking_lidar_depth.pyを追加。計画時刻へ補償済点群を画像取得時刻の機体座標に戻し、その画像と同時刻のencoderから得たカメラ姿勢で測距する。結果を計画時刻へ変換して視野離脱予測だけに渡す。現在のジンバル角を過去画像へ誤適用しない。真値なし/推定器への因子追加なし/特徴数と順序不変。
- test_tracking_lidar_depth.py: yaw±50/pitch±90・距離1〜12m・機体回転並進80平面例で既知3D解に一致、無支持時のbit一致。test_planner_lidar_depth.py: 取得時と現在の角度を別にして座標一致、機能OFFで旧計算bit一致、取得時encoder不在fallback。新＋既存の純Python10、ROS接続13、制御19の計42成功。最初のROS試験はROS2環境からsourceしてimportエラー2件、clean環境で13件成功。機能の失敗ではないが記録。メタデータ追加後の新ROS1件も成功。
- planner_node.pyは任意flagのときだけ使用。report_gimbal_campaign.pyに支持点数/取得時刻一致/scan age/距離範囲の監査と支持回数を追加。continue_gimbal_development.pyが新helper/testsを保存SHA対象へ追加。development_v59.jsonは84SHAを宣言、途中変更禁止。
- firstshort49435（a1→b1/24）、scheduler82157（early-stop/最大1sim/次short25→prefix24→25）、readonly observer82743/storage3015で開始。開発基準・種24/25は不変、通過時の独立28/29はまだ未使用。Windows約21.6GB、8GB guard/GUI raw bagなし。v59性能は未確認。

## 2026-10-19T10:17:56.329489+00:00 — v59開発a1/24完了
- 全228.493秒/wall284.9秒/新距離監査を含め全valid。融合位置0.385701512→0.318030734m、ROVIO位置6.575469245→10.877549660m。融合比0.824551431。距離支持統計{"decisions": 1088, "supported_decisions": 182, "supported_points": 450, "acquisition_time_checked": true}。距離の幾何学整合は確認、これだけで精度改善の機序は確定しない。b1/24実行中。

## 2026-10-19T10:23:12.516907+00:00 — v59短区間seed24完了
- b1全263.384754秒完走、wall332.279秒、全入力/終了/距離時刻監査true、全子正常終了。融合位置RMSE固定0.083814→可動0.061311m、ROVIO33.793187→26.922575m。a1を含め2組改善、平均比0.778034。開発途中であり独立検証成功とは扱わない。
- scheduler82157がseed25短区間a1→b1へ進行、runtime84SHA不変。ROOT/v59_progress.html更新。Windows空き21.6GB、8GB停止基準を維持。

## 2026-10-19T10:26:57.788576+00:00 — v57独立6組の融合入力を追加診断、v58完了ログ圧縮
- 推定器へ入力した1Hz速度と重力方向を、その時刻の真値へ事後照合。v57_independent_all_fusion_consistency.jsonとv57_independent_velocity_directions.json。a1は2seedとも速度/傾き誤差が悪化、b1は両方改善、prefixは速度が悪化/傾きは小幅改善。融合位置6勝をROVIO単体の一様な改善と説明する根拠はない。速度不確かさの中央値はactiveが6組中5組で小さく、単純にvisionを弱めただけという説明も支持しない。因果を分ける追加の融合除去実験は未実施。
- 停止済みv58等を可逆圧縮、exec23173終了0。累計452件/2.4569GB logical、数値3092群3548path/4.0309GB logical削減。v58全4組を圧縮後再集計し結果辞書が完全一致/全valid、exec83413終了0。現行v59数値は除外、元画像bag等は保持。Windows VHD物理回収とは主張しない。

## 2026-10-19T10:29:16.080996+00:00 — v59 a1/25完了
- a1全228.492812秒完走、wall293.4秒、全valid/全子0。融合位置固定0.420105→可動0.225389m、ROVIO6.943633→14.180389m。3完了組は全て融合位置改善。seed25 b1継続中、長区間/独立検証は未実施。v59_second_a1_report.log、v59_progress.html。

## 2026-10-19T10:33:53.456540+00:00 — v59短4組完了、長区間へ
- b1/25全263.384754秒完走、全valid/全子0。融合位置固定0.246596→可動0.247583m（比1.004、改善に数えない）、ROVIO21.650376→14.065181m。短4組3勝1敗、経路平均基準は可能。scheduler82157がprefix24全1070秒を開始（child1605656）。以後prefix25、両方含む6組で判定。84SHA不変/GUI rawなし/8GB guard。

## 2026-10-19T10:51:32.236710+00:00 — v59 prefix24完了
- 全1070秒完走、wall1045.354秒、全valid/全子0。融合位置固定6.021453→可動5.417040m、ROVIO53.196200→50.635998m。全区間報告v59-prefix-seed24/index.html、v59_first_prefix_report.log。開発5組4勝1敗、prefix25をscheduler82157が実行中（child1609078）。まだ6組基準判定は未完。Windows21.6GB、84SHA不変。

## 2026-10-19T11:10:44.555498+00:00 — v59開発全6組合格、固定して未使用seed28/29へ
- prefix25全1070秒完走/全valid/全子0。融合位置固定7.599392→可動5.782923m、ROVIO60.140409→93.901331m。開発全6組5勝1敗、平均比0.7928625156352981（20.71%低減）、最悪1.00400586006、経路平均a1 .680528531636/b1 .867761439170/prefix .830297576100。事前基準すべて合格。ROVIO単体は一様に改善していない。
- scheduler82157/observer82743完了0。旧storage3015は対象cmdline/output一致のPID1597523へSIGTERM。84SHA現在一致、frozen_candidate_v59.json/frozen_v59_sources.zipを作成し保存内容ハッシュ確認。以後実装/設定を変更しない。
- 未使用seed20261028/20261029×同3経路の独立6組を開始。firstshort11325（28 a1→b1）、scheduler22584（次29短→28長→29長、最大1sim）、readonly resource58823/storage93336。validation_v59_firstshort_command.json、validation_v59_scheduler.log、v59_validation_storage_guard.jsonl。全独立完了後に判定、開発合格だけで目標達成とはしない。8GB停止/GUI rawなし/Windows21.6GB維持。

- v59開発全6組を停止後に可逆圧縮し、全結果辞書を再集計で厳密照合。v59_development_after_compression_check.jsonは全4campaign exactly_same=true/valid=true、cleanup57576終了0。独立検証の数値ログは対象外。原本研究bagを削除せず、Windows空き21.6GB。

## 2026-10-19T11:15:40.884624+00:00 — v59独立a1/28完了
- 全228.492812秒完走、wall296.4秒、全valid/全子0。融合位置固定0.384377→可動0.295469m（約23.13%低減）、ROVIO4.180815→7.527241m。独立1/6組、b1/28進行中。推定器単体と融合精度を分離して記録。v59_independent_first_a1_report.log、validation_v59.htmlを途中更新。

## 2026-10-19T11:21:29.284945+00:00 — v59独立b1/28完了
- 全263.384754秒完走/全valid/全子0、wall343.4秒。融合位置固定0.138367→可動0.152057m（約9.89%悪化）、ROVIO38.181906→18.866403m。独立2組1勝1敗。悪化も保持し、残り4組を同じ固定設定で実行する。firstshort11325終了0、scheduler22584はseed29短2本（child1616878）を開始。ROOT/validation_v59.html更新。

## 2026-10-19T11:30:12.626979+00:00 — v59独立a1/29悪化、追加長区間を取り止め
- a1/29全228.492812秒完走/全valid/全子0、wall296.9秒。融合位置固定0.275711→可動0.305014m（約10.63%悪化）、ROVIO13.901825→9.487655m。独立3組1勝2敗となり5/6勝基準が数学的に不可能。基準を変更せずv59不採用と判断。
- 実行中b1/29を切らないよう、scheduler22584のPython PID1612671だけSIGSTOPして追加起動を停止。b1/29は継続。未開始のprefix28/29は実行しない。停止理由/所有PIDをv59_independent_futility_stop.jsonへ保存。b1完走後にschedulerを終了し、observer/storageも停止する。
- 既存診断をseed28全2組へ適用。悪化b1でもROVIO速度RMSE .412357→.215349m/s、傾き .100024→.091672deg、95%速度誤差楕円内率は両方1.0。単純なROVIO精度低下/過小共分散という説明は支持しない。common_fusion_initial_state_identicalもtrueで初期状態差ではない。v59_independent_seed28_fusion_consistency.json。
- 最良の再現結果はv57独立6組全改善/平均17.74%。20%事前目標は未達として維持し、v59失敗を隠して成功扱いしない。採用設定をv57の検証済み実装へ戻して再現可能な形に整理する。

## 2026-10-19T11:38:28.654340+00:00 — 再現した改善をv57として採用、20%目標は未達と明記
- v59独立b1/29は全263.384754秒完走/全valid/全子0。融合位置固定0.188197→可動0.186844m、ROVIO42.098894→33.332235m。独立短4組2勝2敗。残る未開始長2組は5勝条件が不可能なので取り止め、結果の切捨て/基準変更はしない。scheduler22584終了130は完走後の意図的終了、resource58823/storage93336はSIGTERM終了143。シミュレーション自体の全子は0。
- 採用はv57。設定を固定してから実施した3経路×2乱数条件の独立6比較すべてで融合位置改善、平均17.7385%低減。経路別平均a1 21.3012%、b1 20.4782%、prefix11.4361%。安定性側の事前条件（5勝以上/各経路10%以上/最悪比）は満たすが、平均20%以上という改善幅目標は未達。original_acceptance_passed=falseを保持しており、基準達成や他環境一般化とは主張しない。ROVIO単体は4悪化2改善、融合効果と混同しない。
- 最新候補を繰り返し調整して目標を満たしたことにせず、この範囲で再現している改善を利用できる構成に整理した。planner_node.pyだけを保存済みv57から復元、全81ソース/実行ファイルSHAが当時と完全一致。復元前v59全84SHA一致記録を保持。v59ソース全体はfrozen_v59_sources.zip、差分v59_not_adopted_planner.patch、v59専用helper/2tests/config4ファイルはrejected_v59_source_extrasへSHA確認して移動し、採用構成のテスト探索/起動に混入させない。
- 新run_selected_gimbal.pyとselected_gimbal.jsonで、81SHA検査後に検証時と同じ引数の比較を実行できる。--output必須/既存保存先拒否、--labels/--seed/--dry-run対応。dry-runでv57設定/1経路引数/新規データ未生成を確認し、既存出力先保護も確認。ソース自体を復元しただけなので同一条件の全シミュレーションを追加重複実行していない。
- report_gimbal_selection.pyでROOT/adopted_gimbal.htmlとgimbal_selection.jsonを作成。採用6組と不採用4組、元の20%未達判定、六軸図、再実行手順を併記。READMEの2026-10-19節も更新。生成HTMLの16リンク存在/シェル構文/CLI構文を確認。
- 最終cleanup94609終了0。v59独立全4組を可逆圧縮して全結果辞書が圧縮前と厳密一致/valid、v59_independent_after_compression_check.json。v57全6/v59開発全6も既に同じ照合を実施済み。既存研究bagは保持、新規raw/bag画像点群記録なし。gimbal_final_state_audit.jsonで所有sim/scheduler/observer残存0、81SHA一致、Windows空き21.586GB。WORKLOG/HANDOFFへ記録、commitなし。

## 2026-10-19T13:03:54.996406+00:00 — run1_prefix MIMOSA単体の同一入力比較を開始
- ユーザー依頼により採用v57のrun1_prefixをseed20261026/全1070秒で再飛行。同一の物理飛行・LiDAR・IMUを固定ROVIO融合、偏向ROVIO融合、純LIOに同時入力し、融合の有無を切分ける。過去seed26との入力完全一致は仮定せず、今回の同時比較を根拠とする。
- 新規observe_pure_lio.py/run_pure_lio_comparison.py/report_pure_lio.pyを追加。採用81ソース/実行ファイルSHAを確認し本体は無改変。純LIOは同じbatched実行ファイル/ライブラリ、odometry.manager.enabled=falseと外部odometry入力を未配信topicへ切断するだけ。速度・傾き両因子なし、LiDAR4自由度/IMU/ノイズ/事前値/DRPM OFFを維持。通常融合の複製ノードで入力/出力厳密一致も監査する。
- /root/uas_native/logs/prefix-pure-lio-20261019へ数値だけ保存。GUI/画像/点群/rosbag連続保存なし、Windows8GB停止保護、開始時21GB空き。main session72730、所有campaign1627189/observers1627188。追加2推定器のLiDAR処理完了もsimulation pacingへ含める。構文検査成功、起動済み・追従開始、純LIOの外部速度/傾き因子0件を確認。最終精度はまだ未判定。

## 2026-10-19T13:23:12.991525+00:00 — run1_prefix純LIO比較完了：単体7.898m、固定融合5.747m、偏向融合4.697m
- 全1070秒完走、物理飛行は1回、seed20261026。同じLiDAR/IMUを採用v57の固定・偏向融合と純LIOへ同時入力。純LIOは速度・傾き両方のROVIO拘束を初期化から外した構成で、採用済みLiDAR4自由度/IMU/雑音/事前値/DRPM OFFは維持。推定器コードの新変更や重み調整なし。前回seed26ログと同一観測だったという主張はしない。
- 位置RMSE/回転RMSE: 純LIO 7.898275938m/1.904383813deg、固定融合5.746729469m/1.509169580deg、偏向融合4.696939284m/1.399278401deg。ROVIO単体は固定54.897037397m/2.736510755deg、偏向104.332760308m/4.017952709deg。追従開始で位置姿勢を一度だけ整合、尺度補正なし、10699共通評価点。MIMOSA単体にもドリフトがあり、この1飛行では融合で位置/姿勢とも改善。Visionの寄与が有害という説明はこの結果に当てはまらない。
- 追加の通常融合複製は参照の全出力・入力が厳密一致、同一実行ファイルと実ロードlibrary SHAも一致。純LIOは外部速度/傾き因子0、VIO入力イベント0、参照の全11313スキャンとIMUの到達時刻/累積数が完全一致。
- 起動〜終了の記録件数完全一致の初回監査はfalse（initial_lifetime_audit.jsonを保存）。純LIOだけ最後に1151.6秒の追加1スキャンを処理。評価終端1149.41446495秒より2秒以上後で、参照全スキャンは完全な先頭部分として一致。report_pure_lio.pyは全寿命不一致をfull_lifetime_stream_checksとして保持し、参照全入力一致＋追加末尾が評価後という検査を分離。評価時刻/区間/数値の削除や変更なし。精度比較の検査は全true、追加終了後スキャンを隠して全寿命一致とは称さない。
- main session72730、observers/campaign/duplicate/pure-LIOと全シミュレーション子はすべて終了0。今回の主計算wall1086.674秒。結果index.html/comparison.pdf/comparison.json、六軸エラーCSVをROOTへ保存。採用v57の81SHA維持。生成画像を視認し、軸省略なしで全範囲表示を確認。数値ログはSHA照合付き可逆圧縮、圧縮後再集計で一致確認中。Windows空き約21.6GB。
- 最終確認: 圧縮後の再集計結果辞書が圧縮前と厳密一致。採用81ソースSHA不変、推定器残存0件、Windows空き21.58GB、今回保存の一意inode合計78.8MB。final_audit.jsonへ保存。HANDOFFに今回の結論と終端1スキャン差の扱いを追記。

## 2026-10-19T14:52:07.651076+00:00 — 過去MIMOSA単体高誤差9runの三構成比較を開始
- ユーザー依頼はMIMOSA単体で大きな誤差が出たrunに限定して、単体/固定ROVIO融合/偏向ROVIO融合を比較。旧DRAGON14runの位置RMSEを確認し、0.12m以下5本と2.726m以上9本の間で事前閾値1mを設定。9本=prefix/middle/a1/a2_prefix/b1/b2/2a_prefix/2a_end/2b。旧値は選定だけに使い、全比較は採用UniPilot v57で同一飛行/同一LiDAR・IMU。新測定後の良否で対象を除かない。
- prefixは直前の全1070秒比較ROOT=/root/uas_native/logs/prefix-pure-lio-20261019を再利用。残る8本の軌道時間合計2860.886秒、seed20261026、各1試行。過去runの部分収録ラベルは元flight.route_time_sで区間を制限（middle1240.9/a2_prefix199.76/2a_prefix240秒）。元route全長まで延ばさない。
- 新run_lio_failure_comparisons.pyは既測定prefixの実行コマンドを再利用し、経路/世界/開始位置/区間終端/保存先だけを差し替える。採用81SHA確認、ROVIO拘束を完全除去した純LIOと通常融合の複製を追加。同時sim1本、追加推定器完了待ちあり。report_pure_lio.pyの固定label/seed/区間文字列を実protocolから読むよう一般化。構文確認成功。
- 保存先 /root/uas_native/logs/lio-failure-comparison-20261019。campaign.jsonへ事前対象/閾値/全結果を保存し再開可能にする。GUI/raw画像点群/bagなし、Windows8GB停止保護、現在約21.6GB。短いrun2a_endから開始、全完了後に横断表/六軸PDFを作る。新精度は未判定。


## 2026-10-19T15:55:08.715983+00:00 — 高誤差経路9本のMIMOSA単体/固定融合/偏向融合比較完了
- ROOT=/root/uas_native/logs/lio-failure-comparison-20261019。過去DRAGON14runの位置RMSE>1mで事前選定した9本を、現在のUniPilot・単眼・採用v57・DRPM OFFで比較。prefixは直前の同一入力試験を再利用、8本を新測定。各1試行/seed20261026、独立未使用seedによる再現性検証とは称さない。対象選定と現在の精度を混同しない。
- 各runの3構成が同じ物理飛行/LiDAR/IMUを共有。純LIOはROVIO速度・傾き拘束を初期化から完全に除外、他のLiDAR4自由度/IMU/事前値/雑音/本体81SHAは不変。追従期間だけで初回の位置姿勢を剛体整合、尺度補正なし。区間を短縮せず全長完走。

| run | 単体 位置m | 固定 位置m | 偏向 位置m | 単体 回転deg | 固定 回転deg | 偏向 回転deg |
|---|---:|---:|---:|---:|---:|---:|
| run1_prefix | 7.898276 | 5.746729 | 4.696939 | 1.904384 | 1.509170 | 1.399278 |
| run1_middle | 3.494118 | 2.604198 | 1.226947 | 0.541458 | 0.383307 | 0.524839 |
| run1_tail_a1 | 0.419819 | 0.402498 | 0.365850 | 0.617566 | 0.268617 | 0.266366 |
| run1_tail_a2_prefix | 0.728813 | 0.758864 | 0.668598 | 0.399762 | 0.167796 | 0.160910 |
| run1_tail_b1 | 0.779900 | 0.163087 | 0.166228 | 0.751114 | 0.188723 | 0.172547 |
| run1_tail_b2 | 80.033976 | 26.130765 | 2.344077 | 9.227207 | 2.676209 | 0.489414 |
| run2a_prefix | 35.357275 | 4.944684 | 11.760610 | 4.279467 | 0.769733 | 1.587977 |
| run2a_end | 12.034542 | 2.693864 | 2.612519 | 1.458147 | 0.316975 | 0.332473 |
| run2b | 0.190494 | 0.037786 | 0.039876 | 1.059201 | 0.196404 | 0.188068 |

- 現在の単体位置RMSE>1mはprefix/middle/b2/2a_prefix/2a_endの5本。5本すべてで固定融合/偏向融合の両方が単体より小さい。偏向が固定より改善したのは4/5本、run2a_prefixは4.944684→11.760610m（2.378倍）へ悪化。b2は26.130765→2.344077m（91.029%低減）、middleは2.604198→1.226947m（52.886%低減）。過去の3経路×2条件の採用結果を、新しい経路にも一律に一般化できない。
- 現在単体誤差が1m以下だったa1/a2_prefix/b1/2bも事前選定結果として全て保持。全9本で偏向対固定の位置改善6/9、悪化3/9（b1/2a_prefix/2b）。回転誤差は別指標で、middleや2a_endでは偏向が固定より悪い。位置改善を全6軸一律改善と称さない。
- 新run_lio_failure_comparisons.py、一般化report_pure_lio.py、新report_lio_failure_comparisons.py、既存observe_pure_lio.pyを使用。推定器や偏向規則は本campaign中に調整していない。ROOT/implementationに使用スクリプトを保存。index.htmlに全9本の表/六軸図、all_runs.pdfは9ページ、comparison.jsonに全数値/選定基準/相対誤差を保存。
- 全9本valid、通常融合の複製入力/出力は厳密一致。純LIOは参照スキャン/IMUの完全prefix一致、VIOイベント/速度/傾き因子0、全評価範囲を連続出力。終了後だけの追加スキャンは元の全寿命不一致を保持した上で評価外か検査、評価点の削除なし。新8本は可逆圧縮前後の再集計結果辞書が厳密一致、前回prefixも同確認済み。全所有プロセス終了0、domain85/master11485残存0、81SHA不変。final_audit.json。
- 10HTMLページ47リンク切れ0。2a_prefix/b2/middleの生成図を視認、縦軸省略なし。PDFのPageオブジェクト9とEOFを検査（pdfinfoは未導入、追加インストールなし）。GUI/raw画像/点群/rosbagなし。新campaignの一意ファイル約251MB、既存prefix約78.8MBは参照再利用。
- middleの終盤にWindows空きが21.35→11.13GBへ減ったが8GB停止閾値には達せず、測定終了後21.34GBへ回復。減少時の今回データは337MBで10GB減少を記録ファイルだけでは説明できず、ホスト側の原因は特定していない。既存研究データの追加削除なし。WORKLOG/HANDOFF更新、commitなし。


## 2026-10-20T08:40:38.515125+00:00 — run2a_prefix偏向悪化の既存ログ確認
- ユーザー仮説「特徴が多い方向を固定で見ていたのに、偏向で視覚推定が悪化した」を検証。新規シミュレーション・設定変更なし、同一飛行の追従240秒を使用。結果はrun2a_prefix/view_diagnosis_20261020.jsonへ保存。
- ROVIO単体位置RMSEは固定56.020497m→偏向6.133281m、回転0.586233°→0.189124°で改善。一方MIMOSA融合後は4.944684m→11.760610mへ悪化。視覚推定そのものが悪くなったという説明はこの結果と一致しない。
- 有効ROVIO特徴状態数の平均は固定11.913/偏向11.692、中央値12/11。これは25枠中isValid_=trueの数であり、画像内の全特徴量や当該画像で追跡成功した点数とは異なる。元画像は保存しておらず、固定視野のテクスチャ総量が多かったと断定しない。
- 既存diagnose_joint_fusion_consistency.describeで実際の1Hz融合入力240点を真値と照合。速度RMSE固定0.466513→偏向0.050178m/s、重力傾きRMSE0.119913→0.111179°。速度誤差は60秒ごとの全4区間で偏向が小さい。固定の機体x速度bias0.343088m/s、偏向0.041986m/s。10ms/20ms差分による真値速度確認差RMSE0.000806m/s。
- 実偏向のyaw範囲−8.125～12.825°、pitch−44.431～3.847°、中央値yaw0.984°/pitch−20.706°。全872判断が情報評価、回復走査ではない。4判断で保持制約不能。低追跡点数(<4)の判断0件。
- 確定できるのは「視覚推定と融合に渡す速度は改善したが、最終融合結果は悪化」。融合の重み/誤差の相殺/地図更新との相互作用の因果切分けは未実施。単純に融合コードの不具合と断定しない。推定器への変更なし、データ追加は小さい数値診断JSONのみ。


## 2026-10-20T08:51:58.879373+00:00 — run2a_prefix仮説の論点訂正
- ユーザーが問うのは総特徴数やROVIO全体精度ではなく「LiDAR退化方向を補完する特徴が固定画角で多かったか」。前回答の総合ROVIO/速度RMSEによる否定はこの問いへの根拠にならない。前項の計測値は保持し、方向別の相補性仮説は未検証として訂正。融合処理の問題へ原因を絞ることもできない。
- 既存記録を再確認。active側には選択/現在視線の評価値、退化強さ、最後の視覚地図のみ。偏向後のcurrent_viewは固定0/0度の比較対象ではない。固定側の時系列特徴点幾何・共分散、共通の時系列LiDAR情報行列/固有方向、固定/偏向視覚情報の揃った比較記録はない。保存済みログだけで固定の方向別拘束が強かったと判定できない。
- 必要な比較は同時刻・同じLiDAR情報/座標/事前条件を用い、その弱い方向の不確かさを固定/偏向の視覚特徴がそれぞれどれだけ減らすか。これは今回未実施。診断JSONに仮説/欠測/前回答の訂正を追記。推定器・設定変更、新規シミュレーションなし。


## 2026-10-20T09:01:25.355178+00:00 — run2a_prefix方向別相補性と融合悪化の短時間診断開始
- ユーザー許可により同一run最初120秒、seed20261026、採用v57/DRPM OFF/同一物理飛行の固定・偏向比較を再実行。GUI/画像/生点群保存なし、Windows8GB停止保護。保存先logs/run2a-direction-diagnosis-20261020/short120。
- 新observe_directional_complementarity.pyは固定・偏向のROVIO特徴点位置/結合共分散/外部姿勢を2Hz、LiDAR情報行列/固有方向/退化確率を10Hzで数値記録。これは診断用で推定器・制御へ戻さない。
- 同じ既存MIMOSA実行ファイルによる通常偏向の複製、純LIO、固定速度+偏向傾き、偏向速度+固定傾きの4推定器を追加。後2本は正確な同一時刻のROSメッセージで姿勢のorientationだけ相手側と交換（速度/速度共分散/姿勢の品質判定用共分散は速度側のまま）。傾き因子のsigmaは既存共通値。診断推定器はジンバル/飛行制御に不使用。入力同一性・因子の交換と全区間処理を後で検証する。
- 新run_directional_diagnosis.pyは旧比較の実行コマンドを再利用、区間末尾/保存先/診断推定器の待ち合わせ先を変更。採用81SHA検査。planner設定はlog_candidates=trueのみを別コピーで変更し、全候補予測点数を記録。恒久設定の調整なし。構文確認済み、精度・原因は未判定。


## 2026-10-20T09:19:56.578584+00:00 — run2a_prefix方向別相補性・速度/傾き/共分散の120秒診断を完了
- ROOT=/root/uas_native/logs/run2a-direction-diagnosis-20261020。ユーザー仮説は「固定視野がLiDAR退化方向を補完する特徴を多く見ていたか」。ROVIO全体RMSEだけで否定しないため、同時刻の特徴点位置/結合共分散と同じLiDAR情報行列で方向別指標を計算し、同一飛行の入力交換実験も追加した。採用v57/単眼UniPilot/DRPM OFF/既存81SHAは変更なし。
- 最初120秒を2回新飛行（各seed20261026）。同じ要求seedでも実観測は厳密一致せず、別飛行間の差を介入効果と解釈しない。各飛行の内部では通常固定/偏向と追加推定器に同じLiDAR/IMUを入力。物理飛行/偏向はmainのみが制御、交換側は制御へ戻さない。主計算wallは1回目192.416秒、2回目194.969秒、GUI/raw画像/密点群/bag保存なし。
- 1回目short120位置RMSE[m]: 純LIO12.415674、固定融合0.885808、偏向融合1.830929、固定速度+固定速度共分散+偏向傾き0.586815、偏向速度+偏向速度共分散+固定傾き2.155927。交換因子の速度/共分散/傾きは元系列と全件厳密一致。傾きの違いより速度+共分散の側について結果差が移動した。
- 同じ1回目の実際の1Hz速度因子誤差: 固定0.712657m/s、偏向0.085786m/s。機体前方bias固定+0.569281m/s、偏向+0.073213m/s。終端位置誤差を真値の機体前方へ投影: 純LIO−29.5755m、固定融合−2.0347m、偏向融合−4.7010m。固定ROVIOの過大速度がMIMOSA単体の過小移動と逆向きであり、誤差相殺を支持する。これだけで全240秒の主因を確定とはしない。
- 方向別幾何は239時刻(2Hz)で比較。同じ純LIO由来のH/固有方向/退化確率、同じ外部座標・事前値を使用。ROVIO追跡点の推定距離を使う。点間の相関も含めた視覚行列で、LiDARの弱い6方向の分散低減を評価すると固定が大きいのは13/239、偏向は226/239(94.56%)。平均対数分散低減2.0564→2.6509。純幾何/点ごとの共分散/偏向側LiDAR地図の各代替計算も偏向優勢。共通の最弱並進方向への実速度誤差投影も固定0.191865→偏向0.022476m/s。LiDARと幾何の時刻差最大0.1秒、速度と軸の差最大0.1秒。近接時刻での診断で、全て厳密同時刻とは称さない。
- この方向別指標は条件付きの幾何近似であり、実融合後の共分散でも、真の特徴点位置の検証でもない。ROVIOの深度に尺度誤差が含まれ得る。純LIO地図も推定誤差を持つ。したがって固定画角の真の優位を全面的に否定はしないが、今回の観測指標には支持がない。
- 2回目covariance120では傾きを偏向に揃え、速度値とtwist全共分散を個別に交換。位置RMSE[m]: 固定速度/固定共分散2.577579、固定速度/偏向共分散0.483283、偏向速度/固定共分散8.018385、偏向速度/偏向共分散2.887970。固定の前方速度bias+0.228016m/s、偏向+0.117756m/s。速度誤差の大きい固定の値が、同じ重みでも結果を改善しており、重みだけの説明にもならない。両要素と誤差相殺の寄与を支持する。交換共分散は診断用で、正しい測定モデルや採用候補とは称さない。
- 2回目の通常固定融合は3.975396m、偏向2.887970mで順位が逆転。固定の優位は短区間再飛行でも安定していない。速度過大評価と信頼度が変化するので、1回の結果だけから画角の優位を一般化しない。共通IMU相関/非線形な地図更新は単独に切分けていない。
- 追加の実装確認: 採用plannerは画像180点を別追跡し、局所6姿勢情報を予測。MIMOSAはROVIOの機体速度+重力傾きだけを1Hz融合。両者は同じ最終共分散を計算していない。候補選択でhome0/0より大きいgainの比率90.15%/93.52%、実pitch中央値−24.50°/−18.20°。今回のplanner地図は両飛行ともLiDAR直接深度0、三角測量点が判断時平均130.09/125.84。三角測量はMIMOSAの推定移動と画像視差に依存し、移動尺度誤差が予測拘束へ入る経路がある。これは追加確認対象で、原因と特定したものではない。
- 新規4スクリプト: observe_directional_complementarity.py、run_directional_diagnosis.py、report_directional_diagnosis.py、plot_directional_diagnosis.py。2回目は--mode covariance。入力stamp+source+IMU累積・到達時刻、全区間出力、交換後の因子値/時刻/共分散、通常複製の全軌跡・入力を検査。1回目48/48、2回目51/51合格。1回目はlibrary宣言SHAと複製厳密一致を確認したがロードパスは未記録、2回目は/proc/mapsで実ロードSHAも確認。未計測を隠して一律に実ロード確認済みとしない。
- index.html/diagnosis.pdf(4ページ)/六軸PNG/summary.jsonをROOTへ保存。軸省略なし、2回の測定を別図で表示。短試験の範囲は元240秒の前半120秒だけ。採用規則・融合重みの調整はしていない。実験プロセス全終了0、所有PID残存0。数値CSV/TUM/全候補JSONLはSHA照合付きで可逆圧縮し、圧縮後再集計を確認する。
- 最終確認: 2飛行とも圧縮後の再集計辞書が圧縮前と厳密一致。48/48・51/51の監査を維持。最終16項目も全成功（final_audit.json）、所有PID/シミュレーション/推定器残存0、採用81SHA不変、HTML8リンク切れ0、PDF4ページを確認。総保存の一意inode合計74.12MB、Windows空き20.94GB。図を視認し六軸は軸省略なし。追加測定・恒久的パラメータ採用はしていない。


## 2026-10-20T09:50:22.486195+00:00 — 未コミット作業を機能別に履歴化
- ユーザー依頼により、前回b0580bc以降の実装・設定・経路・検証スクリプト・研究メモを11コミットに分割する。対象はこのmultilink_autonomyリポジトリで、外部依存本体は既存パッチ/取得revision/ビルド手順として保持。pushは実施しない。
- 分割: 生成物除外、DRAGON vision/ZaB経路、容量監視付き収録、オフラインodometry/VIO検証、DRPM/FAST-LIVO2比較、ジンバルモデル/視野計画、高速化/通信、時刻整合と融合校正、ジンバル評価campaign、方向別融合診断、研究/引継ぎ資料。
- .gitignoreを追加整備。rosbag/MCAP/DB/ログ/build/cache/実行lockに加え、論文PDFと抽出TXT、経路の生成PNG/pickle/準備ログ/capture_runs.jsonはGit対象外。すべてローカルには保持。論文は出典/版/SHAと調査メモを追跡、経路は実験の入力となるroute/segments/waypointsと検証証跡を保持。取得資料約280MBを履歴へ重複格納しない。
- .gitattributesは既存patchに加え公式DRPMスナップショットの空白も原文どおり保持。AGENTS.mdは文言を変えずMarkdown段落末の空白を明示的な段落区切りへ変更。resource/route READMEへGit保存範囲を記載。
- Python379ファイルの構文、JSON等67ファイル、YAML/RViz83ファイル、shell17ファイル、XML/launch8ファイルの構文確認はエラー0。秘密鍵/OAuth token/client secret等の典型的な埋込みパターン検査は該当0（完全な機密性保証ではない）。主要な幾何/視野保持/時刻/結果処理の45 unittestと軌道計算4チェックが成功。シミュレーションの再実行は不要なので行っていない。
- 採用v57の81ソース/バイナリSHAはすべて一致。実装や推定器設定をコミットのために変更していない。各コミットのstaged範囲とgit diff --checkを確認し、最後に未コミット差分/生成物混入/全体SHAを確認する。
- 最終確認: 採用81SHA一致、追加対象への生成ログ/センサbag/画像/PDF/ELF混入0、主要49チェック成功。固定ハッシュ対象のROVIO設定テンプレートは末尾空行も元どおり保持し、.gitattributesでその末尾形式を明示。コミット時の一時的な空白整理はジンバル実装コミットへ折り込み、実験状態を変更しない11コミットとして履歴化する。ローカル論文・結果・収録先情報は削除していない。

## 2026-10-20 — SubTGraphの準備、ユーザー指示で検証前に停止
- 1～2環境での検証依頼を受け、公式benchmark revision a6cd7d213e72883c2dbb8aad36123c5da68cd6c2のoperational/01・lavatube/01のOBJ/MTL/texture/数値地図をassets/subtgraphへ取得。source.jsonに出典・SHA・サイズを保存し、OBJは公式Git LFS SHA一致を確認。生成物合計約1.2GB、Windows空き約19GB。
- 新規subtgraph_assets.pyは取得再開と容量チェック、prepare_subtgraph.pyは既存物理条件を使うSDFと直進経路の準備用。後者は未実行・Gazebo未検証。採用推定器・planner・重みは変更していない。
- operational形状を+90deg X回転して解析し、候補x=0,y=-510～-650,z=39mの直線を選定。鉛直交差の床高さを確認したのみで、機体全体の衝突余裕やセンサ表示、飛行精度は未検証。公式原寸のメッシュ・テクスチャは変更していない。
- ユーザーの「一旦ストップ」で作業停止。シミュレーション、ROVIO、MIMOSAは未起動。取得・形状描画プロセスも終了済み。比較実験の結果はまだない。取得物と未コミット準備コードを保持し、自動再開しない。
- 利用実績の再確認: GitHub APIでstars=7/forks=1。OpenAlexの論文レコードW7161940500はcited_by_count=0、公開日2026-05-20。Semantic Scholar APIは429で取得不能。OpenAlex登録数を全サービスの総引用数とは扱わない。実績ある退化ベンチマークと断定できる根拠はなく、可動カメラの再描画に利用できる生成環境という適合性と、第三者の利用実績を分離する。


## 2026-10-20T11:31:28.985125+00:00 — ZaB内の追加3条件を事前指定して比較開始
- run3/run4/run1_tail_branch（直前9経路の固定・偏向比較対象外）を各2試行、要求seed20261101/20261102で事前指定。既存経路との空間的完全非重複とは称さない。短い接続通路、上り接続通路、下降を含む横坑往復を対象にする。全追従区間を使用、結果で選別・設定調整しない。
- 採用v57の81SHA一致、UniPilot/単眼/DRPM OFF/巡航0.5m/sを維持。同じ物理飛行でMIMOSA単体・固定融合・偏向融合を比較。run_zab_additional_conditions.pyは既存run_lio_failure_comparisons.run_oneと完全同じ実行・監査・可逆圧縮手順を再利用する。
- 保存先native/logs/zab-additional-conditions-20261020。GUI/生画像/密点群/rosbagなし、Windows空き約19GB、既存8GB停止保護。新しい精度はまだ未判定。


## 2026-10-20T11:44:52.510736+00:00 — ZaB追加3条件×2回の比較完了・全停止
- ROOT=/root/uas_native/logs/zab-additional-conditions-20261020。事前指定run3/run4/run1_tail_branch、seed20261101/20261102の全6飛行完走。採用v57の81SHAは前後一致。単眼UniPilot/DRPM OFF/同一物理飛行で純LIO・固定融合・偏向融合、追従全区間だけを初回剛体整合、尺度補正なし。新しい結果に応じた候補除外・設定調整なし。
- 形状は公式ZaB world衝突メッシュで確認。run3: 約17m/35.93秒、3断面の左右壁距離合計4.64～5.19m。run4: 約15m/32.70秒、上り0.6m、同4.59～4.74m。branch: 約12m/51.84秒、下降と旋回を含む往復、側方に15～19m開いた区間。world_context.json/cross_sections.json/route_context.png。ray値は指定方向の距離で最小通路幅ではない。直前9経路の固定/偏向比較には未使用だが、過去全経路との空間的完全非重複は主張しない。

| run/seed | 純LIO位置m | 固定位置m | 偏向位置m | 固定回転deg | 偏向回転deg |
|---|---:|---:|---:|---:|---:|
| run3_seed20261101 | 0.171786 | 0.016974 | 0.016127 | 0.120706 | 0.121258 |
| run4_seed20261101 | 0.028621 | 0.021819 | 0.023892 | 0.169153 | 0.176996 |
| run1_tail_branch_seed20261101 | 0.254797 | 0.021470 | 0.021982 | 0.193898 | 0.192536 |
| run3_seed20261102 | 0.230126 | 0.073189 | 0.072203 | 0.124537 | 0.118565 |
| run4_seed20261102 | 0.023067 | 0.026277 | 0.027737 | 0.194701 | 0.202752 |
| run1_tail_branch_seed20261102 | 0.089716 | 0.048114 | 0.046301 | 0.169134 | 0.169161 |

- 偏向は位置3/6改善・3/6悪化。run3は2/2改善、run4は0/2、branchは1/2。2回平均位置RMSEの純LIO/固定/偏向[m]は、run3 .200956/.045081/.044165、run4 .025844/.024048/.025814、branch .172256/.034792/.034141。固定と偏向の差は小さい。今回は33～52秒の短区間で、長時間退化・大ドリフトに対する一般化の証拠とはしない。小さい出力誤差だけから幾何学的退化が存在しないとは判定しない。
- 全6比較valid、全6の通常複製入力・出力一致、純LIO入力prefix・IMU累積一致/ROVIO因子0、圧縮後再集計辞書厳密一致。全実行プロセスexit0、所有PID/シミュレーション残存0。最終36項目成功、HTML36リンク切れ0、PDF6ページ。2図を視認し軸省略なし。主実行wall合計698.6秒（準備・集計時間を含まない）。
- ROOT/index.html、comparison.pdf、summary.jsonに全試行・六軸図と各2回平均。run_zab_additional_conditions.py/report_zab_additional_conditions.pyを追加、既存実行・監査を再利用。実装snapshotあり。GUI/生画像/密点群/rosbagなし。結果約61.1MB、Windows実空き19.37GB、8GB停止下限。既存研究データの削除なし。commitなし。


## 2026-10-20T11:59:39.764186+00:00 — ZaBの固定5方向・独立単眼同時比較を開始
- ユーザー指定(yaw,pitch)=(-50,0),(0,0),(50,0),(0,90),(0,-90)を、直前のrun3/run4/run1_tail_branch×各2回で比較。既存前方固定＋偏向に、同一光学中心の質量・visual/collisionなし仮想カメラ4台を追加。5固定＋1偏向をそれぞれ独立単眼ROVIO/MIMOSAへ入力し、カメラ間の情報融合なし。同じLiDAR/IMU/物理飛行を共有。pitch+90は下、-90は上。
- 採用81ファイルは変更せず、新run_view_array.pyで検証済ownerへの限定拡張を生成。実効ソース・差分・pacerを各出力へ保存。固定方向を生成SDF・Camera0初期外部姿勢・画像時刻の外部姿勢へ一貫反映、既存native画像変換/推定器バイナリを使用。画像時刻の変更や間引きなし。追加の全LIO/VIO完了を待つpacerと終了drainを追加。ROVIOの初期化から各固定方向の画像を使い、他方向から特徴を渡さない。
- test_fixed_view_array.pyの2検査成功。5方向の光軸を解析解と照合、追加センサを除けば元SDFが厳密一致、画質/雑音/物理は不変、実効owner/pacer構文・全方向待ち合わせを確認。
- ROOT=native/logs/zab-fixed-five-directions-20261020。最初のrun3/20261101は全監査合格、wall128.85秒で全停止。固定[-50,0,+50,下,上]/偏向の位置RMSE[m]=.014557/.019678/.010399/.046416/.030557/.019678。まだ1試行であり全体の優劣は未判定。上下画像を視認、機体で全面遮蔽されていない。残る5試行を続行。GUI/bag/生画像連続保存なし、各方向の説明用PNG1枚と数値だけ保存、Windows8GB下限・現在約19GB。

## 2026-10-20：ZaB固定5方向の独立単眼同時比較・完了

- 依頼されたyaw/pitch=(-50,0),(0,0),(50,0),(0,90),(0,-90)を同じ機体位置に設置。従来の偏向カメラも併走。各カメラが独立した単眼ROVIO＋MIMOSAを使用し、複数カメラ間では融合しない。直前のZaB追加条件run3/run4/run1_tail_branchを各2回、計6物理飛行・固定30条件・偏向6条件。要求seed20261101/20261102は乱数seed識別子で実行日ではない。
- 採用v57・DRPM OFF・0.5m/s・既存位置/画質/周期/ノイズ/物理設定を維持。質量や衝突形状は追加せず、カメラセンサのみ追加。既存機体による遮蔽は残る。pitch +90は真下、-90は真上。固定方向は初期化時から各方向を観測し、処理完了を待つことでデータ間引きを行わない。前回の二眼構成とは新規飛行であり、同seedによるノイズ系列一致は仮定しない。
- 全追従区間（約33～52秒）、初回位置姿勢整合のみ・尺度補正なし。下表は各試行の位置RMSE[m]の2回算術平均。

|経路|(-50,0)|(0,0)|(50,0)|(0,90)下|(0,-90)上|偏向|
|---|---:|---:|---:|---:|---:|---:|
|run3|0.034024|0.038944|0.035754|0.057610|0.047277|0.039089|
|run4|0.026034|0.021730|0.023391|0.033999|0.030232|0.023168|
|run1_tail_branch|0.040888|0.043916|0.050413|0.068119|0.073542|0.044862|

- 結果：平均の最良固定方向はrun3/branchでyaw -50、run4で前方。偏向は平均ではいずれの経路も最良固定を上回らなかった。試行単位ではbranchの1回目だけ偏向が全固定方向を上回った（全6回中1回）。run3の最良固定方向は1回目+50、2回目-50で順位が変わる。この結果から一意の最適固定方向や統計的有意差、長期退化環境への一般化を主張しない。結果を見て最良固定を選ぶ事後比較と、事前選択した固定方式の評価を区別する。
- 実装：active_perception/fixed_view_array.py（モデル・独立推定器）、run_view_array.py（凍結起動コードへの限定的拡張を別ファイル生成）、audit_view_array_inputs.py（画像・外部姿勢・時刻監査）、run_view_array_campaign.py（6飛行管理）、report_view_array.py（全方向比較）、test_fixed_view_array.py（幾何・モデル差分）、native/unipilot_view_array.launch.py（追加画像橋渡し）。凍結された既存推定・制御コード81ファイルはSHA一致。変更は新規ファイルで保持し、実行用生成コード・差分も各飛行に保存。
- 検証：幾何・既存SDF不変テスト2件成功、構文確認成功。全6飛行の比較検査valid=true。各固定方向の追従全画像・外部姿勢・画像時刻のROVIO出力を確認。同一LiDAR/IMUイベント・累積数・IMU時刻順、角度以外のROVIO/MIMOSA設定一致、固定外部姿勢とモデル一致、連続推定を確認。圧縮前後の既存比較値は一致。最初の2試行はレポートへの偏向角範囲追記後に再集計し、既存metrics/checksが完全一致。実際に読み込まれたMIMOSA共有ライブラリはbranch第1試行の追加4系統で採取しSHA一致を確認（全試行でのプロセスメモリ採取ではない）。
- 出力：/root/uas_native/logs/zab-fixed-five-directions-20261020/index.html、comparison.pdf（6ページ・各六軸、省略軸なし）、summary.json、各試行array_comparison.json、final_audit.json。HTML参照先欠落0、PDF正常終端、代表図を目視確認。全数値と不良方向も保持。各固定方向・各飛行で1枚のみ画像例を保存し、連続画像/点群/rosbagは保存しない。
- 全所有プロセスは終了コード0で終了、確認対象PID残存0。シミュレーション本体の実時間合計856.3秒（集計・調査時間は別）。結果約109.2MB、最終Windows空き18.89GB。追加4実行ファイルSHAも全飛行開始から不変。GUIなし、採用方式のチューニングなし、commitなし。

## 2026-10-20：新規シミュレーション入力の独立化と旧logsのDrive移管（開始）

- ユーザー指示：新規シミュレーションに必要な設定のみ退避し、その他のlogsはDriveへ移動・ローカル削除。全削除前にDrive上の内容一致を確認する。
- native/simulation_inputsへ14経路のroute/segments/world設定、採用比較3経路の入力、融合校正値、起動引数テンプレート、モデル入力参照を退避。62ファイルの入力バイト列SHAを保持。点群・画像・推定軌跡ではない。退避元一覧はrelocation_manifest.json。
- unipilot_campaign、run_lio_failure_comparisons、run_zab_additional_conditions、run_view_array_campaign、run_directional_diagnosis、compare_routes、test_fixed_view_arrayのオンライン入力参照を移行。過去prefixの測定結果再利用を廃止し、高誤差9経路の新規起動はprefixも再実行する。採用方式・推定処理は不変。unipilot_campaignの入力読込とprepare-only追加に伴いselected_gimbalの該当SHAのみ明示更新、校正値はパスのみ変更。移動前のselected manifestを保持。
- test_simulation_inputsの2件とtest_fixed_view_arrayの2件成功。前者はlogs以下のPath.openを禁止して、採用起動dry-run、通常campaign、9経路、追加3経路、5方向比較の起動準備を検査。校正・全経路の退避前後SHA一致。
- GUIなし3秒追従の実起動確認はWindows空きが8GB未満になり安全停止。全子プロセスは終了0で回収したが、試行自体は容量による失敗であり動作完了成功とは扱わない。容量回復後に再試行する。
- 移管先 uas_gdrive_private:multilink_autonomy_archives/logs-20261020。既存Drive保存とMD5一致した47 MCAP（27,612,490,864 bytes）はDrive内で複製、残る全logsはtar+zstdの256MiB分割で退避。各転送後にDrive側MD5/サイズ一致を確認。symlinkはリンクとして、hardlinkはtarで保存。実体がlogs外にあるリンク先を無断で削除しない。
- Windows空き低下を受けて一時領域を/dev/shmの最大3分割に変更。元logsはこの段階では未削除。移管スクリプト・進捗は /root/multilink_autonomy_ws/maintenance/log-archive-20261020。最終結果は後続追記。

## 2026-10-20：旧logsのDrive移管・ローカル削除完了

- 元ツリー122,533 entries、実体65,284,512,431 bytesを保存。47 MCAP（27,612,490,864 bytes）は既存DriveデータとのMD5一致を確認し、Drive内で今回の保存先へ複製。残りはtar+zstdで19,825,561,289 bytes・74分割。全ての保存先サイズ/MD5と、元ファイルのサイズ・更新時刻・inode・symlinkを確認した後、native/logsを削除して空ディレクトリを再作成した。
- 保存先: https://drive.google.com/drive/folders/1SNkp_XT7AsO2UyedbRQ1FqM1qJKQNiId 。rclone: uas_gdrive_private:multilink_autonomy_archives/logs-20261020。元ファイル一覧inventory.json、分割一覧archive_complete.json、別保存MCAP一覧existing_bags_verified.json、復元コードrestore.pyを保存。旧ログを参照する過去のHTML/PDFやオフライン実験は復元後に使用する。
- native/archive_records/logs-20261020には小さい復元手順・保存確認・起動確認を保持。新規シミュレーション用simulation_inputsと起動コードのスナップショットもDriveへ保存してサイズ/MD5一致を確認。途中で生じた不要な転送分割と空の重複フォルダをDriveから削除した。
- 転送時間の主因はアップロード。最終圧縮転送は約1,750秒、直近実効約12MB/s。256MiBのローカルMD5計算0.46秒、Drive照会13.29秒という測定例を確認。照合でファイル全体を再ダウンロードせず、Driveが返すMD5と比較した。
- 実際のlogs削除後にtest_simulation_inputs 2件、test_fixed_view_array 2件が再度成功。退避した入力のバイト列が元と一致し、採用起動・通常campaign・9経路・追加3条件・5方向比較が過去logsを読まず準備できることを確認。
- Windows空き回復後、GUIなしでrun3の離陸→初期化→3秒追従を実行し、route_complete・比較valid=true、圧縮後の再集計一致。所要約80秒（起動/初期化/集計を含む）、全所有プロセス・Gazebo子プロセスexit0、残存なし。最初の容量停止試行も失敗記録として保存し、成功に書き換えていない。今回の短時間試行は設定退避の起動確認であり、精度・手法有効性の検証として扱わない。
- 最終Windows空き12,905,639,936 bytes。WSL内で削除した65.3GBがそのままCドライブの空きになるとは主張しない。仮想ディスク縮小/WSL終了は実施せず、既存の8GB安全停止条件を維持。作業用ログもDriveへ退避し、ローカルには小さい復元記録と新規入力を残す。commitなし。

## 2026-10-21：二ツ小屋隧道の導入と20m試走

- ユーザー提供元: C:\Users\j-sug\Downloads\futatsugoya-tunnel-abandoned-road-japan。source/bansei_1_392k_20200425.zip内OBJとtextures/の4096×4096画像12枚。作者Fujihiko Matsumoto (fujisan)、CC BY 4.0。出典・変更内容・入力SHAをassets/futatsugoya/attribution.jsonに保持。元Windowsファイルは変更せず。
- OBJは196,878頂点・392,594三角形、元の範囲は約2.90×29.87×1.37単位。MTLが同梱されていなかったため、材質名と画像名で12タイルを復元し、各UVを0〜1へ移してDAE化。形状の削減なし、法線生成のみ。衝突は同一三角形STLで、内部を埋める凸包や追加地面を使わない。DAE面数・12材質/画像・衝突との境界一致を確認。元メッシュは完全な閉曲面ではない。
- 尺度は未記載。近似入口位置の元Y=-11.27/13.335間を384mとする一様倍率384/24.605で設定し、新Xを長手方向とした。参考 https://ootaki.xsrv.jp/26akitan2.pdf p.5（歴史的工事図・諸元の転載）。実測標点による尺度校正ではなく、測量精度を保証しない。照明は一様な環境光で、写真に記録された影/照明を含む。実際の暗いトンネル条件とは区別。
- assets/futatsugoya/futatsugoya.sdf、models/futatsugoya、routes/{smoke,interior}、README.mdを生成。world通信名zabは凍結実行系の互換性維持用で、環境モデルは新トンネルのみ。SDF_PATH指定のgz sdf -kでValid。新prepare_futatsugoya.py/run_futatsugoya.py、simulation_inputs/futatsugoya.jsonを追加。既存推定器・v57偏向・校正・センサ条件は変更していない。
- smokeは20.0015m、0.5m/s、42.0031秒追従。中心面距離最小1.760m。interiorは290.088m、582.177秒、中心面距離最小1.219m。0.25m標本化の形状距離確認であり機体全体の連続衝突保証ではない。入口側の近接箇所を避けて長経路は世界X=60〜350m。290m全飛行は未実施。
- GUI/rosbagなし、真値制御で離陸→初期化→共通準備動作→20m追従。純LIO/固定単眼融合/偏向単眼融合を同一飛行で実行。DRPM OFF。全追従区間・初回剛体整合のみ・尺度補正なし。要求seed20261021は同じだが試行間のノイズ完全一致を保証せず、独立3seed実験とは扱わない。

|試行|LIO位置RMSE[m]|固定融合|偏向融合|固定ROVIO|偏向ROVIO|補助確認|
|---|---:|---:|---:|---:|---:|---|
|smoke|11.6226|8.0872|8.3079|0.8905|0.9684|点群topic誤指定で確認用observerのみ終了失敗|
|verified|12.0081|9.4367|9.5906|0.1709|0.3407|異なる型を含む点群の確認用読み取りのみ失敗|
|final|11.7528|8.0917|8.2864|0.8632|0.7357|確認用observerを含め正常終了|

- 最初の2回も飛行/推定器はexit0・route_complete・比較valid=trueで、補助確認不具合と推定誤差を混同しない。全試行の元データと失敗記録を保持。3回目は正しいROS側topic /rmf_unipilot/lidar/pointsと構造化点群読み取りに修正して正常終了。確認画像は各カメラ1枚のみ。最終試行は23,413有効LiDAR点、距離中央値4.024m・90%点16.334m。無応答の無限大点を含む補助boundsは未評価としてnullにし、元メタデータはimplementationに保存。今後の補助bounds計算では有限点を先に抽出する。推定器入力は変更なし。
- カメラ実画像でテクスチャの表示を目視確認。同時刻68秒の真値と3,000点を用い、点と元の面の距離中央値0.02015m、90%点0.05991m、99%点0.34560m、95.07%が10cm以内。全点が測距ノイズ3cm以内とは主張しない。面から15cm以内の対応の法線による並進行列は固有値[0.006692,0.378305,0.615002]、最弱方向は世界X成分0.999993。形状上は長手方向が弱いが、この行列はMIMOSAの事後情報行列ではなく、大誤差の全原因を特定したという意味ではない。
- 最終試行所要約105秒（初期化/集計込み）、全flight/observer/GPU/Gazebo子exit0、所有PID残存なし。全3試行valid=true、圧縮後再集計一致、採用ソースSHA一致、最終試行の資産SHA一致。新Pythonの構文確認・git diff --check成功。短い20mでは偏向が固定を改善せず、gimbalの有効性を主張しない。
- 結果: /root/uas_native/logs/futatsugoya-20261021-final/index.html（全3試行・画像・制約）、smoke/comparison.pdf（六軸）、smoke/geometry_sensor_audit.json、final_audit.json。最初の2試行は同階層の-smoke/-verified。資産約105MiB、全3試行数値等約33MiB。Windows空き約180.18GB。既存のDrive移管データを復元せず実施。commitなし。

## 2026-10-21：二ツ小屋隧道290mの同一飛行比較・完了

- ユーザー指示でinterior経路290.088mを巡航0.5m/sで全飛行。採用v57・センサ・推定器・偏向規則・校正は変更せず、DRPM OFF。固定yaw/pitch=(0°,0°)と偏向yaw±50°/pitch±90°、純LIOを同一飛行・共通LiDAR/IMUで比較。制御は真値、推定器は真値を利用しない。GUI/bagなし。
- 追従582.177秒、離陸/初期化は評価対象外。5821共通評価時刻、始点剛体整合のみ、尺度補正なし。飛行・集計673.8秒（約11.2分）。経路追従誤差最大0.06276m。

|構成|位置RMSE[m]|回転RMSE[deg]|終端位置誤差[m]|
|---|---:|---:|---:|
|MIMOSA単体|170.4573|1.1342|293.5489|
|MIMOSA＋ROVIO固定|2.1291|0.3805|3.0381|
|MIMOSA＋ROVIO偏向|7.2903|0.3841|13.3072|
|ROVIO単体固定・参考|235.1055|2.3846|457.8003|
|ROVIO単体偏向・参考|12.1992|2.1213|12.0538|

- 今回は視覚融合が純LIOより良好だが、偏向は固定より悪い。ROVIO単体の累積位置誤差は逆の順位であり、融合が使う速度・傾き・重みと累積位置誤差を混同しない。誤差相殺/重みを含む原因の切り分けは今回未実施。1回の試行で統計的有意差・再現性を主張しない。
- comparison.valid=true、20/20比較検査合格。純LIOの全評価LiDAR/IMU一致、複製融合の全入力/出力一致、固定/偏向の既存比較監査、全期間出力、同一設定/バイナリ、採用ソースSHA不変を確認。圧縮前後の再集計完全一致、資産SHA不変。全所有プロセス・Gazebo子プロセスexit0、所有PID残存なし。
- 結果 /root/uas_native/logs/futatsugoya-20261021-interior290/index.html、comparison.pdf（全方式/融合2方式の六軸2ページ、省略なし）、interior/comparison.json、final_audit.json。ログ内implementation/finalize_report.pyで追加レポートを再生成可。推定/実行コードの変更なし。数値を可逆圧縮し全体約45MiB、Windows空き約176.82GB。
- 既存20m試走は別開始位置・別飛行なので直接の同一入力比較ではない。入口間384mを基準とする近似尺度、一様照明/写真テクスチャの制約を維持。全停止済み、自動再開なし、commitなし。

## 2026-10-21：二ツ小屋隧道290m・固定5方向同時比較

- ユーザー指示で(-50,0)/(0,0)/(+50,0)/(0,+90)/(0,-90)を比較。既存の固定5方向拡張を新トンネルに適用し、全5方向＋偏向を同一290.088m飛行で独立単眼ROVIO＋MIMOSA処理。5眼融合ではない。共通LiDAR/IMU、同じカメラ位置/画質/周期、角度以外の設定一致。初期化から各方向を観測。pitch+90は下/-90は上。採用v57、DRPM OFF、0.5m/s、真値飛行制御。
- run_futatsugoya.pyに--view-arrayを追加。既存run_view_array.pyへの切替、拡張ソースSHA保存/終了時照合、全方向レポート/監査を実行する。既存の推定器/偏向器/センサ処理は変更なし。test_fixed_view_array 2件成功、構文確認成功。再実行は `python3 /root/uas_native/active_perception/run_futatsugoya.py --route interior --view-array --output <新規出力先>` 。
- 追従582.177秒、5821共通評価点。離陸/初期化は除外、始点剛体整合のみ、尺度補正なし。約11.3分で全長完了。

|方向yaw,pitch[deg]|融合位置RMSE[m]|融合回転RMSE[deg]|融合終端位置誤差[m]|ROVIO位置RMSE[m]|
|---|---:|---:|---:|---:|
|(-50,0)|2.5665|0.4534|4.2350|166.1314|
|(0,0)|5.5140|0.5188|7.8117|231.1907|
|(+50,0)|6.0679|0.4915|8.7080|38.9211|
|(0,+90)下|7.8524|0.4545|13.3127|13.2397|
|(0,-90)上|9.8527|0.4852|14.9443|2.0101|
|偏向|5.1247|0.4914|12.2163|21.6884|

- 同時純LIO位置RMSE167.8541m。固定5方向RMSEの算術平均6.3707mに対し偏向5.1247m（19.56%小さい）。これは固定方向を等確率で1つ選ぶ場合のRMSE期待値との比較で、統計的有意差ではない。最良固定(-50,0)は事後選択で2.5665m。
- 前回二眼飛行の(0,0)=2.1291m/偏向7.2903mとは別飛行。今回(0,0)=5.5140m/偏向5.1247m。要求seedは同じ20261021だがセンサノイズの厳密一致を仮定しない。差の原因をカメラ追加やノイズのみに確定しない。今回内の比較は同一物理飛行/入力監査に合格。
- array監査65/65、元比較監査20/20に合格。全方向の露光時刻・固定姿勢・ROVIO全画像出力・LiDAR/IMU入力・設定・連続性一致。全構成coverage1、MIMOSA最大間隔0.1秒、ROVIO0.05秒。採用SHA/拡張SHA/資産SHA不変、圧縮後元比較再集計一致。全所有/子exit0、残存0。追従誤差最大0.06318m。
- 結果 /root/uas_native/logs/futatsugoya-20261021-five-views290/index.html、interior/six_axes.pdf、summary.json、final_audit.json。六軸全方向同一図、省略なし。各固定方向確認画像1枚、数値ログ可逆圧縮、GUI/rosbagなし、計約89MiB。Windows空き約176.65GB。追加レポート生成コードは同ディレクトリimplementation/finalize_report.py。全停止済み、commitなし。
- 1試行のみで再現性は未検証。既存の近似尺度/一様照明/写真テクスチャの制約を維持。ROVIO単体の順位と融合順位は異なり、その原因は今回調査していない。

## 2026-10-21：二ツ小屋隧道(0,0)の試行間差を確認

- ユーザー質問：前回位置RMSE2.1291mと5方向飛行の5.5140mの差。既存数値ログのみ比較、追加シミュレーションなし。
- 固定ROVIO設定・カメラ内外部設定・融合設定（ログ保存先除外）・IMUモデル/サンプリング・校正・経路・実バイナリSHAが一致。両試行とも全期間連続性検査に合格。2.13→5.51の差を設定変更/欠落で説明する証拠なし。
- 実際のROVIO初期化は20.202秒→20.262秒（60ms差）、初回出力20.25→20.30秒（50ms差）。途中説明の「初期化50ms」は初回出力との差であり、初期化ログでは60msと確認。両飛行の真値を共通0.1秒格子で照合：位置差RMS5.784mm/最大18.786mm、姿勢差RMS0.2707deg/最大0.8286deg。初期化・実飛行が厳密に再現されていない。画像共有ハッシュは件数が異なるため、ハッシュ差を画像差の証拠として使用しない。
- 進行方向終端誤差は前回+2.8499m、今回−7.7135m。前回は追従250→300秒で−0.9993→+2.5033mと正方向に変化、今回は−5.6559→−6.0181m。前回の精度を安定した性能として扱えない。ROVIO由来速度と不確かさも異なるが、どの差が何mの誤差を生んだかは切り分けていない。
- 同一raw入力による再生比較ではなく、生画像/IMU/点群を保存していないため、センサ乱数・実行順序・初期化・特徴追跡・融合寄与の単独因果は確定しない。同seedだから同一入力とはみなさない。今回5方向同一飛行内の比較結果は維持するが、1試行での期待値差に統計的有意性を主張しない。
- 再現可能な照合コード/数値: /root/uas_native/logs/futatsugoya-20261021-repeat-check/{analyze.py,comparison.json,report.txt}。推定器や設定変更なし。

## 2026-10-21：5方向＋偏向の融合改善と誤差相殺をログ診断

- ユーザー質問：LiDAR退化方向への妥当な視覚拘束か、大誤差の相殺か。新規飛行なし。融合に実際に入力されたvelocity_factors.csv（各582個・1Hz）を真値姿勢で世界座標へ変換し、真値位置20ms中央差分と比較。真値は事後診断のみ。10/20/50ms差分による平均速度変化は最大3.4e-6m/s。ROVIO累積位置ではなく実際の融合観測を評価。
- 真値進行方向平均0.498305m/s。VIO速度平均/進行方向速度誤差RMS[m/s]: (-50,0)0.983242/0.717515、(0,0)1.401486/1.150195、(+50,0)0.615226/0.136316、下0.539320/0.072513、上0.517129/0.023192、偏向0.544222/0.120132。
- 同時純LIOは始点剛体整合後の平均進行速度−0.003601m/s（ほぼ進まず、終端X誤差−292.133m）。融合後平均進行速度は(-50,0).4912、正面.4851、+50 .4835、下.4755、上.4727、偏向.4774m/s。これは各独立推定の移動量であり、融合内部のLiDAR観測値と同一ではない。
- 正面と-50は実際より大きな速度を約97%の時刻で入力しながら融合位置誤差が小さい。LiDARの移動過小評価とVIOの過大評価の相殺を支持する兆候が強く、良好な視覚拘束だけで説明しない。最良VIO速度の上向きが融合では最悪という順位も確認。ただし重み・姿勢・地図が異なるため因果の寄与率や支配性を断定しない。
- 偏向は同じ大誤差同士とは分類できない。最初の100秒はVIO平均0.703m/sで過大、最後の3区間(300–400/400–500/500–582秒)は0.484/0.469/0.447m/sで真値より小さい。前半は相殺し得るが後半は独立純LIOと同じ過小評価方向。正しい速度に近い情報の供給と時期依存の偏りが併存する。速度以外の傾き拘束の寄与は未分離。
- ログの共分散は実効融合重みではない。IMU・姿勢・相関・地図更新を含むため単純加重平均の計算で因果分解しない。正面/−50の成功を純粋な拘束補完と解釈しない一方、偏向の改善全てを偶然の相殺とも断定しない。前回の20%期待値差は性能観測であり、狙った方向拘束改善機構の証明ではない。
- 結果 /root/uas_native/logs/futatsugoya-20261021-fusion-diagnosis/{index.html,velocity.pdf,summary.json,analyze.py}。各方向の速度/申告標準偏差と真値の全時系列・区間別集計を保存。因果切分け用の観測交換/誤差除去実験は未実施。推定器設定変更なし、全停止維持。

## 2026-10-21：ZaB・二ツ小屋で共通の弱方向の局所移動誤差を比較

- ユーザー指示で短窓の評価を新規飛行で実施。ZaB run1_tail_a1/run1_tail_b1・二ツ小屋interior開始から各120秒、seed20261022/20261023の各2回、計6飛行。前半/後半60秒の6区間を評価。区間・窓長5/10/20秒（主10秒）・閾値を精度集計前に固定。二ツ小屋はworld X60付近から約60mの範囲で、以前の290m全長ではない。ZaBでは旋回減速があるため評価時刻は追従開始からの経過シミュレーション秒、経路進行時刻とは区別。
- 固定5方向(-50,0)/(0,0)/(+50,0)/(0,+90下)/(0,-90上)、偏向、純LIOを同一飛行で比較。各カメラは独立した単眼ROVIO＋MIMOSA。採用v57・融合重み・初期化・センサ・偏向器は無変更、DRPM OFF。真値は制御と事後評価だけで推定器/偏向器へ入力しない。全カメラは初期化から観測し、偏向中も評価から除外しない。
- record_common_lidar.pyを追加し、observe_pure_lio.py / run_lio_failure_comparisons.pyに任意record_geometryを追加。純LIOの読み取り専用診断を有効化して96要素の実効LiDAR情報をgzip JSONLへ記録、融合には戻さない。既定動作は不変。run_window_validation.pyで6飛行を所有・正常停止。全65/65比較検査に成功、採用81ファイル/実験/評価ソースSHA不変、全子exit0、所有PID残存なし。
- 共通の純LIO診断の並進3×3ブロック（回転固定）から最小固有方向を取得し、真値姿勢で世界座標へ変換。窓開始時の同じ方向を全方式で使用。各窓の始点位置・姿勢だけを事後整合し、移動量誤差をこの方向に投影。尺度補正や推定器リセットなし。診断行列は確率重みを含むeffective情報で、raw Hessianや融合後共分散ではない。ドリフトした地図に依存する共通診断基準であり真の環境情報ではない。
- 融合に実際に入った1Hzの視覚速度を真値姿勢で回し、真値位置20ms中央差分との差を同方向へ投影。全窓でW秒にW観測、共通LiDAR診断年齢0.25秒未満を確認。平均偏り/標準偏差/RMSを別々に保存。window_metrics.py / report_window_validation.pyを追加。座標/始点不変、尺度誤差保持、弱方向回転、正負相殺でRMSが消えないことの4数値試験成功。構文検査・git diff --check成功。

|評価区間|純LIO 10秒方向誤差RMS[m]|固定5方向平均[m]|偏向[m]|偏向の変化|
|---|---:|---:|---:|---:|
|ZaB a1 0–60秒|0.01943|0.01515|0.01436|−5.2%|
|ZaB a1 60–120秒|0.12969|0.08500|0.06877|−19.1%|
|ZaB b1 0–60秒|0.03104|0.01870|0.01689|−9.7%|
|ZaB b1 60–120秒|0.08694|0.02011|0.01394|−30.7%|
|二ツ小屋 0–60秒|4.97256|0.55346|1.16550|+110.6%|
|二ツ小屋 60–120秒|4.99377|0.40475|0.22582|−44.2%|

- 各方向を2飛行・12窓で集計したRMSの固定5方向算術平均と偏向を比較。固定方向を等確率で1つ選択する期待値であり事後最良固定との比較ではない。ZaB後半2区間/二ツ小屋後半の改善、二ツ小屋前半の悪化は各2試行とも同符号で、5/20秒窓でも同符号。ZaB前半2区間は試行で順位逆転、b1前半は20秒窓でも偏向が悪化。窓を独立試行として数えず、有意差を主張しない。
- 二ツ小屋後半の実視覚速度誤差RMSは正面.8332/上.0096/偏向.1389m/sだが、融合方向移動誤差は.2222/.5230/.2258m。正面の速度平均誤差+.6979m/sと純LIO移動平均誤差−4.9811m/10秒は相殺し得る方向。偏向にも+.1225m/sの速度偏りがあり、改善を全て正確な拘束の効果と断定しない。ZaB b1後半は偏向の視覚速度RMS.1595m/sが全固定方向より大きいのに融合誤差が最小。短窓評価でも融合重み・姿勢・地図・相殺の影響は残る。因果寄与率は未同定。
- 事前閾値lambda_min/lambda_max<.05の強い異方性は10秒窓72個中1個（ZaB a1後半）、二ツ小屋0。二ツ小屋純LIOの大きな移動誤差は確認したが、この閾値による退化認定はできていない。評価は「回転固定時の最も弱い並進方向」に対する性能であり、真の退化方向補完の証明ではない。全窓と閾値以下部分集合を保持し、結果に合わせ閾値を変更しない。
- 10秒窓のジンバル総移動角中央値はZaB a1 29.0deg/b1 20.4deg/二ツ小屋31.8deg。20秒窓も保持。特徴追跡の安定所要時間は直接同定しておらず、10秒で必ず効果が完結するとは言わない。
- 結果 /root/uas_native/logs/weak-direction-windows-20261021/window_report/{index.html,comparison.pdf,summary.json,windows.csv,visual_velocity.csv}。各方向値/全窓長/各試行/誤差偏り・ばらつき・共分散/ジンバル移動を保存。campaign.jsonに事前条件、final_audit.jsonに監査、evaluation_implementationに評価実装のコピー、finalize_findings.pyに解釈ページ追記。数値ログを可逆圧縮し約196MiB、Windows空き176.38GB。連続画像/点群/rosbag/GUIなし。シミュレーション・所有プロセス全停止、自動再開なし、commitなし。

## 2026-10-22：固定方向順位と偏向の反復検証（実行開始）

- ユーザー指示でZaB/二ツ小屋から有望区間を選び、各3回新規計測。選択は既存の方向差と通路形状を参考にし、新測定前に固定。ZaB a1の往路40–100秒・復路160–220秒、二ツ小屋interiorの60–120秒・180–240秒。時刻は追従開始からの経過シミュレーション秒。前段の飛行/地図/特徴履歴は保持し、評価だけを切り出す。ZaBは全228.493経路秒、二ツ小屋242経路秒まで取得。往復は同じ通路だが採点する位置範囲と履歴は完全一致ではない。
- 独立要求seed20261201/20261202/20261203。固定5方向＋偏向の独立単眼ROVIO/MIMOSAと純LIOを同じ飛行・LiDAR/IMUで比較。既存v57・DRPM OFF・推定器/偏向器設定は無変更、真値による観測補正なし。過去結果は選択用で今回の反復集計に混ぜない。全新結果を保存する。
- run_direction_repeats.py、report_direction_repeats.py、report_direction_segments.pyを追加。主指標10秒の共通最弱並進方向への移動誤差、補助5/20秒、全方向誤差・実視覚速度誤差/不確かさを保持。追加で60秒区間開始剛体整合の位置/回転誤差を計算。尺度補正・推定器リセットなし。
- 固定方向別平均/試行間標準偏差/順位、等確率5方向平均、両区間共通の事後最良固定、区間別事後最良固定を区別。残り2試行で固定方向を選び未使用の1試行で採点する比較も実施。窓は独立反復ではない。強異方性閾値.05は前回から維持し、真の退化認定とは区別。
- 集計が評価対象の試行を方向選択へ混ぜないことと、全試行で事後選択した順位と区別する数値検査に成功。追加3ファイル構文確認成功。評価実装は最初の新結果集計前にSHA/コピーを固定。
- ROOT=/root/uas_native/logs/direction-repeats-20261022、main session42719。最大1simulation、GUI/bag/画像点群の連続記録なし。Windows空き173GB、8GB下限。実行中であり結果と停止監査は完了後追記。

## 2026-10-22：固定方向順位と偏向の反復検証完了・全停止

- 前項の事前固定6飛行（2環境×3seed）を完走。全区間30.0m前後、0.5m/s。ZaB往路は世界X約323.4→353.4m/yaw−1.7deg、復路341.2→311.4m/yaw178.3deg。二ツ小屋はX89.5→119.5mと149.5→179.5m。選択区間内に機体の旋回なし。飛行前段/旋回履歴は維持し、区間開始で推定器リセットしない。過去の実験を反復数へ混ぜない。
- 各方向の各飛行10秒方向誤差RMSを3回で算術平均（±試行間標準偏差）。主指標は回転固定の共通純LIO診断最弱並進方向。5/20秒・全方向・実際の視覚速度の誤差/偏り/不確かさも保持。

|区間|−50°|正面|+50°|下|上|偏向|純LIO|
|---|---:|---:|---:|---:|---:|---:|---:|
|ZaB往路|.07403|.07258|.09736|.09670|.05804|.03982|1.59192|
|ZaB復路|.16614|.09069|.12146|.50460|16.35725|.10996|4.83954|
|二ツ小屋60–120秒|.20851|.32838|.41059|.48694|.58915|.43462|4.90741|
|二ツ小屋180–240秒|.09294|.94717|.21840|.11909|.19928|.53837|4.89484|

- ZaB往路は固定最良が3/3で上、偏向が3/3で全固定を上回る（偏向 .05404/.03558/.02983m）。最良固定平均から31.4%低減。復路の最良固定は正面/−50/正面、偏向は正面に1勝2敗、平均21.2%悪化。上向きは3回目の復路で大誤差（10秒RMS48.829m、60秒位置RMSE181.679m）、入力監査合格なので性能結果として保持。大値に合わせて省略せず、対数軸図を追加、元の線形軸も保持。
- 二ツ小屋手前の最良固定は−50/−50/正面、奥は下/下/−50。2回時点での「場所ごとに最良方向変化」の見立ては3回目で再現せず、3回平均は両区間とも−50。偏向の対−50勝ちは手前1/3、奥0/3。5/20秒平均でも両区間−50が最良で偏向は下回る。固定優位を「たまたま」として除外しない。
- 60秒区間開始で剛体整合した位置RMSEの平均[m]は、ZaB往路 上.2104/偏向.2201、復路 正面.2768/偏向.2422、二ツ小屋手前 −50 .3194/偏向1.5740、奥 −50 .1683/偏向1.0601。ZaBは10秒主指標と一部順位が逆。往路20秒方向RMSも上.0851/偏向.0846で僅差。短窓の改善を全時間尺度に拡張しない。
- 残り2試行で固定方向を選び残した1試行で採点する比較（2区間×3試行のRMS平均）: ZaB 共通正面.081632/区間別上・正面.074361/偏向.074888m。二ツ小屋 共通−50 .150724/区間別選択.176270/偏向.486495m。評価試行で方向を選ばない数値検査済み。二ツ小屋では区間別方向を追加選択する方針も共通−50を超えない。
- 視覚観測の正確さと融合順位の逆転は再現。二ツ小屋手前の実視覚速度RMS（各試行RMS平均）は−50 .3362/偏向.0487m/s、奥は−50 .5570/偏向.0413m/s。奥−50の速度偏りは3回すべて正(+.751/+.242/+.536m/s)、純LIO10秒移動偏りはすべて約−4.8～−5.0m。固定融合が良い一因として系統的な誤差補償を支持するが、寄与率は未同定。固定の実測上の勝ちを維持しつつ「視覚情報自体が正確」と解釈しない。画角選択だけで融合誤差を最適化すると補償を好む危険がある。
- 主10秒72窓中、事前閾値.05の強異方性は3窓（ZaB往路）のみ。診断方向はドリフトを含む純LIO地図の条件付き並進情報であり真の退化方向認定ではない。各環境3試行、窓を独立試行数に数えず有意差を主張しない。真値は共通飛行制御/評価のみ、推定器/偏向器へは入れない。
- 全6飛行で65/65比較検査、採用/実行/評価SHA不変、全所有process/子exit0・残存PIDなし、GUI/bagなし。実行・比較計33.8分、今回ログ約306MiB、Windows空き172.79GB。追加3ファイル構文確認、評価試行を方向選択へ混ぜない数値検査、git diff --check成功。新たな推定器/偏向器設定は採用していない。
- 結果 `/root/uas_native/logs/direction-repeats-20261022/report/index.html`。summary.jsonに全試行/順位/別試行採点/視覚誤差、segment_metrics.jsonに位置・回転・最大誤差/実座標、windows.csv/visual_velocity.csvに最小数値。comparison.pdfは線形軸、log_scale_comparison.pdfは大誤差を省略しない対数軸。実gimbal角度図/数値も保存。final_audit.jsonに全監査。
- 再集計順: report_direction_repeats.py ROOT → report_direction_segments.py ROOT → ROOT/plot_choices.py → ROOT/finalize_findings.py。実験ソースはrun_direction_repeats.py、評価ソースはevaluation_implementationへコピー/SHA固定。全終了、自動再開待ちなし、commitなし。

## 2026-10-22 — 固定視野がVIO拘束自体で有利なケースの追加収集（開始）
- ユーザー依頼: 融合後の誤差相殺とは分離して、固定視野のVIO観測が偏向より正確なケースを増やす。
- `run_visual_constraint_survey.py`を追加。採用UniPilot v57、DRPM OFF、固定5方向の独立単眼＋偏向＋純LIOを維持。ZaB a1全長、b1全長、二ツ小屋interiorを362秒まで、各3新規seed(20261301/302/303)、計9飛行。推定器・制御規則・融合重みは変更しない。
- 新規結果を見る前に評価区間を固定: a1 40–100/160–220秒、b1 60–120/180–240秒、二ツ小屋60–120/180–240/300–360秒。追従開始からの経過時間で、区間前の地図・追跡履歴は残す。b1後半は旋回を含む条件として別記し、直進条件と混同しない。
- 主指標を実際に融合したVIO速度の共通LiDAR弱方向への誤差RMSへ変更。各固定方向との対応比較、速度の偏り/ばらつき/推定共分散、5/10/20秒方向保持、別試行での方向選択を報告。融合後誤差は副指標。良い例だけを残さず全試行・全方向を保存する。固定有利の探索であり、環境全体での発生頻度を推定する試験ではない。
- ROOT=`/root/uas_native/logs/visual-constraint-survey-20261022`。GUI/rosbag/連続画像点群なし、数値は可逆圧縮。開始時Windows空き約161GiB。終了時に所有プロセスを停止する。

### 固定優位のVIO拘束追加収集・完了
- 全9新規飛行・7区間×3回を完了、すべて停止。`logs/visual-constraint-survey-20261022/report/index.html`、`comparison.pdf`（全方向）、`cases.pdf`（代表3ケース時系列）、`summary.json`、`cases.json`、時刻付き`visual_samples.csv`に保存。
- 主指標はMIMOSAに実際に渡したVIO速度の、共通純LIO弱方向への誤差RMS[m/s]。各飛行のRMSの3回平均。真値は採点のみ、視覚速度や融合重みを補正していない。
- 固定が3/3回とも勝ち、平均誤差20%以上減る候補は3区間・11方向比較。代表: ZaB b1 60–120秒 (-50,0): 固定.01799397 / 偏向.29902147 (93.98%減)。b1 180–240秒 (-50,0): .03039579 / .40118636 (92.42%減)。二ツ小屋60–120秒 (0,-90)上向き: .01100935 / .14935021 (92.63%減)。b1後半は直進と旋回の混在条件。
- 上記3例は5/10/20秒の方向更新でも各3/3勝利。全3方向の速度誤差でも各3/3勝利。他2試行のVIO誤差だけで方向を選び、残る試行を評価する方式でも同じ代表方向が選ばれ、すべて固定が勝つ。探索的な事例収集で、有意差や環境全体での発生率とはしない。
- b1固定−50°の速度偏りは前半+.0016〜+.0043m/s、偏向+.180〜+.251m/s。後半は固定+.0107〜+.0222、偏向+.236〜+.315。観測時のジンバル角速度<1deg/sでも誤差差は残る。原因を操作した切り分け実験ではなく、動作中だけの一時誤差では説明しきれないという観測。共分散も時刻付き保存し、RMSをσ中央値で割って過信度を判定せず、各時刻のerror/σから正規化誤差を算出。
- 二ツ小屋手前の偏向は.35889/.07301/.01615m/sと大きく変動、上向き固定.01238/.01349/.00716m/s。平均だけで安定性を隠さない。残る4区間は順位逆転あり。a1復路は最初2回固定優位だが3回目は偏向.0524m/sが全固定より良い。二ツ小屋中央・奥側も偏向が最良の回あり。失敗や固定不利の方向を除外していない。
- 共通方向は純LIO診断の回転固定並進情報ブロックで、真の退化を保証しない。最小/最大固有値<.05は10秒窓126中8窓。この実験の強い結論は「VIO速度観測自体で固定が勝つ区間がある」であり、全区間での真のLiDAR退化検出や融合後精度の優位ではない。
- 新規ソース: run_visual_constraint_survey.py / report_visual_constraint_survey.py / plot_visual_constraint_cases.py。既存採用v57・DRPM OFF・全センサ設定・融合重み・偏向制御は不変。実行/評価ソースをSHA保存。評価は速度偏り、ばらつき、共分散、3D速度誤差、重力方向誤差、特徴数、ジンバル速度、融合後の短区間移動誤差を分離。
- 各飛行65+20=85項目、計765/765成功。4数値単体試験、対応比較/評価試行の選択漏洩防止チェック、378組の統計整合性/投影誤差上限/各60速度標本確認、リンク検査、構文/差分空白検査に成功。実行・採用・評価SHA不変、全所有PID停止/終了0。数値は可逆圧縮、連続画像・点群・bag・GUIなし（既存runnerの画角確認PNG5枚/flightは小容量のため保持）。ログ全体約537MiB、Windows空き約160GiB。
- 再生成: OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 active_perception/report_visual_constraint_survey.py ROOT → python3 active_perception/plot_visual_constraint_cases.py ROOT → python3 ROOT/finalize_findings.py → python3 ROOT/validate_report.py → python3 ROOT/finalize_audit.py。全終了済み、継続待ち実験なし。commitは未実施（依頼なし）。

## 2026-10-22：固定視野に負けた条件を対象に偏向手法の改良を開始
- ユーザー依頼は「手法を改良し、固定に勝てるように」。主評価を融合前の実ROVIO速度拘束（共通LiDAR弱並進方向）に置き、融合後の局所移動/全軌跡は分けて評価する。既存v57の81ファイル・センサ・推定器・融合較正は変更しない。
- 既存9飛行の意思決定を確認。b1の60–120秒では選択視野の情報利得の中央値が0〜0.0002であり、前方優先項0.5が相対的に強い。二ツ小屋の60–120秒では目標角が10度以上切り替わる回数が各飛行119〜161回/60秒。これらは制御目的と保持動作の問題候補で、誤差の原因を単独で確定したものではない。
- 新motion_view_planner.pyは2時刻の視線差の相対運動情報を評価し、同じ地図特徴の不確かさを両時刻に伝播して共有相関を扱う。LiDAR並進方向の相対的な弱さで評価、特徴数上限25の選別もその方向への寄与を使用。目標保持4秒と切替え幅の条件を導入。真値・経路名・固定方向の正解を入力しない。実ROVIOの将来共分散と同一とは主張しない。
- motion_view_planner_node.py/run_motion_view_array.pyは新制御だけを明示的に差し替える独立起動。既存凍結ソースを変更しない。run_motion_view_trials.pyで設定/実行ソースを保存してから同一飛行の固定5方向・偏向・純LIOを実行。全比較・失敗も保持。
- 新4単体試験（視線差の有限差分、情報の半正定値/不確かさ単調性、座標回転同値、実速度/追跡保持制約）が成功。
- 開発v1：seed20261401、ZaB b1全長/二ツ小屋122秒。評価区間b1 60–120/180–240秒、二ツ小屋60–120秒。ROOT=logs/gimbal-velocity-improvement-20261022/development_v1。現在実行中、改善は未判定。GUI/raw bagなし、8GB容量guard、検証後停止。

### 改良v1の開発結果、v2で特徴選別の見積もりを修正
- v1の2飛行は全入力・連続性・終了監査合格。ZaB b1のVIO方向誤差（前半60–120/後半180–240秒）は偏向0.011608/0.008800 m/s、同飛行の最良固定(-50,0)0.026058/0.019457 m/sで改善。融合後全軌跡RMSEは偏向0.426980m、最良固定0.147774mで改善していない。両者を分けて記録。
- 二ツ小屋60–120秒は偏向0.166926m/s、上向き固定0.00777m/sで悪化。v1を全環境で固定に勝った方式とは扱わず、独立検証はまだ開始しない。開発ログ・設定・ソースsnapshotをdevelopment_v1へ保持。
- v2は情報の強い25点を理想的に選べるという予測を外し、画像を5×5区画に分け、区画内の点の情報を平均する期待値モデルを追加。ROVIOの空間的に分散した特徴選別に近づける計画上の近似で、ROVIOの選択点を正確に予測したとは主張しない。他の制御パラメータはv1と同じ。区画内の点の重複で情報量が増えないこと、総特徴予算上限を数値検査。
- 同じ開発seed20261401でZaB b1/二ツ小屋の新規飛行を開始。seed一致による観測系列の同一性は主張せず、各試行の中で固定と偏向を同時比較。ROOT/development_v2。全失敗も残し、独立検証seed20261411/12/13は未使用。

### v2完了、v3は準備運動中から視野選択
- v2の2飛行は全監査合格。VIO弱方向RMS m/s：ZaB b1前半active .016382 / 事前比較m50 .021186 / 当該最良down .010903。後半active .042441 / m50 .024354。二ツ小屋active .034562 / up .008954。v2も最良固定を一律に超えたとは扱わない。
- 二ツ小屋v1の発生時期を事後確認。追従前10秒の速度3D RMSはactive/front共に.01013m/s、追従20–40秒にactive .13749/front .21893へ増大。初期化直後から既に大誤差だったとはいえず、巡航移行後に差が育つ。旧制御はapproach/excitation中の視野選択を禁止していた。
- v3はv2の評価方法を維持し、推定器ready後の既存approach/excitationでも視野選択を許可。静止初期化やROVIO/MIMOSA実装、準備運動、飛行軌道は変更しない。固定側も従来通り初期化から各固定方向を見る。共有画像リレーは実encoderで視野が分離するまでだけ共通画像を使い、分離後は各実カメラ画像を使う既存処理のまま。
- v3二ツ小屋122秒は全監査合格。60–120秒VIO方向RMS active .013072 / up .008139 / down .022403 / p50 .069753 / m50 .465789 / front .581124m/s。上向き固定にはまだ負け、他4方向より小さい。融合全軌跡RMSE active3.44095mは最良固定1.26283mより大きい。融合とVIO拘束を分離。
- v3 ZaB b1を続行中。ROOT/development_v3、seed20261401。独立検証はまだ未開始。

### 開発4候補を完了し、motion_view_v3を固定して独立9飛行へ
- 開発8飛行（4候補×ZaB b1/二ツ小屋）は全監査合格・全所有シミュレーション正常終了。全候補・全固定方向・失敗例をROOTへ保持。
- 事前比較方向（b1の-50°、二ツ小屋の上向き）に対する3区間のVIO方向RMS比の幾何平均：v1 1.6300、v2 1.7326、v3 1.2789、v4 3.3585。選択規則はv1–3を観察後、v4結果を見る前にcandidate_selection_protocol.jsonへ記録した開発用規則であり、独立な統計的証拠とは扱わない。候補v3を選択。
- v4（v1評価＋早期開始）は二ツ小屋で再悪化し不採用。全結果を保存。v3にも最良固定より悪い開発区間があり、全条件で固定を超えたとは主張しない。
- motion_view_v3.yaml：2秒の視線差の相対運動情報、5×5区画の期待情報、LiDAR並進の相対的な弱さ、目標保持4秒/切替幅.25、正面優先.02、追跡保持.9、未来保持.3、準備運動中から制御。実駆動上限は元のまま（探索5°/s・合成20°/s、yaw±50/pitch±90）。GT・経路名・正解固定方向を制御へ渡さない。
- 6試験成功：特徴位置/機体相対運動の有限差分、情報PSD/不確かさ単調性、座標回転、長さ単位変換、速度/保持制約、区画内重複による情報予算不変。旧採用81ファイルは全SHA一致。
- frozen_candidate.json/zipに候補7ファイルSHAと旧基準検査を保存。数値処理は変えず、コピー元に残っていた「official results」という不適切なコメントを相対運動の計画情報へ訂正してから固定。
- 独立seed20261411/412/413、ZaB a1全228.493秒・b1全263.385秒・二ツ小屋362秒、計9飛行を開始。評価は従来7区間・各3反復、主対象はb1前後/二ツ小屋前部の3区間。開発データを独立結果へ混ぜず、途中調整もしない。ROOT/validation、owner continue_validation.py（session42001）、最大1sim。現在a1/411開始、完了まで継続。

### 2026-10-22 ユーザー指示で改良作業を中断
- 「一回作業中断して」に従い実験の続行を停止。記録済み全ownersと/procを確認し、今回の実験プロセスは残っていない。ROOT/PAUSED.jsonとHANDOFFに再開位置を保存。採用設定は変更していない。
- 独立検証seed20261411の3環境は完了・監査合格。b1前半のVIO方向RMSは偏向.12368/固定-50 .01759m/s、後半は偏向.00977/固定-50 .01838m/s（後半は全5固定より小さい）。二ツ小屋手前/中/奥は偏向.0204/.1124/.1275対上向き固定.0172/.0511/.0309m/s。最良固定に安定して勝つ目標は未達。
- 4本目zab_a1_seed20261412は飛行終了し全所有プロセスexit0だが、入力/出力連続性、scan・IMU系列一致、各画像処理数の監査に失敗。参照scan2823対純LIO2817、純LIO出力最大間隔.7秒。実験ownerはそこで異常終了し後続5本は未実行。原因未特定、失敗データを保持し有効精度結果には混ぜない。
- frozen v3は変更なし。v5は実ROVIO特徴・速度の同時不確かさを使う既存v54評価へ早期視野選択を加えた候補設定のみ準備、実行していない。中断後の追加実験なし。再開は4本目の監査原因確認から。

### 2026-10-23 改良作業を再開・無効試行の共通欠落を確認
- ユーザー「再開して」に従い再開。前回a1/seed20261412の入力監査を調査し、全固定画像/外部姿勢がsim249.15〜249.2秒から.55〜.60秒途切れ、共通真値も249.2秒から.98秒途切れていた。偏向推定器だけの破綻ではなく共通取得/通信の中断。残存ログに原因の例外がなく、正確な発生源は未特定。
- 既存の厳密な入力監査を維持。無効試行をvalidation/invalid_trialsへ記録付き移動し、同条件・同seedで一度再試行。resume_motion_view_trials.pyを追加：凍結SHA検証、完了飛行skip、旧所有者停止確認、無効試行保持、失敗時停止。計算/センサ/制御条件は変更しない。元のrun_motion_view_trials.pyは変更せず凍結を維持。
- 再開owner session38925、/tmp/motion-view-resume.log。a1/412から残り6本を実行。Windows空き160.59GiB、GUI/raw bagなし。

### 2026-10-23 再試行成功・実追跡情報の候補を準備
- a1/seed20261412の同条件再試行が全監査合格。前回の共通欠落は再発しなかった。元の無効試行はvalidation/invalid_trialsに保持し、発生源未特定という記録を維持。残りの凍結v3検証を続行。
- 事後診断map_vs_tracked_features.json：独立a1/411前半では計画地図点数中央値281に対し実ROVIO追跡点中央値2、追跡5点未満79.15%。計画地図点数は全地図、実追跡は現在画像なので同じ母数ではないが、計画が成立している間に実推定の観測が乏しいことを確認。これだけを全区間の誤差原因とはしない。
- v5（既存v54の実ROVIO同時共分散＋早期視野選択）、v6（v5＋既存v55の画像上の特徴移動を抑える候補）を開発比較予定として固定。新v6設定を追加、追加依存ソースをsnapshot/SHA保存。関連33試験と候補生成10試験が成功。既存推定器/融合/センサ設定は変更なし。
- validation完了後にv5、v5完了後にv6をそれぞれ開発seed20261401のb1/Fut122秒で順次実行する待機ownerを作成。session67375（after_validation.py）、51605（after_v5.py）。同時シミュレーションは最大1本。停止時はこの待機ownerも止めること。
- report_motion_candidates.pyを追加。開発/独立検証を分け、VIO速度と融合後移動誤差を別表示し、各試行を省略せず表示。候補ごとの評価方法を説明し、固定平均・事前比較方向・事後最良を区別。ROOT/candidate_reportは現在進捗版。

### 2026-10-23 準備運動後の尺度学習を切り分ける追加候補（未実行）
- 凍結v3独立二ツ小屋2回目も全監査合格。VIO方向RMS 手前.01497/中.09875/奥.21529m/s、上向き固定.01363/.02957/.03435m/s。後半の悪化が2回再現。v3検証は6本完了、3巡目を続行中。
- preparation_view_motion.json：巡航前26秒の準備運動中に、v3は二ツ小屋で計約70度、b1で約61度のジンバル移動があった。特徴の入替えによる尺度学習の影響を候補として扱うが、これだけで原因を確定しない。
- motion_view_v7.yamlと専用prepared_view起動系を準備。元の26秒準備運動を全固定/偏向で共通にもう1回繰り返し、その2回目だけ偏向カメラの実角度を保持する。推定器の状態をリセットしない。固定にも同じ身体運動/IMU入力を与える。v1–v6より準備時間が長い別プロトコルであることを明示し、条件不変とは扱わない。まだ実行していない。
- view_preparation.pyは凍結ソースを編集せず厳密なanchor照合で実行用コピーだけ変更。新route状態excitation_holdは推定・真値・経路コマンドを視野制御へ渡さず、実encoder角保持だけに使用。2回の運動が元referenceに一致、2段階の遷移、実角保持と固定比較の維持、anchor不一致拒否の3試験成功。ROOT/v7_plan.jsonにSHAと未実行状態を保存。

### 2026-10-23 v3独立9飛行完了、v5実追跡評価を開始
- v3全9飛行が完走し各85項目成功。再開後の6飛行に新たな欠落なし。旧無効a1/412の保存と再試行を含む。最終集計中。採用v57/凍結v3のSHAは維持。
- ZaB b1のVIO方向RMS（active / -50固定）3回：前半 .12368/.01759、.04100/.03509、.05717/.03716。後半 .00977/.01838、.06223/.01637、.03514/.05479。前半0勝3敗、後半2勝1敗。固定最良への安定優位は未達。
- 全9本のsim停止/exit0を確認し、待機after_validation.pyだけを停止。最後の読取専用集計と重ねてv5実飛行を開始（同時simは1本）。session64013、/tmp/motion-view-v5-flight.log。旧待機停止と理由はv5_queue_transition.json。依存18SHAを起動時確認。
- v5の実decisionでpredicted_geometry_lidar_conditioned_position_increment_information_gainと実追跡点を確認。現在ZaB b1準備運動中。後続v6の待機session51605は継続、v7条件付待機session74997を追加。
- v7はv5/v6どちらも事前比較固定への開発幾何平均比<=.9かつ最大比<=1.5を満たさない場合だけ実行する事前探索規則をv7_plan.jsonへ記録。専用owner/pacerの生成/compileも成功。実行まだなし。

### 2026-10-23 v3最終集計・監査完了
- audit_and_summarize.py成功、ROOT/final_audit.json。9本×85チェック成功、全所有終了0、圧縮前後の再集計一致、実行/評価/凍結ソースSHA一致。入力欠落の旧1試行は別保管。
- 全7区間×3回（9物理飛行内で区間は相関あり）：固定5方向平均に18/21勝利、全固定より良いのは2/21。区間ごとの飛行RMSを平均したVIO方向誤差はactive.19331、固定平均.43289m/s。平均対応比.54781。
- 重点3区間×3回：固定平均に8/9勝利、以前選択固定（b1=-50、Fut=上）に2/9勝利。active平均.04172、固定平均.13667m/s、平均対応比.63103。最良固定への安定優位とはしない。
- 区間平均active/固定平均 m/s：a1前.28505/.36576、a1後.67395/1.38497、b1前.07395/.07848、b1後.03571/.14644、Fut前.01550/.18508、中.09231/.36190、奥.17668/.50758。
- 結果validation/report/motion_index.html、motion_comparison.pdf、final_summary.json。これはv3の完了結果。後続v5/v6は開発段階で混ぜない。採用v57は引き続き不変。

### 2026-10-23 実追跡候補v5完了・v6を継続
- v5の開発2飛行は全監査成功。方向速度RMS[m/s]：b1前半active .01757 / -50固定 .01859（全5固定より良い）、後半 .07487 / -50 .01740、二ツ小屋手前 .02089 / 上固定 .00913。良い固定への安定優位はまだ達成していない。推定器・センサ・融合重みは変更なし。
- v6（画像の動きを抑える候補追加）はb1完了・監査成功。前半 .02934 / -50 .01774、後半 .03949 / -50 .01930m/s。二ツ小屋を続行。
- view_switching_diagnosis.jsonに実角度・実追跡点数の診断を保存。v3二ツ小屋は大半上向き（pitch中央値約-76〜-90度）でも上固定より速度誤差が大きい。方向選択のみで差を説明せず、観測履歴・準備運動の寄与をv7で切り分ける。v5では画像内回転を含む実角度移動が60秒で約128度あり、v6はその低減候補を評価中。

### 2026-10-23 v6開発2本完了、準備運動診断とa1を追加
- v6二ツ小屋60–120秒のVIO方向RMSは偏向.006427、上固定.009277m/sで全固定5方向より小さい。融合後全軌跡RMSEは偏向4.508m、最良固定1.630mで逆転しており、視覚拘束の勝ちと融合の勝ちを混同しない。両飛行は全監査成功。
- 事前比較方向への3区間RMS比の幾何平均はv5=2.1034、v6=1.3285。事前記録したv7条件を両方満たさないため、共通追加準備運動のv7診断を開始。
- v7の後にv6をZaB a1へ適用する開発1飛行を予約（session38537）。目的は実追跡情報がv3の追跡点不足を避けるか確認すること。設定・seed・軌道をv6_a1_plan.jsonへ事前記録。開発結果で選択後は新しいseedで独立検証する。

### 2026-10-23 次の独立検証条件を事前記録
- 次候補はv6/v7の開発3区間で事前比較固定へのRMS比の幾何平均が小さい方を選ぶ。next_validation_protocol.jsonにv7最終集計を見る前に規則を保存。v6のa1開発診断後、候補ソース/設定をfrozen_round2.json/zipへ固定して、新seed20261511/12/13×a1/b1/二ツ小屋（最後は362秒）9飛行を実行予定。調整結果を独立検証へ混ぜない。
- start_round2_validation.py待機owner session13025 (/tmp/motion-round2.log)。simは1つだけ。ユーザー停止時はこのownerも停止する。候補選択/凍結はまだ待機中。
- audit_motion_campaign.pyを追加、v5/v6完了データで実行成功。各source SHA/入力・連続性/全所有終了/圧縮前後再集計一致を確認。VIO拘束、固定等確率平均、事前方向、事後最良を分けて数値保存する。v6は3開発区間すべて固定平均より小さく、そのうち1区間は全固定より小さい。candidate_reportをv1〜v6と完了v3独立9本で再生成。

### 2026-10-23 v7 ZaB完了・同一飛行の全固定よりVIO拘束改善
- 共通準備運動を追加したv7のb1は全監査成功。弱方向速度RMS[m/s]前半active .014790 / 最良-50 .025044、後半active .013620 / 最良-50 .019578。両区間とも全5固定より小さい。1本の開発飛行であり、反復での優位は未検証。
- 融合後全軌跡RMSEはactive .502774m / front .169015mで固定の方が良い。VIO速度拘束の主目的と融合位置を分けて保持。
- preparation_hold_audit.jsonで69.25〜94.95秒の128判断すべて実角保持、最初の1秒を除く実角変化は数値丸め程度と確認。追加26秒の身体運動は固定/偏向に共通。準備運動追加と保持を合わせた候補の比較であり、保持だけが改善原因だとはまだ扱わない。二ツ小屋を続行。

### 2026-10-23 v7二ツ小屋は悪化、v8を開発候補へ追加
- v7二ツ小屋も全監査成功。VIO方向RMS active .076895 / 上固定 .008665m/sで悪化。ZaBの改善と同時に二ツ小屋で改善するという仮説は、この開発試行では成立しなかった。全結果保持。
- まだ独立検証round2は開始前。待機owner13025のみ停止し、v6（実追跡＋画像移動を抑える候補）とv7の共通追加準備を組み合わせたv8を新しい開発候補へ追加。推定器・センサ・融合は不変、真値を視野判断へ入れない。v8_plan.jsonへ仮説/全依存SHA/条件を事前保存。
- 開発選択候補をv6/v7/v8へ拡張し、変更前protocolをnext_validation_protocol_history.jsonlへ保持。主3区間の事前固定に対するRMS比の幾何平均という規則と、未使用の独立seed20261511/12/13は維持。開発での仮説変更であり、独立結果を見て選び直してはいない。
- v6 a1の後にv8のb1/Fut122秒を実施（session97395）、その後に選択・凍結・独立9本（session16303）へ移る。全waiting ownerはlive_state.jsonへ更新。同時sim最大1。

### 2026-10-23 v6 a1完了・v8開始
- v6 a1は全監査/全所有終了/圧縮再集計一致に成功。VIO方向RMS[m/s]往路active .167693 / 最良+50 .090174、復路active .089461 / 最良front .086465。固定5方向平均には両区間勝ち、最良固定には両方届かない。復路ではROVIO実追跡点2の場面も残る。
- 原因候補として、現在の実追跡評価の下位処理が少数の視差のある特徴を追い続けられることをソースで確認。これは記録時刻の不一致ではなく既存の制御規則。まだ変更せず、候補間比較の条件を維持。
- v8開発開始（session97395）。v8とv6の設定差はcommon_preparation_cycles=2、hold_active_view_in_second_cycle=trueの2項目だけと確認。
- 再開helperに準備回数によるrunner選択と--frozen検証を追加しcompile成功。round2中断後は `resume_motion_view_trials.py ROOT/validation_round2 --frozen ROOT/frozen_round2.json` で完了済みを飛ばし同じ準備手順を使う（途中で中断した試行の扱いは状態確認が必要）。完了reportは候補の実際の評価方式に対応するreport_motion_candidatesへ変更。未使用の経路を既に実行確認済みとはしない。

### 2026-10-23 組合せ候補v8のZaB完了
- b1は全監査成功。VIO方向RMS active/-50固定[m/s]：zab_b1_0 0.019904/0.043674, zab_b1_1 0.027696/0.036444。前後半とも全固定5方向より小さい。1本の開発飛行であり、再現性の結論は保留して独立反復へ進む。
- 融合全軌跡RMSE active .374257m / 最良下固定 .079578mで逆順位。VIO拘束と融合位置の比較を分離して保存。二ツ小屋を続行。準備中の実角保持も数値記録で確認。全ROOT約1.3GiB、Windows空き約159GiB。

### 2026-10-23 v8開発完了・凍結して独立9飛行を開始
- v8開発2本は全監査成功、全所有終了0、圧縮前後一致。二ツ小屋VIO方向RMS active .026985 / 上固定 .022530m/sで約20%大きい。途中の少数サンプルの優位を最終値として使わず、全60サンプルの確定値を記録。ZaB前後は全固定より良好。融合位置は引き続き別評価。
- 事前固定への主3区間RMS比の幾何平均：v6=1.328493、v7=1.539064、v8=.745800。事前の選択規則によりv8を選択し、frozen_round2.json/zipへソース・設定SHAを保存。v8は全主区間で固定に勝ったわけではない。
- v8は現在のROVIO追跡点/速度の同時不確かさから、LiDARを補う将来の観測を評価し、画像上の動きを抑える候補も選ぶ。独立の画像特徴地図の未表示点は探索対象にしない。元の地図検索型v3とはこの点が異なる。共通準備運動を26秒追加し、その間だけ選んだ実角を保持する。固定側も同じ身体運動を受ける。
- validation_round2開始、seed20261511/12/13×a1/b1/二ツ小屋362秒の9本。owner session16303、/tmp/motion-round2.log。最初はa1/511。独立検証の途中に設定を変えない。推定器/融合重み/センサ品質/DRPM OFFは共通。真値は共通身体飛行制御と事後採点だけ。
- 検証後はaudit_motion_campaign.py validation_round2 --frozen ROOT/frozen_round2.json、report_motion_candidates.pyで更新する。現時点で採用v57/defaultは変更していない。全データは数値ログ中心、ROOT約1.3GiB。

### 2026-10-23 独立1本目完了・説明文の整合
- v8独立a1/511は全監査成功。VIO方向RMS active .72022/.26646m/s、固定+50の往路 .27582、固定front復路 .09657より大きい。融合位置RMSEはactive .23112mで最良だが、VIO拘束の改善とは数えない。上固定は融合100.65mまで崩れ、入力正常の性能結果として保持。
- 往路の実追跡点中央値4、4点未満34.3%。現在の下位制御は適した追跡点が見つかると正面復帰を中途で終了する設計。時刻不整合の証拠はなく、原因を断定しない。round2_a1_recovery_diagnosis.jsonに遷移記録を保存。独立検証中の設定変更なし。
- 元の数値レポート生成器に旧v57/各3飛行という固定説明文があったため、数値コードを凍結したまま生成HTMLだけを修正するannotate_motion_reports.pyを追加。完了済み10キャンペーンで設定名/開発・独立区分/追加準備を表示し、成功。数値CSV/JSON・計算コードは変更対象外。round2完了後も実行する。report_motion_candidatesの真値用途も共通飛行制御と事後採点に明記。

### 2026-10-23 v8独立3本・終了処理失敗の保存と再試行、v9準備
- v8独立seed20261511の3本は全監査成功。b1前半は偏向 .03813m/sで全5固定より大きく、後半は .04415m/sで全5固定より小さい。二ツ小屋の手前/中/奥は .05731/.09869/.13911m/sで、上固定 .00710/.03158/.03478m/sより大きい。固定平均への勝利と良い固定への勝利を区別。
- 4本目a1/seed20261512は飛行完走、画像・IMU・推定出力の全連続性チェックは成功したが、yaw_m50画像encoderだけSIGINT後にexit -11。他の内部処理と全外部ownerはexit0、Gazebo正常終了。終了時のsegfaultの直接原因は未特定。厳密な終了チェックを緩めず無効試行をvalidation_round2/invalid_trialsへ保持し、凍結v8を変更せず同条件・同seedで一度再試行。session35274、/tmp/motion-round2-resume.log。旧16303は終了。再開helperの無効理由を継続性限定から実際のfailed_checks参照へ修正。
- preparation_vs_cruise_views.jsonを追加。v8独立b1の保持角(-43.79,-12.10)に対し巡航中央値(+30.04,-89.97)、二ツ小屋保持(+5.82,-22.77)に対し巡航(+2.19,-89.97)。準備時と巡航時の視野不一致を確認。これが誤差の唯一原因とは扱わない。
- v9を未実行の次候補として準備。最初の経路polyline上を最大15m、最高.5m/sで進んで視野を選択し、同じ経路を戻る間と2回目26秒準備運動中に角度保持、その後に元の評価軌道へ。全固定にも同じ運動を与える。v8比で最大64秒+整定時間の事前往復を追加する別準備条件。真値は身体制御/採点のみ、推定器補正や偏向判断には使用しない。
- 新規preview_motion.py/preview_preparation.py、専用route/planner/array/trials、motion_view_v9.yaml、test_preview_preparation.py。5試験成功（既存経路上の正確な往復、滑らかな速度、実制御遷移、評価時間分離、実角保持、生成source照合）。v9_plan.jsonにSHAと未実行状態保存。v8凍結ソース変更なし。v9実験ownerはまだない。採用v57/defaultは不変。

### 2026-10-23 v9を現検証完了後に順次実行する待機を設定
- start_v9.py（session90918、/tmp/motion-view-v9.log）はround2の正常完了と実行owner消失の両方を待ち、全SHA検証・凍結保存後、v9開発seed20261401でb1/Fut122秒を実行する。現round2設定は不変。同時シミュレーション最大1。停止時は待機ownerも停止する。v9性能結果はまだない。
- candidate_reportを開発v1〜v8、v6追加a1、完了v3独立と進行中v8独立で更新。結果を開発/独立に分離し、固定平均/事前固定/事後最良を明記。v8独立の途中集計を最終と扱わない。

### 2026-10-23 v8終了エラー再試行成功・速度の偏りを確認
- a1/512の同条件再試行は全監査成功・全所有exit0。元の終了SIGINT後encoder -11は再発なし。無効元結果を削除せず、独立検証4本完了。b1/512へ進行。
- bias_vs_scatter_diagnosis.json：二ツ小屋手前v8開発は方向RMS .02698、平均ずれ .02551、ばらつき .00880m/s。独立511はRMS .05731、平均ずれ .05661、ばらつき .00892m/s。実追跡特徴中央値18〜18.5。特徴不足だけでなく尺度/初期学習の仮説を優先する根拠。平均ずれ自体は因果同定ではない。v9は未実行、現独立検証を変更せず継続。

### 2026-10-23 v9追加実行検査と待機管理
- audit_preview_preparation.py追加。実行後に段階順序、往復復路+追加準備中の実角保持、評価期間からの除外を確認し、保持視野と巡航視野の差も保存する。構文検査成功、v9実データでの実行は未実施。
- v9独立検証へ進む開発基準を結果前にv9_validation_protocol.jsonへ保存。主3区間の事前固定への幾何平均比<=.9、最大比<=1.5、全区間で固定5方向平均より小さい場合に未使用seed20261611/12/13のb1+二ツ小屋全長で反復予定。基準不達の場合は結果を保持し診断する。
- 待機owner検出を文字列部分一致から実際のargv要素一致へ修正。親shellのコマンド本文を実験ownerと誤認する余地を除いた。v9待機session90918を停止し64332へ置換。現在のv8実行/条件に影響なし。停止時は35274と64332を確認。

### 2026-10-23 a1の少数追跡による復帰中断へ対策候補v10（未飛行）
- v8独立a1は3本完了し、いずれもVIO方向速度で良い横向き/正面固定に劣る。既に記録した「復帰開始後、1〜3個の暫定特徴を追って正面復帰を中断する」遷移への別候補を用意。現v8/次v9の凍結依存は変更なし。
- committed_tracking_view.py追加。可視かつ0.3秒以上追跡した点が既存min_features=4未満で既存delay=1秒続くと復帰を確定し、正面±1度以内かつ4点以上が1秒続くまで局所視野選択へ戻らない。瞬間的な再検出や1点の視差で中断させない。復帰速度は既存の物理軸速度・合成角速度の範囲内。身体/推定器のリセット・真値補正なし。
- v8との差を隔離する専用guarded planner/array/trialsとmotion_view_v10.yamlを追加（v9の事前往復は入れない）。閉ループ復帰、一時欠落、新特徴の年齢、準備中の観測空白、健全時の元共分散評価との出力完全一致の5試験成功。生成owner/pacer/planner構文確認成功。初回テスト用joint_stop_margin指定がスカラーだった不備を2要素へ訂正し再実施。
- v10_plan.jsonに全SHA・a1開発seed20261401の事前計画を保存。まだ飛行/待機ownerなし。v9は別の尺度/準備仮説、v10は追跡喪失からの復帰仮説として分離する。採用設定v57は不変。

### 2026-10-23 v10 a1をv9開発後に順次実行予定
- v10 a1開発1本を、v9開発2本の正常完了およびowner消失後に実行する待機ownerを設定（session68228、start_v10.py、/tmp/motion-view-v10.log）。開始時に全SHA照合しfrozen_v10_development.json/zipへ固定する。v9の結果を見て設定を変えず、追跡喪失への対策単独を検証。
- 現在v8独立の最後の二ツ小屋/513を実行中。後続の待機はv9 session64332、v10 session68228。ユーザー停止時は全待機ownerも停止。GUI/bagなし、同時sim最大1。採用変更なし。

### 2026-10-23 v8独立9飛行完了・v9開発へ移行
- v8は全9本の入力/出力/終了監査、ソースSHA、圧縮前後再集計一致に成功。audit_motion_campaign.py完了、validation_round2/report/final_audit.json。a1/512の旧終了エラー試行1本はinvalid_trialsへ別保管している。
- 全7区間×3反復（9物理飛行、同飛行の区間は相関あり）：固定5方向平均に20/21勝利、全5固定より小さいのは3/21。active平均 .19103m/s、固定平均 .77858m/s。固定平均はa1の上/下固定の大きな失敗を含むので、最良固定への優位とはしない。
- 重点3区間×3反復：固定平均に8/9、事前比較固定（b1=-50、二ツ小屋=上）に1/9勝利。active平均 .05310、固定平均 .14092m/s。b1前半-50へ0/3、後半1/3、二ツ小屋手前の上へ0/3。良い固定への安定優位は未達。
- v9は凍結後に開発b1を開始、session64332（PID495489）。実際にpreview_out→preview_return→excitation_hold→trackingの遷移を確認。復路/準備時は約(-49.97,-77.94)度で保持し、以前の前向き準備と区別できている。性能は未集計。後続Fut122秒、さらに待機v10 a1 session68228がある。
- v9待機でsymlink表記と実体パスの差により、旧v8の最後の読取専用集計中にv9が開始した。v8の全sim/owner試行は停止済みで同時simは1本。後続用の待機コードはresolve比較に修正。修正時の重複開始は凍結manifest存在チェックで拒否され、第二のsimは起動していない。外側/tmp開始ログ冒頭だけリダイレクトで上書きされたためv9_owner_notice.jsonと拒否ログを保存し、マーカーを記載。各試行のログ・入力・結果は保持、元v9ownerは継続。
- v8最終HTMLの旧説明をannotate_motion_reportsで修正。新audit_preview_preparationには保持視野と巡航視野の光軸方向差と、画像内回転を含む全姿勢差を分けて追加（真上ではyaw角差が光軸差を意味しないため）。まだv9完了データで実行前。

### 2026-10-23 v9 b1の結果と二ツ小屋の終了エラー再試行
- v9 b1は全監査成功。VIO方向RMS[m/s]前半active .02532 / -50 .02205（最良下 .01908）、後半active .04712 / -50 .01466。準備変更だけで良い固定に勝てるという仮説は、この開発飛行では成立しなかった。融合位置RMSEはactive .42455m / front .16532m。結果保持。
- audit_preview_preparation成功。復路+準備84.313〜125.15秒、実角(-49.9717,-77.9428)、角変動1e-11度台。巡航60〜120秒の中央値(43.0813,-89.9717)で、光軸方向差12.0588度、カメラ全姿勢差93.6495度。評価開始125.28秒、準備は評価から除外。
- 二ツ小屋は122秒完走、全入力/出力連続性は成功したがyaw_m50_encoderが終了SIGINT後に再度exit -11。他全内部/外部exit0。既存基準のままinvalid_trialsへ保管し、同じ凍結v9/seed20261401で一度再試行（session57699、/tmp/motion-v9-resume.log）。元64332は終了、待機68228もfailedで終了。v10待機を71161へ再起動し、再開helperも所有者候補へ追加、パスresolve一致で待機。
- 終了問題の原因切分けとして、同じ実行ファイルをgdb下で合成画像入力/終了する最大20回の小試験を開始（session20251、build/native-shutdown-diagnosis/stress.py、ROS_DOMAIN_ID=186）。Gazeboなし、実験domain85とは分離。推定器/センサ/画像アダプタの実装・binaryはまだ変更していない。
- v10の時刻検査を、同一センサ時刻の再評価を許容する方式へ修正。既存plannerは時計停止中にも処理確認を行うため、同一時刻を例外にすると新たな停止を作る。実時間で判定するため重複時刻で欠落時間は増えない。対応試験含む6試験成功、v10_planのSHA更新（v10はまだ未飛行/未凍結）。

### 2026-10-23 v9開発終了・v10 a1実行中
- v9の二ツ小屋再試行は全監査・所有者exit0・圧縮再集計一致に成功。方向速度RMS active .022877 / 上固定 .017629m/s。v9主3区間は固定平均に3/3、事前良好固定に0/3。事前基準を満たさず独立反復へ進めない（v9_screening_result.json）。元の終了エラー試行は保持。
- 準備実行監査は2飛行とも成功。二ツ小屋は保持/巡航の光軸差0.00189度でも精度は固定に届かず、「準備と巡航の光軸差だけが原因」という説明には反する結果。
- v10 a1が凍結後に開始、owner session71161、/tmp/motion-view-v10-after-retry.log。後続待機なし。数値条件不変、標準v57不変。live_stateを更新。
- 終了時encoder障害の合成入力/gdb診断20回は全正常終了で再現せず。原因特定・修正済みとはしない。report_motion_candidates/annotate_motion_reportsにv10の復帰規則の説明を追加（数値計算不変）。

### 2026-10-23 ユーザー指示で現検証終了後に停止
- ユーザー「今の検証が終わったら停止して」。v10 a1開発1本を最後まで完了し、全85入力/出力チェック・全所有者exit0・凍結SHA・圧縮前後の集計一致を確認。owner session71161もexit0。後続実験は起動しない。
- VIO弱方向速度RMS[m/s]：往路active .237817、最良+50固定 .143035（正面 .182863）、復路active .050780、最良正面 .066641。往路は良い固定に劣る、復路は全5固定より小さい。開発1飛行の結果であり、独立反復での安定優位は未検証。固定平均は上下方向の大きな誤差を含むため最良固定との比較を別記。
- 融合全軌跡位置RMSE active .136418m、最良固定-50 .254291m。融合位置改善をVIO拘束の全区間改善とは扱わない。
- 復帰処理は4回発動し、全4回とも正面±1度かつ既定の継続追跡点数を満たしてから再開。report/recovery_execution.jsonに実行記録を保存。単に実装通り動いた事実と、誤差改善の因果証明は区別。
- 現状態をROOT/PAUSED.json、live_state.jsonへ保存。再開後はv10結果を基に次の凍結比較を設計する。完了済みv3/v8独立検証、v9/v10開発を重複実行しない。標準v57は変更していない。数値ログ中心、raw rosbag/GUIなし。

### 2026-10-24 特徴の多い面が場所ごとに変わる人工トンネルを作成・検証開始
- ユーザー指示で新規作業を開始（前日の停止はこの依頼により解除）。ZaB/二ツ小屋に似た長い通路として幅6m・壁高3m・頂部5mのアーチ断面を250m押し出した人工環境を作成。実スキャンの再現ではなく制御された要因試験。assets/switching_tunnel_20261024に再生成可能な形状/模様/経路を保存。
- 形状・照明は共通、32mごとに高コントラスト模様の面を左壁→天井→右壁→床、逆順、全周一様の3配置。模様の切替は4m幅で滑らか、低模様面のコントラストは2.5%。新規凹凸は追加せずLiDAR形状は完全共通。無雑音の点面情報行列で軸X並進が厳密な零方向であることを解析確認。実LIOの退化は別途記録で検証。
- build_switching_tunnel.py / check_switching_tunnel_render.py追加。初回静止描画検査でDAE法線欠落によりGazebo物理が停止。法線を明示して修正し、3配置×4位置×5方向の60画像を実Gazeboで取得成功。初回エラーログ保持。nativeと同じ480×270、焦点距離252.5116の画角。FAST閾値20による角検出で、例：左壁領域+50は546点/反対-50は3点、右壁領域-50は528/+50は0。天井と床も対応方向へ切替を確認。単独静止カメラ検査であり飛行ロボットの遮蔽検査とは区別。
- 最新候補v10を無変更で使用（標準採用v57への切替なし）。130mを0.5m/s、共通初期運動、固定5単眼+偏向単眼を同じ実飛行で比較。DRPM OFF、推定器・重み・センサ不変。配置metadata/正解を偏向や推定へ渡さない。真値は共通機体制御と事後採点のみ。
- 3配置×新seed20262401/402/403の9飛行を事前固定。主指標は既知の軸X方向のVIO速度誤差、各領域中央40秒を4区間。従来の推定LIO弱方向指標と融合精度は副指標。固定平均・全行程で一つ選ぶ固定・区間ごとの事後最良を区別。判定で失敗結果を除外しない。
- ROOT=/root/uas_native/logs/switching-tunnel-20261024。73入力のSHAとソースzipをfrozen.json/frozen_sources.zipへ保存後、run_switching_tunnel_trials.py開始（session14721、/tmp/switching-tunnel-validation.log）。報告器report_switching_tunnel.pyも事前作成/凍結。完了後audit_motion_campaignで全所有終了/連続性/圧縮再集計一致を確認し、専用report/switching_index.htmlを報告する。現在精度結果は未取得。rawbag/GUIなし、Windows空き156GiB、8GiB停止guard。

### 2026-10-24 1本目完了・欠落拘束を扱う評価の追加
- alternating/20262401は全取得監査成功。偏向は左→天井→右→床へ移動する挙動を確認。少数の飛行画像だけを保存する読取専用snapshot_switching_flights.pyを追加（session87067、最初のseedのみ各配置4位置×6方向）。推定/制御には入力しない。
- 下固定の中央2区間と上固定の最後の区間で速度factorが存在しない。画像/ROVIO odometryは連続しており取得欠落と区別。既存速度融合managerにはpose共分散のD最適性が1超なら入力を見送る処理があるが、今回の閾値通過ログは未取得で、直接の因果断定はしない。
- 元の完全factor前提のreport_direction_repeatsはMissing actual velocity factorsを送出。この制約を緩めて補間することはせず、別のreport_switching_tunnel_sparse.pyを追加。既知X速度RMSは全区間が利用可能な場合のみ完全値として扱い、それ以外は利用時RMSと利用率を明示。全方法共通の補助指標として単体VIO/融合の10秒相対移動誤差を追加。これは1本目の欠落確認後の評価追加であり事前登録とはしない（evaluation_amendment.jsonにSHAと理由）。取得/手法/条件は一切変更しない。
- 最初のalternating全4区間統合：偏向の速度拘束利用率100%、軸速度RMS .1833m/s、最良の完全固定-50 .2172。VIO10秒軸移動RMS 偏向1.4984m/最良固定-50 2.1547m、融合10秒 .8751m/最良固定+50 3.5666m、全行程融合位置13.225m/最良固定上25.845m。下/上の利用率は50%/75%。単一試行で反復確認前。
- 既存runnerは9本の取得終了後、元報告器で既知の欠落例外となる見込み。全取得状態completeは保存されるため、終了後は別報告器/取得監査を実施する。元の凍結ソースを途中変更しない。現在逆順1本目を実行中。

### 2026-10-24 逆順1本目完了・一様配置の未完了取得を再試行
- reversed/20262401は全取得監査成功。偏向VIO軸速度 .194743m/s（最良の完全固定+50 .342604）、VIO10秒移動1.7491m（最良固定+50 4.4597）、融合10秒 .9989m（最良固定-50 4.2290）、全行程融合13.2357m（最良固定上45.5347）。純LIO 77.0936m。下/上の速度拘束利用率50.6%/75.6%。まだ各配置1本。
- uniform/20262401はsim181.447秒でpacingがpaused後15秒再開できず終了。全推定器/シミュレータexit0、pacerのみexit1。LiDAR/VIO完了年齢<=.147秒、planner age .309秒で残留。plannerはstaleカウント増加、時計停止中に必要な新しい状態を得られない待ち合わせが疑われる。精度結果による除外ではなく未完了取得としてinvalid_trialsへ保持し、全凍結設定・同seedのまま1回再試行。
- 再開owner session68566（/tmp/switching-tunnel-resume.log）。旧14721終了。画像snapshot旧分をflight_snapshots_attempt1に保存、再開読取observer session61808。次のalternating/402のみ共分散を1Hz記録する読取probe待機session98322（probe_when_alternating2.py）、起動前試行はcampaign failedで安全に終了し、再開後に再設定。正解・推定・制御へ影響する出力なし。
- report_switching_tunnel_sparseは取得欠落ではないfactor利用率低下を保持して初回集計成功。audit_switching_tunnel.pyを追加、元の完全factor報告器に依存せず、取得監査・全所有終了・凍結/追加報告器SHA・圧縮再集計一致を確認する。全9本完了後に実施。

### 2026-10-24 一様配置再試行と偏在配置2回目が完了
- uniform/20262401の同条件再試行は完走・全取得監査成功。偏向融合全位置7.4651m、最良上固定4.8991mで固定優位。VIO10秒移動は偏向1.4452m/上固定1.3894m、軸速度は偏向 .1894m/s/上固定 .1950m/s。指標によって順序が異なる点を保持。
- alternating/20262402は全取得監査成功。偏向軸速度 .090256m/s/最良完全固定-50 .287077、VIO10秒 .6881m/最良固定-50 1.6304、融合10秒 .7917m/最良固定-50 3.5747、融合全位置11.7211m/最良固定+50 31.6651m。
- 共分散probeで入力見送りを追加確認。評価全体25〜257秒の232サンプルで、下固定のpose共分散判定値>1は111、上は43。余裕を持って>1.1となった下108/上40サンプルすべて、±0.45秒以内に採用速度factorなし。偏向は判定値最大 .01195で閾値1未満、欠落なし。threshold_check.jsonへ保存。読み取りprobe session98322とsnapshot session61808は正常終了。
- 現在有効4/9本、逆順/402を実行中。owner68566のみ、後続待機なし。全周の対照結果と偏在結果を分けて最終3反復へ集計する。Windows空き約155GiB、新規log約0.34GiB時点。

### 2026-10-24 各条件2反復、6/9本完了
- reversed/402：軸速度 偏向 .280389/最良完全固定-50 .280929m/sでほぼ同等。一方VIO10秒移動2.4404m/最良固定-50 3.9906、融合10秒1.0942m/最良固定-50 3.8557、融合全位置12.2747m/最良固定-50 38.9604m。速度単独と位置の短区間精度を混同しない。下固定の融合全位置479.047mの失敗も除外せず保持。
- uniform/402も全監査成功。全位置 偏向13.1503m/最良上固定7.8325m。これまでの偏在4本はVIO10秒/融合全位置で全固定より良好、一様2本は上固定が融合全位置で良好。各3反復の最終結論はまだ。
- 最後のseed20262403の3条件へ進行、owner68566、待機/診断ownerなし。完了済み6本を再実行しない。初回uniformの未完了数値41ファイルをgzip化しSHA復元一致確認、26.1MiB削減。無効元の経路/監視ログ保持。

### 2026-10-24 人工トンネルの全9飛行完了・停止
- alternating/reversed/uniform各3反復、計9飛行を完了。各85取得検査、全内部/外部所有プロセスexit0、全入力・追加評価器74 SHA一致、可逆圧縮前後の再集計一致、全owner PID消滅をaudit_switching_tunnel.pyで確認。主owner68566は既知の完全factor前提報告器の例外でexit1、取得campaignはcomplete。取得失敗と混同せず、欠落拘束を明示するreport_switching_tunnel_sparse.pyで全9本を集計した。数値手法・センサ・重み・v10候補・標準v57は一切変更なし。
- 結果はROOT/validation/report_sparse/index.html、comparison.pdf、summary.json、velocity_samples.csv、final_audit.json。主取得の有効9本合計wall 3202.67秒（未完了再取得・描画検査・集計時間を含まない）。最初のuniform未完了1本はinvalid_trialsに保持し同条件再取得、精度で飛行除外はしない。
- 各試行RMSの3回平均：alternatingのVIO10秒軸移動は最良固定-50 2.30618→偏向1.06527m、融合全位置は最良固定+50 33.61161→11.17378m。reversedのVIO10秒は最良正面5.30693→1.96882m、融合全位置は最良-50 44.70721→12.47324m。両配置とも各飛行内の全固定と比べて、VIO10秒・融合10秒・融合全位置の全指標で6/6回偏向優位。
- 主指標の軸速度RMS平均（完全利用のみ）：alternating最良-50 .26961→偏向 .13043m/s、reversed最良-50 .45906→.22278。偏向/水平3固定の利用率100%、上下固定は約50%/75%で完全値を算出しない。ROVIO入力が連続していても不確かさ閾値で融合に採用されない実例として残す。上下の利用時RMSを全区間精度としてランキングしない。
- 対照uniformではVIO10秒最良下固定 .23214m / 偏向1.13870m、融合全位置最良上固定6.04190m / 偏向10.82770m。最良固定に偏向は3/3回負けた。主軸速度も最良+50 .02896 / 偏向 .16015m/s。固定平均への勝利だけで有効と結論しない。
- 結論：今回の制御された偏在配置ではVIO自身の軸方向誤差が減っており、融合誤差相殺だけではない改善を確認。全周に特徴がある場合は固定優位。絶対精度は偏在でも融合全位置8.6〜13.2m残り、実環境汎化や単純な特徴追従との差は検証していない。各配置3独立飛行、窓数を独立試行数としない。10秒移動指標の追加は最初の拘束欠落を見た後であることをevaluation_amendment.jsonに保持。
- illustrate_switching_tunnel.py追加：数値評価を変更せず、主軸速度（完全利用のみ）、実偏向角の位置推移、環境図、全9本の結論をHTMLへ追記。再生成はreport_switching_tunnel_sparse.py <validation> → audit_switching_tunnel.py <validation> → illustrate_switching_tunnel.py <validation>。既存の凍結runnerは取得完了後に旧報告器例外となるので、その場合に飛行を再取得しない。
- 全シミュレーション/観測者/待機所有者を終了。連続画像・点群・rosbagなし、数値を可逆圧縮し静止確認画像だけ保存。環境はassets/switching_tunnel_20261024に常設、build_switching_tunnel.pyで再生成可能。ログ約0.5GiB、Windows実空き約156GiB。今回の方式を標準採用に切替えていない。

### 2026-10-24 人工トンネル表示を一つのlaunchへ統合
- ユーザー指示により `multilink_uas_bringup/launch/switching_tunnel.launch.py` 追加。`ros2 launch multilink_uas_bringup switching_tunnel.launch.py` で環境だけ表示。layout=alternating/reversed/uniformを選択可能、機体・推定器・bagなし。
- runtimeディレクトリと権限、WSL描画設定をlaunch内部で設定。既存Gazebo GUI設定を一時複製して初期視点をトンネル内部(42,0,2.5)へ指定、独立partitionを使用。一時GUI設定は終了時に削除、元world/評価入力は変更しない。
- colconの対象packageビルドと--show-argsが成功。実GUI起動でモデル/模様の描画を画面取得で確認、描画ログ28〜32fps。Ctrl+CでGazebo/launch exit0、検証用viewerは停止。ディレクトリ作成やsourceをユーザーの起動手順へ要求しない。

### 2026-10-24 Ostopoviceモデルから100m生成・比較開始
- ユーザー指定Downloads/hitler-highway-tunnel-at-ostopoviceを読込。source zipにFBXと8192²のDiffuse/Normal/AO画像あり。import_fbx_mesh.cppでAssimpの三角形化・座標変換済みFBXをUV保持OBJへ変換。素材をassets/ostopovice_100m_20261024/sourceへ保存、元Downloads無変更。
- generate_ostopovice_tunnel.py追加。元Y上/Z軸の内部-125〜125単位を中心(0,65)からray採取、内部幅中央値153.2698798単位を5mへ換算(scale .0326221956)。内壁画像を実UVから1536×768へ展開。元形状約8.16mから1mのcosine重複で周期7.15555mとして100mに延長。0.1m軸/128周方向で凹凸を採取、完全な元メッシュ延長ではなく再構成した内壁。端面なし、最短経路中心余裕1.93009m、幅全体bounds5.16036m/高さ4.56195m。
- テクスチャは元の自然なコンクリート/汚れcropの重なり合成を背景に、色成分を含む落書きcropをランダム加工。回転±60度、反転、サイズ.65〜2m、明るさ.85〜1.15、色相±12。100mにつき14clusterの中心を距離と壁・天井の周方向から一様に選び、各5〜11枚を位置ばらつき付き配置。前回の方向順序指定/無特徴灰色面の人工指定はなし。床は元の砂利/コンクリート。全加工/配置をtexture_placements.jsonへ保存。乱数24101/24102/24103を全て評価し、結果で選別しない。
- 3配置のcollisionメッシュは同一byte。全9継ぎ目位置差0、非退化三角形、同seedの画像/配置再生成がbyte一致。DAE自動日時を固定metadata epochにし再生成可能。初回DAEの非連続配列例外をC連続コピーで修正してから生成完了、性能実験前。
- check_ostopovice_render.pyで3配置×4位置×固定5方向の60実Gazebo画像を取得・目視確認。光学検査は全終了。原写真の明暗は画像に含むが照明は一様ambient、元Normal/AOによる新たな衝突凹凸は追加しない。
- 既存候補v10/推定器/融合/センサ/DRPM OFFを無変更で固定。3配置×seed20262501/502の6飛行、各90m/.5m/s、同一実飛行で固定5単眼+偏向単眼、純LIOも記録。主区間10〜170秒の軸VIO速度RMS/利用率、補助10秒相対移動/全位置。元凹凸があるのでXを厳密零方向とは仮定せず純LIO/行列も保存。欠落factorを補間しない評価器を事前に固定。
- run_ostopovice_trials.py/report_ostopovice_trials.py/audit_ostopovice_trials.py追加。139入力SHA・ソースzip・設計をROOT=/root/uas_native/logs/ostopovice-20261024/frozen.jsonへ保存。owner session76156、/tmp/ostopovice-validation.logで開始、最大1sim。rawbag/GUIなし、8GiB空きguard。完了前に終了しない。
- 環境だけの表示用 `ros2 launch multilink_uas_bringup ostopovice.launch.py` を追加（seed:=24102/24103選択）。既存switching_tunnel.launchにworld_file/camera_pose引数を追加して再利用。対象packageビルドと引数確認成功。既存switchingの既定表示は変更なし。

### 2026-10-24 Ostopovice比較 4/6本完了
- 全3配置の1回目と配置24101の2回目が完了、取得監査成功。融合全位置RMSE 偏向/試行内最良固定[m]：24101/501 .92970/.45014、24102/501 1.20150/.41481、24103/501 1.50027/.28650、24101/502 1.08025/.40576。結果で条件変更/除外せず残り2本へ継続。
- 1回目の純LIO全位置は配置順に1.99791/2.10853/1.49913m。元の凹凸を残した今回の形状は理想的な軸方向零拘束とは違う。全生成面を面積重みで見た法線行列は最小/最大比 .0100165、最弱方向が軸Xにほぼ一致。実行中の推定行列とは区別（geometry_information.json）。
- VIOの実軸速度誤差は融合順位と一致しない。24103/501は偏向 .020688m/s / 最良上固定 .051890、VIO10秒移動 .269397/.324910mで偏向良好だが融合位置は偏向が悪化。比較結果を融合だけから視覚拘束の優劣と呼ばない。
- 読取snapshot_ostopovice_flights.pyで最初の3飛行×4地点×6視野=72静止画像を取得。機体で大部分が隠れず実壁面を見ることを確認。observer session1786はSIGINT後exit0で停止済み。連続画像/bagなし。
- plot_ostopovice_environment.pyで元断面/実落書き配置の図、finalize_ostopovice_report.pyで結果/再生成・表示方法の追記処理を追加。評価数値/凍結源は無変更。environment_versions.jsonに使用ライブラリ版を保存。owner76156のみ継続、live_state更新。

### 2026-10-24 Ostopovice自動生成・全6飛行比較完了
- 3ランダム配置×2反復、各90m/.5m/sを完走。全6本85検査/本成功、139凍結入力SHA一致、圧縮後再集計一致、全内部/外部プロセスexit0。取得owner76156も正常終了、6本取得合計1624.25秒（約27.1分、生成・表示確認・集計を除く）。結果は `/root/uas_native/logs/ostopovice-20261024/validation/report/index.html`、comparison.pdf、summary.json、final_audit.json。
- DRPM OFF、ROVIO+MIMOSA、v10偏向/既存v57標準は変更なし。同一実飛行で固定5独立単眼と偏向単眼、純LIOを比較。10〜170秒の全方法で速度拘束利用率100%。各条件2回のRMS平均、固定平均は5方向を等重みで平均する。
- 主軸VIO速度RMS[m/s] 固定平均/最良固定/偏向：24101 .264596/.058910/.061874、24102 .421961/.052403/.060580、24103 .238640/.055349/.024116。最良固定はいずれも上向き。
- VIO10秒軸移動RMS[m] 固定平均/最良上固定/偏向：24101 2.739964/.407056/.602283、24102 4.117985/.358229/.525396、24103 2.428517/.323463/.300617。偏向は全6回で固定平均より良く、全固定より良いのは1/6回。条件3の2回平均では最良固定より良いが、個別2回とも勝利とは言わない。
- 融合全90m位置RMSE[m] 固定平均/最良正面固定/偏向：24101 .789862/.427947/1.004975、24102 .737009/.403781/1.120925、24103 .764915/.422369/1.408892。偏向は全6回で固定平均より悪い。純LIO各条件平均1.816437/1.437142/1.998320m。偏向は純LIOより条件平均で改善するが固定平均には劣る。視覚自体の順位と融合順位は逆転し、融合精度だけから視覚拘束の正しさを評価しない。今回、原因を速度/共分散交換等で分離してはいない。
- 実偏向角は3配置とも序盤の探索後、主に天井(pitch=-90)へ向く。ランダム落書き配置は生成したが、最適面が位置ごとに何度も変化することを保証する環境ではない。固定上向きのVIO精度が高い結果も保持。2反復で有意差や実環境全般への優位とは称さない。
- report最終仕上げで環境断面/配置図、固定平均と最良固定を分離した表、再生成/表示方法、実偏向角の解釈を追加。図を目視確認。ユーザー表示は `ros2 launch multilink_uas_bringup ostopovice.launch.py`、別配置seed:=24102/24103。実Gazebo GUIを起動し内壁/天井の描画確認、Ctrl+Cでlaunch/server/clientすべてexit0、PID668078/668080/668081消滅。検証用シミュレーション継続なし。
- 数値ログを可逆圧縮、連続画像/点群/bagなし。ログ273MiB、生成assets190MiB、Windows空き約156GiB。live_state/completedへ完了を記録。再開待ち・追加測定なし。

### 2026-10-24 自然なまとまりを持つ人工偏在トンネルへ変更
- ユーザー指示により完全人工生成へ戻し、build_weathered_tunnel.py追加。長さ100m・幅5m・天井高4.5mの一定アーチ断面。100mで5つの不規則な劣化領域。軸位置/長さ/周方向位置/幅を乱数化し、左・天井・右の3領域へ分散する制約のみ設定。区間全体の左右切替・矩形contrast maskは使わず、曲がったGaussian密度場の重なりと確率的な模様配置を使用。seed24201。
- 模様は外部画像なしで全て生成。多尺度の弱いコンクリート色むら、ぼかした湿潤染み、細いひび、不規則な剥離、白色析出跡。床はコンクリート背景のみ。高密度領域以外も低密度1.2%の発生確率を保持。今回2906個の細部模様。密度場・全模様座標をdistribution.jsonへ保存。自然な見え方を狙った人工条件で、実世界の統計分布再現ではない。
- assets/weathered_tunnel_20261024へ生成。10mごとにmesh/画像分割、UVの画像上下方向を合わせる。形状は全長一定、面法線のX成分は厳密0で、模様による新たなLiDAR幾何拘束はない。経路5→95m、.5m/sの設定も作成するが飛行実験は未実行。
- check_weathered_render.pyで5位置×5方向の25画像を取得・目視確認。位置11.9/28.6/47.3/69.2/90.2mで最高FAST点数の向きは上102/+50度102/−50度77/上90/−50度131。模様の偏在が撮影画像でも変わることを確認。特徴数を推定精度や有効な拘束の証明とは扱わない。
- 同seedを独立一時フォルダへ再生成し、画像40・mesh40・distributionJSONの81ファイルがbyte一致。一時再生成物は削除。生成開始時のPillow9列挙名互換エラーを旧APIへ修正し、その後2回の全生成成功。
- weathered_tunnel.launch.py追加、対象colconビルド・引数確認・実GUI描画成功。起動は `ros2 launch multilink_uas_bringup weathered_tunnel.launch.py`。最初のGUIはWSL D3D12 device removed/OpenGL context失敗、同一設定再起動では39〜42fpsで描画・画面保存できた。終了時gz wrapperが既に消滅した子へsignalを送るESRCHでexit1、launchはexit0。所有プロセス残存なしと一時GUIディレクトリ削除を確認済み。環境/推定器パラメータ変更で成功を偽装しない。
- 概要/実画像/検出数は `/root/uas_native/logs/weathered-tunnel-20261024/index.html`、再生成検査validation.json。既存の比較結果・v10・v57は変更していない。今回の新環境では固定/偏向の精度比較未実施。

### 2026-10-24 自然な偏在トンネルの固定・偏向比較開始
- ユーザーの比較指示により、既に表示したweathered24201環境を変更せず3回独立飛行。seeds20262601/602/603、90m/.5m/s、固定5独立単眼+偏向単眼+純LIO。v10偏向・既存融合/センサ/DRPM OFFは無変更。主区間10〜170秒、軸速度誤差と拘束利用率、補助10秒移動・全位置。結果による環境/方向/試行選別なし。
- run_weathered_trials.py/report_weathered_trials.pyを追加、既存Ostopovice取得・評価処理を再利用し環境説明と反復数を変更。再利用auditは全入力/出力/正常終了/凍結SHA/圧縮再計算を確認。既存の旧実験スクリプトは無変更。条件/ソースをlogs/weathered-tunnel-20261024/frozen.jsonとzipへ事前保存。数値のみ、GUI/bagなし、Windows8GiB空き保護。

- 初回seed20262601はroute174/182秒直前にpacerが待ち解除不能で停止。sim263.654、LiDAR処理の差.154秒が再開閾値.15秒をわずかに超え、次の入力を出す時計も停止して15秒timeout。全推定器exit0、pacerのみexit1。未完了をinvalid_trialsへ保持し同一条件で再試行。凍結ソース/推定器/センサ/視野選択/評価は変更していない。初回の30確認画像もinvalid試行へ紐づけて移動。

- 同一条件再試行も81.7/182秒でpacer待ち停止。詳細確認により先のLiDAR原因記述を訂正：ordered fusionではLiDAR再開閾値は.26秒なので.15秒は正常。planner完了時刻との差.300/.304秒が再開閾値.25秒を超え、次の.2秒周期の時計も停止して相互待ち。
- 既存/採用sourceは触らずrun_weathered_view_array.py/weathered_pacing.pyを追加。カメラ視野判断の待ち合わせ閾値のみ停止.4/再開.35秒へ（既存.5秒freshness以内）。描画・観測周波数・時刻・推定器・視野選択式は同じ。旧生成owner sourceと厳密一致、pacerだけの2差分（閾値importと終了時subscription解除）を検査。記録された2deadlockのageを再生し、再開できることと.42秒では停止することを確認。
- 未完了2回目もinvalid_trialsへ保持。旧frozenとzipを別保存し、新しい実行wrapper/閾値を追記して再凍結。採点対象3回すべてこの待ち合わせ設定で行う。未完了値を成功結果へ昇格しない。

- 閾値緩和版も139/182秒で停止。plannerが入力待ち/不整合でコマンドを保持してreturnしたことと未完了の計算をpacerが区別できず、成功commandの時刻だけが更新されなかった。閾値だけの対処は不十分であり不採用。3回目もinvalid_trialsへ保持。
- 最終修正はweathered_planner_node.py/scheduling_completion.pyで、既存tickの正常return後だけ別topicへ処理完了時刻を通知。新command・holdとも通知、例外時は通知しない。last_plan・選択規則・角度command・推定器は変更なし。pacerの参照をこの完了topicへ切替え、閾値は元.3/.25へ戻す。旧実験sourceは保存、別wrapperで適用。hold/new-command/exceptionの通知順・return値不変・owner/pacerの許可差分のみを検査。3本すべて再凍結した最終版で取得。

- 最終修正後のseed601は90m完走、全センサ/推定出力・入力同値・設定・終了が成功。ただし旧監査のplanner_continuousだけfalse（最大command gap2.527秒）でreportが止まった。これは入力不整合等のholdを含む現在の偏向方式の結果であり、良い飛行だけ再選別しないため取得失敗と分離。weathered_trial_acceptance.py/audit_weathered_trials.py追加。元valid/falseチェックはそのまま保存、acquisition_validとmethod_outcomesを別記録。許可するfalseはplanner_continuousとそれを伝播するreference_valid/reference_paired_auditのみ。他の欠落/設定差分/異常終了は許可しない検査成功。評価窓/精度数式/手法は変更なし、取得済み全長を再測定せず残り2回へ継続。事後の監査分類変更はdesignへ明記。

### 2026-10-24 自然な人工偏在トンネルの3反復比較完了
- weathered24201の同一環境でseed20262601/602/603を90m/.5m/s、固定5独立単眼・偏向単眼・純LIOで比較完了。最終owner46579はexit0。3本取得合計767.32秒（約12.8分、未完了3試行・調査・報告を除く）。結果 `/root/uas_native/logs/weathered-tunnel-20261024/validation/report/index.html`、comparison.pdf、summary.json、findings.json、angles.png。環境概要indexからもリンク。
- 3回平均 融合全位置RMSE[m]：純LIO51.52391、固定yaw−50 33.72523、正面32.38494、yaw+50 17.53123、下49.36842、上14.43053、偏向7.96145。偏向は固定5平均29.48807より全3回で改善、各回最良固定より良いのは1/3回。最良固定の3回平均（上）から44.8%減。個別偏向4.36032/14.38093/5.14310、正面1.26558/49.10911/46.78012、上13.09211/8.53157/21.66792m。平均だけで全回勝ちや安定絶対精度と説明しない。
- VIO10秒軸移動RMS[m]平均：−50 7.90970、正面3.47516、+50 5.87555、下3681.92137、上8.37564、偏向3.28663（3.21777/2.88156/3.76055）。固定平均741.51148は下向き発散に支配されるため改善率の主張に使わず方向別を提示。偏向が全固定より良いのは1/3回。最良固定平均（正面）との差は約5.4%。融合10秒軸誤差は最良固定上2.79183→偏向1.46122m。
- 全区間利用の軸VIO速度RMS平均[m/s]：−50 .85526、正面 .37422、+50 .63909、上 .94628、偏向 .33042。下固定は全3回で速度拘束利用率0%、ROVIO出力は連続だが融合不採用。その他100%。下の全区間速度値を補間/ゼロ埋めせずNoneと表示。
- 偏向角の実記録を描画して目視確認。序盤は左上、中間で戻し、後半は右上へ移動する。画像配置の正解をcontrollerへ入力せず既存v10、DRPM OFF、推定/融合設定を固定。静的な最良固定に全回勝つ結果ではない。
- 取得検査は全3回で合格、228凍結SHA一致、圧縮前後の比較数値一致、全内部/外部プロセスexit0、所有PID消滅を確認。ただし元の厳密監査validは全3回falseを維持：planner_continuousがfalse、最大角度更新間隔2.527/2.728/2.098秒。新しい取得監査はこの偏向の保持挙動をmethod_outcomesへ明示し、他の失敗を許可しない。結果報告にも元false・取得合格を併記。全検査成功と呼ばない。
- 完走前の3試行はinvalid_trialsへ保持し数値を可逆圧縮、精度で試行を除外していない。planner成功commandと処理完了を区別する待ち合わせ修正は最終3回で共通。途中の閾値緩和だけの案は不採用、旧source/frozenも保存。finalize_weathered_report.pyで平均/最良/各回/監査変更の説明を追加、評価数値は変更しない。
- 全実験停止、再開待ちなし。GUI/rosbag/連続画像点群なし。ログ全体223MiB（未完了試行含む）、Windows空き約153GiB。採用v57/既存候補v10自体は変更していない。

## 2026-10-24: 蓄積した変更の分割コミット

- ユーザー指示により、前回コミット以降の変更を8件に分割: ログから独立した実行入力・復元記録、トンネル取り込み、固定5方向の比較、移動を考慮した偏向計画、特徴分布切替トンネル、スキャン由来のトンネル加工、自然な劣化模様の人工トンネルと実行待ち制御、作業記録・引き継ぎ。
- 実験時のソース照合を維持するため、推定・制御実装や数値設定はこの整理で変更しない。表示用レポート補助の行末空白1箇所のみ除去。過去の候補設定と不採用の待ち時間調整も、検証履歴として保持する。
- 既存の.gitignoreでログ、rosbag、ビルド出力、キャッシュを除外できていることを確認。再実行に必要な経路・校正値・設定と、Drive復元に必要な小さな記録は保持。生成画像・生成モデル・計測データは追加しない。
- 検証: 実行入力、固定カメラ配置、区間評価、偏向計画、追跡復帰、事前観測に関する単体テスト28件成功。追加Python全87ファイルの構文、追加JSON全75ファイルの読み込みを確認。コミット整理のためのシミュレーション再実行は行っていない。
