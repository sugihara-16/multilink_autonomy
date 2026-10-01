> 2026-09-30 飛行・SLAM追記：統合WSで `mode:=flight_slam` を実装・検証済み。
> LiDAR＋IMUのみをMIMOSAへ入力し、約1.5 m上昇→2 m四方の移動→着陸を確認した。
> 飛行制御はGazebo真値、SLAM推定による閉ループ制御・ループ閉じ込みは未実装。
> 採用結果は `native/logs/20260930-154020-97154/`（位置RMSE 0.01573 m）。
> 操作手順と制約は統合WS直下のREADMEを参照。以下の「MIMOSA未導入」は過去の記録。

> 2026-09-30 移設追記：`uas_native` の全ファイルは
> `/root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/` へ移動した。
> `/root/uas_native` は互換symlink。現在の操作手順はワークスペース直下の `README.md` を参照。
> 対話Bashは `.bashrc` でROS2＋統合WSを自動読み込みする。
> Python仮想環境は端末では有効化せず、stackのRLプロセス内だけで有効化する。
> `ros2 launch multilink_uas_bringup unipilot.launch.py` が現在の起動入口。
> 以下に残る旧パス・手順・「自動sourceしない」等の記述は移設前の履歴。

# 研究・実装・実行環境の引き継ぎ

> 運用変更（2026-09-29）：新WSL Ubuntu-22.04の既定ログインを **root** とし、構築した環境一式を **`/root`（`~`）へ移動済み**。起動は `cd ~/uas_native` → `./run.sh stack`。ビルド済みファイルの旧絶対パスを保つ互換用symlinkだけを `/home/developer` に残している。参考コピー・過去のログの旧パスは構築時の記録。developerアカウントは削除していない。

更新: 2026-09-30。実装・環境の確認記録は2026-09-29時点。2026-09-30に今後の作業場所と統合ワークスペースの方針を追記。

**今後の主題は、相補的なセンサをDRAGONの複数リンクに分散し、関節運動で観測配置を変えるactive perceptionである。LiDARを全リンクに複製すること自体は、今後の主題にしない。** これまでの4 LiDAR実装は、可動センサの統合と比較評価のための既存資産として残す。

直近では、既存UniPilot / Unified Autonomy Stack（UAS）を基準に環境ごとの性能を調べる準備として、新しいWSL `Ubuntu-22.04` にネイティブ実行環境を構築した。**センサ・地図生成・プランナー・RL/CBFの接続確認まで完了。MIMOSAを使ったSLAM評価と自律飛行評価は未実施。aerial_robotは新環境へまだ導入していない。**

## 1. 研究テーマと、次に明らかにしたいこと

### 1.1 目的

DRAGONのようなロータ分散型マルチリンク機体に、UniPilotで使うような**異なる性質を持つセンサ群（complementary sensors）**を分散配置する。環境に応じてリンク姿勢を変え、センサの向き・位置・相対配置を調整し、状態推定・地図生成・タスク遂行を安定させられるかを調べる。

センサの具体的な種類、台数、取付リンクは未確定。LiDAR、カメラ、IMU、必要に応じてRadarなどが検討対象だが、全部を追加する決定はしていない。「複数モダリティ」と「同種LiDARの台数追加」は区別する。

マルチリンク機体を採用する本来の理由は、マニピュレーションや狭隘空間の潜り抜け能力である。SLAMのためだけに機体を複雑化する説明は弱い、というのが議論の出発点。これらの機体能力を、分散したセンシングでどう強化できるかを研究上の文脈とする。

### 1.2 ユーザー・教授との議論で整理した仮説

- 相補的なセンサを各リンクへ分散すると、搭載重量を分散でき、関節運動で配置を変えられる。ただし重量分散の実際の利益は、配線・追加機構・慣性・制御負荷も含めて未評価。
- 森林・トンネルなどで観測の拘束が方向によって弱い場合、配置変更によって不足する情報を取得できる可能性がある。
- Hessian等から弱い推定方向を把握し、それを補う観測が得られる関節運動を計画する、というactive perceptionを将来の方法論候補にする。**このプランナーはまだ実装していない。**
- gimbalは主に回転を変える。マルチリンクでは、制約付きながら並進を含むセンサ配置変更ができる。この追加自由度が、実際に必要な環境条件を示したい。マルチリンクでも任意の位置・姿勢を実現できるとは仮定しない。

検証したい中心命題は次の2つ。

1. 固定配置では推定が退化・不安定化するが、センサ配置を変えると改善する環境があるか。
2. その中に、gimbalによる回転だけでは不十分で、並進を含むマルチリンクの配置変更が有効な環境群があるか。

いずれも**未実証の仮説**。トンネル等の環境名だけで成立を主張しない。観測可能な幾何・テクスチャ・遮蔽・視野・照明等の条件、機体とセンサの可動範囲を記述して議論する。単にセンサを回しただけで、環境に存在しない拘束が生まれるわけではない。

### 1.3 これまで退けた、または不十分と考えた説明

- 「LiDARを増やせばSLAMが安定する」だけでは不十分。基部の単一LiDARで十分な環境も多く、追加質量や可動外部パラメータの誤差を正当化できない。
- 「先頭リンクで先に地図を作り、後続リンクが通る」だけでは分散センシングの必要性が薄い。後続がほぼ同じ場所を通るなら、既存地図で足りる可能性を検討する。
- 特殊な曲がり角・開口部を1つ作って有効性を示すだけでは一般性が弱い。
- 機体全体の並進・回転、固定配置の工夫、gimbalで同じ利益が得られないかを必ず比較する。
- 画像による滑り・変形観測、手先カメラ等を採用するだけでは、既存の地上マニピュレーションとの方法論上の違いが弱い。
- エンコーダより高精度に関節角をSLAMで推定することを、今回の主要な利点には置かない。関節角は既存エンコーダを使う。

NTNU ARLのactive perception、UA-MPC、AEOS、RoboPanoptes等を会話で検討したが、今回の問題設定をそのまま解決する先行例と結論したわけではない。本資料では、過去の文献回答の数値・主張を再検証なしに転記しない。論文で引用する段階では一次資料を確認する。

### 1.4 これから行うこと

ユーザーが決めた順序は、**様々な環境を用意 → 既存UniPilot / UASで基準性能を測る → DRAGONへ相補センサを分散して比較**。

そのための再開案は以下。実験条件や手法まで確定済みという意味ではない。

1. 新WSLの既存構成を確認し、評価対象の推定器・センサ入出力を確定する。UAS標準シミュレーションの真値姿勢を、SLAMの推定結果と誤認しない。
2. 必要なIMU・カメラ等をシミュレータで有効化し、時刻、座標系、ノイズ、外部パラメータを確認する。MIMOSA等の推定器を実際に接続する。
3. 構造の豊かな部屋、長い廊下・トンネル、森林等を候補に、固定配置の基準性能を収集する。現在準備できているUASワールドはSubT洞窟であり、多環境ベンチマークの整備は未完了。
4. 観測の弱くなる条件を見つけてから、同じセンサ構成で「固定」「gimbalによる回転」「関節による回転＋並進」を比較する。必要なら機体全体を動かす条件も追加する。
5. 基部軌道を揃える比較と、タスクを自由に遂行する比較を分ける。センサ台数・点数・処理量増加の利益と、配置変更そのものの利益を分けて評価する。
6. 有効な環境群が確認できた段階で、弱い推定方向と候補動作による観測変化を結ぶ関節計画へ進む。Hessian改善だけでなく実際の推定誤差・失敗率・タスク達成・計算負荷で確かめる。

現時点では**aerial_robotのROS2移植・新WSLへの導入は後回し**。この資料を読んだだけで、その作業や飛行実験を開始する必要はない。

### 1.5 今後の作業場所とワークスペース構成（2026-09-30追記）

**新WSL `Ubuntu-22.04` のroot環境を今後の作業場所とし、UASとaerial_robotを接続する専用ワークスペースを設ける方針。** UAS本体とaerial_robot本体はそれぞれのワークスペースで管理し、日常の研究開発は統合用の `~/multilink_autonomy_ws` を中心に行う。

以下は目標とする配置案。`aerial_robot_ws`、`multilink_autonomy_ws`およびそのパッケージは、この資料への追記では作成していない。パッケージ名・分割は必要に応じて調整し、最初から全てを作らなくてよい。

```text
/root/
├── unified_autonomy_stack/       # 既存：UAS本体と内部のworkspaces
├── aerial_robot_ws/              # 今後：aerial_robot本体と必要な依存
├── multilink_autonomy_ws/           # 今後：研究・統合の中心
│   └── src/
│       └── multilink_autonomy/
│           ├── multilink_uas_bringup/   # 全体起動、接続設定、実験条件の切替
│           ├── multilink_sensors/       # センサ配置、可動外部パラメータ処理
│           ├── multilink_simulation/    # Harmonic向け機体モデル、環境
│           ├── multilink_perception/    # 推定器接続、active perception
│           └── multilink_evaluation/    # 記録、比較、作図
├── ros1_noetic/                  # 既存：GBPlanner等が使うROS1依存基盤
└── uas_native/                   # 既存：環境構築、起動、接続検証の補助ファイル
```

役割分担と実装時の指示：

- 統合launch、研究用のセンサ設定・環境・比較評価コードは統合ワークスペースに置く。UASやaerial_robotの内部変更が必要な場合だけ、それぞれのリポジトリを変更し、差分を管理する。本体のソースを統合側へ丸ごと複製しない。
- `uas_native`はUAS公式リポジトリでも、今後の研究コードを全て置く場所でもない。現在の動作確認済み入口を維持し、統合用launchが整った段階で必要な設定を整理して引き継ぐ。
- ワークスペースを追加するだけではROS1／ROS2やGazeboの互換性は解決しない。aerial_robotをどのROS構成で動かし、Harmonicの物理・センサ・制御へどう接続するかを、導入前に確認する。現時点で移植済み・対応済みとは扱わない。
- ROS1とROS2のパッケージを無条件に同じワークスペースへ混ぜたり、両方のsetupを同じ端末へ重ねたりしない。両ROSが必要なら、ビルドと実行環境を分け、bridgeするtopic・型・時刻・TFを明示する。統合側にも両ROSの独自パッケージが必要になれば、ROSバージョンごとにワークスペースを分割する。
- 旧 `Ubuntu-20.04` の `/root/ros_ws` と `/root/mimosa_ws` は、過去実装・実験の参照と再現用として保持する。新WSLの同名パスとは別物であり、そのままHarmonicへ持ち込めるとは仮定しない。

進行順序は、まず新環境で **UniPilot＋実際の推定器** を動かし、記録・評価の基盤を作る。その後aerial_robot／DRAGONを接続する。最終的には、統合ワークスペースから「UniPilot基準構成」「DRAGON固定センサ配置」「DRAGON可動センサ配置」を共通の環境・評価条件で切り替えて起動できる構成を目指す。今回の依頼はこの方針の記録であり、ワークスペース作成・移植・飛行の開始指示ではない。

## 2. 2つの環境の位置づけ

パスはディストリビューションごとに別。新WSLの `/root` に旧WSLのファイルがあるわけではない。

| 項目 | 旧環境：DRAGON / MIMOSA | 新環境：UniPilot / UAS |
|---|---|---|
| WSL名 | `Ubuntu-20.04`（Windows側の既定のまま） | `Ubuntu-22.04` |
| ユーザー | `root` | `root`（構築時はdeveloper） |
| ROS / Gazebo | ROS1 Noetic / Gazebo Classic 11.15.1 | ROS2 Humble + ROS1 Noetic / Gazebo Harmonic、NTNU gz-sim 8.9.0 |
| ソース | `/root/ros_ws`、`/root/mimosa_ws` | `/root/unified_autonomy_stack` |
| 起動・管理 | `dragon_mimosa`パッケージ | `/root/uas_native` |
| SLAM | MIMOSAのLiDAR＋IMUを実行・比較済み | 標準構成ではMIMOSA未起動、真値odometry使用 |
| 自動飛行 | 通常launchではしない。明示的なdemo実行は飛行する | 検証ランチャーは制御出力を機体から切断 |

両環境とも同じWindowsホストのRTX 4070 Laptop GPU / WSLgを使用。GUIを出さない場合でもGPUセンサの描画コンテキストは必要。

## 3. DRAGONでMIMOSAを使うために実装したこと

以下の相対パスは、原則 `/root/mimosa_ws/src/mimosa/dragon_mimosa` 基準。読解用ソースのコピーは [handoff_reference/legacy_source](handoff_reference/legacy_source) にある。

### 3.1 bringup・単一LiDAR連携

- `dragon_mimosa`パッケージを作成。Dragon v1.5、Livox MID360相当点群、FC IMU、MIMOSA設定、Gazeboワールドを統合した。
- `fc_imu_bridge.py`でFCメッセージを `sensor_msgs/Imu` へ変換。`livox_custom_bridge.py`でLivox CustomMsgの点ごとの時刻を保ったPointCloud2へ変換する。実機接続は未検証。
- `config/mimosa.yaml`はLiDAR geometric / IMUを有効化。Radar、外部odometry、Ouster向けphotometricは無効。姿勢の真値をMIMOSAへ渡さない。
- `mapping_room.world`は16×12 mの壁・柱・箱・仕切り・天井梁を持つ室内。飛行中も鉛直方向の拘束が得られるように梁を設けた。**退化を起こしやすい一般環境の代表という位置づけではない。**
- Dragonの `bringup.launch` にモデル選択、xacroオプション、Gazebo GUI / RVizの独立指定を追加した。
- Dragon側RVizにマップ、推定軌跡、FC座標、統合／個別点群をまとめた。MIMOSA用RVizを別に開かない。
- `align_visualization.py`が最初の同期姿勢だけで表示座標を合わせる。以後の推定誤差を追従補正しない。MIMOSAの地図Publisherはラッチ化済み。
- 重複arming入力で状態がSTARTへ戻り、FC指令が止まる不具合を `flight_navigation.h` で修正。ARM_OFF以外のstartを無視し、遅延したarming応答が飛行状態を巻き戻さないようにした。通信failsafeそのものは維持。実キーボード入力で確認済み。

### 3.2 過去の4 LiDAR＋4 IMU分散構成

**各センサを独立したファクターとして追加した実装ではない。既存の因子の残差・重みを多センサ用に変更したものでもない。** 前処理で「仮想LiDAR 1台＋仮想IMU 1台」にまとめ、既存MIMOSAへ入力する。

```text
各linkのLiDAR ─┐
URDF＋関節角 ─┴─ 点ごとの時刻でFC座標へ変換 ─ 統合点群 ─┐
各linkのIMU ──── 関節運動・レバーアームを補正 ─ 仮想IMU ─┤→ MIMOSA 1台
                                                          └→ 地図・FC姿勢
```

| 内容 | 実装・仕様 |
|---|---|
| センサ配置 | link1〜4に同一相対配置。LiDAR光学原点 `[0.132, 0, 0.077] m`、IMU原点 `[0.3245, -0.001, 0.028] m`、取付回転は単位回転 |
| 基準body | link2上のFC原点。MIMOSAでは `mimosa_fc` |
| 生LiDAR | `/dragon/linkN/livox/lidar`、各10 Hz |
| 生IMU | `/dragon/linkN/imu`。link2はSpinalと同じGazebo IMUの生値を分岐。他リンクには独立IMU追加 |
| 関節 | `/dragon/joint_states`。`history.py`で時刻履歴、`kinematics.py`でFKと微分項 |
| 統合 | `scripts/multilink_frontend.py`、`config/multilink.yaml` |
| 出力 | `/dragon/multilink/points`（10 Hz）、`/dragon/multilink/imu`（100 Hz）、`/dragon/multilink/status` |

LiDARは各点の取得時刻の関節角を補間してFC座標へ移す。スキャン間の時刻差を点フィールド `t` [ns] に保持し、機体全体の運動のdeskewはMIMOSAに任せる。入力 `line` 上位2ビットに元リンク番号を保持する。TFやGazebo真値からセンサの相対姿勢を与える構成ではない。

IMUは、相対姿勢による回転、関節由来の相対角速度、レバーアームの接線・遠心加速度、関節運動に伴う相対速度・加速度とコリオリ項を補正した後に重み付き統合する。単純平均ではない。各IMUの姿勢真値は使わず、接地時に値を固定するFCの前処理も避ける。

必要な時刻を挟むIMU・エンコーダ標本がない場合、最新値で代用しない。4台のLiDARは50 ms以内で組み合わせ、対応する統合IMUを先に配信する。さらにMIMOSAのLiDAR managerにIMU到着待ちを追加し、ROSのトピック間配送順序の違いによるバッファ範囲外参照を防いだ。Dragon設定では実時間0.5秒でtimeoutし、間に合わないスキャンはグラフ登録前にスキップする。本体既定は0、単一スレッドのbag処理では待機を有効化しない。

個々のIMUのバイアス・時刻ずれ・取付誤差をオンライン推定する仕組みはない。統合後の共通IMUバイアスをMIMOSAが推定する。実機では別途校正が必要。ループ閉じ込みやMIMOSAによる飛行制御は追加していない。

**既定と比較Bの違いに注意：** `gazebo.launch` は4 LiDAR＋4 IMU統合が既定。一方 `demo.launch` と後述A/B比較は、4 LiDARでも **link2の生FC IMU 1系統**をMIMOSAへ渡す。LiDAR台数の差を比較するためで、多IMUの利益を示す実験ではない。

### 3.3 GPU LiDARと、変形後バイアスの修正

`src/gazebo_livox.cpp`をCPU `ray` / GPU `gpu_ray`両対応にした。既定はGPU。出力は360×32レイ、水平360°・垂直−7〜52°、10 Hz。GPU内部1437×125から縦横4サンプルごとに抽出する。ヘッダは処理完了時刻でなく `LastMeasurementTime()`。

このセンサはMID360の視野の近似。実機の非反復走査・強度特性を再現せず、Gazeboの同一スキャン内の相対時刻 `t` は0。GPUはvisual、CPUはcollision形状を観測するので、両者が一致するとは限らない。

初期計測では、GUI＋RVizありのRTFがCPU 0.28 → GPU 1.00に改善。ただしこれは当該ワールド・マシンでの結果であり、将来の性能保証ではない。

その後、4 LiDARの変形後に位置誤差が増える原因を調べ、**Gazebo Classic 11のGPU距離画像と走査角の1内部サンプルずれ**を特定した。関節振動が原因と結論したわけではない。プラグイン内で再投影キャンバスを半画素移動して画素中心へ合わせ、既に修正済みの実装には二重適用しない。システムのGazebo本体は書き換えていない。

無ノイズの箱で面距離RMSEは、水平32.13 → 1.39 mm、傾斜28.35 → 1.48 mm。CPU/GPU・2姿勢等の6条件で回帰テスト済み。`test/test_gpu_lidar.py`を参照。初期の [PERFORMANCE.md](handoff_reference/legacy_docs/PERFORMANCE.md) の幾何誤差値は、この修正前の記録として読む。

### 3.4 基部を保持する変形と地図更新

- 元の `rosrun dragon transformation_demo.py` は関節と基部姿勢を指令するため、基部の位置・姿勢が一定という比較条件にならなかった。
- `dragon_mimosa/scripts/transformation_demo.py`で、元デモの姿勢指令を別topicへ逃がし、FCの位置・姿勢を100 Hzで保持する補助制御を追加した。既存制御がCoG目標を使うため、FC誤差からCoG目標を更新する。Gazebo内の姿勢を強制固定する方法ではない。
- 基部変位は完全には0にならない。修正後飛行の変形中は位置RMS 5.71 mm / 最大14.13 mm、姿勢RMS 0.845° / 最大2.02°だった。
- 基部移動だけを条件にした地図キーフレームでは、関節だけ動かしても地図が更新されなかった。MIMOSAの `geometric.*` / `geometric_config.*` に `map_keyframe_max_interval` を追加した。
- Dragon設定は並進0.15 m、回転5°に加え、センサ時刻で0.5秒経過した最初のスキャンを追加。本体既定0は従来動作。A/B両条件に同じ設定を使う。関節や情報利得に基づく能動的なキーフレーム選択ではない。

### 3.5 旧環境の起動手順

既にビルド済み。初回再構築時のunderlayは `/root/ros_ws/devel_catkin`。`devel` や `devel_isolated` と取り違えない。

```bash
# 旧WSL Ubuntu-20.04内
cd /root/mimosa_ws
source /root/ros_ws/devel_catkin/setup.bash
catkin config --extend /root/ros_ws/devel_catkin
catkin build mimosa dragon_mimosa -j4 -p2
source /root/mimosa_ws/devel/setup.bash

# Dragon、Gazebo、MIMOSA、Dragon側RVizをまとめて起動。armingしない。
roslaunch dragon_mimosa gazebo.launch
```

`multilink:=false`で従来の単一LiDAR構成、`gpu_lidar:=false`でCPU、`gazebo_gui:=false rviz:=false`で画面を省く。**`multilink:=false`の通常launchはFCメッセージ変換経由のIMUであり、保存済みA/B比較の「同じ生FC IMU＋同じ点群から切り出すA」と全く同じ条件ではない。**

比較B相当の飛行デモは次の2操作。bringupを先に別途起動しない。ROS masterも `roslaunch` が必要に応じて起動する。

```bash
# 端末1：このlaunchだけでは飛ばない
source /root/mimosa_ws/devel/setup.bash
roslaunch dragon_mimosa demo.launch

# 端末2：明示的な飛行実験時だけ実行。ここから自動arming・離陸する
source /root/mimosa_ws/devel/setup.bash
rosrun dragon_mimosa transformation_demo.py
```

デモは離陸 → 2 mホバー・基部保持 → 15秒待機 → 変形 → 15秒待機 → 復元 → 着陸。bag、イベント、パラメータ、URDF、ログは毎回 `~/.ros/dragon_mimosa/transformation_*/` に保存。旧 `mapping_flight.py` やartifactsの飛行スクリプトも指令を送るので、起動確認の代わりには実行しない。

## 4. 単一LiDARと4 LiDARの比較で、何が分かったか

採用する最新結果は、旧WSLの `/root/mimosa_ws/artifacts/lidar_bias_fix_20260921/new_flight/`。原因と修正は [RESULTS.md](handoff_reference/experiments/lidar_bias_fix_20260921/RESULTS.md)、図一覧は [PLOTS.md](handoff_reference/experiments/lidar_bias_fix_20260921/PLOTS.md)。

Aはlink2の1台、Bは4台。同じ修正後のflight bagから作り、同じ生FC IMUを使う。A点群がBのlink2成分とbyte単位で一致することを検証済み。**同じ4台搭載機体の記録から入力を変えた推定器比較であり、1台搭載機と4台搭載機の質量・飛行性能比較ではない。**

| 最新修正後の指標 | A：1 LiDAR | B：4 LiDAR |
|---|---:|---:|
| 変形前15秒：位置RMSE | 10.489 mm | 8.774 mm |
| 変形中：位置RMSE | 10.416 mm | 8.902 mm |
| 変形後15秒：位置RMSE | 10.341 mm | 8.797 mm |
| 地図completeness、変形直前→直後、閾値10 cm | 58.27→58.66% | 66.59→69.13% |
| 地図completeness、評価終了、閾値10 cm | 58.82% | 69.45% |
| 変形中の地図更新 | 33回 | 33回 |
| 全飛行評価区間のLiDAR callback中央値 | 4.434 ms | 8.626 ms |

この部屋では両方とも安定し、Bは地図の被覆を増やした。位置精度の差は小さく、処理量は増えた。**これだけで分散LiDARの一般的な有効性も、無効性も示したとは言えない。** 修正前Bの変形後RMSE 17.543 mmはシミュレータの点群バイアスを含むため、分散センシングの原理的欠点の証拠に使わない。

### 4.1 評価の定義と取り違えやすい点

- `/dragon/ground_truth`はすでにFC原点。link2→FCオフセットを二重に足さない。古い評価にはこの誤りがあり、過去に報告した約9.8 cm等は採用しない。
- 推定時刻にGT位置を線形補間、姿勢をSLERP。最初の同期時刻だけでSE(3)を合わせる。全軌跡フィット、スケール補正、変形開始時の再合わせはしない。位置誤差は3次元距離、姿勢誤差は相対回転角。
- 飛行制御は既存のGazebo真値モード。GTは制御・評価・表示に使用し、MIMOSA推定入力にはしない。
- 地図完成率は、固定GT表面パッチ中心 `s_i`、面積 `a_i`、時刻tの推定地図 `M_t` に対し、`C_tau(t) = 100 * sum_i a_i * 1[distance(s_i, M_t) <= tau] / sum_i a_i`。
- 旧室内ではROI `[-7.875,-5.875,0]`〜`[7.875,5.875,5] m`。箱の表面を約10 cmパッチに分け、他物体内部の面を除外。計72,014パッチ・720.14 m²。観測されない遮蔽面も固定分母に含む。「現時点で見える面の何%」ではない。
- 地図は軌跡と同じ初期SE(3)で整列し、共通5 cm voxelで間引く。閾値は10 cmと5 cm。各地図にICPをかけない。地図更新間は直前の値を保持し、離陸前の地図も含むのでhover開始時は0%ではない。
- Completenessだけでは誤った余分な点を罰しない。推定地図点→GT表面のaccuracy、表面距離、F-scoreも保存。GT姿勢で置いた生点群の累積coverageとは別指標。
- 「Weakest translation localizability」は `min(localizability_trans_final)`。当該ソースでは幾何LiDAR因子の `JᵀJ` の並進3×3ブロックの最小固有値の**平方根**に相当する（`mimosa/include/mimosa/utils.hpp`、`lidar/geometric_factor.hpp`）。全センサ・IMUを含む推定器全体の最小固有値や共分散ではない。点数や重みの影響があるため、値の増大だけで退化解消と結論しない。
- 誤差0.20 mまたは5°超が0.5秒以上、出力欠落0.3秒超等の判定は、この比較用の実装上の定義。公認ベンチマークの共通基準ではない。

作図のユーザー指定：全時系列図はhover開始以降だけ表示。既存図の横軸原点はtakeoffのままで、hoverを0秒へ振り直したわけではない。変形区間を薄く着色、軸数字を大きくし、RMSEの水平線・「Interval RMSE」は不要。completenessは縦軸を拡大し、上下の省略記号と0%・100%の端点を表示する。

### 4.2 原本の実験データ

全て旧WSLの `/root/mimosa_ws/artifacts/` 以下。次は用途別の索引。

| ディレクトリ | 内容・扱い |
|---|---|
| `dragon_mimosa_arming/` | 重複arming・意図的通信断のfailsafe確認 |
| `dragon_multilink/`、`dragon_gpu/` | 初期統合・GPU速度検証。旧GT原点評価とGPUバイアス修正前の結果に注意 |
| `dragon_transform/` | 初期の飛行・変形・bag比較。固定基部の最新A/Bとは別 |
| `2026-09-17-19-47-30/` | ユーザー提供 `/root/2026-09-17-19-47-30.bag` の解析 |
| `lidar_ab_20260921/` | 最初のA/B。基部も動く |
| `lidar_ab_fixed_base_20260921/` | FC保持。変形中に地図が更新されなかった条件 |
| `lidar_ab_fixed_base_refresh_20260921/` | 時間更新を追加。GPUバイアス修正前 |
| `lidar_bias_fix_20260921/same_bag_direction_correction/` | 元bagの方向だけ補正した原因切り分け |
| `lidar_bias_fix_20260921/new_flight/` | **修正後の新規飛行・最新A/B** |

最新ディレクトリに `flight.bag`、`inputs.bag`、`A/results.bag`、`B/results.bag`、JSON/CSV、`prepare_inputs.py`、`replay.py`、`analyze.py`、各plot/validateスクリプトがある。研究用スクリプトには絶対パスや既存ROS masterの前提が残る。再解析は旧環境で行うか、パス・依存を確認して移植する。古い実験には別ディレクトリへのsymlinkもある。軽量な引き継ぎコピーはbagを含まず、完全な再実行用データセットではない。

## 5. Git状態：コミットだけ取得しても最新状態にはならない

2026-09-29確認時点。

| リポジトリ | ブランチ / HEAD | remote |
|---|---|---|
| `/root/mimosa_ws/src/mimosa` | `develop/multi_link` / `0c21851` | `junichiro`: sugihara-16/mimosa、`origin`: ntnu-arl/mimosa |
| `/root/ros_ws/src/aerial_robot` | `develop/mimosa` / `f284e65d` | `origin`: sugihara-16/aerial_robot、`upstream`: jsk-ros-pkg/jsk_aerial_robot |

9月18日に整理したコミット：MIMOSA側 `a6c9222`（map latch）、`280d41c`（IMU待機）、`cafd846`（Dragon連携）、`ac950ea`（全リンク統合）、`0c21851`（文書）。Dragon側 `e378a65e`（重複arming）、`a6dbd785`（生IMU）、`55cb2682`（launch引数）、`f284e65d`（統合RViz）。

その後の変更は現在未コミットで残る。

- MIMOSA側：時間キーフレーム、GPU画素対応修正、IMU入力選択、demo launch/script、GPU回帰テスト、関連依存・ドキュメント。
- Dragon側：離陸高度2 m、制御ゲイン、URDFの色・Gazebo material等。既存のローカル変更として保全する。これらのゲインを実機用の確定値とみなさない。
- `aerial_robot_3rdparty`には別件のMuJoCo変更が残る。本件の整理で巻き戻さない。
- 本引き継ぎ文書・参考コピーも新規未追跡ファイル。**今回の資料作成ではcommit / push / checkout / resetを行っていない。**

詳細なHEAD・status・差分は [legacy_git](handoff_reference/legacy_git) に保存。コミット済み変更のpatchと、HEAD以降の作業ツリーpatchを分けている。未追跡のdemo等は [legacy_source/dragon_mimosa](handoff_reference/legacy_source/dragon_mimosa) にもコピーした。patchを既存作業ツリーへ重ねて適用しない。

`dragon_mimosa`は `spinal`、`dragon`、`aerial_robot_base`、`aerial_robot_msgs`等に依存するが、jsk_aerial_robotリポジトリの自動取得・revision固定は未整備。MIMOSA本体にはDragon依存を追加していない。再構築にはDragon側の変更とunderlayが必要。

## 6. 新WSLのUAS環境

### 6.1 接続と通常操作

WindowsのPowerShell等から：

```powershell
wsl -d Ubuntu-22.04 --cd /root
```

新WSL内：

```bash
cd /root/uas_native
./run.sh status          # 引き継ぎ資料作成時は null（停止中）
./run.sh stack           # 観測・接続検証構成を起動。機体へ制御出力しない
./run.sh check-stack     # ROS2センサ、ROS1地図・TFを確認
./run.sh stop            # このランチャーが起動したプロセス群を停止
```

`./run.sh sim`はシミュレータ単体。`./run.sh check-inference`は出力切断を確認してからRLへ目標を与え、RL/CBFの数値を検査する。検査後はstop/stackで残った目標を消す。`./run.sh rviz`は点群・姿勢用のROS2 RViz設定。GUIの操作・見え方はこの構築作業では未検証で、ROS1 VoxbloxメッシュをそのRViz2で表示できたという主張もしない。

`manager.py`が必要なROS1 master、bridge、sim、GBPlanner、CBF、RLを管理するので、手動で別のroscoreを起動する必要はない。内部ではROS1 GBPlanner用のmasterが必要。

### 6.2 実行構成と検証範囲

```text
Gazebo Harmonic（洞窟・UniPilot）
  → ros_gz_bridge → ROS2点群・真値odometry・TF
       ├→ RL → CBF → /validation/cbf_safe_cmd（機体入力とは切断）
       ├→ CBF用点群選択 → CBF
       └→ ros1_bridge → ROS1 GBPlanner / Voxblox
```

**UASは自律化スタックであり、MIMOSAそのものではない。** この固定revisionの標準シミュレーションはGazebo真値 `/rmf_unipilot/odom` を使う。公式のMIMOSA用manifestは別に存在し、ROS1側の構成。ROS2で動くUASを用意したことから「ROS2版MIMOSAやカメラ直接融合が動いた」と推論しない。

ローカルの公式 `docs/slam.md` ではカメラはexperimental、外部VIOのodometry経由の接続はWIPと記載されている。LiDAR photometricはカメラ画像を直接融合する機能と同義ではない。今後のセンサ・branch選定では、必要機能をソースと実入力で確認する。

- シーンは `darpa_cave_01`。SubTメッシュはGit LFSの実体を取得済み。
- 標準UniPilotモデルはLiDARのみ有効。IMU・RGB/depthカメラは無効で、bridge宣言だけあってもメッセージは出ない。
- LiDARは500×48、24,000点、10 Hz、最大100 m、距離ノイズ標準偏差0.03 mの汎用GPU rayモデル。出力は `x,y,z,intensity,ring`、点ごとの時刻なし。実機センサの完全再現ではない。
- 既定CBF出力は `/validation/cbf_safe_cmd` へremapし、`/rmf_unipilot/cmd/acc`へ接続していない。**自動離陸させないための設定を維持する。**
- 2026-09-29、30秒確認で生点群10 Hz、odom100 Hz、RTF約0.995、機体変位0 m。ROS1側は点群300件、odom2,991件、メッシュ31件、world→LiDAR TFを確認。
- 実入力による15秒推論でRL147件（約9.8 Hz）、CBF750件（約50 Hz）、全出力有限値、機体変位0 m。
- 以上はセンサ・推論・地図生成の接続確認。SLAM精度、退化回避、自律飛行、探索の成功率は未評価。

### 6.3 パス・バージョン・環境分離

| 内容 | 新WSL内パス／設定 |
|---|---|
| 起動設定・手順 | `/root/uas_native/README.md`、`run.sh`、`manager.py` |
| UAS | `/root/unified_autonomy_stack`、main `cb78bf7f57d074fc0999dd23bb1cfa1b1754d44d` |
| 全ソースrevision | `/root/uas_native/source-commits.json` |
| Gazebo fork | `/root/gz-sim`、`1897a13938a3670e575b0f0ae45e0b5c02c427ed`、`/usr`へ導入 |
| ROS2 | Humble、`ws_sim` / `ws_cbf` / `ws_rl` |
| ROS1基盤 | `/root/ros1_noetic/install_isolated`、92パッケージをJammy上でソースビルド |
| ROS1 planner | UAS内 `workspaces/ws_gbplanner`、必要28パッケージをビルド |
| Python | `/root/.venvs/uas`、Python3.10、PyTorch2.1.0+cu118、NumPy1.23.5 |
| 通信 | `ROS_DOMAIN_ID=74`、`ROS_MASTER_URI=http://127.0.0.1:11374`、`GZ_PARTITION=uas_native` |
| 検証ログ | `/root/uas_native/logs/build-and-checks/final-{topics,planner,inference}.json`、`logs/20260929-171728/` |

ROSをshell初期化へ自動sourceしていない。必要なら**別々のクリーンな端末**で `source /root/uas_native/env.sh`（ROS2＋venv）または `source /root/uas_native/env_ros1.sh`（ROS1）を実行。同一shellで混ぜない。bridgeは意図的に両方を扱う専用構成。

bridgeは先行Docker検証でJammy上にビルドした再配置可能なアーカイブを利用。新WSLの他の主要workspaceはネイティブに再ビルド済み。実行時にDockerは不要。

### rootへのディレクトリ移動

`uas_native`、`unified_autonomy_stack`、`ros1_noetic`、`gz-sim`、`.venvs`と関連設定の実体は `/root` にある。起動・ビルドスクリプトとvenvの入口も新パスへ更新した。旧 `/home/developer/...` は互換用symlinkであり、二重にデータを保持しているわけではない。既存のbuild/installメタデータは旧パスを含むので、このリンクを削除しない。今後クリーン再ビルドする場合は新パスを使う。移動記録は `~/uas_native/relocation.json`。旧developerのROSログは `~/.ros/developer_history/` に保全した。

移動後の再検証：LiDAR 10 Hz、ROS1地図生成・TF、RL/CBF推論を確認。定常観測時RTF約0.988、機体変位0 m。検証後は停止済み。ログは `~/uas_native/logs/root-relocation/`。新規ROS2購読者がDDSのendpointを先に発見しノード名をまだ取得していない場合、推論検査が誤って拒否する問題も修正した。不明な名前は最大5秒待ち、未解決または許可外のpublisherなら目標を送らず拒否する。

### 6.4 構築中に必要だった修正・運用条件

- Jammy上のROS1：`rosconsole-log4cxx.patch`、`pcl-ros-cxx17.patch`。GBPlanner側もC++17、`CMAKE_CXX_EXTENSIONS=OFF`。古い手動の `-std=c++11` より優先させる必要があった。
- `grid-map-onetbb.patch`：旧 `task_scheduler_init` を `task_arena`へ移し、`TBB::tbb`へリンク。NaNを含む平面でarea/raster×スレッドautomatic/1/2の6条件、逐次・並列差0を確認。
- `eigen_checks`のCMakeのため、plannerの `CATKIN_ENABLE_TESTING=ON`を維持。`bin/rosversion`は既存rospkgモジュールを呼ぶ補助CLI。
- FastDDSのUDP送受信バッファを4 MiBへ設定。点群配送欠落でCBFがtimeoutした問題を解消。`fastdds.xml`を外さない。
- NVIDIA CUDA認識とOpenGL描画を別々に確認済み。実GazeboのOgre2 rendererは `D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)`。Linux NVIDIAカーネルドライバを追加しない。
- WSLgを利用するserver-only起動。`--headless-rendering`によるEGL構成ではない。`DISPLAY=:0`、`GALLIUM_DRIVER=d3d12`等はenv設定を参照。
- `/etc/wsl.conf`は `systemd=false`。複数WSL間のWindows実行ファイルハンドラへの干渉対策として `systemd-binfmt.service` / `binfmt-support.service`をmask済み。UASはsystemdに依存しない。
- `developer`は開発用のsudo NOPASSWD。既定ログインはrootへ変更済み。既定WSLは旧Ubuntu-20.04のまま。新WSLだけの再起動後にユーザー・CUDA/OpenGL・ROS1解決を確認済み。
- Gazebo forkは `/usr`へ入るので、aptでGazeboを更新したらforkの再ビルド要否を確認する。

詳細と再構築スクリプトは [native_README.md](handoff_reference/uas/native_README.md)。旧Docker検証は `/root/uas_sim_validation`、そのソースは旧WSLの `/root/unified_autonomy_stack`。新ネイティブ環境と取り違えない。旧Docker構成はdomain73 / master11373 / partition `uas_validation`で、現行ネイティブとは別。

## 7. 再開時に守ること・資料の使い方

1. 通常の起動確認でarming・離陸しない。飛行は明示的に依頼された実験・操作で行う。旧デモの存在を再実行の指示と解釈しない。
2. 旧ROS1/Gazebo Classicと新Humble/Harmonicは別環境。旧ワークスペースを上書きしてROS2へ移行しない。新環境へのaerial_robot導入はまだ依頼されていない。
3. 最新の未コミット変更を保全する。単にremoteをcloneしても最新GPU修正・demo・地図更新は揃わない。
4. 評価ではGT原点、初期alignment、基部残留運動、地図更新条件、センサ生成誤差を確認する。新旧の不具合修正前後の結果を混ぜない。
5. 有効性を証明する結論を先に決めず、改善する条件・改善しない条件を記録する。旧室内のA/Bは、新テーマの一般性の証明ではない。
6. 起動・停止はプロジェクトのランチャーを使う。全WSLを落とす `wsl --shutdown` や、無差別なROS/Gazeboプロセスkillは避ける。
7. 今回は引き継ぎ資料だけを作成。既存WORKLOGへの追記、実装変更、再飛行は行っていない。

この文書は旧WSLの `dragon_mimosa/for_codex/HANDOFF.md` と、新WSLの `/root/uas_native/HANDOFF.md` に同じ内容を置く。隣の `handoff_reference/` に旧実装の主要ソース、Git差分、最新比較結果、環境資料をコピーした。これは参照スナップショットで、以後の修正は本来のソースへ行う。

コピーの元パスとSHA-256は `handoff_reference/manifest.json`。bag、ビルド生成物、Git履歴全体は同梱していない。参考資料内にコピー範囲外への相対リンクがある場合、記載した旧WSLの原本を参照する。旧環境はWindowsから `wsl -d Ubuntu-20.04 -u root --cd /root` で開ける。

新しいチャットへの開始文例：

> `/root/uas_native/HANDOFF.md` と同ディレクトリの `README.md` を読んで、この研究と環境を引き継いでください。主題はcomplementary sensorsのmulti-linkへの分散配置とactive perceptionです。旧4 LiDAR実装は参考資産です。まず現状を把握し、私の次の指示に従ってください。自動離陸やaerial_robotの導入はまだ行わないでください。
