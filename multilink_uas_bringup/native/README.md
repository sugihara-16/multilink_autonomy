> 2026-09-30 移設追記：`uas_native` の全ファイルは
> `/root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/` へ移動した。
> `/root/uas_native` は互換symlink。現在の操作手順はワークスペース直下の `README.md` を参照。
> 対話Bashは `.bashrc` でROS2＋統合WSを自動読み込みする。
> Python仮想環境は端末では有効化せず、stackのRLプロセス内だけで有効化する。
> `ros2 launch multilink_uas_bringup unipilot.launch.py` が現在の起動入口。
> 以下に残る旧パス・手順・「自動sourceしない」等の記述は移設前の履歴。

# Native WSL Unified Autonomy Stack

既定ログインはユーザーの希望でrootへ変更済み（2026-09-29）。環境の実体も `/root` へ移動済み。起動時は `cd /root/uas_native` を使う。構築・検証当時のユーザーはdeveloper。

Ubuntu-22.04 (WSL2) に直接インストールする検証環境。aerial_robot は導入しない。
元の Ubuntu-20.04 と Docker 検証環境は保持する。

## 接続

Windows Terminal / PowerShell:

```powershell
wsl -d Ubuntu-22.04 --cd /root
```

WSL 内:

```bash
cd /root/uas_native
./run.sh stack
./run.sh status
./run.sh check-stack
./run.sh stop
```

`stack` は観測・接続検証モード。CBF の出力は `/validation/cbf_safe_cmd` へ出し、
機体の `/rmf_unipilot/cmd/acc` へは接続しない。自動離陸しない。
`check-inference` はこの切断をチェックした上でポリシーに目標を送り、RL/CBF の出力を検査する。
機体へは送らない。検査後は stop/stack でポリシーの目標をリセットする。

`./run.sh sim` はシミュレータ単体。`./run.sh rviz` は点群・姿勢表示用の ROS2 RViz を追加起動。
headless は Gazebo GUI を開かない意味で、GPU センサ描画には WSLg を使う。
Windows の WSL 全体を shutdown する必要はない。

## 構成

- ROS2 Humble / Gazebo Harmonic。NTNU gz-sim fork は `1897a13938a3670e575b0f0ae45e0b5c02c427ed`。
- ROS1 Noetic は `/root/ros1_noetic` に必要なソースをビルド。Focal のバイナリは Jammy に混ぜない。
- UAS: `/root/unified_autonomy_stack`。リビジョンは `source-commits.json`。
- ROS2 ワークスペース: ws_sim, ws_cbf, ws_rl。
- ROS1 ワークスペース: ws_gbplanner。
- ROS1 の rosversion コマンドは bin/rosversion から既存の rospkg モジュールを呼び出す。
- ros1_bridge: 先行検証で Jammy 上にビルド済みの配布可能なアーカイブを展開。
  bridge fork: `8f1b7e903e1fb4462218db659bb4981228c12206`。
- RL の Python 依存は `/root/.venvs/uas` に分離。PyTorch 2.1.0+cu118 / NumPy 1.23.5。
- ROS_DOMAIN_ID=74, ROS master=127.0.0.1:11374, GZ_PARTITION=uas_native。
- ランタイムに Docker は使用しない。

通常の端末には ROS を自動 source しない。ROS1/ROS2 は別々の端末で使用する:

```bash
source /root/uas_native/env.sh       # ROS2 + Python venv
# 別のクリーンな端末:
source /root/uas_native/env_ros1.sh   # ROS1
```

developer は開発用の sudo NOPASSWD ユーザー。Windows の GPU ドライバを使用し、
Linux NVIDIA カーネルドライバはインストールしない。

## 範囲

標準 UniPilot モデルの LiDAR と Gazebo 真値 odometry を用いる。
この構成は MIMOSA を起動しない。SLAM 精度、飛行性能、探索性能の検証とは区別する。
IMU/カメラは標準モデルで無効化されており、この環境構築でセンサ仕様は変更しない。

## 再構築

導入スクリプトは同ディレクトリに保存。bootstrap.sh は root で実行する依存導入用。
通常はrootで build_gazebo.sh / build_noetic.sh / build_workspaces.sh / build_planner.sh を実行。
既存ソースやモデルは元の検証環境からコピーし、ビルド生成物を除外して再構築。
ROS1 リリースソース一覧は noetic.rosinstall。
rosconsole には Debian の log4cxx 0.12 対応パッチを適用している:
https://sources.debian.org/src/ros-rosconsole/1.14.4-2/debian/patches/0002-be-compatible-with-log4cxx-0.11-and-0.12-0.13.patch/
パッチ本体は rosconsole-log4cxx.patch として保存。
同じライブラリのヘッダが C++17 を要求するため、pcl_ros を C++17 に変更したパッチも保存。
GBPlanner 側は CMAKE_CXX_STANDARD=17 / CMAKE_CXX_EXTENSIONS=OFF を指定する。
grid_map_filters は oneTBB の task_arena に移行し、TBB::tbb へリンクする。
変更は grid-map-onetbb.patch に保存。公式移行資料:
https://uxlfoundation.github.io/oneTBB/main/tbb_userguide/Migration_Guide/Task_Scheduler_Init.html
`bash check_normals.sh` で area/raster × スレッド数 automatic/1/2 の逐次・並列一致を確認できる。
NaN を含む平面データで 6 ケースすべて差分 0 を確認済み。

WSL 間で共有される Windows 実行ファイルハンドラの消失を防ぐため、新ディストリビューションの
systemd-binfmt.service と binfmt-support.service は mask 済み。
さらに distro の自動停止時の干渉が再発したため /etc/wsl.conf の systemd=false を設定。
UAS の起動管理は manager.py が担当し、systemd サービスには依存しない。
既定ユーザーは root（構築時はdeveloper）。既定ディストリビューションは既存の Ubuntu-20.04 のまま。
Gazebo fork は /usr にインストールするため、Gazebo パッケージの更新後には再ビルドが必要。

## 検証結果

2026-09-29、Ubuntu-22.04 / developer ユーザーで確認。

- ROS1 基盤: 92 パッケージ（最初の 91 + eigen_conversions）をソースから構築。
- GBPlanner: 必要な 28 パッケージすべてビルド成功。既存コードのコンパイラ警告は残る。
- Gazebo: NTNU fork 8.9.0、Ogre2 の GL_RENDERER は D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)。
- PyTorch: 2.1.0+cu118、NumPy 1.23.5。CUDA 行列演算およびポリシー読み込み成功。
- 全構成の 30 秒観測: 点群 24,000 点 / 10 Hz、間引き 4,800 点 / 10 Hz、CBF 選択点群 323 点 / 約 10 Hz。
- 全構成の RTF 約 0.995、機体の最大移動量 0 m。
- ROS1 側 30 秒: 点群 300 メッセージ、姿勢 2,991 メッセージ、メッシュ 31 メッセージ。
- メッシュ最大 22 ブロック / 15,390 頂点（1 メッセージあたり）。world→LiDAR の TF 有効。
- センサ入力を用いた 15 秒の推論確認: RL 出力 147 メッセージ（約 9.8 Hz）、CBF 出力 750 メッセージ（約 50 Hz）。全出力が有限値、機体移動 0 m。
- 法線フィルタ互換修正: 6 ケースで逐次・並列の差分 0。

ログは logs/ に保存。センサ・推論・地図生成の接続検証であり、SLAM 精度や自律飛行性能の測定ではない。

構築時は再起動後のdeveloperログイン、CUDA/OpenGL、ROS1のパッケージ解決を確認。その後rootへ切り替え、既定rootログイン・ROS2 import・CUDA認識を確認。
確認後の UAS は停止済み。`run.sh status` は null（停止中）を返す。
ROS1/ROS2 の環境は別端末で読み込む。混在させると ROS_DISTRO の警告が出る。

## rootホームへの移動（2026-09-29）

UAS、ROS1基盤、Gazeboソース、venv、関連設定の実体と所有者をrootへ移した。
通常は `cd ~/uas_native` から操作する。起動・ビルド入口とvenvも `/root` に更新済み。
既存build/installの絶対パスを維持するため、旧 `/home/developer/...` に互換用symlinkを残す。
これらはデータの複製ではなく、既存ビルド利用中は削除しない。履歴資料は当時のパスのまま保全。
移動記録は `relocation.json`、旧developerのROSログは `/root/.ros/developer_history/`。

移動後の再検証：LiDAR 10 Hz、ROS1地図生成・TF、RL/CBF推論を確認。定常観測時RTF約0.988、機体変位0 m。検証後は停止済み。ログは `~/uas_native/logs/root-relocation/`。新規ROS2購読者がDDSのendpointを先に発見しノード名をまだ取得していない場合、推論検査が誤って拒否する問題も修正した。不明な名前は最大5秒待ち、未解決または許可外のpublisherなら目標を送らず拒否する。
