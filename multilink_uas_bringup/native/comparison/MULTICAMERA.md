# FAST-LIVO2 複数カメラ拡張

DRAGONのlink1・link3・link4を、同じLiDAR・IMU推定へ入力するための、このプロジェクト独自の拡張です。公式FAST-LIVO2に元からある機能ではありません。

## 動作

- 同一の撮影timestampを持つ画像を1組にする。異なる時刻の画像は組み合わせず、欠落を古い画像で補完しない。重複・期限切れ・未完成・満杯の待機列による破棄はカウントする。
- LiDAR・IMU処理は元のまま1回。カメラごとに内部校正、LiDARからの取り付け変換、画像の参照地図を持つ。
- 設定に列挙した順に、同じ推定状態へ画像補正を適用する。次のカメラは直前の補正後の状態と共分散（不確かさ）を引き継ぐ。独立した3個のodometryを平均する方式ではない。
- 逐次更新のため、非線形な画像処理ではカメラの順番に依存する。今回の標準順はlink1→link3→link4。全画像の残差を一括して最適化する実装でも、カメラ間ステレオ対応を取る実装でもない。
- 画像同士の対応付けは各カメラ内で行い、LiDAR地図と推定状態を共有する。推定途中で関節を変形させる動的な取り付け変換には未対応。

## 設定

ROS1パラメータ `multicamera/cameras` に使用するカメラ名を列挙する。1台・2台・3台を選べる。省略時は従来の単眼処理を使用する。

```yaml
multicamera:
  cameras: [link1, link3, link4]
  max_pending_frames: 100
  link1:
    topic: /comparison/image/link1
    camera: {cam_model: Pinhole, cam_width: 480, cam_height: 360, scale: 1.0,
             cam_fx: 138.56406460551023, cam_fy: 138.56406460551023,
             cam_cx: 240.0, cam_cy: 180.0, cam_d0: 0.0, cam_d1: 0.0, cam_d2: 0.0, cam_d3: 0.0}
    # Rcl: カメラ座標←LiDAR座標の3×3回転を行優先で9要素
    # Pcl: 同じ変換の並進3要素。p_camera = Rcl * p_lidar + Pcl
  # link3/link4にもそれぞれtopic/camera/Rcl/Pclが必要
vio:
  exposure_estimate_en: false
```

上は構造の説明用で、取り付け変換が未記入なのでそのまま実行しない。実行用の全値は `logs/zab-offline/fast_multicamera/link1_link3_link4/<run>/trial1/ros_parameters.yaml` に生成される。撮影時の校正を使うため、取り付けを変更した場合は更新が必要。

3台構成では固定露出を前提とする。元のFAST-LIVO2には露出推定の状態が1個しかないため、それを独立したカメラ間で共有しない。露出推定を有効にした複数カメラ設定は明示的に拒否する。独立した自動露出、非同期カメラ、カメラごとの時間ずれ補正、複数カメラCOLMAP出力は未対応。

## 保存データでの実行

ROS2をsource済みの普段のシェルからは、ROS1用の環境を混ぜずに以下で実行する。仮想環境のactivateは不要。

```bash
# 端末1。Ctrl+Cでmasterを停止できる。
env -i HOME=/root USER=root PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash /root/uas_native/comparison_ros1.sh roscore -p 11376

# 端末2。trial名は既存結果を上書きしない新しい名前にする。
env -i HOME=/root USER=root PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  bash /root/uas_native/comparison_ros1.sh python3 /root/uas_native/run_fast_multicamera.py \
  --label run1_tail_a2_prefix --cameras link1 link3 link4 --trial manual1
```

入力bagは `prepare_fast_multicamera.py` で以前の検証済みカメラ比較bagから構成する。LiDAR・IMUは複製せず、撮影時刻を変更しない。生成済み入力bagを再作成する必要はない。出力は各試行ディレクトリのodometry.csvとログ・設定。

## ROSトピック経由

生成済み設定を渡して `roslaunch fast_livo multicamera.launch config:=<ros_parameters.yaml>` を上記ROS1環境内で実行する。GUIは起動しない。ROS1の画像・点群・IMUが設定されたトピックへ来ている必要がある。`/comparison/image/link*` は校正済み480×360画像であり、Gazeboの未加工720×540画像をそのまま入れるトピックではない。ROS2シミュレーションlaunchをFAST-LIVO2へ切り替える変更は含まない。

最終的な複数カメラ補正後の姿勢は `/multicamera/odometry` に撮影時刻で出力する。従来の `/aft_mapped_to_init` はLiDAR更新時の出力のまま。補正後の出力が必要な場合は前者を使う。画像・色付き点群の可視化は先頭カメラを代表表示する。

今回の瞬時LiDAR・20Hz画像では、元FAST-LIVO2の「直前に点群がない画像時刻は視覚処理を行わない」動作を維持するため、実際の画像補正は約10Hz。オフラインCSVには元と同じ20Hzの推定状態が出る。欠落画像を使って補正したという意味ではない。

## 再構築と検証

`build_livo_comparison.py` が、固定した公式revision→既存互換パッチ→fast-livo2-multicamera.patchの順に適用し、synchronized_images.hpp・offline adapter・launchを配置する。通常のslam_ros1_wsは変更しない。

- `test_synchronized_images.cpp`: 完全一致時刻、到着順逆転、重複、欠損、古い画像、待機列上限、1台構成。
- `verify_fast_multicamera.py`: 実データの時刻・各カメラの制約数と補正寄与・共分散の引継ぎ・共通LiDAR/IMU処理回数・精度。
- `test_fast_multicamera_ros.py`: ROS購読と撮影時刻付き補正後odometry出力。起動したプロセスは試験終了時に停止する。
- `test_fast_multicamera_configuration.py`: 不正回転、トピック重複、不適切な共有露出設定を拒否。
