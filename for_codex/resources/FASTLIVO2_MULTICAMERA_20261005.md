# FAST-LIVO2の複数カメラ対応（2026-10-05）

link1・link3・link4の同時刻画像を、共通のLiDAR・IMU推定へ反映できるようにした。3台構成で同じ2飛行を各3回処理し、全6試行で位置誤差の最大値1m未満、発散なし。これは今回のデータでの動作検証であり、任意の環境での安定性の保証ではない。

## 実装

カメラごとに校正・参照画像地図を分離し、1つの位置・姿勢・速度・IMUバイアス・共分散へ逐次画像更新する。前のカメラの補正後状態と共分散を、次のカメラの補正前状態として渡す。IMU・LiDARをカメラごとに再適用しない。設定順link1→link3→link4で処理するため、非線形更新の順序依存は残る。一括最適化やカメラ間ステレオ対応を追加した意味ではない。

multicamera/cameras未指定なら従来の単眼経路。明示指定では1台・2台・3台を使用可能。入力timestamp完全一致で画像セットを構成し、欠損・重複・古い画像を区別、キューを制限。時刻のdoubleへの変換も旧処理のROS Time.toSecに合わせた。元のヘッダ時刻・点群・IMU内容を変更しない。

対応範囲は同期撮影・固定露出・固定取り付け変換。露出状態が元実装に1個しかないため、複数カメラでexposure_estimate_en=trueは拒否する。独立した自動露出、非同期カメラ、変形中の外部校正更新は未対応。ROS2シミュレーションlaunchの切替は行わず、現在のROS1比較環境とROS1トピック入力へ対応した。

補正後のROS出力は/multicamera/odometry。各カメラの補正を完了した状態の位置・姿勢を、撮影時刻で出力する。従来の/aft_mapped_to_initはLiDAR更新時の出力として維持。可視化画像は先頭カメラ代表。今回の瞬時LiDAR入力では元FASTの空点群時の画像更新スキップを維持し、視覚補正は約10Hz、オフラインの状態CSVは20Hz。

## 3台構成の結果

露出推定OFF、各試行を同一の最終版実行ファイルで計算。離陸前から初期化し、評価対象は追従区間63.66–113.14秒 / 63.78–263.96秒。開始時に一度だけ位置・姿勢整合し、共通10Hz評価。数値はRMSE（二乗平均平方根誤差）。

| 飛行 | 試行 | 位置RMSE [m] | 姿勢RMSE [deg] |
|---|---|---:|---:|
| run2a_end | 1 | 0.091817 | 0.394688 |
| run2a_end | 2 | 0.083057 | 0.463877 |
| run2a_end | 3 | 0.086656 | 0.407542 |
| run1_tail_a2_prefix | 1 | 0.366206 | 0.671750 |
| run1_tail_a2_prefix | 2 | 0.401847 | 1.164463 |
| run1_tail_a2_prefix | 3 | 0.393443 | 1.179959 |

## 動作確認用の対照

| 条件 | 飛行 | 位置RMSE [m] | 姿勢RMSE [deg] |
|---|---|---:|---:|
| legacy_exposure | run2a_end | 0.032282 | 0.530787 |
| legacy_fixed | run1_tail_a2_prefix | 809.174713 | 28.660617 |
| legacy_fixed | run2a_end | 0.025767 | 0.590967 |
| link1_link4 | run2a_end | 0.114424 | 0.470265 |
| link3 | run2a_end | 0.022132 | 0.469216 |

対照は各1回であり、反復を揃えた精度比較ではない。legacy_fixedは旧単眼経路で露出推定OFF、legacy_exposureは旧単眼・露出推定ON、link3は新入力経路の1台構成、link1_link4は新入力経路の2台構成。同じ入力でも数値が変わる既知の問題は解決していない。固定露出の単眼長区間対照は発散した。過去の単眼比較とは露出条件も違うため、今回の数値だけで複数カメラの優位性を断定しない。

## 検証

- 最終版11試行すべてexit0。追従時刻989/4004点は元データと一致し、出力20Hzの欠落なし。共通LiDAR/IMU読み込み数、LIO/VIO処理呼び出し数、実行ファイルSHAが全条件で一致。
- 3台構成は1試行あたり短区間1111回、長区間2619回ずつ全カメラで画像処理を実行。各カメラで1000回以上の実際の位置更新を確認。隣接カメラ間の共分散traceは前の補正後と次の補正前で完全一致。同期セットの欠損・重複・古い入力・待機列破棄はいずれも0。
- 元の画像セット数は2226/5243。実際の画像処理回数が少ないのは初期化と上記の空点群時スキップによる既存動作。画像を失ったまま全て使ったとは扱わない。
- C++単体試験: 時刻の一致、到着順逆転、重複、欠損、古い入力、容量上限、1台構成。
- ROSトピック経由の10秒再生で98個の補正後odometryを受信。全timestampが元画像集合に属し、単調増加、位置有限・単位Quaternionを確認。
- 不正回転行列、重複トピック、複数台で露出推定ONは起動時に拒否。
- 固定公式ソースから既存パッチ→新パッチを適用して今回のソースを再現。再適用判定も成功。最終ビルドは警告・失敗なし。
- 起動したROS master、推定器、bag再生、launchは停止済み。GUIなし、元bagと以前の比較結果は保持。

## 保存先・使い方

- 使い方: native/comparison/MULTICAMERA.md。
- 起動設定: native/logs/zab-offline/fast_multicamera/link1_link3_link4/<run>/trial1/ros_parameters.yaml。
- 実行: native/run_fast_multicamera.py --label <run> --cameras link1 link3 link4 --trial <新しい名前>。ROS1環境はcomparison_ros1.shを使う。
- ROS起動: roslaunch fast_livo multicamera.launch config:=<設定ファイル>。GUIなし。校正済み画像・LiDAR・IMUのROS1入力が別途必要。
- 結果: native/logs/zab-offline/fast_multicamera/evaluation.json、各条件/<run>/trial*/odometry.csv・errors.csv・config.yaml・run.log。
- pre_timestamp_fixは修正前の調査用出力で、最終評価へ混ぜない。
- 再構築用: native/comparison/fast-livo2-multicamera.patch、synchronized_images.hpp、fastlivo_offline.cpp、fastlivo_multicamera.launch。build_livo_comparison.pyへ組み込み済み。
