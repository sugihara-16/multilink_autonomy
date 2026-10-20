# DRPMの距離方向ノイズ修正と精度再検証（2026-10-05）

ノイズの等方近似は今回のセンサと一致しなかったため修正した。**修正後も精度は改善せず、2例とも旧方式より位置誤差が増えた。**

ノイズの方向の不一致を保存データとシミュレータの実装から確認し、修正した。今回の2例では精度は改善せず、旧等方近似より位置誤差が増えた。ノイズ近似の誤りだけでは現在のMIMOSA+DRPMの性能不足を説明できない。

| 飛行 | 方式 | 位置RMSE [m] | 姿勢RMSE [deg] |
|---|---|---:|---:|
| run2a_end | legacy_isotropic | 4.399628 | 0.159267 |
| run2a_end | propagated_isotropic | 3.692832 | 0.283232 |
| run2a_end | radial | 23.967582 | 0.573889 |
| run2a_end | no_drpm | 0.511148 | 0.199296 |
| run1_tail_a2_prefix | legacy_isotropic | 4.453920 | 0.225301 |
| run1_tail_a2_prefix | propagated_isotropic | 7.326136 | 0.121508 |
| run1_tail_a2_prefix | radial | 5.882769 | 0.230812 |
| run1_tail_a2_prefix | no_drpm | 4.255743 | 0.303098 |

3方式×2飛行、全6ケースexit 0。旧方式を再実行した結果は、前回と全出力時刻・位置・四元数が完全一致（回転差を計算すると丸め誤差約6.4e-15°）。各方式の追従中の出力数は989/4004、最大間隔0.05秒、時刻欠落0。処理時間の合計は約11分。

短い飛行では、PCA伝播のみの対照3.69mに対し、同じ伝播計算で距離方向に変更すると23.97m。長い飛行では、伝播のみ7.33mに対し距離方向5.88m。方向モデルの影響は飛行によって異なるが、いずれも旧方式やDRPMなしを上回る位置精度にはならなかった。

DRPMは修正後も実際に働いている。例えば短い飛行では、100回の因子計算ごとに記録した確率のうち、p<0.5の方向数の平均は旧方式3.33方向、修正後1.84方向。ただし初期化から全区間のサンプルであり、各方式の推定姿勢・地図・対応点が異なるので、同一時刻の直接比較や悪化原因の確定には用いない。

[六軸プロット](/root/uas_native/logs/zab-offline/drpm_noise/plots/index.html) / [全図PDF](/root/uas_native/logs/zab-offline/drpm_noise/plots/comparison.pdf) / [評価値](/root/uas_native/logs/zab-offline/drpm_noise/evaluation.json) / [条件・時刻の検証](/root/uas_native/logs/zab-offline/drpm_noise/verification.json)。旧結果は保持。GUI/Gazeboは起動せず、検証用の推定器・ROS masterは終了。

## 実装と検証の要点

- 保存済みURDFはGaussian range sigma=0.03m。離陸前91スキャンで、光線別の距離標準偏差中央値0.029936/0.029942m、直交方向RMS約0.15/0.18μm。距離方向ノイズと整合。
- 点の共分散をσ²uuᵀへ変更。地図点には取得時の共分散を保持し、LiDARの取り付け位置と座標回転も反映する。
- 面の向きの誤差は各点の共分散をPCAの微分を通じて伝播。面の誤差を別途実測して与える必要はない。人工ノイズ各10万回・4条件で法線共分散の誤差0.50〜0.97%。近傍切替や全実データの面での保証ではない。
- 従来の等方近似、伝播計算だけを更新した等方近似、距離方向への修正版を比較。同じ点群・IMU・単眼link3 ROVIO、同じ別飛行で校正した固定6×6融合重み。評価は従来の追従区間・共通10Hz・開始時のみ剛体整合。
- 実装はnative/comparison/drpm.hpp、mimosa-drpm-noise.patch。再構築はbuild_livo_comparison.py。評価用はrun_drpm_noise.py / plot_drpm_noise.py / verify_drpm_noise.py / verify_drpm_sensor_noise.py。
- `drpm_noise_model`: 0=従来、1=伝播等方、2=距離方向。今回の修正版実験は2を明示指定。従来比較を再現するため既定値0を保持。通常slam_ros1_wsは変更していない。

## 残る制約

現DRPMはLiDAR因子への情報量の重み付けによる移植で、原論文式24/26の全更新則の再現ではない。共通IMU、地図姿勢、点の再利用による相関も未補正。LiDAR残差自体の重みは従来の固定σ=0.03mを維持。この結果をDRPM原手法の優劣とは解釈しない。

今回の光線計算は全点時刻オフセット0の瞬時スキャン用。実機の動きを伴う回転走査では各点の取得姿勢を保持する追加対応が必要。

詳しい式、ノイズの確認、再現条件は `/root/uas_native/logs/zab-offline/drpm_noise/README.md`。検証データと六軸図は同ディレクトリ。Gazebo一次資料: [GpuLidarSensor](https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8/src/GpuLidarSensor.cc)、[Lidar](https://raw.githubusercontent.com/gazebosim/gz-sensors/gz-sensors8/src/Lidar.cc)。
