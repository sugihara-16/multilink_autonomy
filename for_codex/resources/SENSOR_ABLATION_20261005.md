# センサ除外比較

| 飛行 | 方式 | 位置RMSE [m] | 姿勢RMSE [deg] |
|---|---|---:|---:|
| run2a_end | MIMOSA + adopted DRPM: LiDAR + IMU | 12.108117 | 1.410846 |
| run2a_end | FAST-LIVO2: LiDAR + IMU | 12576.923355 | 27.904979 |
| run2a_end | ROVIO: image + IMU | 1.130972 | 0.045204 |
| run1_tail_a2_prefix | MIMOSA + adopted DRPM: LiDAR + IMU | 33.395138 | 3.265019 |
| run1_tail_a2_prefix | FAST-LIVO2: LiDAR + IMU | 7439.778538 | 55.208741 |
| run1_tail_a2_prefix | ROVIO: image + IMU | 4.889198 | 0.061220 |

DRPMは採用した公式更新＋距離方向ノイズ（3cm、最大10反復）を維持し、ROVIO入力だけ無効化。実際に読み込んだトピックはLiDARとIMUだけです。比較対象FAST-LIVO2は測定済みfast_lio（最初から画像なし）を再利用。ROVIOも既存の単眼link3結果を再利用。離陸前から推定し、追従区間だけ共通10Hzで評価。開始時に一度だけ位置・姿勢を整合し、以降の補正や尺度調整はしません。縦軸は省略なし。大きな誤差でDRPMの曲線が読みにくいため、DRPM単独の全範囲図も併記しています。

FAST-LIVO2画像なしのkm単位の値は発散した推定結果です。既存の診断では短い飛行は追従開始前に発散しています。離陸等を評価区間から除いても、それまでに生じた速度・姿勢・地図の誤りはリセットしません。「画像ありで初期化して追従中だけ画像を止める」既存条件とは別です。

## FAST-LIVO2 LiDARなし

FAST-LIVO2のLiDARなしは現行公式実装では単体VIOとして未対応です。画像照合にLiDAR由来の三次元点を使い、画像とIMUだけから地図を初期化・更新する経路がありません。単純にlidar_en=0としてもVIOモードにはなりません。無効な設定での停止やIMUだけの予測をVIO精度として数値化せず、未対応と記録しています。ROVIO単体の保存済み結果は掲載しています。

固定revision: 0d2c0346107b75b59934975adec9a6eeeb913c64。根拠はfast_without_lidar_audit.jsonの公式ソース位置参照。画像更新だけの独立実験には三次元点の初期化・管理等の新規実装が必要であり、今回の設定切替の比較には含めません。


## 検証・解釈

- DRPM LIOの2試行は正常終了。採用構成から変えた設定は外部ROVIOのenabledだけ（保存先は別）。実際に読み込んだのは/comparison/imuと/comparison/pointsのみ。元bagのSHA、実行ファイルと共有ライブラリは採用構成と一致。
- 追従中のDRPM/FAST LIOは495/2002点、ROVIOは989/4004点で、元の時刻列に欠落なし。共通10Hz格子で評価。既存FAST/ROVIOのRMSEを再現。
- DRPM LiDAR制約は全495/2002スキャンで生成。停止条件を満たしたのは112/495、412/2002で、それ以外は採用構成と同じ最大10反復の結果を使用。
- ROVIO入力を外すと位置・姿勢とも悪化した。この除外ではROVIOが運ぶ画像情報とIMU情報を同時に除いているので、画像の効果と共通IMUを二重に使う影響を分離した実験ではない。
- 起動したmaster11376・推定器は停止。GUI/Gazeboは起動していない。既存FAST/ROVIOは再測定せず、元結果を保持。

[六軸プロット](/root/uas_native/logs/zab-offline/sensor_ablation/plots/index.html) · [PDF](/root/uas_native/logs/zab-offline/sensor_ablation/plots/comparison.pdf)

公式ソースの根拠：[モード選択と画像補正](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/src/LIVMapper.cpp#L162)、[LiDAR点からの視覚地図生成](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/src/vio.cpp#L804)。
