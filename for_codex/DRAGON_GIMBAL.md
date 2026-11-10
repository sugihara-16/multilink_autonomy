# DRAGONの固定四角形＋link4ジンバルカメラ

LiDAR＋IMUはlink1、単眼カメラはlink4。機体の6関節を四角形のまま保ち、カメラのyaw/pitchだけで視野を選ぶ。センサ設定はUniPilotから継承し、aerial_robot_baseの既定センサ条件には戻さない。既定の融合は、以下の検証済みDRPM修正版。

## UniPilot条件整合と融合修正（2026-11-10）

`dragon_gimbal.launch.py` の既定は `unipilot_matched:=true fusion_profile:=auto`。UniPilotのROVIO実行ファイル、v10視野選択（LiDAR情報あり）、ジンバル速度制限と計算完了待ちを使用する。MIMOSAは共通の実行ファイルに、以下の修正を含む別の共有ライブラリを読み込む。

- **DRPMをON**。公式更新則を使う既存4自由度版（方位＋並進）と、公称3cmの等方的な点の不確かさを使用する。全6自由度の公式プログラムそのものではない。実センサの横方向に3cmの雑音を追加したわけではない。
- 平面推定の近傍を5点から20点へ増やす。公式の法線推定と0.1radの不確かさ判定は維持し、LiDAR補正を捨てずに弱い方向を抑える。
- 視野選択には全6方向の測定されたLiDAR幾何を渡す。融合で傾き2方向を更新しないという制限を、観測不能と混同しない。
- 物理センサ、ROVIO、IMU設定、視覚の融合重み、v10の採点・探索処理は変更しない。IMUの不確かさと数値更新条件を変える別候補は独立飛行で発散したため不採用。

- 8秒静止→空中で推定器初期化の成功を確認→12秒接近→26秒の準備運動2周期→90m追従。2周期目は視線を保持し、追加40秒旋回は行わない。追従はUniPilotと同じ2秒の直線加減速。
- カメラはlink4に取り付けたまま、四角形のFC基準で前方を向ける。光学中心は `(0.8,0,-0.025)` m。UniPilotの前方18cmから80cmへ延長したのは、DRAGON本体の自己遮蔽を避けるため。生成器がこの姿勢をlink4座標へ変換する。
- LiDARはlink1のFC基準 `(-0.15,0,0.30)` m。UniPilotのz=0.05mをそのまま用いるとDRAGONのローター部品の境界内になるため25cm上げた。実際の位置を推定器へ設定し、測距ノイズ・走査角・距離閾値・周期は変えていない。支持棒は理想形状で質量/たわみを再現しない。
- カメラ変換もUniPilotの回転専用処理を使う。実測関節角から光学中心の位置を計算し、既存の10µm並進許容を超えれば停止する。関節を固定した今回専用であり、機体を変形させる実験では従来のSE3版が必要。
- `inspect:=true distance:=0` は同じ新配置で全画角を確認する検査モード。通常比較の準備手順とは別で、従来の検査用カメラ旋回を行う。
- `unipilot_matched:=false` で以下に記載した旧後方向配置と旧制御を再現する。

新しい試験の保存先は `/root/uas_native/logs/dragon-unipilot-matched-20261110/`。数値と確認結果は `report/index.html` とWORKLOGを参照。機体、飛行制御、取り付け距離、別飛行のセンサ系列はUniPilotと同一ではない。

修正前の条件整合試験は、90m×3回の平均がROVIO10.34m、融合41.15mだった。これは失敗比較として保持している。

今回の最終試験は `logs/dragon-fusion-fix-20261110/view_seed20262801/02/03`。同じ模様24301、90m、0.5m/s、3回すべて保持。追従開始姿勢だけで整列し尺度補正なし。

| 条件 | 試行1 | 試行2 | 試行3 | 平均 |
|---|---:|---:|---:|---:|
| UniPilot参照・融合位置RMSE [m] | 5.502 | 6.381 | 11.870 | 7.918 |
| DRAGON修正前 | 51.808 | 19.573 | 52.065 | 41.149 |
| DRAGON修正後 | 1.025 | 7.100 | 4.114 | 4.080 |

修正後ROVIOは2.310/7.473/4.236m、平均4.673m。全3本で取得・設定・ロードライブラリ検査成功、追従中のLiDAR補正は5,460回中5,460回成立し、各時刻で3方向を保持。全所有プロセス停止済み。レポートは `/root/uas_native/logs/dragon-fusion-fix-20261110/report/index.html`。既存UniPilotはDRPMなしの過去の別飛行であり、機体だけを変えた同一入力比較ではない。今回確認したのはDRAGON融合不良の修正と性能回復で、修正後の固定対偏向の再比較はまだ行っていない。

## 起動

```bash
ros2 launch multilink_uas_bringup dragon_gimbal.launch.py
```

Gazebo GUIを開き、離陸→推定器初期化・準備運動→90m/0.5m/sの自動飛行→着陸・全停止。途中終了は実行端末のCtrl+C。GUI不要なら `headless:=true`、短く試すなら `distance:=20`。 旧DRPMなし融合を再現する場合は `fusion_profile:=baseline`。`auto` は通常の条件整合モードで `drpm_isotropic20_view` を選び、旧構成・取り付け検査では `baseline` を維持する。固定カメラ対照は `mode:=fixed`。既定の条件整合モードでは固定側も同じ機体の準備運動を行い、カメラは正面を保持する。旧モードでは共通の初期旋回後、軌道追従中を固定する。

取り付け確認は次のコマンド。ホバリング中にyaw/pitchの両端を含む9方向と原点復帰を観測し、各1枚だけ画像を残す。連続画像・rosbagは保存しない。

```bash
ros2 launch multilink_uas_bringup dragon_gimbal.launch.py inspect:=true distance:=0
```

## 旧構成の配置（unipilot_matched:=false）

生成元は `multilink_uas_bringup/native/dragon_gimbal.py`。`MOUNT_XYZ=(0.10,-0.60,0)` m、`MOUNT_RPY=(0,0,-pi/2)` rad。link4の外側へ60cm張り出し、高さはリンクと同じ。従来の関節偏向用カメラは `(0.10,-0.08,0)` だったため、外側へ52cm移している。初期方向は従来どおりlink4の-y向きで、四角形では機体の後方になる。

- yaw±50°、pitch±90°。正のpitchは下向き。
- 両回転軸と光学中心は同一点。カメラの重心・支持機構を詳細再現した実機設計ではなく理想ジンバル。
- 回転リンク質量は各2g。支持棒は可視形状を持つが、その重量・たわみはモデル化していない。
- 実測8軸（機体6軸＋カメラ2軸）を画像時刻に補間し、ROVIOにカメラの位置・姿勢を渡す。指令角を代用しない。
- LiDAR、IMU、画像の設定はUniPilot由来のまま。モデルごとに一致を検査する。

旧構成は `dragon_articulated.launch.py`、新構成は `dragon_gimbal.launch.py`。旧モデルの取り付け位置や既定の関節探索を上書きしない。

## 旧構成の自己遮蔽と計画

起動時に、画像全体の視野と機体形状の重なりを検査する。モデルの全visualを囲む箱に15mmの余裕を加え、ローターには回転中の範囲を囲む半径25cmの境界を追加。yaw/pitchを2°刻みで4,641姿勢検査し、格子間の姿勢も考慮して画角に3°の余裕を加える。画像のマスクや機体visualの非表示で遮蔽をごまかしていない。結果は各出力先の `occlusion_audit.json`。

視野の採点は現在のDRAGON関節偏向版と共通。ROVIOの特徴点と不確かさ、現在のLiDAR情報、観測済み特徴地図を使う。探索対象だけを2軸ジンバルへ替え、10°刻みの候補209個と局所候補を評価する。LiDAR情報が更新されない場合は視覚による観測継続に移る。過去UniPilotの採点・探索実装と完全同一という意味ではない。

ジンバル指令は100Hz、合成角速度14°/s以下。Gazebo各軸の速度上限は20/√2°/s。指令が2秒以上途切れた場合は実測角度を保持する。

形状検査と画像確認は固定四角形についてのもの。関節変形をこのlaunchに追加する場合は、変更後の形状で遮蔽を検査し直す。

## 検証記録

`/root/uas_native/logs/dragon-gimbal-20261027/` に形状検査、端角度の画像、20m動作検証と90m飛行を記録。最終結果・制約はWORKLOGの同日項目を参照。

## 修正ライブラリの再構築

既存のUniPilot/MIMOSAビルドを前提に、追加部分は次で再構築できる。既存のUniPilotライブラリは上書きしない。

```bash
source /root/uas_native/env.sh
python3 /root/uas_native/active_perception/build_drpm_four_dof.py
python3 /root/uas_native/active_perception/build_drpm_support.py
python3 /root/uas_native/active_perception/build_drpm_view_information.py
```

直接実験runnerを使う場合は `--gimbal --unipilot-matched --fusion-profile drpm_isotropic20_view` を指定する。runner単体の既定は過去実験との互換性のためbaselineのまま。`drpm_isotropic20_calibrated` は不採用の診断候補なので使用しない。

## 関節版との対称比較

`mechanism_comparison:=true` で、関節版と同じSE3対応ROVIO、観測地図＋速度予測の視野選択、準備動作を使う。共通の前向き配置と修正済み融合を維持する。既定のv10/回転専用ROVIOの試験とは区別する。対応する関節側コマンドと条件は `DRAGON_ARTICULATED.md` を参照。

2026-11-10の共通制御比較は `logs/dragon-mechanism-paired-20261110-v3/report/index.html`。共通制御ジンバルの融合平均38.882m、関節平均21.904m（3回中1回58.503m）。既存の既定v10ジンバル平均4.080mとは別の視野制御/カメラ変換構成である。既定v10は維持し、共通制御を新たな採用方式へ自動昇格していない。
