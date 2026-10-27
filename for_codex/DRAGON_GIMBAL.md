# DRAGONの固定四角形＋link4ジンバルカメラ

LiDAR＋IMUはlink1、単眼カメラはlink4。機体の6関節を四角形のまま保ち、カメラのyaw/pitchだけで視野を選ぶ。センサ設定とROVIO＋MIMOSA（DRPM OFF）は現行DRAGON関節偏向版から継承し、aerial_robot_baseの既定センサ条件には戻さない。

## 起動

```bash
ros2 launch multilink_uas_bringup dragon_gimbal.launch.py
```

Gazebo GUIを開き、離陸→推定器初期化・準備運動→40秒のカメラyaw観測→90m/0.5m/sの自動飛行→着陸・全停止。途中終了は実行端末のCtrl+C。GUI不要なら `headless:=true`、短く試すなら `distance:=20`。固定カメラ対照は `mode:=fixed`。固定条件も同じ初期観測を行い、軌道追従中だけ固定する。

取り付け確認は次のコマンド。ホバリング中にyaw/pitchの両端を含む9方向と原点復帰を観測し、各1枚だけ画像を残す。連続画像・rosbagは保存しない。

```bash
ros2 launch multilink_uas_bringup dragon_gimbal.launch.py inspect:=true distance:=0
```

## 配置

生成元は `multilink_uas_bringup/native/dragon_gimbal.py`。`MOUNT_XYZ=(0.10,-0.60,0)` m、`MOUNT_RPY=(0,0,-pi/2)` rad。link4の外側へ60cm張り出し、高さはリンクと同じ。従来の関節偏向用カメラは `(0.10,-0.08,0)` だったため、外側へ52cm移している。初期方向は従来どおりlink4の-y向きで、四角形では機体の後方になる。

- yaw±50°、pitch±90°。正のpitchは下向き。
- 両回転軸と光学中心は同一点。カメラの重心・支持機構を詳細再現した実機設計ではなく理想ジンバル。
- 回転リンク質量は各2g。支持棒は可視形状を持つが、その重量・たわみはモデル化していない。
- 実測8軸（機体6軸＋カメラ2軸）を画像時刻に補間し、ROVIOにカメラの位置・姿勢を渡す。指令角を代用しない。
- LiDAR、IMU、画像の設定はUniPilot由来のまま。モデルごとに一致を検査する。

旧構成は `dragon_articulated.launch.py`、新構成は `dragon_gimbal.launch.py`。旧モデルの取り付け位置や既定の関節探索を上書きしない。

## 自己遮蔽と計画

起動時に、画像全体の視野と機体形状の重なりを検査する。モデルの全visualを囲む箱に15mmの余裕を加え、ローターには回転中の範囲を囲む半径25cmの境界を追加。yaw/pitchを2°刻みで4,641姿勢検査し、格子間の姿勢も考慮して画角に3°の余裕を加える。画像のマスクや機体visualの非表示で遮蔽をごまかしていない。結果は各出力先の `occlusion_audit.json`。

視野の採点は現在のDRAGON関節偏向版と共通。ROVIOの特徴点と不確かさ、現在のLiDAR情報、観測済み特徴地図を使う。探索対象だけを2軸ジンバルへ替え、10°刻みの候補209個と局所候補を評価する。LiDAR情報が更新されない場合は視覚による観測継続に移る。過去UniPilotの採点・探索実装と完全同一という意味ではない。

ジンバル指令は100Hz、合成角速度14°/s以下。Gazebo各軸の速度上限は20/√2°/s。指令が2秒以上途切れた場合は実測角度を保持する。

形状検査と画像確認は固定四角形についてのもの。関節変形をこのlaunchに追加する場合は、変更後の形状で遮蔽を検査し直す。

## 検証記録

`/root/uas_native/logs/dragon-gimbal-20261027/` に形状検査、端角度の画像、20m動作検証と90m飛行を記録。最終結果・制約はWORKLOGの同日項目を参照。
