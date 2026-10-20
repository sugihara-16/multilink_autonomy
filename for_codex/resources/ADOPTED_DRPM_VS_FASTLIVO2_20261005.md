# 採用DRPMとFAST-LIVO2の比較

両飛行とも位置誤差はFAST-LIVO2、姿勢誤差は採用MIMOSA＋DRPM＋ROVIOの方が小さい結果です。融合方式、地図処理、重み等が異なるため、この差をtight couplingだけの効果とは断定しません。採用MIMOSAのLiDAR制約は追従中の全スキャンで生成されましたが、停止条件を満たしたのは133/495、788/2002スキャンです。未収束のスキャンも10回目の結果を採用した設定として比較しています。

| 飛行 | 方式 | 位置RMSE [m] | 姿勢RMSE [deg] |
|---|---|---:|---:|
| run2a_end | MIMOSA + DRPM + ROVIO (official update, radial noise) | 1.239010 | 0.358439 |
| run2a_end | FAST-LIVO2 (LiDAR + IMU + image) | 0.035253 | 0.662300 |
| run1_tail_a2_prefix | MIMOSA + DRPM + ROVIO (official update, radial noise) | 0.865113 | 0.537693 |
| run1_tail_a2_prefix | FAST-LIVO2 (LiDAR + IMU + image) | 0.373302 | 3.711495 |

同じ2飛行・link3の単眼カメラ・LiDAR・IMU。保存済み推定値を再集計し、推定器の再実行はしていません。MIMOSAはROVIOを融合し、公式DRPM更新＋距離方向ノイズ（標準偏差3cm）、最大10反復を使用。FAST-LIVO2は画像補正ありの既存fast_full条件です。離陸前から推定した結果のうち軌道追従区間のみを共通10Hzで評価し、開始時の位置・姿勢整合は一度だけ。軸省略・途中の再整合・スケール補正なし。元の推定時刻は変更せず、誤差CSVには補間評価時刻を明記しています。

採用設定: comparison/adopted_drpm.json。ROVIOの融合重みは既存の別飛行校正行列を維持。採用方針は記録し、通常のシミュレーションlaunchは変更していません。再現: python3 /root/uas_native/plot_adopted_drpm_vs_fastlivo.py

[六軸プロット](/root/uas_native/logs/zab-offline/adopted_drpm_vs_fastlivo2/plots/index.html) · [PDF](/root/uas_native/logs/zab-offline/adopted_drpm_vs_fastlivo2/plots/comparison.pdf)
