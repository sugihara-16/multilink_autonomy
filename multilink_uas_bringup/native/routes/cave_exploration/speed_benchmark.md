# 2026-10-01 シミュレーション速度の簡易測定

固定四角形DRAGON、洞窟、3カメラ＋LiDAR＋IMU、ROVIO＋外部odometry融合MIMOSA、真値制御でホバリング。Gazebo GUI/RVizなし。物理刻み1 ms、センサ解像度・更新周期は変更していない。各設定5秒待機後、実時間20秒を測定。

| 目標倍率 | 実測倍率 | ROVIO遅延95%値（sim秒） | IMU観測Hz（sim） |
|---|---|---|---|
| 1.0 | 0.5899 | 0.115 | 987.5 |
| 1.5 | 0.6219 | 0.115 | 983.2 |
| 2.0 | 0.6058 | 0.115 | 988.2 |
| 4.0 | 0.6192 | 0.115 | 984.3 |
| 1.0 | 0.6011 | 0.115 | 983.2 |

結論：この負荷・ホバリング位置では約0.6倍で頭打ち。目標4倍にしても実時間より速くならない。軌道26.75分は同じ速度なら実時間約45分に相当するが、移動中の速度は未測定。

カメラ3台・ROVIOは全条件で最大stamp間隔0.05秒、LiDAR/MIMOSAは0.1秒。ROVIO遅延95%値は0.115秒。IMU観測は983–988 Hzで、1倍時にも観測欠落がある。観測用best-effort購読での結果なので、推定器入力の欠落率とは断定しない。測定プロセス自体の負荷を含む。

品質同一性や長距離移動時の推定精度を証明する試験ではない。独立LIO/VIO構成ではなく既存融合構成を使用。具体的なCPU/GPUボトルネックの切り分けは未実施。終了時に実行倍率1へ復帰。

元データ: `/root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/logs/20261001-154958-353663/speed_benchmark.json`
物理刻みの実測確認: 同ディレクトリ `speed_physics_stats.txt`。再測定は既存DRAGON VIOを起動してから、env_dragon.shをsourceし `python3 benchmark_sim_speed.py --output <JSON> --flight-log <起動ログdir>`。
