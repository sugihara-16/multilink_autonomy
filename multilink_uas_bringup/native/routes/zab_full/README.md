# ZaB全域・記録専用走査

4つの通過可能な衝突メッシュ上の連結区間を別々の離着陸で収録する。
main north/main south/connection2 interior/connection1 interior。
合計移動約1958.53 m、独立通路約1457.85 m、重複約500.69 m。
各連結区間の全辺を通る開いた巡回路を、最小重みの奇数次数点ペアリングとEuler路で構成。
0.5 m/s巡航、始終端は2秒の加減速、通路間の向き変更は停止して旋回。
経路時間は約74分(sim)。離着陸・センサ確認は追加。

配布メッシュの接合面が機体包絡で通過できない箇所がある。
connection1/connection2は通路内部も別runで記録し、接合面近傍の非通過部分だけを除く。
また、メッシュ末端の床欠損と狭窄は、安全に飛行・着陸できる位置までに制限。
したがって全メッシュ頂点や衝突で閉じた面を越える飛行を「全道通過」とは数えない。
通過可能な通路網の全辺を対象とし、対象外は準備ログ・blocked_joinに記録。

`campaign.json` が実行順・経路・長さ、run*/route.jsonとsegments.jsonが飛行入力。
`coverage.png` が俯瞰。`corridors.gpickle` は証明済み通路網（このプロジェクト内生成物）。
Gitでは実行に使う経路定義と検証情報を保存し、俯瞰図・pickle・準備ログ・ローカルの収録先一覧は除外する。
外部由来のpickleを読み込まないこと。
準備過程はpreparation/に保存。メッシュ原本はWS assets/zab/models。
グリッドは0.5 m、連絡路の修復は0.2 m。経路を0.05〜0.1 m間隔で三角形距離照合。
広い通路は半径1 m球、狭い連絡路は水平半径1 m・垂直半径0.5 mの包絡を用いる。
終端床は±0.65 mの9点とspawn位置で有無・高低差を検査。

例（run番号を変更してそれぞれ実行）：
```bash
ros2 launch multilink_uas_bringup zab.launch.py headless:=true rviz:=false estimators:=false record:=true route:=/root/uas_native/routes/zab_full/run3
```

LIO/VIO、ROS1 core/bridge、画像整流、評価グラフを起動しない。
GT飛行制御、3カメラ/LiDAR/IMU生データ、GT/関節/制御/TFを記録。
物理刻み1 ms、画像解像度・センサ頻度は従来どおり。
起動時に倍率2/4/無制限を各12実秒計測し、配信頻度が維持される最速設定を選択。
配信頻度が落ちる場合は倍率1へ戻す。測定はcapture_status.json。
着陸完了後は記録を確定し、全構成を停止する。
