# Linux GPU比較環境

ホストのROS/Gazeboと機体開発環境を変更せず、Ubuntu22.04・ROS2 Humble・Gazebo HarmonicとROS1推定器を専用コンテナへ構築する。

- ホスト保存先: `../multilink_autonomy_runtime/root`（リポジトリ外）。
- ソース: このリポジトリをコンテナの `/root/multilink_autonomy_ws/src/multilink_autonomy` に共有。
- 操作: `bash multilink_uas_bringup/native/container/run.sh bash -lc '<コマンド>'`。
- コンテナはNVIDIA Container ToolkitのあるDockerを使用。依存パッケージはコンテナの書込み層、ソース依存・ビルド・環境モデルはruntime/rootに保持する。コンテナを削除した場合はbootstrapを再実行する。
- X11のGPU描画はホストのDISPLAYとXauthorityをコンテナへ渡す。`xhost +`は使用しない。

構築順はbootstrap、fetch_sources、prepare_sources、`../fetch_noetic.py`、build_noetic、build_simulation、build_bridge、build_estimators。シェルスクリプトはbash、Pythonスクリプトはpython3で、すべてrun.sh経由でコンテナ内から実行する。各処理の出力を `/root/setup-logs` へ保存する。推定器は記録済みrevisionとpatchを使う。

試験環境は構築後に次のコマンドで生成する。既存試験中の環境ファイルは再生成しない。

```bash
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/build_weathered_tunnel.py /root/multilink_autonomy_ws/assets/weathered_holdout_20261026/24301 --seed 24301 --length 100
```

初回のX11認証は `xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | docker exec -i multilink-comparison-20261111 xauth nmerge -` で現在の表示先だけを共有する。

環境構築後の試験コマンド（新規出力先が必須）:

```bash
bash multilink_uas_bringup/native/container/run.sh bash -lc 'source /root/uas_native/env.sh; python3 /root/uas_native/active_perception/run_practical_comparison.py --output /root/uas_native/logs/practical-comparison-20261111'
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/audit_practical_comparison.py /root/uas_native/logs/practical-comparison-20261111
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/report_practical_comparison.py /root/uas_native/logs/practical-comparison-20261111
```

先に短距離確認をする場合は別の出力先で `--arms gimbal joint --seeds 20262801 --distance 10` を指定する。短距離確認の値を90m反復平均へ混ぜない。

ROS1 bridgeについて過去資料のfork revision `8f1b7e...` の配布元は確認できなかったため、今回は公式0.10.3 (`174d7aad...`) をビルドする。既存のreliable_bridgeを接続し、通信検査を実施してから精度比較する。過去環境とのバイナリ一致は主張しない。

実験は合成環境weathered24301、90m、0.5m/s。採用済みジンバルv10・回転専用ROVIO・修正版DRPM融合を維持し、関節版とは共通10Hz評価時刻・開始姿勢だけの整列・尺度補正なしで比較する。新しいUniPilot試験を環境移植の参照とする。環境構築、両方式の10m予備試験、90m・3方式各3回の本試験まで完了。結果は [比較レポート](../logs/practical-comparison-20261111-v1/report/index.html)。融合位置RMSE平均はUniPilot8.290270m、ジンバル6.759937m、関節3.475956m。全9試行の取得/動作/終了検査に合格。

Linux移植で必要だった修正:

- Gazeboの描画とセンサーライブラリを8.3.0へ揃える。8.3.0には従来の描画worker最適化が取り込まれている。`build_gazebo_runtime.py` はsystem本体を書き換えず、隔離したruntimeへ配置する。センサーのまとめ描画と停止対応は継続使用。
- DRAGONの慣性データ/関節状態の受信待ち容量を2000へ拡大し、1kHz入力の欠落を防ぐ。
- シミュレーション専用のSpinal制御時計を物理時刻へ合わせる。計算待ちの一時停止を500msの指令途絶と誤認しない。実機MCUの時計と途絶閾値は変更しない。

ROS1/ROS2通信検査は `container/test_bridge.py` で実施済み。時刻・値・共分散を含む慣性データ100件が一致し、欠落と循環重複は0件。過去ビルドのハッシュを書き換えず、新規ビルドの識別情報を試行ごとに保存する。

終了時の監査はコンテナ内で実行する（保存されたプロセス番号がコンテナ内の番号のため）。実行・評価処理の固定、取得、画像とカメラ姿勢の対応、融合ライブラリの実ロード、距離センサー補正の継続、所有プロセスの停止を検査する。表示用レポートだけの取得後変更は別項目で記録する。

## 5環境条件の比較

基準、壁模様の配置変更、コントラスト0.45倍、環境光RGB0.35、幅5m→7mの5条件を生成する。後4条件は基準から1要因だけ変える。全条件で90m・0.5m/s・物理時間刻み1msを維持する。各条件に3方式×3回、計45回を新規実行する。

```bash
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/build_environment_conditions.py /root/multilink_autonomy_ws/assets/environment_comparison_new
bash multilink_uas_bringup/native/container/run.sh bash -lc 'source /root/uas_native/env.sh; python3 /root/uas_native/active_perception/run_environment_comparison.py --conditions /root/multilink_autonomy_ws/assets/environment_comparison_new/conditions.json --output /root/uas_native/logs/environment-comparison-new --jobs 5'
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/report_environment_comparison.py /root/uas_native/logs/environment-comparison-new
```

生成先と出力先は新規ディレクトリを指定する。`--jobs`は条件の同時実行数（1〜5）。各条件内の試行順は事前に固定し、通信領域・ポート・出力先を分離する。GPUはセンサー画像の描画に使い、推定・制御の計算待ち中はシミュレーション時間を止める。センサー周期や数値計算の精度は下げない。

取得・動作検査に不合格があっても、全予定試行を実行し結果を残す。全区間が取得できた試行は失敗フラグ付きで平均に含める。3回すべての評価区間がそろわない場合は3回平均を表示しない。試験中は実行・評価ソースや設定、バイナリ、環境ファイルを変更しない。

今回の記録先は `logs/environment-comparison-20261111-v1`。単一の合成トンネルを変化させる比較であり、5つの実地環境や機構だけの効果を検証する試験ではない。終了後は `docker stop multilink-comparison-20261111` で専用コンテナを停止できる。

全45試行を82.7分で実行済み。[5条件の比較レポート](../logs/environment-comparison-20261111-v1/report/index.html)に3回平均・ばらつき・全試行・不合格内容を保存した。全試行で評価区間を取得したが、既定の取得/動作/終了検査は11試行に不合格項目があり、除外せず集計している。全試験プロセスと専用コンテナは停止済み。各3値の補助図と詳細説明は、成果物内の `report/plot_environment_runs.py <reportディレクトリ>` で再生成できる。

## 以前の模様切替トンネルでの3方式比較

`build_switching_comparison.py` は以前の `build_switching_tunnel.py` をそのまま呼び、左壁→天井→右壁→床、その逆順、全方向に豊富な模様の3条件を作る。経路は[20,0,2.5]→[150,0,2.5]、130m・0.5m/s。各条件でUniPilotジンバル、DRAGONジンバル、DRAGON関節を各3回、計27回実行する。

```bash
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/build_switching_comparison.py /root/multilink_autonomy_ws/assets/switching_comparison_new
bash multilink_uas_bringup/native/container/run.sh bash -lc 'source /root/uas_native/env.sh; python3 /root/uas_native/active_perception/run_environment_comparison.py --conditions /root/multilink_autonomy_ws/assets/switching_comparison_new/conditions.json --output /root/uas_native/logs/switching-comparison-new --jobs 3'
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/active_perception/report_environment_comparison.py /root/uas_native/logs/switching-comparison-new
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/container/analyze_switching_comparison.py /root/uas_native/logs/switching-comparison-new
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/container/annotate_switching_report.py /root/uas_native/logs/switching-comparison-new
bash multilink_uas_bringup/native/container/run.sh python3 /root/uas_native/container/compact_comparison.py /root/uas_native/logs/switching-comparison-new
```

全区間の平均・標本標準偏差・個別値、以前と同じ4区間の誤差、10秒間の進行方向移動誤差、実測視線角を保存する。区間ごとの姿勢/尺度の再調整はしない。固定方向カメラを主比較に加えた試験ではなく、採用済みの3実用構成を比較する。

連続画像・点群・rosbagは保存しない。終了・分析後、数値/ログをgzipで可逆圧縮し、同じ実装のコピーはファイル実体を共有する。圧縮前後のSHA一致を `storage_manifest.json` に記録。既存の実験ディレクトリを削除しない。圧縮後も上記2つの報告コマンドで再集計できる。終了時監査のプロセス番号照合は取得時のコンテナで行い、後日の再集計では当時保存した監査結果も参照する。

今回の出力は `logs/switching-comparison-20261111-v1`。全27回を79.93分で実行し26回完走。全周条件の関節版1回は姿勢崩れで未完走となり、その3回平均は表示しない。逆順関節初回の終了処理エラーも保持する。[比較レポート](../logs/switching-comparison-20261111-v1/report/index.html)に平均・ばらつき・各値・視線・失敗を保存した。圧縮後約320MiB、主集計535数値は圧縮前後で完全一致。主な位置推定の平均誤差は正順でUniPilot、逆順で関節版、全周の完走比較でUniPilotが最小だった。

## 旧weathered24201でのUniPilot再検証（2026-11-12）

旧固定比較に用いた汚れ・ひびの配置24201で、UniPilotのみ90 m・0.5 m/s・seed20262601/602/603を実行した。結果は `logs/weathered-unipilot-20261112-v1/report/index.html`。過去平均7.96145 m、今回5.326814 m、全3回完走・取得/終了検査成功。位置誤差の完全一致や統計的同等性を示したものではない。

新規実行は `build_weathered_tunnel.py <新しい環境先> --seed 24201 --length 100` で環境を作り、`run_practical_comparison.py --output <新しい結果先> --arms unipilot --seeds 20262601 20262602 20262603 --world <環境先>/world.sdf --distance 90 --origin 5 0 2 --domain-base 171 --port-base 11771 --continue-failures` を環境読み込み後に実行する。

今回の再集計は `python3 /root/uas_native/container/report_weathered_reproduction.py /root/uas_native/logs/weathered-unipilot-20261112-v1`。結果隣接の同名 `.design.json` は取得前条件と旧記録値。数値は可逆圧縮済みで、再集計一致を確認済み。終了監査は取得時のコンテナ内で保存済み。結果は実ディスク41 MiB、専用コンテナ停止済み。

## weathered24201のDRAGON追加比較（2026-11-12）

`run_weathered_dragon_comparison.py --output <新規結果先> --reference /root/uas_native/logs/weathered-unipilot-20261112-v1` を環境読み込み後に実行する。先行UniPilotの凍結ファイル一致を確認し、同じ3seedの各組でDRAGONジンバル/関節を実行、3組を並行させる。結果別の再選別なし。

今回の結果は `logs/weathered-dragon-20261112-v1/report/index.html`。ジンバル3回平均18.015248 m、関節1/3完走で3回平均なし（601/602は姿勢崩れ、完走603は8.401577 mだがactuator異常終了あり）。603の離陸前の通信待ち失敗は保存し、同設定1回の起動再試行を別ディレクトリに保持。変更理由はacquisition_amendment.json。

再集計は `python3 /root/uas_native/container/report_weathered_dragon_comparison.py /root/uas_native/logs/weathered-dragon-20261112-v1`。全監査は取得時に保存済み。先行UniPilotの圧縮データを参照するため、そのディレクトリも保持する。圧縮後47 MiB・全summary再集計一致、専用コンテナ停止済み。

## UniPilot固定5方向とジンバルの同時比較（2026-11-13）

最終結果は `logs/unipilot-array-heldout-20261113-v2/report/index.html`。10模様配置×3回=30飛行で、固定正面/左右50度/上下とジンバルを同時撮影し、それぞれ独立した単眼ROVIO+MIMOSAへ入力した。v10、DRPM OFF、90m/.5m/s、全182秒。全30完走、主推定180系統の全区間取得、評価区間の入力一致・センサー欠落なし。

比較用の6推定器は、`build_rovio_lossless_output.py`で生成した隔離バイナリを共通使用する。ROVIOの送信待ち1件で発生した推定結果の欠落を、待ち容量2000へ変更して解消。推定計算は変更していない。2000件のバースト送信試験、3並列18系統の予備確認を実施済み。

再実行は環境を読み込んだコンテナで `python3 /root/uas_native/active_perception/run_unipilot_array_campaign.py --output <新しい保存先> --jobs 3 --worlds <world.sdfを10個指定>`。各世界のrouteは[5,0,2]から90mの経路を持つ必要がある。実行中はソース・バイナリ・環境を変更しない。このownerは全予定試行を保存し、集計→可逆圧縮→同じsummaryの再計算一致まで実行する。ホストの`execute_unipilot_array.py`は出力先と世界を指定した当該試験用で、最後のDocker停止まで行うため、再利用時は新しい出力名にする。

最終平均RMSEは−50 22.245m、正面31.134m、+50 85.615m、下86.727m、上16.749m、ジンバル6.066m。配置単位の片側検定は5比較Holm補正後すべてp=.004883。大きな発散も含む平均で、中央値と全試行の図も保存。初期化/準備/終了の注意事項、補助ROVIOの1数値破綻を記録し、全監査合格とは扱わない。入力品質が揃う6配置の補助集計でもジンバルの平均が最小。範囲はこの単純形状・模様配置の生成条件に限定する。

約851MBの実ディスク使用、主summary圧縮再計算一致、全所有プロセスと専用Docker停止済み。詳細な実行/統計/注意事項/保存の監査は結果ディレクトリ内のJSONとWORKLOG末尾を参照。
