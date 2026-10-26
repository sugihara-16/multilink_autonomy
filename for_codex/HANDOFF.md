## 2026-10-26 最新：未使用3配置での偏向採点比較、全30本完了・全停止
- 最新結果は `/root/uas_native/logs/information-holdout-20261026/report/index.html`、comparison.pdf、findings.json。3新規模様配置×各方式5回、90m/.5m/s、単眼ROVIO+MIMOSA・DRPM OFF。視覚のみの採点 vs LiDAR情報を使う採点。形状・センサ・推定器・融合・偏向v10は共通。固定との比較ではない。
- 全3配置の平均でLiDARあり改善。3配置平均の視覚速度誤差 .389103→.325277m/s（16.4%減）、VIO10秒3.650041→3.034276m、融合10秒1.824442→1.525933m、全位置10.932243→8.715399m（20.3%減）。速度で良いのは15組中7組、全位置10組。速度の最大値/ばらつきは全3配置で減少、毎回の改善や統計的有意は主張しない。24301の速度中央値は逆順位。
- 30本の取得検査合格・速度利用率100%、751凍結入力一致、全終了コード0、210PID消滅。元の厳密valid=false（planner_continuousのみ）は保持して別記。圧縮データから30本再計算して数値厳密一致。初回未完了の補助上固定異常はincomplete_attemptsへ保持。全30本は補助4推定器を外した同一構成で実施、6カメラの描画と主推定器は不変。
- 中断依頼の状態確認時には既に全30本が完了。再開後は最終集計のみ実施。現在のシミュレーション/待機runnerなし。ログ約723MiB、Windows空き約156GiB。全予定完了、再取得・継続起動は不要。
- 再集計は run_information_holdout.py --output <ROOT> --report-only → finalize_information_holdout.py <ROOT>。run_information_holdout.pyの計測部分と実行wrapperは凍結済み、変更時は新キャンペーンとして扱う。詳細はWORKLOG末尾。
- 以下は過去の履歴。古い「進行中」やsession番号を現在の状態と扱わない。

## 2026-10-24 最新：人工トンネル6/9本完了、最後の3本を実行中
- ROOT=/root/uas_native/logs/switching-tunnel-20261024。owner session68566、/tmp/switching-tunnel-resume.log。現在alternating_seed20262403。先頭6本有効完了。最終3条件が終わるまで新設定・新実験なし。待機/追加observerは全て終了。
- 偏在4本はVIO10秒移動/融合全位置で全固定より良好。全周rich対照2本は上固定が融合全位置で良好。速度RMSだけだとreversed/402は偏向 .280389と最良固定 .280929m/sでほぼ同等。全失敗値保持。
- 完了後の元の報告器例外は既知（固定上/下の受理速度factor欠落）。取得status=completeと9本の各validを確認してから `report_switching_tunnel_sparse.py ROOT/validation` と `audit_switching_tunnel.py ROOT/validation` を実行。元のfrozen.jsonと追加評価evaluation_amendment.jsonを維持。report_sparse/index.htmlが正しい最終入口。
- 途中集計更新sessionは現在起動したreportのみ（/tmp/switching-second-round-report.log）。旧session14721は停止、61808/98322の読取診断も正常終了。snapshot初回保持、共分散probe threshold_check.jsonで未受理と閾値超過の対応確認済。
- 最終結果/WORKLOG保存、全所有sim停止、Windows容量確認まで完遂。以下は履歴。

## 2026-10-24 最新：人工トンネル2本完了、一様配置の再試行中
- ROOT=/root/uas_native/logs/switching-tunnel-20261024。owner session68566、/tmp/switching-tunnel-resume.log。validationでalternating/reversedのseed20262401は有効完了。uniform同seedはpacerの時計停止待ちタイムアウトで未完了をinvalid_trialsへ保持し同条件再試行中。旧14721終了。設定/凍結SHAは不変。
- 追加読取observer session61808（飛行画像少数）、次のalternating/402共分散probe待機session98322。停止時は待機含め停止。live_state参照。
- 重要：固定の上/下で不確かなVIO速度factorを渡さない区間が生じ、旧報告器は完全factor前提のため失敗する。取得は全画像/odometry連続で正常。9本取得後の旧報告エラーは既知。評価は別のreport_switching_tunnel_sparse.py validation、監査はaudit_switching_tunnel.py validationを使用。evaluation_amendment.jsonに追加評価のSHA/理由を記録。初期主指標を欠落補間で偽装しない。
- 1本目2配置とも偏向は固定全方向よりVIO10秒移動/融合位置が良好。ただし各1回で再現性確認中。全9本完了まで設定を変えず継続。以下は履歴。

## 2026-10-24 最新：特徴偏在の人工トンネルで9飛行を検証中
- 新ユーザー依頼で再開。ROOT=/root/uas_native/logs/switching-tunnel-20261024。owner session14721、/tmp/switching-tunnel-validation.log、validation/campaign.json。待機ownerなし、現在runnerが3配置×3seedを順次実行。同時sim1、GUI/rawbagなし。
- assets/switching_tunnel_20261024：幅6m/高さ5mアーチの一定断面、模様だけ左壁→天井→右壁→床/逆順/全周一様。実Gazebo静止画像60枚で偏在確認済。初回DAE法線欠落による停止を修正後成功。
- 最新候補v10、推定器/センサ/融合条件は固定。73入力SHAをROOT/frozen.jsonへ保存。130m、0.5m/s。主指標は既知退化軸XのVIO速度RMS、領域中央40秒×4。各固定は独立した単眼、配置正解をplannerに渡さない。
- 全9本完了後、audit_motion_campaign.py validation --frozen ROOT/frozen.json、report/switching_index.htmlの結果を確認。run_switching_tunnel_trials.pyは完了済みをスキップ可能だが、失敗試行は原因確認してから扱う。まだ精度結果なし。新規手法調整はせず結果を全部保存。標準v57不変。以下は履歴。

## 2026-10-23 最新：現検証を完了しユーザー指示で停止
- ユーザー「今の検証が終わったら停止して」。ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022、PAUSED.json/live_state.json。現在owner/待機ownerなし。新しい実験を開始せず、次の明示的な再開指示を待つ。
- 最後のv10 a1は完了/全監査成功。frozen_v10_development.json/zip。development_v10_a1/report/index.html、final_audit.json、final_summary.json、recovery_execution.json。復帰4回すべて所定条件で完了。
- VIO方向速度RMS 往路active .237817対最良+50 .143035、復路active .050780対最良正面 .066641m/s。復路は全固定より良好、往路は劣る。開発1飛行で独立再現性は未検証。融合位置RMSE .136418m対最良固定 .254291mは別指標。
- v9は主3区間で事前良好固定に0/3、事前screen不達。v8独立9本は完了、主9組で良好固定に1/9。全完了結果は保持、重複実行不要。標準v57未変更。再開後はv10診断を踏まえて次の候補/独立比較を設計。以下は履歴であり、古い実行sessionを再起動しない。

## 2026-10-23 最新：v9完了・v10 a1実行中
- ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022。現在session71161、development_v10_a1、/tmp/motion-view-v10-after-retry.log。frozen_v10_development.json/zip。後続待機なし。同時sim1。v10は追跡不足時に正面復帰を完了する別候補、6試験成功。
- v9開発2本は監査済み。主3区間で固定平均には3/3、事前良好固定には0/3。v9_screening_resultの事前基準不達で独立反復しない。Fut旧終了エラーはinvalid_trialsに保持。合成encoder終了試験20回は再現なし、修正は未実施。
- v10終了後audit_motion_campaign --frozen frozen_v10_development.json、annotate_motion_reports、復帰動作確認、全候補レポート更新。改善目標は未達。標準v57/推定器/センサ/融合重み不変、GUI/bagなし。以下は履歴。

## 2026-10-23 最新：v8独立9本完了、v9開発実行中
- ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022。現在session64332/PID495489、development_v9のb1が走行中。その後Fut122秒。/tmp/motion-view-v9.log冒頭に上書き注記あり（v9_owner_notice.json参照）。試行内のログ/数値は正常。凍結frozen_v9_development.json/zip。
- 後続v10 a1開発1本の待機session68228、start_v10.py、/tmp/motion-view-v10.log。v10はv8＋少数追跡時の復帰継続、v9の往復準備とは別候補。5試験成功、まだ未飛行。v10_plan.jsonにSHA/条件。停止時は64332と68228を含む所有者を止める。
- v8 validation_round2は全9本・最終監査完了。全7区間×3回で固定平均に20/21、全固定に3/21勝。主3区間×3回では事前良好固定に1/9で目標未達。詳細report/final_summary.json。元a1/512終了エラー1本をinvalid_trials保持、再試行合格。旧owner35274は終了。再実行不要。
- v9完了後audit_motion_campaign --frozen frozen_v9_development.json、audit_preview_preparation、annotate_motion_reports、report_motion_candidates。v9_validation_protocol.jsonの事前基準で新seed20261611/12/13への反復判断。v10 a1は別に検証。採用v57/default・推定器/センサ/融合重み不変。GUI/bagなし、Windows空き約162GiB。同時sim最大1。以下は履歴。

## 2026-10-23 進行中：v8独立反復、次にv9開発比較
- 現ownerはsession35274（/tmp/motion-round2-resume.log）。ROOT/validation_round2は5/9本完了、二ツ小屋/512を実行中。a1/512終了エラーの再試行は成功、無効元データ保持。設定変更なし。
- 次のv9待機ownerはsession64332（start_v9.py、/tmp/motion-view-v9.log）。旧90918は待機検出修正のため停止。現在の9本完了とowner終了後、v9のb1/Fut122秒を順次実行。同時sim最大1。停止時は待機ownerも止める。
- v9_validation_protocol.jsonに開発通過条件と未使用seed20261611/12/13を事前記録。audit_preview_preparation.pyは準備実行・評価除外・実視野保持・巡航視野差を確認する新読取用チェック（v9実データ実行は未了）。現在の凍結38ソースは全一致。
- 目標未達、既定v57不変。最後は全所有sim停止。数値中心、Windows空き約163GiB。以下は履歴。

## 2026-10-23 最新：v8独立検証の4本目を同条件で再試行中
- ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022。validation_round2で3本完了、a1/seed20261512再試行中。owner session35274、/tmp/motion-round2-resume.log。旧16303は終了、待機ownerなし。同時sim最大1。
- 旧4本目は画像/推定の連続性は全て成功したが、終了SIGINT直後にyaw_m50_encoder exit -11。invalid_trialsへ保持し、凍結v8/seed不変で再取得。残り5本を順次継続。独立3本の結果はWORKLOG末尾。最良固定に安定して勝つ目標は未達。
- v9は実装/5試験済み、未飛行。先に既存経路を最大15m往復し、巡航で選んだ視野を保って追加準備運動。固定にも同じ身体運動。v8より準備時間/観測が増える別条件。v9_plan.json参照。v8凍結依存は変更しない。
- v8完了後はaudit_motion_campaign.py --frozen ROOT/frozen_round2.json、説明修正annotate_motion_reports.py、比較report_motion_candidates.py。採用v57/default不変。全sim停止は検証終了時、GUI/bagなし。以下は履歴。

## 2026-10-23 12:51 UTC 最新：v8独立9飛行を実行中
- ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022。validation_round2、owner session16303、/tmp/motion-round2.log。開始はa1/seed20261511。残りも同設定で継続。追加待機ownerはない（現在のrunnerが9本を順次起動）。live_state.json参照。
- 開発v1〜v8完了（v6はa1追加も完了）。v8は実ROVIO追跡/速度の同時不確かさ＋画像移動を抑える候補＋共通準備運動26秒追加/視野保持。未表示の画像地図点を検索するv3とは異なる。固定にも同じ追加身体運動、推定器/融合/センサは不変、DRPM OFF、真値は飛行制御/事後採点のみ。
- v8開発の主VIO方向RMS active/事前固定：b1前半 .019904/.043674、後半 .027696/.036444、二ツ小屋 .026985/.022530m/s。2区間勝ち/1区間負け。事前比較比GM=.74580でv6/v7より小さく選択。設定はfrozen_round2.json/zipで固定し、未使用seed511/512/513×a1/b1/Fut362秒の9本へ。独立結果を見て途中調整しない。
- 新しい独立結果はまだない。採用v57/defaultは変更していない。完了後 `python3 /root/uas_native/active_perception/audit_motion_campaign.py ROOT/validation_round2 --frozen ROOT/frozen_round2.json` で監査・集計し、report_motion_candidatesでHTML/PDF更新。v3独立9本は完了済みで再実行しない。
- 中断再開helperは追加準備を自動判別：`resume_motion_view_trials.py ROOT/validation_round2 --frozen ROOT/frozen_round2.json`。途中中断の試行は先に状態確認し、結果を捨てない。GUI/bagなし、全simは検証終了後停止。ユーザーの自走改善指示を継続中。commitなし。以下のsessionや待機情報は履歴。

## 2026-10-23 12:33 UTC 更新（この節を優先）
- ROOT gimbal-velocity-improvement-20261022。v3独立9本、v5/v6各2本、v7各2本は完了/全監査成功。v7はb1前後で全5固定よりVIO方向誤差が小さかったが、二ツ小屋は .076895対上固定.008665m/sで悪化。詳細WORKLOG末尾。
- 実行中はv6 a1開発1本（session38537 /tmp/motion-view-v6-a1.log）。後続v8（v6実追跡評価＋v7共通追加準備の組合せ）の2本が待機：start_v8.py session97395 /tmp/motion-view-v8.log。
- 独立round2待機はstart_round2_validation.py session16303 /tmp/motion-round2.log。旧待機13025は停止。v6/v7/v8開発主3区間で事前固定へのRMS比幾何平均が小さい候補を選び、SHA/zip保存後、未使用seed20261511/12/13×a1/b1/Fut362秒の9本へ自動移行。独立結果はまだ一切見ていない。方針更新前後をprotocol/historyで保持。
- 同時sim1、ユーザー停止時は全待機ownerも停止。live_state.jsonを確認。未完了なので自走継続。採用v57/default変更なし、commitなし。以下は履歴であり古いsessionを再起動しない。

## 2026-10-23 改良実験を継続中（最新）
- ROOT=/root/uas_native/logs/gimbal-velocity-improvement-20261022。live_state.jsonで所有者を確認。v3独立9本完了・9×85監査成功、final_audit.json。1本の共通取得欠落はinvalid_trialsへ保持し同条件再試行成功。再開後の6本に再発なし。採用v57は変更なし。
- v3独立結果：全7区間×3回の21組で固定5方向の均等平均に18勝3敗、全固定に2勝19敗。主3区間×3回の9組で事前比較方向（b1 -50、二ツ小屋 上）に2勝7敗。固定平均への優位と良い固定への敗北を区別する。詳細final_summary.json。
- v5（実ROVIO特徴/速度共同不確かさ＋早期開始）開発2本完了。主方向誤差 active / 事前固定：b1前半 .01757/.01859、後半 .07487/.01740、二ツ小屋 .02089/.00913 m/s。
- v6（v5＋画像移動を抑える候補）開発2本完了。b1前半 .02934/.01774、後半 .03949/.01930、二ツ小屋 .006427/.009277 m/s。二ツ小屋では全5固定より良いが、1試行の開発結果。独立検証ではない。
- v7はv3に準備運動26秒を全固定・偏向共通で追加し、その2回目だけ偏向の実角度を保持する別プロトコル。フィルタ/センサ/重みは変更せず、真値補正なし。条件付owner after_v6.py session74997 (/tmp/motion-view-v7.log) が開始。v5/v6の事前スクリーニングは両方未達。v7_plan.json/SHA固定、3試験成功。
- v7完了後、v6のZaB a1開発1本を実行するowner after_preparation_trial.py session38537 (/tmp/motion-view-v6-a1.log)が待機。a1ではv3の計画地図があるのに実ROVIO追跡点が極端に少なかったため、実追跡評価の汎化を確認。全失敗保持。開発seed20261401、独立検証用の次seedは未使用の20261511/12/13。
- 同時sim最大1。ユーザーが停止したら待機ownerも停止する。まだ自走実験中、終わっていない。source/config/各結果はROOT、比較入口candidate_report/index.html（更新要）。固定に安定して勝つ目標は未達、採用切替/commitなし。
- 古いafter_validation.py/session67375は意図的停止、v5実行64013は終了、v6owner51605は終了。古い以下の進捗は履歴。GUI/raw bagなし、Windows空き約160GiB、8GiB停止guard維持。

## 2026-10-23 再開中（以下の中断指示は解除済み）
- ユーザー「再開して」。ROOT gimbal-velocity-improvement-20261022、resume_motion_view_trials.pyがvalidationの未完了分を実行中（session38925、/tmp/motion-view-resume.log）。無効a1/412はinvalid_trialsへ保存、同条件で再試行。凍結ソースと採用設定は不変。
- 欠落は全固定画像/共通真値に同時発生。発生源は未特定、厳密な監査を維持して継続。完了済み3本を再実行しない。PAUSED.jsonはresumedへ更新。

## 2026-10-22 — 偏向手法の改良（ユーザー指示で中断・全停止）
- 最新指示「一回作業中断して」に従って停止。再開指示まで実験を開始しない。ROOT `/root/uas_native/logs/gimbal-velocity-improvement-20261022`、再開記録 `PAUSED.json`。記録済みownersと/procを確認し、今回の実験プロセス残存なし。
- 依頼は「手法を改良し、固定に勝てるように」。開発4候補×2飛行完了、全監査合格。motion_view_v3をfrozen_candidate.json/zipに固定。既存採用v57の81ファイル・センサ・推定器・融合較正は不変。採用切替/commitなし。6単体試験成功。
- v3は2画像の相対運動情報、LiDARの相対的に弱い並進方向、5×5画像区画の期待情報、4秒目標保持、準備運動中から視野選択。真値や経路名を制御へ入れない。主評価は融合前の実ROVIO速度誤差（共通LiDAR弱方向）。融合後の位置精度は分けて扱う。
- 独立検証はseed20261411のZaB a1/b1/二ツ小屋の3飛行が完了・監査合格。4本目a1/20261412は飛行完了・全所有終了0だが入力連続性等の監査に失敗してownerが終了。`validation/campaign.json`はfailed、4本目エントリはrunningのまま（実プロセスはない）。生の失敗記録を保持。
- 4本目失敗：comparison.jsonでreference_valid、純LIOの連続入力/scan/IMU完全一致/出力連続性がNG。array_comparison.jsonでも全方向の入力/画像/出力連続性がNG。純LIOのscan2817対参照2823、最大出力間隔0.7秒。原因の特定は未完了。速度制限違反や手法精度不良と決めつけない。無効試行を正常な精度比較へ混ぜない。
- 初回独立検証VIO方向RMS[m/s]：b1前半偏向.12368/固定-50 .01759、後半偏向.00977/固定-50 .01838（後半は全固定5方向より良い）。二ツ小屋手前/中/奥：偏向.0204/.1124/.1275、上向き固定.0172/.0511/.0309。a1も最良固定には負ける。改善は区間によって逆転し、固定に安定して勝つという目標は未達。
- 未実行はb1/Fut seed412、a1/b1/Fut seed413の5飛行。a1/412は原因解決後に必要なら再試行。元のcontinue_validation.pyを先頭から再実行しない（既存出力へ衝突、凍結情報を上書きする）。runnerにresume機能なし。別継続処理で完了済みを保持し、実験条件のSHAを確認する。
- 次候補motion_view_v5.yamlは既存v54の実ROVIO特徴・速度の同時不確かさ評価＋準備運動中の視野選択。設定だけ用意、未実行。v3の結果を見て改良した次候補には別の未使用seedで独立検証が必要。v3を黙って書き換えない。
- 結果/再開参考：各development_v*/report/motion_index.html、validation/report/summary.json、/tmp/motion-view-validation.log、ROOT/progress.py。最終report_motion_view_trials.pyとaudit_and_summarize.pyは独立9本完了前提なので未完了状態を最終結果と呼ばない。GUI/raw bagなし、数値ログ保持、Windows空きguard8GB。

## 2026-10-22 — 固定優位のVIO拘束追加収集（完了）
- 新規9飛行、ZaB a1/b1と二ツ小屋の7区間×3回。ROOT `/root/uas_native/logs/visual-constraint-survey-20261022`。全停止済み・継続待ちなし。レポート `report/index.html`、全比較PDF、代表時系列cases.pdf、summary/cases.json、visual_samples.csv。
- VIO速度観測そのものの精度で固定が3/3回優位な代表3例: b1 60–120秒(-50,0) .017994 vs 偏向.299021m/s、b1 180–240秒(-50,0) .030396 vs .401186、二ツ小屋60–120秒(0,-90上) .011009 vs .149350。各飛行RMSの平均。5/10/20秒の共通方向更新・全3D速度誤差でも各3/3勝利。他2試行で方向を選んでも同じ方向が選ばれ、評価試行で勝つ。
- 全候補は3区間11方向比較（独立11環境ではない）。他4区間は順位逆転を含み、固定が恒常的に良いとはしない。a1復路3回目は偏向が全固定より正確。二ツ小屋手前偏向は.359/.073/.016m/sと変動が大きいので平均のみで説明しない。
- 採用v57・DRPM OFF・融合重み・偏向規則は変更なし。共通純LIO弱方向への速度誤差が主指標、融合後の誤差相殺で選んだケースではない。方向は漂流する純LIO地図由来の診断指標で、強異方性基準は126窓中8窓。真の退化を保証した実験ではない。
- 各85項目×9飛行=765/765成功、全所有終了0、採用/実行/評価SHA一致。約537MiB、Windows空き約160GiB。詳細はWORKLOG最新節。新規ソースrun_visual_constraint_survey.py/report_visual_constraint_survey.py/plot_visual_constraint_cases.py。レポート再生成手順はWORKLOGに記載。commit未実施。

## 最新：方向順位の反復検証（2026-10-22、完了・全停止）

- ROOT=/root/uas_native/logs/direction-repeats-20261022、入口report/index.html。ZaB a1往路40–100秒/復路160–220秒、二ツ小屋60–120秒/180–240秒、各3回新規飛行（seed20261201/2/3）。履歴保持・評価のみ切出し、各約30m。v57/DRPM OFF・固定5方向＋偏向＋純LIO、真値補正なし、設定採用変更なし。
- 主10秒方向誤差RMS平均[m]: ZaB往路 固定最良上.05804/偏向.03982（3/3で全固定より良好）、復路 正面.09069/偏向.10996（正面に1勝2敗）。上向きは復路3回目に破綻（10秒48.83m、60秒位置181.68m）し、入力検査合格の性能結果として除外せず保存。
- 二ツ小屋は手前 −50 .20851/偏向.43462、奥 −50 .09294/偏向.53837。偏向は−50に手前1勝2敗・奥0勝3敗。奥の最良は下/下/−50で3回平均は−50。2回時点の最良方向切替は3回目に再現せず、最終平均では両区間−50が良い。固定優位を偶然として無効扱いしない。
- 60秒位置RMSEではZaB往路 上.2104/偏向.2201、復路 正面.2768/偏向.2422で主10秒と一部逆転。二ツ小屋 手前−50 .3194/偏向1.5740、奥−50 .1683/偏向1.0601。全窓長/全試行値を保持し、一指標の勝ちを普遍的勝ちへ拡張しない。
- 視覚速度そのものは二ツ小屋偏向が正確。奥の速度RMSは−50 .5570/偏向.0413m/s、−50の速度偏りは3回全て過大、純LIO移動は全て過小。系統的な誤差補償を支持する再現例。固定の融合成績の勝ちと、観測自体の正確さを分ける。因果寄与率未同定。
- 残り2試行で方向を選んで1試行へ適用する評価も保存。ZaB 共通正面.08163/区間別.07436/偏向.07489、二ツ小屋 共通−50 .15072/区間別.17627/偏向.48649m。強異方性閾値該当は主72窓中3だけで、真の退化方向認定とは区別。
- 全6×65/65監査成功、採用/実行/評価SHA不変、全所有停止正常終了。ログ約306MiB、Windows空き約173GB。GUI/bagなし。source: active_perception/run_direction_repeats.py、report_direction_repeats.py、report_direction_segments.py。再集計は上2report→ROOT/plot_choices.py→ROOT/finalize_findings.py。WORKLOG末尾に詳細。再開待ちなし、commitなし。

## 二ツ小屋隧道290m・固定5方向（2026-10-21、完了・全停止）

- 同じ飛行で固定5方向＋偏向を独立単眼ROVIO＋MIMOSA処理。v57/DRPM OFF/0.5m/s、角度以外同条件、追従582.177秒。新--view-arrayで再実行可。
- 融合位置RMSE[m]: (-50,0)2.5665、(0,0)5.5140、(+50,0)6.0679、(0,+90)下7.8524、(0,-90)上9.8527、偏向5.1247。同時純LIO167.8541。固定5方向等確率選択のRMSE期待値6.3707より偏向19.56%低い。最良固定は事後選択。1試行で有意差/再現性未評価。
- 前回二眼飛行とはノイズ厳密同一を仮定しない。今回65/65全方向監査、元20/20、全期間coverage1。採用/拡張/資産SHA不変、全所有/子exit0・残存なし。ログ89MiB、Windows空き176.65GB。
- 結果 /root/uas_native/logs/futatsugoya-20261021-five-views290/index.html、interior/six_axes.pdf、summary.json、final_audit.json。全停止、自動再開不要、commitなし。

## 二ツ小屋隧道290m比較（2026-10-21、完了・全停止）

- ユーザー指示の290.088mを0.5m/s、追従582.177秒で完走。採用v57/DRPM OFFは変更なし。固定(0°,0°)、偏向、純LIOを同じ飛行で比較、GUI/bagなし。
- 位置RMSE[m]: MIMOSA単体170.4573、固定融合2.1291、偏向融合7.2903。今回は偏向が固定より悪い。ROVIO単体は固定235.1055/偏向12.1992で逆順位。融合の原因切分けは未実施、誤差相殺とは断定しない。
- 全5821評価時刻、初期剛体整合のみ、全監査合格・圧縮後再集計一致・採用/資産SHA不変。全所有/子exit0・残存なし。所要約11.2分、ログ45MiB、Windows空き176.82GB。
- 結果 /root/uas_native/logs/futatsugoya-20261021-interior290/index.html、comparison.pdf（六軸2ページ）、final_audit.json。1試行、既存の近似尺度/照明条件の制約は維持。自動再開不要、commitなし。

## 二ツ小屋隧道の導入（2026-10-21、20m確認完了・全停止）

- 新環境assets/futatsugoya/futatsugoya.sdf、手順assets/futatsugoya/README.md。作者Fujihiko MatsumotoのCC BY 4.0モデル。元39.3万三角形と4K画像12枚を保持。材質対応を復元してGazebo用に変換。
- 元の実寸不明につき近似入口間=384mで一様拡大。これは測量校正ではない。一様環境光＋写真テクスチャで、実際の暗闇の再現ではない。world名zabは通信互換用で、形状は新モデルのみ。
- 新prepare_futatsugoya.py/run_futatsugoya.pyとsimulation_inputs/futatsugoya.json。採用v57/センサ/推定器/校正は変更なし。DRPM OFF。新規端末から `python3 /root/uas_native/active_perception/run_futatsugoya.py --output /root/uas_native/logs/futatsugoya-new-check` 。GUI/bagなし、Ctrl+C可。--route interiorは290m（形状距離確認済み、全飛行未実施）。既存出力へ上書きしない。
- 20m/0.5m/s、真値制御の同一飛行で純LIO・固定融合・偏向融合を確認。最終試行位置RMSE[m]は11.7528/8.0917/8.2864、ROVIOは固定0.8632/偏向0.7357。短い区間で大ドリフトがあり、偏向の改善は出ていない。
- 最初の2回も飛行と比較は正常だが補助点群確認に不具合があり、修正後の3回目は補助確認も成功。全3試行保持。独立3seedの性能実験ではない。LiDARと元形状の距離中央値2.0cm、95.1%が10cm以内。法線による形状診断は長手方向が弱い。詳細・制約はWORKLOG末尾。
- 結果 /root/uas_native/logs/futatsugoya-20261021-final/index.html、smoke/comparison.pdf、final_audit.json。全所有プロセス終了、採用SHA不変。資産約105MiB/実験ログ計33MiB、Windows空き約180GB。自動再開不要、commitなし。

## 旧logsのDrive移管（2026-10-20、完了・全停止）

- ユーザー指示で旧native/logs全体をDriveへ移管し、ローカルから削除。以下の過去の実験結果パスは保存当時の記録であり、今はローカルに存在しない。旧データから自動再開しない。
- 保存先: https://drive.google.com/drive/folders/1SNkp_XT7AsO2UyedbRQ1FqM1qJKQNiId 。元ファイル実体65,284,512,431 bytes、122,533 entries。圧縮ログ19,825,561,289 bytes（74分割）＋別保存47 MCAP 27,612,490,864 bytes。全分割・全MCAPのサイズとMD5一致、元ツリー不変を確認して削除。リンク先がlogs外にある実体は削除していない。
- 新規オンライン実験に必要な入力は `/root/uas_native/simulation_inputs/README.md` に集約。14経路・採用3経路・融合校正・起動テンプレートを保持。採用v57の推定/偏向方式・数値設定は不変。オンライン起動側の過去logs参照を置換し、9経路比較のprefixも新規実行する。
- 復元手順・保存確認は `/root/uas_native/archive_records/logs-20261020/`。`restore.py --destination <空のディレクトリ>` でlogs/を復元する。`--without-bags` で別保存47 MCAPを省略可能。Google DriveのフォルダIDを固定して取得。過去の図・オフライン再解析には復元が必要だが、新規シミュレーションには不要。
- 削除後の入力/起動準備テスト2件、固定5方向モデルテスト2件成功。さらに旧logsが空の状態でrun3の離陸・初期化・3秒追従を完了、固定/偏向/純LIOの検査valid=true。全所有プロセス/子プロセスexit0、残存なし。短い起動確認であり新たな性能比較の結論には使わない。
- 最終Windows空き12.91GB。WSL内データ削除量とCドライブの実空き増加は同一ではない。VHDのオフライン圧縮やWSL終了は行っていない。8GB停止条件は維持。

<!-- CURRENT_FIXED_FIVE_VIEWS -->
## ZaB固定5方向同時比較（2026-10-20、完了・全停止）
- ROOT=/root/uas_native/logs/zab-fixed-five-directions-20261020。直前のrun3/run4/run1_tail_branch各2回で、固定5方向と偏向を同じ飛行で独立単眼ROVIO＋MIMOSA処理。5眼融合ではない。採用v57・DRPM OFFは不変。pitch+90下/-90上。各カメラの位置・画質・周期は同一、固定姿勢以外の推定設定は同一。
- 2回平均位置RMSE[m]：

|経路|(-50,0)|(0,0)|(50,0)|(0,90)下|(0,-90)上|偏向|
|---|---:|---:|---:|---:|---:|---:|
|run3|0.034024|0.038944|0.035754|0.057610|0.047277|0.039089|
|run4|0.026034|0.021730|0.023391|0.033999|0.030232|0.023168|
|run1_tail_branch|0.040888|0.043916|0.050413|0.068119|0.073542|0.044862|

- 固定方向への依存あり。平均では偏向は最良固定を超えず、試行別で全固定を超えたのはbranch第1試行のみ（1/6）。run3の最良方向は試行ごとに変化。短い約33～52秒×2回の検証であり、最良固定は事後選択。長期一般化や有意差を示すものではない。前回二眼飛行とは同seedでも入力ノイズ一致を仮定しない。
- 6/6 valid、全固定方向の露光・外部姿勢・ROVIO出力時刻、LiDAR/IMU共通、設定一致、連続推定を確認。凍結81SHA・追加4SHA不変、全子exit0、所有PID残存0。結果約109MB、Windows空き18.89GB。index.html/6ページcomparison.pdf/summary.json/final_audit.json。画像は方向ごと1例のみ、bagなし。
- 実装はfixed_view_array.py/run_view_array.py/run_view_array_campaign.py/audit_view_array_inputs.py/report_view_array.py/test_fixed_view_array.pyとunipilot_view_array.launch.py。詳細はWORKLOG末尾。全完了で自動再開不要。採用方式は変更していない、commitなし。
<!-- /CURRENT_FIXED_FIVE_VIEWS -->

<!-- CURRENT_ZAB_ADDITIONAL_CONDITIONS -->
## ZaB追加3条件の固定/偏向比較（2026-10-20、完了・全停止）
- ROOT=/root/uas_native/logs/zab-additional-conditions-20261020。run3/run4/run1_tail_branchを事前指定し各2回（要求seed20261101/20261102）。採用v57・UniPilot単眼・DRPM OFFは変更なし。純LIO/固定融合/偏向融合を同じ飛行・LiDAR・IMUで評価。短い追従33～52秒、初回剛体整合・尺度補正なし。
- 2回平均位置RMSE[m]：run3 単体.200956/固定.045081/偏向.044165、run4 .025844/.024048/.025814、branch .172256/.034792/.034141。偏向は3/6改善、3/6悪化。run3は2/2改善、run4は2/2悪化、branchは1勝1敗。差は小さく、新環境や長期退化への一般化を示す結果ではない。
- 全6valid・圧縮後再計算一致・81SHA不変・全子exit0・残存0。GUI/bag/生画像点群なし、保存約61MB、Windows空き約19.37GB。index.html/6ページcomparison.pdf/summary.json/final_audit.json。WORKLOG末尾に全数値。採用方式の調整なし、commitなし。
- 再実行コードrun_zab_additional_conditions.py（事前条件・再開状態・既存run_one再利用）、report_zab_additional_conditions.py。全完了なので自動再開不要。SubTGraphは却下されたまま、今回使用していない。
<!-- /CURRENT_ZAB_ADDITIONAL_CONDITIONS -->

<!-- CURRENT_LIO_FAILURE_COMPARISON -->
## 高誤差経路9本の三構成比較（2026-10-19、完了・全停止）
- ROOT=/root/uas_native/logs/lio-failure-comparison-20261019。旧DRAGON14runの単体位置RMSE>1mで事前選定した9本を、現在のUniPilot v57・単眼・DRPM OFFで比較。prefixは直前結果を再利用、8本を新測定。各run1試行/seed20261026。現在単体誤差が小さいrunも除外せず保持。
- 現在も単体位置RMSE>1mの5本（単体→固定融合→偏向融合、m）: prefix 7.898→5.747→4.697、middle 3.494→2.604→1.227、b2 80.034→26.131→2.344、2a_prefix 35.357→4.945→11.761、2a_end 12.035→2.694→2.613。融合は単体より改善するが、偏向は5本中4改善1悪化。2a_prefixは固定比2.378倍悪化。全9本では偏向対固定6改善3悪化。現在単体1m以下の4本も全結果ページに保持。
- 位置と回転は区別。middleの回転RMSEは単体0.541°/固定0.383°/偏向0.525°、位置改善が全軸改善を意味しない。既存の採用3経路×2条件の結果を他経路へ一律に一般化できない。本比較で設定調整なし、採用v57はそのまま。
- 各run同一物理飛行/LiDAR/IMU。純LIOはROVIO速度/傾き両拘束を初期化から除去、LiDAR4自由度/IMU等は同一。全9本比較検査valid、参照複製の全入力/出力厳密一致、純LIOの参照全入力prefix一致と評価後追加末尾を分離検査。全区間/尺度補正なし、圧縮後再集計も厳密一致。
- 結果ROOT/index.html（全表・六軸図）、all_runs.pdf（9ページ）、comparison.json、final_audit.json。新run_lio_failure_comparisons.py/report_lio_failure_comparisons.py、一般化report_pure_lio.py、既存observe_pure_lio.py。実装snapshotをROOT/implementationへ保存。再開不要、campaign.json complete。
- 全所有プロセス終了0、domain85/master11485残存0、採用81SHA不変。GUI/画像点群/bag追加なし、新結果約251MB、既存prefix約78.8MB参照。Windows空き21.34GB（終盤一時11.13GBまで減少したが終了後回復、原因未特定、8GB停止には至らず）。詳細WORKLOG末尾。commitなし。
<!-- /CURRENT_LIO_FAILURE_COMPARISON -->

## 追加検証：run1_prefixのMIMOSA単体（2026-10-19、完了・全停止）
- 同一1飛行/同一LiDAR・IMUで採用v57の純LIO、固定ROVIO融合、偏向ROVIO融合を同時比較。全1070秒、seed20261026。純LIOはROVIOの速度・傾き拘束を初期化から除去し、他の設定・実行ファイルは不変。カメラ/機体制御には純LIOを使わない。
- 位置RMSE/回転RMSE: 純LIO **7.898m/1.904°**、固定融合 **5.747m/1.509°**、偏向融合 **4.697m/1.399°**。ROVIO単体は固定54.897m/偏向104.333m。この試行ではMIMOSA単体にもドリフトがあり、融合は位置・姿勢誤差を減らした。前回の採用検証ログとの同一入力比較ではなく、今回新しく同時取得した1飛行の結果。
- 結果 `/root/uas_native/logs/prefix-pure-lio-20261019/index.html`、`comparison.pdf`、`comparison.json`。通常融合の複製は全出力/入力が厳密一致。純LIOの参照全11313スキャン/累積IMU一致、VIOイベント・速度/傾き因子0。終了時に純LIOだけ追加1スキャンを評価後に処理したため、全寿命の件数一致はfalseとして保持。評価区間/数値を変えず、完全参照prefix一致と評価後追加を別検査し精度比較valid=true。
- 全プロセス終了0、採用81SHA不変。GUI/画像点群/rosbag保存なし。数値は可逆圧縮。実装 `active_perception/{run_pure_lio_comparison.py,observe_pure_lio.py,report_pure_lio.py}`。採用ジンバル方式/設定は下記v57のまま。

<!-- CURRENT_GIMBAL_TASK -->
## 現在の採用構成（2026-10-19、検証プロセス停止済み）
- ユーザー依頼はROVIO＋MIMOSAのジンバルである程度安定した改善まで自走、ログ最小化/Windows容量管理。最新「再開して」に対してv59の開発/独立検証まで実施し、再現性のあるv57を採用可能な形に整理した。ROOT=/root/uas_native/logs/gimbal-improvement-20261017、実装=/root/uas_native/active_perception。
- 採用v57: 3経路×2乱数条件（26/27）の独立6組全て融合位置改善、平均17.7385%低減（比.8226147923506177）。経路平均a1 21.3012%/b1 20.4782%/prefix11.4361%。5勝以上/各経路平均10%以上/最大悪化条件は通るが、事前平均20%目標は未達。元のpassed=falseを保持。一般環境での保証や20%基準達成とは言わない。ROVIO単体は4悪化2改善であり、融合改善と混同しない。
- 結果入口ROOT/adopted_gimbal.html（採用6組・不採用4組・六軸図へのリンク・再実行方法）、gimbal_selection.json。詳細validation_v57.html/validation_summary_v57.json。v57凍結frozen_candidate_v57.json/frozen_v57_sources.zip。
- v57独立fixed→active m: a1/26 .472135→.334259、b1/26 .226205→.195271、a1/27 .284→.246、b1/27 .278→.202、prefix26 7.151→5.954、prefix27 6.796→6.380。全valid/全子0。長区間は1070秒（route_statusの元3778秒ではない）。長区間に数m誤差は残る。
- 現在のplanner_node.pyは保存済みv57へ復元し、全81ソース/バイナリSHAが検証時と厳密一致。selected_v57_source_verification.json。新run_selected_gimbal.py＋selected_gimbal.jsonが81SHA確認後に同じ実験条件で再実行する。例: python3 /root/uas_native/active_perception/run_selected_gimbal.py --output /root/uas_native/logs/gimbal-repeat-new 。--labels run1_tail_a1 / --seed / --dry-run対応。既存出力先拒否。未依頼の再実行を勝手に開始しない。
- v59（視認性用8mを画像時刻に整合したLiDAR支持距離で補う）は開発6組5勝/平均20.71%低減で合格したが、未使用28/29の独立短4組は2勝2敗。a1/28 .384377→.295469、b1/28 .138367→.152057、a1/29 .275711→.305014、b1/29 .188197→.186844m。3組時点で2敗となり5/6勝が不可能だったため、実行中4組目を完走して未開始長2本を取り止めた。全実施結果保持、v59_independent_futility_stop.json、validation_v59.html。26～29は今後の未使用独立条件として再利用しない。
- v59全84SHAの最終一致記録を保存してから復元。v59全ソースzip/差分patchを保持し、専用helper/2tests/config4ファイルはROOT/rejected_v59_source_extrasへ移動（rejected_v59_source_archive.json）。v59を再現する場合はfrozen_v59_sources.zipの実装が必要。現在の採用nodeはv57。
- 共通: camera yaw±50/pitch±90、合成20deg/s、scan5deg/s、実駆動制約。既観測画像＋LiDAR支持面/推定移動三角測量で地図化。実ROVIO追跡点は視認性用8m。融合はROVIO速度＋重力傾き1Hz、DRPM融合OFF、LiDAR4DOF、重力9.8、初速σ1、同時刻観測後map/output、センサbias OU、legacy IMU白色雑音。校正ROOT/native_fusion_velocity_gravity_calibration.json。真値は飛行制御/独立校正/事後評価だけ。
- v57全6の事後診断ではa1は速度/傾きとも悪化、b1両方改善、prefixは速度悪化/傾き小幅改善。v59の悪化b1/28でも速度/傾きは改善、95%速度誤差楕円内率は固定可動とも1.0。初期状態は完全一致。単純にVIOが良ければ融合も良いという説明はできず、因果除去実験は未実施。
- v58は短4組中b1/25が1.64倍悪化して不採用、全結果保持。旧v57容量停止2件は失敗側保持して同seed/条件/全長を再取得、共有リンク修正記録v57_stopped_archive_reference_repairs.json。今後アーカイブ移動時は共有リンクの付替え必須。
- 現在所有sim/scheduler/observer残存0。v59 scheduler22584は完走後の意図的終了130、resource58823/storage93336はSIGTERM143、simの子は全0。cleanup94609は終了0。Windows空き21.586GB、今後も8GB停止を維持。GUI/画像/点群/bag追加保存なし、元研究bag保持。
- v57独立6/v58短4/v59開発6/v59独立4を停止後SHA照合可逆圧縮、再集計で全結果辞書厳密一致。必要な校正、数値、設定、凍結実装、失敗記録を保持。Windows VHD物理縮小とは主張しない。
- 再実行CLIのdry-run/既存出力保護、採用81SHA、結果16リンク、生成コマンド構文を確認。README2026-10-19節、WORKLOG末尾に詳細。commitなし。次に追加改善を頼まれた場合は、20%基準未達とv59不再現を出発点にし、過去結果を成功へ書換えない。
<!-- /CURRENT_GIMBAL_TASK -->


> **最新（2026-10-14 14:24 UTC）：追加2時間の高速化を終了。欠落監査を通ったa1全長は394.04→306.96秒（1.284倍、22.1%短縮）。5倍は未達。検証用プロセス残存0。**
> 実施枠12:28:12～14:28:12 UTC。コンパクトな地図索引、同一入力の計算再利用、C++画像/点群受け渡し・CSV記録、時計重複除去、終了端の処理完了待ちを隔離実装。物理1ms、LiDAR10Hz/500×48、IMU200Hz、両カメラ20Hz/480×270とノイズを維持。通常GUI起動は変更しない。
> 採用可能な速度測定は `native/logs/speed6-final-prefetch/`（timestamp監査true、全子プロセスexit0）。neighbors再測定294.64秒はIMU周期1件分の欠落があるため、欠落なしの性能結果とはしない。prefetchが欠落を防ぐと実証したわけではない。単体の検索/行列/センサ変換は原実装と一致、オンライン推定精度同等は未確認。
> 全3区間試行 `speed6-full` はa1/b1完走、prefix328.849/1070秒で追従誤差0.18757mのため停止。全比較完了・5倍判定false。b1にもIMU1件分の欠落あり。UniPilot単眼ROVIOの大きなドリフトは未解決で、固定/偏向精度比較へ採用しない。
> 曲がり角の先読み減速（任意 `--corner-speed-limit`）は制御だけの切り出し試験で最大追従誤差17.6→7.0cm。経路/停止上限は維持するが角で減速するため、速度比較の最終a1試験には使用していない。prefix全長・推定器同時負荷は未検証。
> 再現コマンドはREADMEの高速化節、実装/試行/制約はWORKLOG末尾、集計は `native/logs/speed6-final-summary.json`。旧DRAGON図・データは不変。自動再開しない。
>
> 以下は前回2時間枠の記録。

> **最新（2026-10-14）：UniPilot固定/偏向比較の「最低5倍・2時間」高速化は未達。検証用プロセスは停止済み。**
> センサ生成/ICP並列化は部分検証済みだが、全3軌道比較は未完了。最終prefixは327.84/1070秒で追従誤差0.181m（既定上限0.18m）により停止、a1は207.38/228.49秒で中断、b1未実行。旧比較結果は不変。
> センサ単独RTF3.7616は全体の高速化倍率ではない。長区間はMIMOSAの地図更新・推定処理待ちが支配的、UniPilot単眼推定の大きなスケールドリフトも残る。高速化入口は実験用で、既定起動へ採用しない。
> 監査: `native/logs/speed5-final-audit/performance_audit.json`（完走・5倍・全区間品質の判定はfalse）。再現手順と未検証事項はREADMEの2026-10-14節、詳しい事実はWORKLOG末尾。旧手動ジンバルGUIは維持。自動再開しない。

> **最新（2026-10-04）：LIO/LIVO退化対策の文献調査完了。推定器の追加導入・実験は未実施。**

> **最新（2026-10-04）：単眼LIVO 4条件・2飛行の比較完了。**
> `run2a_end` / `run1_tail_a2_prefix`、カメラlink3の1台に統一。MIMOSA OFF/ON/DRPMには同じ単眼ROVIOをodometry因子として入力。元の14飛行の結果と図は不変。
> 位置RMSE(m)は OFF=12.557/4.881、ON=同一（標準閾値で作動0回）、FAST-LIVO2=0.0204/0.3478、DRPM=4.631/9.759。姿勢RMSE(deg)はOFF/ON=1.490/76.106、FAST-LIVO2=0.579/4.117、DRPM=5.229/123.004。今回の設定ではFAST-LIVO2が最良、DRPMは姿勢崩れを解決していない。
> 図：`native/logs/zab-offline/livo_comparison/plots/index.html` / `comparison.pdf`（2飛行×6軸、軸省略なし）。説明・数値：同ディレクトリの`../README.md`、版管理要約`resources/LIVO_COMPARISON_20261004.md`。`tracking/`に真値/単眼ROVIO/4条件の元座標の姿勢CSV。全8ケースexit0、989/4004点、20Hz、元時刻の欠落/余分0。
> 実装は独立`/root/multilink_autonomy_ws/comparison_ws`。通常SLAM workspace/launchを変更していない。再現用patchとDRPM実装は`native/comparison/`、準備・再生・評価スクリプトは`native/*livo_comparison*`。標準ONの代入漏れ、同期瞬時LiDARのゼロ積分区間、同時刻最終姿勢の出力を修正。DRPMは論文Eq28の情報量を使うグラフへの移植であり、独立ICPの再現ではない。各方式の内部重みは個別設定を保持、最適調整済みの比較ではない。
> 所有ROSプロセス停止済み、GUI/Gazebo起動なし。入力キャッシュ約3.3GBを追実験用に保持、C:空き約16GB。詳細はWORKLOG末尾。

> `resources/README.md`に指定4候補の方式・実証範囲・公開実装状況・比較の優先順位を記載。既存2 PDFを保持して11 PDFを追加、計13論文。`resources/manifest.json`に取得元・版・ページ数・SHA-256・確認した公式repoのcommitを保存。全PDFの読取りとローカル参照を確認済み。
> FAST-LIVO2は通常LIVO基準として適切。Switch-SLAM公式コードは未確認。SA-LIVO v2は公式コード公開予定で、見つかった第三者実装はv1/v2混在・独自拡張があるため公式と同一視しない。
> 指定arXiv:2410.10784はDRPM（点群位置合わせ用の退化判定・抑制部品）であり、単独LIOではない。論文実験は脚odometry、ROVIO、外部odometry、レーダー等を併用。純LIO比較には共通LIOへ組み込み、補助入力を外して別途検証する必要がある。
> 比較候補: MIMOSA退化対策OFF/ON、FAST-LIVO2画像なし/あり、FAST-LIO2、DRPM組込み、X-ICP系（後続perfectlyconstrainedの公式比較実装）、LVI-SAM。GenZ-ICPはIMUなしLOとして別枠。COIN-LIOは有効な反射強度の確認が前提。現MIMOSAはproject_on_degneneracy=false・回転閾値0。再確認した有効処理は弱い点の並進成分への投影のみで、回転成分への適用はコメントアウトされている。3カメラROVIOと公式単眼FAST-LIVO2の視野差、瞬時LiDARの点時刻0、IMU誤差モデルの整合を比較条件に含める。
> 既存推定結果・設定・元bagは変更なし。GUI・ROS推定器・シミュレーションの起動なし。

> **最新（2026-10-03）：全14飛行のVIO精度データと六軸プロットを更新済み。計算用ROSプロセスは停止。**
> 新CSV: `native/logs/zab-offline/vio_recomputed/`（truth/LIO/VIOの追従期間のみ）。新図: 従来と同じ `error_plots/index.html` / `all_flights.pdf`（14ページ、六軸、縦軸省略なし）。旧図は `error_plots_before_vio_recompute/`。詳細はWORKLOG最終節とcomparison.json/validation.json。
> 前回の `gyro_matched_depth10` を全14件に共通適用。11件を再処理、検証済み3件を再利用。2飛行の初期化時刻1 ms差を修正して再処理し、全14件の初期化を旧結果と同じ2.003秒へ統一。真値は推定に使用していない。VIO84,830件の元整数ns一致、LIO/真値不変、初期化1回・遅延IMU警告0を検証済み。
> 全14飛行で位置RMSE低下。全評価点の位置RMSE42.621→1.698 m。最大位置誤差は13/14飛行で低下（run1_prefix110.366→4.904 m、run1_middle93.254→3.191 m）。run3の最大値だけ0.254→0.274 mへ小幅増加。run1_middle末尾の既知2時刻未出力は維持し補作しない。
> **新VIOはCSV。旧results配下のMCAPは旧VIOのまま保持しており、新結果と混同しない。** 元センサーbag・本番設定は変更していない。診断用出力はvio_audit/recomputed_diagnosticsへ分離。再生成は `python3 /root/uas_native/plot_offline_errors.py`（完了済み新CSVを選択）。
> Drive新CSV: https://drive.google.com/drive/folders/1FaOAITQ6HfaM39PfEn24vhHZvG3vC7_h 、新図: https://drive.google.com/drive/folders/1Ty3-Dp3NqgZl1L_Il6zk8YuPtIRtJz5w 、旧図: https://drive.google.com/drive/folders/1TaSxK-xpIxs9fNBn7t-R0xUL_BP3Kkmy 。新CSV117・新図64・旧図62、計243ファイルのサイズ/MD5一致を確認。元の図/結果は保全。
> GUI/Gazeboは起動せず、専用master11375・推定器は終了。入力キャッシュ空、C:空き約26 GB。ユーザーkeyboard PID20007は停止していない。

> **最新（2026-10-03）：VIO追加監査と45条件の追実験が完了。全所有プロセス停止済み。**
> レポート: `native/logs/zab-offline/vio_audit/report/index.html`、六軸/状態図6ページPDF、45条件のCSV/設定ZIP。Drive: https://drive.google.com/drive/folders/1RjLjMmoimR0zpeQ6wXGD4nkU4_v8qWCi （21ファイルMD5一致）。WORKLOG最終節に詳細。
> 入力破損/接続ミスを示す証拠なし。全14runの校正整合、代表3runの全センサ時刻/固定関節/IMU物理整合を確認。直接再計算baselineは3件すべて旧VIO位置出力と完全一致。通信キューや高速再生ではドリフトを説明できない。
> **旧VIO誤差は環境だけの退化とは扱わない。** ジャイロの誤差モデルが模擬センサに合わず、実際にはない偏りと向きのずれを推定する。モデル整合だけでa2_prefix/b2/2a_endの最大Yaw誤差8.695/16.967/1.121deg→0.022/0.583/0.020deg。さらに距離初期値2→10mで最大位置誤差6.554/22.387/0.543m→0.912/1.713/0.086m。3飛行・45条件で比較、他11飛行は未検証。原本入力不変、通常条件にGT入力なし（GTを使うバイアス診断2条件は明示して区別）。
> 本番設定・元推定結果・既存ROVIO実行ファイルは変更していない。比較候補は `report/candidate_sim_rovio.patch` と各実験のrovio.info。設定はシミュレーションの既知ノイズ用で、実機へそのまま適用しない。ステレオON/75特徴だけで完全解決する結果ではない。
> 検証用master11375、推定器、転送は終了。一時入力NPZ55個（約4.07GB）・原本キャッシュは削除、元Driveデータは保持。再検証にはprepare_vio_audit.pyで入力を再生成する。verify_vio_audit.pyの全45条件検証済み結果はvalidation.json。GUI/Gazebo起動なし、ユーザーkeyboard PID20007は停止していない。

> **最新（2026-10-03）：指定9runの環境・LIO/VIOドリフト調査完了。**
> 調査資料: `native/logs/zab-offline/environment_analysis/report.html`。全軌道と27時点の元点群・3カメラ81画像を調査。Drive: https://drive.google.com/drive/folders/1kLCPB1GBx318shxmcG3uDHnePMUHIHv9 （163ファイルMD5一致）。run2_tail_b2はユーザー確認でrun1_tail_b2へ訂正。
> LIOは進行方向の形状情報の弱さを多くのrunで確認。run2a_endは真値22.4mに対しLIO進行ほぼ0、VIO最大誤差0.54m。姿勢破綻を伴うrunやrun2bの約22.9kmの発散は、環境以外に退化保護OFF/重力方向/バイアス/地図対応の問題も候補。VIOは暗部影響のあるrunと、画像特徴が豊富でもずれるrunを区別。ROVIO全体25特徴・ステレオ距離初期化OFF・低加速度運動・IMU加速度bias_mean0.1が候補だが因果未確定。内部状態の記録と一要因ずつの比較実験を資料で提案し、実施はしていない。
> スクリプト: analyze_zab_runs.py、probe_zab_sensors.py、map_zab_analysis.py、summarize_zab_probes.py、report_zab_environment.py。入力は採用metadataで選択・MD5照合、キャッシュ削除済み。元データ・推定設定・既存誤差図は変更していない。GUI/推定器/シミュレーション起動なし。

> **最新（2026-10-03）：LIO/VIO六軸誤差プロット作成済み。**
> `native/logs/zab-offline/error_plots/index.html`が一覧、`all_flights.pdf`が全14飛行・14ページ。各飛行のPNG/PDFとLIO/VIO誤差CSVも保存。Drive: https://drive.google.com/drive/folders/1Ty3-Dp3NqgZl1L_Il6zk8YuPtIRtJz5w （全61ファイルMD5一致）。再生成は`python3 /root/uas_native/plot_offline_errors.py`。
> 青LIO/橙VIOを各軸に重ねる。独立14飛行を各1図、同一飛行内の分割は統合。追従開始の共通時刻で位置/姿勢を一度だけ整合、真値を推定時刻へ補間（外挿なし）。世界XYZの位置誤差と真値機体基準の相対回転xyz Euler角を表示。元データは不変。ユーザーの再指定により全図の縦軸省略を撤去。各軸は全誤差範囲を表示。整合条件/表示範囲/除外件数はevaluation.json、説明はREADME.txt。

> **最新（2026-10-03）：採用14bagの独立LIO/VIO後処理・Drive保存・最終照合が完了。全所有プロセス停止済み。**
> 真値392,184件、MIMOSA LIO42,416件、3カメラROVIO84,830件。追従窓の合計4,241.56秒。離陸・ホバリング・向き合わせは初期化入力にのみ使用、保存対象から除外。画像・点群は結果に含めない。元センサーbagはDriveに保持。
> 出力: `native/logs/zab-offline/results/`（14件のMCAP/CSV＋window/result/manifest/README、550,021,374 bytes）。Drive: https://drive.google.com/drive/folders/1Jj7Jw1ttaG-dmqws_mg5JQHp-0OoSbC- （元dataset配下`odometry_tracking_20261003`）。
> `verify_offline_odometry.py`で全MCAPの3トピック限定・header/保存時刻/CSV整数ns一致・元センサー時刻対応・位置/姿勢/速度/件数/有限性を検証済み。bag間は時計がリセットされるため、元run識別子とstampの組で照合する。座標系は推定器の元のものを保持し、真値への空間的な補正はしていない。
> 制約: run1_middleの末尾1323.15/1323.20秒は後続入力不足でVIO2件が未出力。補作せずresult.jsonへ明示。途中欠落・余分な時刻0。4倍上限のrun1_tail_a1でMIMOSA SIGSEGVがあり、同区間を等速で再処理成功。以後等速上限、原因は未特定。失敗部分はdiagnosticsに隔離し成果物には含めない。
> 実装: `offline_odometry.py` / `process_zab_odometry.py` / `verify_offline_odometry.py` / `package_offline_odometry.py`。監督exec65086はexit0。計算用ROSプロセス・転送処理は終了し、入力キャッシュ削除済み。GUI/Gazeboは起動していない。Drive側の100ファイルすべてサイズ・MD5一致（remote_check.log）、差分0。自動再開しない。

> **現在（2026-10-03 01:27 CEST）：ZaB全域の再収録・Drive保存・最終検証が完了。起動したシミュレーションと記録処理はすべて停止済み。自動再開しない。**
> 条件: 関節固定DRAGON、巡航0.5m/s、GUI/LIO/VIOなし、物理・センサー品質維持。メッシュから通過可能とした25本の道路を全被覆（4つの連結成分）。独立した14bagで保存、各bagのシミュレーション時計は0から始まる。
> 保存先: https://drive.google.com/drive/folders/1eV2i-Y1p07k9A-G2gMWxIWM4B7jNNIYU （rclone `uas_gdrive_private:multilink_autonomy_zab_20261002-190350`）。
> 採用bagの合計: 27,962,075メッセージ、228,918,754,159 bytes（約228.9 GB、MCAP内部の可逆Zstd圧縮）。元SQLite換算395,631,882,240 bytes。5系統（IMU、LiDAR、カメラ3台）すべて周期欠落・時刻逆順0。元データとMCAPの内容・順序・時刻・型/QoSを完全比較済み。
> **採用対象はDrive直下のdataset_manifest.jsonを使用。** ローカル `native/logs/zab-full-campaign/dataset_manifest.json` にも同じ一覧。`remote_final_verification.json`で417ファイル（bag375分割＋各bagのmetadata/検証結果/設定アーカイブ）のサイズ・MD5照合一致。両一覧もDriveへ照合付きで保存済み。
> run1_prefixは異常前のchunk0..79のみ。run2a_prefix（20261003-002709-102783）はLiDAR欠落前のchunk0..21のみをmetadataから参照し、終盤はrun2a_endで約5m重ねて取り直した。未採用chunk22..25、旧full_metadata_with_gap.yaml/full_validation_with_gap.jsonは診断用に保持。excluded_attemptsと20261002-220031-79824も採用対象外。
> 最終監査: `python3 /root/uas_native/summarize_zab_campaign.py`。Drive再照合: env.shをsourceし、UAS_CLOUD_REMOTEにremote.txtの値を設定して `python3 /root/uas_native/verify_zab_remote.py`。秘密情報は/root/.config/rcloneのみ。認証は維持。
> 収録改善: gz_ros2_controlのpause中write抑止、Spinal SIMULATIONのROS clock化、実Windows容量監視、ROS launchの独立プロセス化、IMU reliable/depth2000、最終2bagからLiDAR reliable/depth50。停止応答の一時失敗は3回まで再試行。品質設定は低下させていない。
> 容量制御は変換/転送2並列、raw>2.5GiBまたはWindows<8GBでpause、raw<1.2GiBかつWindows>=9GBでresume、flight reserve6GB。長時間GazeboでWindows容量が急減する原因の厳密な解析は未了。短区間分割と可逆圧縮で今回の収録は完遂。

> **2026-10-02：ユーザー指示により100 m飛行run 20261002-160953-24661のrosbagを削除済み。CSV・評価結果・経路は保持。以下のbag保存済みの記述は削除前の状態。**

> **2026-10-02：ZaB 100 m直線・1 m/s指令の飛行とrosbag保存が完了。シミュレーション/RVizは停止済み。**
> run：native/logs/20261002-160953-24661/。bag 163.972 sim秒・約12.38 GB・12分割。着陸、非arming、manager status=null。
> 実測進行100 m、巡航平均1.00215 m/s、最大時間付き追従誤差0.24969 m、直線からの最大横断誤差0.02045 m。
> 再実行：`ros2 launch multilink_uas_bringup zab.launch.py headless:=true record:=true route:=/root/uas_native/routes/zab_straight`。
> このコマンドは離陸・100 m飛行・着陸・記録後の全体終了まで自動実行する。通常起動は飛行なし・記録OFF。
> 詳細：run内session_summary.json、routes/zab_straight/README.md、WORKLOG最終追記。

> **2026-10-02 訂正：ユーザー指摘によりキーボードGUIを撤去、元の端末操作へ戻した。**
> 起動コマンドは同じ。base_link基準・XY0.4 m/sを維持。Qt依存なし。
> 複数キーは直近0.25秒の受信入力を合成する近似方式。端末ではキー解放・厳密な同時長押しを検出できない。
> 以下の操作用Qtウィンドウに関する記述は旧状態。詳細はREADME末尾。

> **2026-10-02：対話Bashにaerial_robot_baseとROS2 bridgeを追加、キーボードをbase_link基準・XY速度2倍・同時押し対応へ変更。**
> `source ~/.bashrc` 後、`ros2 run aerial_robot_core keyboard_command.py --ros-args -p robot_ns:=/dragon`。
> 操作用Qtウィンドウを選択して押下中のみ移動。解放・フォーカス喪失で停止。base_linkの傾きを含む3D方向。
> 外部変更の再現差分はnative/aerial-keyboard.patch。仮想環境・ROS1は端末に読み込まない。
> 検証用シミュレーションは起動していない。

> **2026-10-02：ユーザー指摘を受け、エージェント起動のZaB・推定系・RVizを停止済み。manager status=null。**
> 今後は継続起動の明示指示がない限り検証後に停止し、ユーザー自身の端末から起動してもらう。

> **2026-10-02 更新：ZaBに洞窟と同じ独立LIO/VIO・RViz・6軸誤差表示を接続。**
> 起動は `ros2 launch multilink_uas_bringup zab.launch.py headless:=true`（Gazebo GUIなし、RVizあり）。
> ZaBはrosbagを記録しない。洞窟用dragon_compareのrosbagも既定OFF（明示的なrecord:=trueのみON）。
> CSV・誤差プロット・実行ログは保存。自動arming/飛行/洞窟経路追従は起動しない。
> 現在の状態はmanager statusで確認。停止はmanager stopまたは起動端末のCtrl+C。
> 以下の「ZaBは推定器を起動しない」は旧状態。

> **2026-10-02：rosbag破棄・ZaB導入完了。シミュレーションは停止済み。**
> 10月1日の比較試験2回分のrosbag（合計約57.15 GB）をユーザー指示で削除。CSV・誤差・グラフ・設定は保持。
> ZaB起動：`ros2 launch multilink_uas_bringup zab.launch.py`。DRAGONは地上・非arming、3カメラ＋LiDAR＋IMU。
> `headless:=true`で画面なし、`robot:=false`で環境だけ。arming・飛行・rosbag・推定器は起動しない。
> 資産：WS直下 `assets/zab/`。再構築：`python3 /root/uas_native/setup_zab.py`。詳細は `native/ZAB.md`。
> 現在のHarmonicでworld読込、GUI、4制御器active、3カメラ20 Hz、LiDAR10 Hz、IMU約1 kHzを確認。
> 既存洞窟用軌道はZaBには使わない。ZaBでの飛行・LIO/VIO評価は未実施。以降の旧「bag保存」は過去の状態。

> **2026-10-01 ユーザーの「終了していいよ」で飛行・計測を停止済み。自動再開しない。**
> manager status=null。最新run `native/logs/20261001-160857-363502/` のbagを正常に確定。
> 記録627.034 sim秒、45分割。最終結果は同ディレクトリ `session_summary.json`。
> 以下の「実行中」「終了まで継続」は過去の指示・記録であり、現在は終了している。

> **2026-10-01 16:09 再起動：現在の稼働ログは `native/logs/20261001-160857-363502/`。**
> 前回はRVizがSIGABRT（std::system_error: Invalid argument）で終了し、旧監視が全構成を停止した。
> 前回bagはmetadata.yamlまで確定済み。今回はRVizだけを再起動するwatchdogを導入し、GUI障害で飛行・記録を止めない。
> supervisor PID 363502 / launch PID 363472。開始地点から新しい試行として再離陸・軌道追従する。
> 「終了」まで継続というユーザー指示は有効。以下の旧PID・旧runは過去の記録。

> **2026-10-01 独立LIO/VIOの洞窟飛行・記録を実行中。ユーザーが「終了」と言うまで継続する指示。**
> 起動：`dragon_compare.launch.py headless:=true rviz:=true`。Gazebo画面なし、RVizに両軌跡と6軸誤差。
> 現在のログ：`native/logs/20261001-160414-360546/`。manager状態を確認してから操作し、二重起動しない。
> MIMOSAは外部odometry無効、ROVIOと独立。真値制御で離陸→始点へ移動→596 m経路を反復。
> 初期の約16 mを追従し、最大追従誤差0.063 m、記録・6軸評価を確認。全周完走はまだ確認していない。
> 終了指示時はmanager stopで今回の構成を停止し、bag確定・最終結果を確認する。自動停止しない。
> 追従安全条件を超えた場合はGazeboを一時停止し、`route_status.json` に理由が残る。
> 初期のFlightNav.headerは実時刻だったためsim約65秒でuse_sim_timeを有効化。指令比較はbag受信時刻またはroute_tracking.csvを使う。
> 現在のソースは次回起動からuse_sim_time=trueを既定とする。その他のセンサ/推定値の時刻は初めからsim時刻。

> 2026-10-01 洞窟走査軌道：`native/routes/cave_exploration/` に596.318 mの経路と26.750分の時間付き軌道を作成。
> 固定四角形DRAGON、FC基準の世界座標。三叉路・東北側通路・南側の折れ曲がりを通って戻る。
> メッシュ照合で中心から壁・床まで1.95 m以上、半径1 mの機体包絡に対して0.95 m以上の余裕。
> 離陸・着陸と追従飛行は未検証。生成方法・制約・俯瞰図は成果物のREADME参照。
> 次の研究段階は複数環境・固定センサ配置でLIOとVIOを別々に評価し、それぞれの退行条件を調べること。
> この評価ではROVIOをMIMOSAへ融合しない。現在のdragon_vio起動は融合するため、比較試験には別設定が必要。

> 2026-10-01 カメラ配置更新：3台の光学中心を各リンク基準 `(0.10,-0.045,0)` mへ移動。
> リンクと同じ高さの側面取り付け、共通yaw=-127.5度。以前の高さ0.16 mの配置は廃止。

> 2026-10-01 DRAGON VIO追加：`dragon_vio.launch.py` でlink1・link3・link4の外向き3カメラを
> ROVIOへ入力し、MIMOSAへ融合。link2のLiDARとFC IMUを維持。四角形を保って離陸・ホバリングする。
> カメラ・IMU間の校正は固定であり、変形中のVIOには未対応。起動方法・制約はREADMEを参照。
> 外部aerial_robot_modelのrotor_tf_publisher修正も必要（`native/dragon-rotor-tf.patch`）。

> 2026-09-30 VIO追加：UniPilotの `mode:=vio_slam` を実装。前方・左右の3カメラすべてを
> 1つのROVIOへ入力し、MIMOSAの外部odometryとして融合する。DRAGONの構成は変更していない。
> `ros2 launch multilink_uas_bringup unipilot.launch.py mode:=vio_slam headless:=false rviz:=true`。
> 3台の取付・校正は明示したシミュレーション用の仮定配置。実機個体の校正ではない。
> 左右185度生画像は保持し、ROVIO用には120度のpinhole画像へ整流。全3台を使用。
> ビルド・詳細・検証結果はREADMEとWORKLOGを参照。

> 2026-09-30 DRAGON追記：現在の対象はUniPilotデモからDRAGONの単一LiDAR＋FC IMUへ変更。
> 環境はUniPilotと同じUASの `darpa_cave_01.sdf` をそのまま使い、spawnも `(40, 5, 0.5)`。
> 起動は `ros2 launch multilink_uas_bringup dragon_slam.launch.py headless:=false rviz:=true`。
> 真値odometryで離陸・その場でホバリングし、MIMOSAはLiDAR＋IMUのみから地図を作る。
> 分散LiDAR・関節変形・水平飛行・RLは起動しない。aerial_robotは `/root/ros2/aerial_robot_base_ws` の現行ROS2実装を使用。
> 以後の実装記録は同じディレクトリの `WORKLOG.md` に追記する（AGENTS.mdに規則を記載）。
> 詳細・検証結果はWS直下READMEとWORKLOGを参照。以下は過去の経緯。

> 2026-09-30 飛行・SLAM追記：統合WSで `mode:=flight_slam` を実装・検証済み。
> LiDAR＋IMUのみをMIMOSAへ入力し、約1.5 m上昇→2 m四方の移動→着陸を確認した。
> 飛行制御はGazebo真値、SLAM推定による閉ループ制御・ループ閉じ込みは未実装。
> 採用結果は `native/logs/20260930-154020-97154/`（位置RMSE 0.01573 m）。
> 操作手順と制約は統合WS直下のREADMEを参照。以下の「MIMOSA未導入」は過去の記録。

> 2026-09-30 移設追記：`uas_native` の全ファイルは
> `/root/multilink_autonomy_ws/src/multilink_autonomy/multilink_uas_bringup/native/` へ移動した。
> `/root/uas_native` は互換symlink。現在の操作手順はワークスペース直下の `README.md` を参照。
> 対話Bashは `.bashrc` でROS2＋統合WSを自動読み込みする。
> Python仮想環境は端末では有効化せず、stackのRLプロセス内だけで有効化する。
> `ros2 launch multilink_uas_bringup unipilot.launch.py` が現在の起動入口。
> 以下に残る旧パス・手順・「自動sourceしない」等の記述は移設前の履歴。

# 研究・実装・実行環境の引き継ぎ

> 運用変更（2026-09-29）：新WSL Ubuntu-22.04の既定ログインを **root** とし、構築した環境一式を **`/root`（`~`）へ移動済み**。起動は `cd ~/uas_native` → `./run.sh stack`。ビルド済みファイルの旧絶対パスを保つ互換用symlinkだけを `/home/developer` に残している。参考コピー・過去のログの旧パスは構築時の記録。developerアカウントは削除していない。

更新: 2026-09-30。実装・環境の確認記録は2026-09-29時点。2026-09-30に今後の作業場所と統合ワークスペースの方針を追記。

**今後の主題は、相補的なセンサをDRAGONの複数リンクに分散し、関節運動で観測配置を変えるactive perceptionである。LiDARを全リンクに複製すること自体は、今後の主題にしない。** これまでの4 LiDAR実装は、可動センサの統合と比較評価のための既存資産として残す。

直近では、既存UniPilot / Unified Autonomy Stack（UAS）を基準に環境ごとの性能を調べる準備として、新しいWSL `Ubuntu-22.04` にネイティブ実行環境を構築した。**センサ・地図生成・プランナー・RL/CBFの接続確認まで完了。MIMOSAを使ったSLAM評価と自律飛行評価は未実施。aerial_robotは新環境へまだ導入していない。**

## 1. 研究テーマと、次に明らかにしたいこと

### 1.1 目的

DRAGONのようなロータ分散型マルチリンク機体に、UniPilotで使うような**異なる性質を持つセンサ群（complementary sensors）**を分散配置する。環境に応じてリンク姿勢を変え、センサの向き・位置・相対配置を調整し、状態推定・地図生成・タスク遂行を安定させられるかを調べる。

センサの具体的な種類、台数、取付リンクは未確定。LiDAR、カメラ、IMU、必要に応じてRadarなどが検討対象だが、全部を追加する決定はしていない。「複数モダリティ」と「同種LiDARの台数追加」は区別する。

マルチリンク機体を採用する本来の理由は、マニピュレーションや狭隘空間の潜り抜け能力である。SLAMのためだけに機体を複雑化する説明は弱い、というのが議論の出発点。これらの機体能力を、分散したセンシングでどう強化できるかを研究上の文脈とする。

### 1.2 ユーザー・教授との議論で整理した仮説

- 相補的なセンサを各リンクへ分散すると、搭載重量を分散でき、関節運動で配置を変えられる。ただし重量分散の実際の利益は、配線・追加機構・慣性・制御負荷も含めて未評価。
- 森林・トンネルなどで観測の拘束が方向によって弱い場合、配置変更によって不足する情報を取得できる可能性がある。
- Hessian等から弱い推定方向を把握し、それを補う観測が得られる関節運動を計画する、というactive perceptionを将来の方法論候補にする。**このプランナーはまだ実装していない。**
- gimbalは主に回転を変える。マルチリンクでは、制約付きながら並進を含むセンサ配置変更ができる。この追加自由度が、実際に必要な環境条件を示したい。マルチリンクでも任意の位置・姿勢を実現できるとは仮定しない。

検証したい中心命題は次の2つ。

1. 固定配置では推定が退化・不安定化するが、センサ配置を変えると改善する環境があるか。
2. その中に、gimbalによる回転だけでは不十分で、並進を含むマルチリンクの配置変更が有効な環境群があるか。

いずれも**未実証の仮説**。トンネル等の環境名だけで成立を主張しない。観測可能な幾何・テクスチャ・遮蔽・視野・照明等の条件、機体とセンサの可動範囲を記述して議論する。単にセンサを回しただけで、環境に存在しない拘束が生まれるわけではない。

### 1.3 これまで退けた、または不十分と考えた説明

- 「LiDARを増やせばSLAMが安定する」だけでは不十分。基部の単一LiDARで十分な環境も多く、追加質量や可動外部パラメータの誤差を正当化できない。
- 「先頭リンクで先に地図を作り、後続リンクが通る」だけでは分散センシングの必要性が薄い。後続がほぼ同じ場所を通るなら、既存地図で足りる可能性を検討する。
- 特殊な曲がり角・開口部を1つ作って有効性を示すだけでは一般性が弱い。
- 機体全体の並進・回転、固定配置の工夫、gimbalで同じ利益が得られないかを必ず比較する。
- 画像による滑り・変形観測、手先カメラ等を採用するだけでは、既存の地上マニピュレーションとの方法論上の違いが弱い。
- エンコーダより高精度に関節角をSLAMで推定することを、今回の主要な利点には置かない。関節角は既存エンコーダを使う。

NTNU ARLのactive perception、UA-MPC、AEOS、RoboPanoptes等を会話で検討したが、今回の問題設定をそのまま解決する先行例と結論したわけではない。本資料では、過去の文献回答の数値・主張を再検証なしに転記しない。論文で引用する段階では一次資料を確認する。

### 1.4 これから行うこと

ユーザーが決めた順序は、**様々な環境を用意 → 既存UniPilot / UASで基準性能を測る → DRAGONへ相補センサを分散して比較**。

そのための再開案は以下。実験条件や手法まで確定済みという意味ではない。

1. 新WSLの既存構成を確認し、評価対象の推定器・センサ入出力を確定する。UAS標準シミュレーションの真値姿勢を、SLAMの推定結果と誤認しない。
2. 必要なIMU・カメラ等をシミュレータで有効化し、時刻、座標系、ノイズ、外部パラメータを確認する。MIMOSA等の推定器を実際に接続する。
3. 構造の豊かな部屋、長い廊下・トンネル、森林等を候補に、固定配置の基準性能を収集する。現在準備できているUASワールドはSubT洞窟であり、多環境ベンチマークの整備は未完了。
4. 観測の弱くなる条件を見つけてから、同じセンサ構成で「固定」「gimbalによる回転」「関節による回転＋並進」を比較する。必要なら機体全体を動かす条件も追加する。
5. 基部軌道を揃える比較と、タスクを自由に遂行する比較を分ける。センサ台数・点数・処理量増加の利益と、配置変更そのものの利益を分けて評価する。
6. 有効な環境群が確認できた段階で、弱い推定方向と候補動作による観測変化を結ぶ関節計画へ進む。Hessian改善だけでなく実際の推定誤差・失敗率・タスク達成・計算負荷で確かめる。

現時点では**aerial_robotのROS2移植・新WSLへの導入は後回し**。この資料を読んだだけで、その作業や飛行実験を開始する必要はない。

### 1.5 今後の作業場所とワークスペース構成（2026-09-30追記）

**新WSL `Ubuntu-22.04` のroot環境を今後の作業場所とし、UASとaerial_robotを接続する専用ワークスペースを設ける方針。** UAS本体とaerial_robot本体はそれぞれのワークスペースで管理し、日常の研究開発は統合用の `~/multilink_autonomy_ws` を中心に行う。

以下は目標とする配置案。`aerial_robot_ws`、`multilink_autonomy_ws`およびそのパッケージは、この資料への追記では作成していない。パッケージ名・分割は必要に応じて調整し、最初から全てを作らなくてよい。

```text
/root/
├── unified_autonomy_stack/       # 既存：UAS本体と内部のworkspaces
├── aerial_robot_ws/              # 今後：aerial_robot本体と必要な依存
├── multilink_autonomy_ws/           # 今後：研究・統合の中心
│   └── src/
│       └── multilink_autonomy/
│           ├── multilink_uas_bringup/   # 全体起動、接続設定、実験条件の切替
│           ├── multilink_sensors/       # センサ配置、可動外部パラメータ処理
│           ├── multilink_simulation/    # Harmonic向け機体モデル、環境
│           ├── multilink_perception/    # 推定器接続、active perception
│           └── multilink_evaluation/    # 記録、比較、作図
├── ros1_noetic/                  # 既存：GBPlanner等が使うROS1依存基盤
└── uas_native/                   # 既存：環境構築、起動、接続検証の補助ファイル
```

役割分担と実装時の指示：

- 統合launch、研究用のセンサ設定・環境・比較評価コードは統合ワークスペースに置く。UASやaerial_robotの内部変更が必要な場合だけ、それぞれのリポジトリを変更し、差分を管理する。本体のソースを統合側へ丸ごと複製しない。
- `uas_native`はUAS公式リポジトリでも、今後の研究コードを全て置く場所でもない。現在の動作確認済み入口を維持し、統合用launchが整った段階で必要な設定を整理して引き継ぐ。
- ワークスペースを追加するだけではROS1／ROS2やGazeboの互換性は解決しない。aerial_robotをどのROS構成で動かし、Harmonicの物理・センサ・制御へどう接続するかを、導入前に確認する。現時点で移植済み・対応済みとは扱わない。
- ROS1とROS2のパッケージを無条件に同じワークスペースへ混ぜたり、両方のsetupを同じ端末へ重ねたりしない。両ROSが必要なら、ビルドと実行環境を分け、bridgeするtopic・型・時刻・TFを明示する。統合側にも両ROSの独自パッケージが必要になれば、ROSバージョンごとにワークスペースを分割する。
- 旧 `Ubuntu-20.04` の `/root/ros_ws` と `/root/mimosa_ws` は、過去実装・実験の参照と再現用として保持する。新WSLの同名パスとは別物であり、そのままHarmonicへ持ち込めるとは仮定しない。

進行順序は、まず新環境で **UniPilot＋実際の推定器** を動かし、記録・評価の基盤を作る。その後aerial_robot／DRAGONを接続する。最終的には、統合ワークスペースから「UniPilot基準構成」「DRAGON固定センサ配置」「DRAGON可動センサ配置」を共通の環境・評価条件で切り替えて起動できる構成を目指す。今回の依頼はこの方針の記録であり、ワークスペース作成・移植・飛行の開始指示ではない。

## 2. 2つの環境の位置づけ

パスはディストリビューションごとに別。新WSLの `/root` に旧WSLのファイルがあるわけではない。

| 項目 | 旧環境：DRAGON / MIMOSA | 新環境：UniPilot / UAS |
|---|---|---|
| WSL名 | `Ubuntu-20.04`（Windows側の既定のまま） | `Ubuntu-22.04` |
| ユーザー | `root` | `root`（構築時はdeveloper） |
| ROS / Gazebo | ROS1 Noetic / Gazebo Classic 11.15.1 | ROS2 Humble + ROS1 Noetic / Gazebo Harmonic、NTNU gz-sim 8.9.0 |
| ソース | `/root/ros_ws`、`/root/mimosa_ws` | `/root/unified_autonomy_stack` |
| 起動・管理 | `dragon_mimosa`パッケージ | `/root/uas_native` |
| SLAM | MIMOSAのLiDAR＋IMUを実行・比較済み | 標準構成ではMIMOSA未起動、真値odometry使用 |
| 自動飛行 | 通常launchではしない。明示的なdemo実行は飛行する | 検証ランチャーは制御出力を機体から切断 |

両環境とも同じWindowsホストのRTX 4070 Laptop GPU / WSLgを使用。GUIを出さない場合でもGPUセンサの描画コンテキストは必要。

## 3. DRAGONでMIMOSAを使うために実装したこと

以下の相対パスは、原則 `/root/mimosa_ws/src/mimosa/dragon_mimosa` 基準。読解用ソースのコピーは [handoff_reference/legacy_source](handoff_reference/legacy_source) にある。

### 3.1 bringup・単一LiDAR連携

- `dragon_mimosa`パッケージを作成。Dragon v1.5、Livox MID360相当点群、FC IMU、MIMOSA設定、Gazeboワールドを統合した。
- `fc_imu_bridge.py`でFCメッセージを `sensor_msgs/Imu` へ変換。`livox_custom_bridge.py`でLivox CustomMsgの点ごとの時刻を保ったPointCloud2へ変換する。実機接続は未検証。
- `config/mimosa.yaml`はLiDAR geometric / IMUを有効化。Radar、外部odometry、Ouster向けphotometricは無効。姿勢の真値をMIMOSAへ渡さない。
- `mapping_room.world`は16×12 mの壁・柱・箱・仕切り・天井梁を持つ室内。飛行中も鉛直方向の拘束が得られるように梁を設けた。**退化を起こしやすい一般環境の代表という位置づけではない。**
- Dragonの `bringup.launch` にモデル選択、xacroオプション、Gazebo GUI / RVizの独立指定を追加した。
- Dragon側RVizにマップ、推定軌跡、FC座標、統合／個別点群をまとめた。MIMOSA用RVizを別に開かない。
- `align_visualization.py`が最初の同期姿勢だけで表示座標を合わせる。以後の推定誤差を追従補正しない。MIMOSAの地図Publisherはラッチ化済み。
- 重複arming入力で状態がSTARTへ戻り、FC指令が止まる不具合を `flight_navigation.h` で修正。ARM_OFF以外のstartを無視し、遅延したarming応答が飛行状態を巻き戻さないようにした。通信failsafeそのものは維持。実キーボード入力で確認済み。

### 3.2 過去の4 LiDAR＋4 IMU分散構成

**各センサを独立したファクターとして追加した実装ではない。既存の因子の残差・重みを多センサ用に変更したものでもない。** 前処理で「仮想LiDAR 1台＋仮想IMU 1台」にまとめ、既存MIMOSAへ入力する。

```text
各linkのLiDAR ─┐
URDF＋関節角 ─┴─ 点ごとの時刻でFC座標へ変換 ─ 統合点群 ─┐
各linkのIMU ──── 関節運動・レバーアームを補正 ─ 仮想IMU ─┤→ MIMOSA 1台
                                                          └→ 地図・FC姿勢
```

| 内容 | 実装・仕様 |
|---|---|
| センサ配置 | link1〜4に同一相対配置。LiDAR光学原点 `[0.132, 0, 0.077] m`、IMU原点 `[0.3245, -0.001, 0.028] m`、取付回転は単位回転 |
| 基準body | link2上のFC原点。MIMOSAでは `mimosa_fc` |
| 生LiDAR | `/dragon/linkN/livox/lidar`、各10 Hz |
| 生IMU | `/dragon/linkN/imu`。link2はSpinalと同じGazebo IMUの生値を分岐。他リンクには独立IMU追加 |
| 関節 | `/dragon/joint_states`。`history.py`で時刻履歴、`kinematics.py`でFKと微分項 |
| 統合 | `scripts/multilink_frontend.py`、`config/multilink.yaml` |
| 出力 | `/dragon/multilink/points`（10 Hz）、`/dragon/multilink/imu`（100 Hz）、`/dragon/multilink/status` |

LiDARは各点の取得時刻の関節角を補間してFC座標へ移す。スキャン間の時刻差を点フィールド `t` [ns] に保持し、機体全体の運動のdeskewはMIMOSAに任せる。入力 `line` 上位2ビットに元リンク番号を保持する。TFやGazebo真値からセンサの相対姿勢を与える構成ではない。

IMUは、相対姿勢による回転、関節由来の相対角速度、レバーアームの接線・遠心加速度、関節運動に伴う相対速度・加速度とコリオリ項を補正した後に重み付き統合する。単純平均ではない。各IMUの姿勢真値は使わず、接地時に値を固定するFCの前処理も避ける。

必要な時刻を挟むIMU・エンコーダ標本がない場合、最新値で代用しない。4台のLiDARは50 ms以内で組み合わせ、対応する統合IMUを先に配信する。さらにMIMOSAのLiDAR managerにIMU到着待ちを追加し、ROSのトピック間配送順序の違いによるバッファ範囲外参照を防いだ。Dragon設定では実時間0.5秒でtimeoutし、間に合わないスキャンはグラフ登録前にスキップする。本体既定は0、単一スレッドのbag処理では待機を有効化しない。

個々のIMUのバイアス・時刻ずれ・取付誤差をオンライン推定する仕組みはない。統合後の共通IMUバイアスをMIMOSAが推定する。実機では別途校正が必要。ループ閉じ込みやMIMOSAによる飛行制御は追加していない。

**既定と比較Bの違いに注意：** `gazebo.launch` は4 LiDAR＋4 IMU統合が既定。一方 `demo.launch` と後述A/B比較は、4 LiDARでも **link2の生FC IMU 1系統**をMIMOSAへ渡す。LiDAR台数の差を比較するためで、多IMUの利益を示す実験ではない。

### 3.3 GPU LiDARと、変形後バイアスの修正

`src/gazebo_livox.cpp`をCPU `ray` / GPU `gpu_ray`両対応にした。既定はGPU。出力は360×32レイ、水平360°・垂直−7〜52°、10 Hz。GPU内部1437×125から縦横4サンプルごとに抽出する。ヘッダは処理完了時刻でなく `LastMeasurementTime()`。

このセンサはMID360の視野の近似。実機の非反復走査・強度特性を再現せず、Gazeboの同一スキャン内の相対時刻 `t` は0。GPUはvisual、CPUはcollision形状を観測するので、両者が一致するとは限らない。

初期計測では、GUI＋RVizありのRTFがCPU 0.28 → GPU 1.00に改善。ただしこれは当該ワールド・マシンでの結果であり、将来の性能保証ではない。

その後、4 LiDARの変形後に位置誤差が増える原因を調べ、**Gazebo Classic 11のGPU距離画像と走査角の1内部サンプルずれ**を特定した。関節振動が原因と結論したわけではない。プラグイン内で再投影キャンバスを半画素移動して画素中心へ合わせ、既に修正済みの実装には二重適用しない。システムのGazebo本体は書き換えていない。

無ノイズの箱で面距離RMSEは、水平32.13 → 1.39 mm、傾斜28.35 → 1.48 mm。CPU/GPU・2姿勢等の6条件で回帰テスト済み。`test/test_gpu_lidar.py`を参照。初期の [PERFORMANCE.md](handoff_reference/legacy_docs/PERFORMANCE.md) の幾何誤差値は、この修正前の記録として読む。

### 3.4 基部を保持する変形と地図更新

- 元の `rosrun dragon transformation_demo.py` は関節と基部姿勢を指令するため、基部の位置・姿勢が一定という比較条件にならなかった。
- `dragon_mimosa/scripts/transformation_demo.py`で、元デモの姿勢指令を別topicへ逃がし、FCの位置・姿勢を100 Hzで保持する補助制御を追加した。既存制御がCoG目標を使うため、FC誤差からCoG目標を更新する。Gazebo内の姿勢を強制固定する方法ではない。
- 基部変位は完全には0にならない。修正後飛行の変形中は位置RMS 5.71 mm / 最大14.13 mm、姿勢RMS 0.845° / 最大2.02°だった。
- 基部移動だけを条件にした地図キーフレームでは、関節だけ動かしても地図が更新されなかった。MIMOSAの `geometric.*` / `geometric_config.*` に `map_keyframe_max_interval` を追加した。
- Dragon設定は並進0.15 m、回転5°に加え、センサ時刻で0.5秒経過した最初のスキャンを追加。本体既定0は従来動作。A/B両条件に同じ設定を使う。関節や情報利得に基づく能動的なキーフレーム選択ではない。

### 3.5 旧環境の起動手順

既にビルド済み。初回再構築時のunderlayは `/root/ros_ws/devel_catkin`。`devel` や `devel_isolated` と取り違えない。

```bash
# 旧WSL Ubuntu-20.04内
cd /root/mimosa_ws
source /root/ros_ws/devel_catkin/setup.bash
catkin config --extend /root/ros_ws/devel_catkin
catkin build mimosa dragon_mimosa -j4 -p2
source /root/mimosa_ws/devel/setup.bash

# Dragon、Gazebo、MIMOSA、Dragon側RVizをまとめて起動。armingしない。
roslaunch dragon_mimosa gazebo.launch
```

`multilink:=false`で従来の単一LiDAR構成、`gpu_lidar:=false`でCPU、`gazebo_gui:=false rviz:=false`で画面を省く。**`multilink:=false`の通常launchはFCメッセージ変換経由のIMUであり、保存済みA/B比較の「同じ生FC IMU＋同じ点群から切り出すA」と全く同じ条件ではない。**

比較B相当の飛行デモは次の2操作。bringupを先に別途起動しない。ROS masterも `roslaunch` が必要に応じて起動する。

```bash
# 端末1：このlaunchだけでは飛ばない
source /root/mimosa_ws/devel/setup.bash
roslaunch dragon_mimosa demo.launch

# 端末2：明示的な飛行実験時だけ実行。ここから自動arming・離陸する
source /root/mimosa_ws/devel/setup.bash
rosrun dragon_mimosa transformation_demo.py
```

デモは離陸 → 2 mホバー・基部保持 → 15秒待機 → 変形 → 15秒待機 → 復元 → 着陸。bag、イベント、パラメータ、URDF、ログは毎回 `~/.ros/dragon_mimosa/transformation_*/` に保存。旧 `mapping_flight.py` やartifactsの飛行スクリプトも指令を送るので、起動確認の代わりには実行しない。

## 4. 単一LiDARと4 LiDARの比較で、何が分かったか

採用する最新結果は、旧WSLの `/root/mimosa_ws/artifacts/lidar_bias_fix_20260921/new_flight/`。原因と修正は [RESULTS.md](handoff_reference/experiments/lidar_bias_fix_20260921/RESULTS.md)、図一覧は [PLOTS.md](handoff_reference/experiments/lidar_bias_fix_20260921/PLOTS.md)。

Aはlink2の1台、Bは4台。同じ修正後のflight bagから作り、同じ生FC IMUを使う。A点群がBのlink2成分とbyte単位で一致することを検証済み。**同じ4台搭載機体の記録から入力を変えた推定器比較であり、1台搭載機と4台搭載機の質量・飛行性能比較ではない。**

| 最新修正後の指標 | A：1 LiDAR | B：4 LiDAR |
|---|---:|---:|
| 変形前15秒：位置RMSE | 10.489 mm | 8.774 mm |
| 変形中：位置RMSE | 10.416 mm | 8.902 mm |
| 変形後15秒：位置RMSE | 10.341 mm | 8.797 mm |
| 地図completeness、変形直前→直後、閾値10 cm | 58.27→58.66% | 66.59→69.13% |
| 地図completeness、評価終了、閾値10 cm | 58.82% | 69.45% |
| 変形中の地図更新 | 33回 | 33回 |
| 全飛行評価区間のLiDAR callback中央値 | 4.434 ms | 8.626 ms |

この部屋では両方とも安定し、Bは地図の被覆を増やした。位置精度の差は小さく、処理量は増えた。**これだけで分散LiDARの一般的な有効性も、無効性も示したとは言えない。** 修正前Bの変形後RMSE 17.543 mmはシミュレータの点群バイアスを含むため、分散センシングの原理的欠点の証拠に使わない。

### 4.1 評価の定義と取り違えやすい点

- `/dragon/ground_truth`はすでにFC原点。link2→FCオフセットを二重に足さない。古い評価にはこの誤りがあり、過去に報告した約9.8 cm等は採用しない。
- 推定時刻にGT位置を線形補間、姿勢をSLERP。最初の同期時刻だけでSE(3)を合わせる。全軌跡フィット、スケール補正、変形開始時の再合わせはしない。位置誤差は3次元距離、姿勢誤差は相対回転角。
- 飛行制御は既存のGazebo真値モード。GTは制御・評価・表示に使用し、MIMOSA推定入力にはしない。
- 地図完成率は、固定GT表面パッチ中心 `s_i`、面積 `a_i`、時刻tの推定地図 `M_t` に対し、`C_tau(t) = 100 * sum_i a_i * 1[distance(s_i, M_t) <= tau] / sum_i a_i`。
- 旧室内ではROI `[-7.875,-5.875,0]`〜`[7.875,5.875,5] m`。箱の表面を約10 cmパッチに分け、他物体内部の面を除外。計72,014パッチ・720.14 m²。観測されない遮蔽面も固定分母に含む。「現時点で見える面の何%」ではない。
- 地図は軌跡と同じ初期SE(3)で整列し、共通5 cm voxelで間引く。閾値は10 cmと5 cm。各地図にICPをかけない。地図更新間は直前の値を保持し、離陸前の地図も含むのでhover開始時は0%ではない。
- Completenessだけでは誤った余分な点を罰しない。推定地図点→GT表面のaccuracy、表面距離、F-scoreも保存。GT姿勢で置いた生点群の累積coverageとは別指標。
- 「Weakest translation localizability」は `min(localizability_trans_final)`。当該ソースでは幾何LiDAR因子の `JᵀJ` の並進3×3ブロックの最小固有値の**平方根**に相当する（`mimosa/include/mimosa/utils.hpp`、`lidar/geometric_factor.hpp`）。全センサ・IMUを含む推定器全体の最小固有値や共分散ではない。点数や重みの影響があるため、値の増大だけで退化解消と結論しない。
- 誤差0.20 mまたは5°超が0.5秒以上、出力欠落0.3秒超等の判定は、この比較用の実装上の定義。公認ベンチマークの共通基準ではない。

作図のユーザー指定：全時系列図はhover開始以降だけ表示。既存図の横軸原点はtakeoffのままで、hoverを0秒へ振り直したわけではない。変形区間を薄く着色、軸数字を大きくし、RMSEの水平線・「Interval RMSE」は不要。completenessは縦軸を拡大し、上下の省略記号と0%・100%の端点を表示する。

### 4.2 原本の実験データ

全て旧WSLの `/root/mimosa_ws/artifacts/` 以下。次は用途別の索引。

| ディレクトリ | 内容・扱い |
|---|---|
| `dragon_mimosa_arming/` | 重複arming・意図的通信断のfailsafe確認 |
| `dragon_multilink/`、`dragon_gpu/` | 初期統合・GPU速度検証。旧GT原点評価とGPUバイアス修正前の結果に注意 |
| `dragon_transform/` | 初期の飛行・変形・bag比較。固定基部の最新A/Bとは別 |
| `2026-09-17-19-47-30/` | ユーザー提供 `/root/2026-09-17-19-47-30.bag` の解析 |
| `lidar_ab_20260921/` | 最初のA/B。基部も動く |
| `lidar_ab_fixed_base_20260921/` | FC保持。変形中に地図が更新されなかった条件 |
| `lidar_ab_fixed_base_refresh_20260921/` | 時間更新を追加。GPUバイアス修正前 |
| `lidar_bias_fix_20260921/same_bag_direction_correction/` | 元bagの方向だけ補正した原因切り分け |
| `lidar_bias_fix_20260921/new_flight/` | **修正後の新規飛行・最新A/B** |

最新ディレクトリに `flight.bag`、`inputs.bag`、`A/results.bag`、`B/results.bag`、JSON/CSV、`prepare_inputs.py`、`replay.py`、`analyze.py`、各plot/validateスクリプトがある。研究用スクリプトには絶対パスや既存ROS masterの前提が残る。再解析は旧環境で行うか、パス・依存を確認して移植する。古い実験には別ディレクトリへのsymlinkもある。軽量な引き継ぎコピーはbagを含まず、完全な再実行用データセットではない。

## 5. Git状態：コミットだけ取得しても最新状態にはならない

2026-09-29確認時点。

| リポジトリ | ブランチ / HEAD | remote |
|---|---|---|
| `/root/mimosa_ws/src/mimosa` | `develop/multi_link` / `0c21851` | `junichiro`: sugihara-16/mimosa、`origin`: ntnu-arl/mimosa |
| `/root/ros_ws/src/aerial_robot` | `develop/mimosa` / `f284e65d` | `origin`: sugihara-16/aerial_robot、`upstream`: jsk-ros-pkg/jsk_aerial_robot |

9月18日に整理したコミット：MIMOSA側 `a6c9222`（map latch）、`280d41c`（IMU待機）、`cafd846`（Dragon連携）、`ac950ea`（全リンク統合）、`0c21851`（文書）。Dragon側 `e378a65e`（重複arming）、`a6dbd785`（生IMU）、`55cb2682`（launch引数）、`f284e65d`（統合RViz）。

その後の変更は現在未コミットで残る。

- MIMOSA側：時間キーフレーム、GPU画素対応修正、IMU入力選択、demo launch/script、GPU回帰テスト、関連依存・ドキュメント。
- Dragon側：離陸高度2 m、制御ゲイン、URDFの色・Gazebo material等。既存のローカル変更として保全する。これらのゲインを実機用の確定値とみなさない。
- `aerial_robot_3rdparty`には別件のMuJoCo変更が残る。本件の整理で巻き戻さない。
- 本引き継ぎ文書・参考コピーも新規未追跡ファイル。**今回の資料作成ではcommit / push / checkout / resetを行っていない。**

詳細なHEAD・status・差分は [legacy_git](handoff_reference/legacy_git) に保存。コミット済み変更のpatchと、HEAD以降の作業ツリーpatchを分けている。未追跡のdemo等は [legacy_source/dragon_mimosa](handoff_reference/legacy_source/dragon_mimosa) にもコピーした。patchを既存作業ツリーへ重ねて適用しない。

`dragon_mimosa`は `spinal`、`dragon`、`aerial_robot_base`、`aerial_robot_msgs`等に依存するが、jsk_aerial_robotリポジトリの自動取得・revision固定は未整備。MIMOSA本体にはDragon依存を追加していない。再構築にはDragon側の変更とunderlayが必要。

## 6. 新WSLのUAS環境

### 6.1 接続と通常操作

WindowsのPowerShell等から：

```powershell
wsl -d Ubuntu-22.04 --cd /root
```

新WSL内：

```bash
cd /root/uas_native
./run.sh status          # 引き継ぎ資料作成時は null（停止中）
./run.sh stack           # 観測・接続検証構成を起動。機体へ制御出力しない
./run.sh check-stack     # ROS2センサ、ROS1地図・TFを確認
./run.sh stop            # このランチャーが起動したプロセス群を停止
```

`./run.sh sim`はシミュレータ単体。`./run.sh check-inference`は出力切断を確認してからRLへ目標を与え、RL/CBFの数値を検査する。検査後はstop/stackで残った目標を消す。`./run.sh rviz`は点群・姿勢用のROS2 RViz設定。GUIの操作・見え方はこの構築作業では未検証で、ROS1 VoxbloxメッシュをそのRViz2で表示できたという主張もしない。

`manager.py`が必要なROS1 master、bridge、sim、GBPlanner、CBF、RLを管理するので、手動で別のroscoreを起動する必要はない。内部ではROS1 GBPlanner用のmasterが必要。

### 6.2 実行構成と検証範囲

```text
Gazebo Harmonic（洞窟・UniPilot）
  → ros_gz_bridge → ROS2点群・真値odometry・TF
       ├→ RL → CBF → /validation/cbf_safe_cmd（機体入力とは切断）
       ├→ CBF用点群選択 → CBF
       └→ ros1_bridge → ROS1 GBPlanner / Voxblox
```

**UASは自律化スタックであり、MIMOSAそのものではない。** この固定revisionの標準シミュレーションはGazebo真値 `/rmf_unipilot/odom` を使う。公式のMIMOSA用manifestは別に存在し、ROS1側の構成。ROS2で動くUASを用意したことから「ROS2版MIMOSAやカメラ直接融合が動いた」と推論しない。

ローカルの公式 `docs/slam.md` ではカメラはexperimental、外部VIOのodometry経由の接続はWIPと記載されている。LiDAR photometricはカメラ画像を直接融合する機能と同義ではない。今後のセンサ・branch選定では、必要機能をソースと実入力で確認する。

- シーンは `darpa_cave_01`。SubTメッシュはGit LFSの実体を取得済み。
- 標準UniPilotモデルはLiDARのみ有効。IMU・RGB/depthカメラは無効で、bridge宣言だけあってもメッセージは出ない。
- LiDARは500×48、24,000点、10 Hz、最大100 m、距離ノイズ標準偏差0.03 mの汎用GPU rayモデル。出力は `x,y,z,intensity,ring`、点ごとの時刻なし。実機センサの完全再現ではない。
- 既定CBF出力は `/validation/cbf_safe_cmd` へremapし、`/rmf_unipilot/cmd/acc`へ接続していない。**自動離陸させないための設定を維持する。**
- 2026-09-29、30秒確認で生点群10 Hz、odom100 Hz、RTF約0.995、機体変位0 m。ROS1側は点群300件、odom2,991件、メッシュ31件、world→LiDAR TFを確認。
- 実入力による15秒推論でRL147件（約9.8 Hz）、CBF750件（約50 Hz）、全出力有限値、機体変位0 m。
- 以上はセンサ・推論・地図生成の接続確認。SLAM精度、退化回避、自律飛行、探索の成功率は未評価。

### 6.3 パス・バージョン・環境分離

| 内容 | 新WSL内パス／設定 |
|---|---|
| 起動設定・手順 | `/root/uas_native/README.md`、`run.sh`、`manager.py` |
| UAS | `/root/unified_autonomy_stack`、main `cb78bf7f57d074fc0999dd23bb1cfa1b1754d44d` |
| 全ソースrevision | `/root/uas_native/source-commits.json` |
| Gazebo fork | `/root/gz-sim`、`1897a13938a3670e575b0f0ae45e0b5c02c427ed`、`/usr`へ導入 |
| ROS2 | Humble、`ws_sim` / `ws_cbf` / `ws_rl` |
| ROS1基盤 | `/root/ros1_noetic/install_isolated`、92パッケージをJammy上でソースビルド |
| ROS1 planner | UAS内 `workspaces/ws_gbplanner`、必要28パッケージをビルド |
| Python | `/root/.venvs/uas`、Python3.10、PyTorch2.1.0+cu118、NumPy1.23.5 |
| 通信 | `ROS_DOMAIN_ID=74`、`ROS_MASTER_URI=http://127.0.0.1:11374`、`GZ_PARTITION=uas_native` |
| 検証ログ | `/root/uas_native/logs/build-and-checks/final-{topics,planner,inference}.json`、`logs/20260929-171728/` |

ROSをshell初期化へ自動sourceしていない。必要なら**別々のクリーンな端末**で `source /root/uas_native/env.sh`（ROS2＋venv）または `source /root/uas_native/env_ros1.sh`（ROS1）を実行。同一shellで混ぜない。bridgeは意図的に両方を扱う専用構成。

bridgeは先行Docker検証でJammy上にビルドした再配置可能なアーカイブを利用。新WSLの他の主要workspaceはネイティブに再ビルド済み。実行時にDockerは不要。

### rootへのディレクトリ移動

`uas_native`、`unified_autonomy_stack`、`ros1_noetic`、`gz-sim`、`.venvs`と関連設定の実体は `/root` にある。起動・ビルドスクリプトとvenvの入口も新パスへ更新した。旧 `/home/developer/...` は互換用symlinkであり、二重にデータを保持しているわけではない。既存のbuild/installメタデータは旧パスを含むので、このリンクを削除しない。今後クリーン再ビルドする場合は新パスを使う。移動記録は `~/uas_native/relocation.json`。旧developerのROSログは `~/.ros/developer_history/` に保全した。

移動後の再検証：LiDAR 10 Hz、ROS1地図生成・TF、RL/CBF推論を確認。定常観測時RTF約0.988、機体変位0 m。検証後は停止済み。ログは `~/uas_native/logs/root-relocation/`。新規ROS2購読者がDDSのendpointを先に発見しノード名をまだ取得していない場合、推論検査が誤って拒否する問題も修正した。不明な名前は最大5秒待ち、未解決または許可外のpublisherなら目標を送らず拒否する。

### 6.4 構築中に必要だった修正・運用条件

- Jammy上のROS1：`rosconsole-log4cxx.patch`、`pcl-ros-cxx17.patch`。GBPlanner側もC++17、`CMAKE_CXX_EXTENSIONS=OFF`。古い手動の `-std=c++11` より優先させる必要があった。
- `grid-map-onetbb.patch`：旧 `task_scheduler_init` を `task_arena`へ移し、`TBB::tbb`へリンク。NaNを含む平面でarea/raster×スレッドautomatic/1/2の6条件、逐次・並列差0を確認。
- `eigen_checks`のCMakeのため、plannerの `CATKIN_ENABLE_TESTING=ON`を維持。`bin/rosversion`は既存rospkgモジュールを呼ぶ補助CLI。
- FastDDSのUDP送受信バッファを4 MiBへ設定。点群配送欠落でCBFがtimeoutした問題を解消。`fastdds.xml`を外さない。
- NVIDIA CUDA認識とOpenGL描画を別々に確認済み。実GazeboのOgre2 rendererは `D3D12 (NVIDIA GeForce RTX 4070 Laptop GPU)`。Linux NVIDIAカーネルドライバを追加しない。
- WSLgを利用するserver-only起動。`--headless-rendering`によるEGL構成ではない。`DISPLAY=:0`、`GALLIUM_DRIVER=d3d12`等はenv設定を参照。
- `/etc/wsl.conf`は `systemd=false`。複数WSL間のWindows実行ファイルハンドラへの干渉対策として `systemd-binfmt.service` / `binfmt-support.service`をmask済み。UASはsystemdに依存しない。
- `developer`は開発用のsudo NOPASSWD。既定ログインはrootへ変更済み。既定WSLは旧Ubuntu-20.04のまま。新WSLだけの再起動後にユーザー・CUDA/OpenGL・ROS1解決を確認済み。
- Gazebo forkは `/usr`へ入るので、aptでGazeboを更新したらforkの再ビルド要否を確認する。

詳細と再構築スクリプトは [native_README.md](handoff_reference/uas/native_README.md)。旧Docker検証は `/root/uas_sim_validation`、そのソースは旧WSLの `/root/unified_autonomy_stack`。新ネイティブ環境と取り違えない。旧Docker構成はdomain73 / master11373 / partition `uas_validation`で、現行ネイティブとは別。

## 7. 再開時に守ること・資料の使い方

1. 通常の起動確認でarming・離陸しない。飛行は明示的に依頼された実験・操作で行う。旧デモの存在を再実行の指示と解釈しない。
2. 旧ROS1/Gazebo Classicと新Humble/Harmonicは別環境。旧ワークスペースを上書きしてROS2へ移行しない。新環境へのaerial_robot導入はまだ依頼されていない。
3. 最新の未コミット変更を保全する。単にremoteをcloneしても最新GPU修正・demo・地図更新は揃わない。
4. 評価ではGT原点、初期alignment、基部残留運動、地図更新条件、センサ生成誤差を確認する。新旧の不具合修正前後の結果を混ぜない。
5. 有効性を証明する結論を先に決めず、改善する条件・改善しない条件を記録する。旧室内のA/Bは、新テーマの一般性の証明ではない。
6. 起動・停止はプロジェクトのランチャーを使う。全WSLを落とす `wsl --shutdown` や、無差別なROS/Gazeboプロセスkillは避ける。
7. 今回は引き継ぎ資料だけを作成。既存WORKLOGへの追記、実装変更、再飛行は行っていない。

この文書は旧WSLの `dragon_mimosa/for_codex/HANDOFF.md` と、新WSLの `/root/uas_native/HANDOFF.md` に同じ内容を置く。隣の `handoff_reference/` に旧実装の主要ソース、Git差分、最新比較結果、環境資料をコピーした。これは参照スナップショットで、以後の修正は本来のソースへ行う。

コピーの元パスとSHA-256は `handoff_reference/manifest.json`。bag、ビルド生成物、Git履歴全体は同梱していない。参考資料内にコピー範囲外への相対リンクがある場合、記載した旧WSLの原本を参照する。旧環境はWindowsから `wsl -d Ubuntu-20.04 -u root --cd /root` で開ける。

新しいチャットへの開始文例：

> `/root/uas_native/HANDOFF.md` と同ディレクトリの `README.md` を読んで、この研究と環境を引き継いでください。主題はcomplementary sensorsのmulti-linkへの分散配置とactive perceptionです。旧4 LiDAR実装は参考資産です。まず現状を把握し、私の次の指示に従ってください。自動離陸やaerial_robotの導入はまだ行わないでください。

## 2026-10-04 ROVIO校正・画像OFF比較の最新結果
- 前回の2飛行・単眼link3でROVIOの相対姿勢誤差を測定し、別飛行の実測行列でMIMOSAの融合を校正。7条件×2飛行完了。
- 結果・六軸プロット: `/root/uas_native/logs/zab-offline/livo_calibrated/plots/index.html`。詳細は`resources/LIVO_CALIBRATED_COMPARISON_20261004.md`。
- 校正MIMOSA+DRPMは姿勢RMSE0.159°/0.225°に安定したが、位置4.40m/4.45mで、同じ校正のDRPMなし(0.51m/4.26m)より改善しない。現DRPM移植は原論文の全更新則の再現ではない。
- FAST-LIVO2画像ありは位置0.035m/0.373m。画像なし標準LIOと同一時刻の画像更新OFFは発散。短いrunは追従前に発散したため「追従開始後だけ画像OFF」も実施し、短い方13.77m、長い方発散。この設定では画像更新の効果が確認できた。通常MIMOSA LIOは13.50m/1.11mだが長い方は姿勢56.28°で、位置だけでは良好と判定できない。
- 実測校正は固定の6×6行列。共通IMU相関/時間相関は未補正。元の入力時刻・旧結果は保持。検証プロセスはすべて停止済み。

## 最新追記: 2026-10-05 DRPMの距離方向ノイズ修正
- センサは距離方向sigma3cm。点群でも確認し、DRPMのσ²I近似を距離方向σ²uuᵀへ対応。地図の取得時光線と、点→面法線への誤差伝播を実装。
- 同じ2例・同じ校正済み単眼ROVIO融合重みで3条件×2を完了。修正後位置RMSEはrun2a_end=23.97m、run1_tail_a2_prefix=5.88mで、従来4.40m/4.45mより悪化。モデルの不一致は解消したが精度低下は解消していない。
- 結果: `/root/uas_native/logs/zab-offline/drpm_noise/plots/index.html`。詳細 `resources/DRPM_RADIAL_NOISE_VALIDATION_20261005.md`。
- 修正版は比較用設定 `lidar/geometric/scan_to_map/drpm_noise_model: 2`。0は従来再現用、1は伝播式だけ変更した等方対照。通常SLAM環境は未変更。今回の瞬時スキャン用であり、実機回転走査では点ごとの取得姿勢を保持する拡張が必要。
- 現移植と原論文全更新則との差、共通IMU・地図姿勢などの相関は未対処。全出力時刻一致、検証プロセス停止、元データ/旧結果保持。


## 最新追記: 2026-10-05 DRPM公式更新と誤差近似の比較
- 完了：公開DRPM計算部を改変せず導入し、更新式×誤差モデルを分離。前回と同じ2飛行・校正済み単眼ROVIO。詳細は[DRPM_OFFICIAL_COMPARISON_20261005.md](resources/DRPM_OFFICIAL_COMPARISON_20261005.md)、図は `/root/uas_native/logs/zab-offline/drpm_official/plots/index.html`。
- 主比較の位置RMSE（短いrun2a_end / 長いrun1_tail_a2_prefix）：両方公式4.13 / 41.12m、更新だけ公式＋距離方向モデル1.24 / 0.87m、誤差近似だけ公式9.05 / 42.16m。位置は更新だけ公式、姿勢は両方公式が最小。DRPMなしに対して全指標を改善したわけではない。
- 公式Eq24更新を独立した点群位置合わせに適用し、Eq28情報でMIMOSAへ融合。3条件の周辺構造を共通化した。「公式」はDRPM計算部で、CompSLAM全体を同じにした意味ではない。主比較第3条件も旧直接グラフと構造が異なる。旧直接グラフを別途再実行し、前回の全出力と完全一致を確認した。
- 等方モデルでは対応点が少なく、多数の位置合わせが10反復上限へ到達。両方公式の30反復確認でも9.44 / 39.47mで、大きなドリフトは解消しない。採用回数・停止状況をverification.jsonへ保存。無収束の結果を収束済みと扱わない。
- comparison_wsのみ変更。新設定drpm_registration_modeは0=旧グラフ、1=公式更新の独立位置合わせ、2=重み付け式の独立位置合わせ。drpm_noise_modelは0=公式等方、2=距離方向。新経路は近傍5点、最大10反復（比較設定）、今回の瞬時スキャン用。
- 12試行全てexit0、時刻一致、公式ソース一致、数値・座標変換試験成功。検証用master11376/11377と推定器は停止済み。元bag・旧結果保持、GUIなし。


## 最新採用方針: 2026-10-05 公式DRPM更新＋距離方向ノイズ
- ユーザーがofficial_update条件を今後の比較基準として採用。`native/comparison/adopted_drpm.json`に記録。MIMOSA＋単眼link3 ROVIO、drpm_registration_mode=1、drpm_noise_model=2、反復10回。前回の直接グラフ重み付けや両方公式条件に戻して比較しない。
- FAST-LIVO2画像ありとの比較を[ADOPTED_DRPM_VS_FASTLIVO2_20261005.md](resources/ADOPTED_DRPM_VS_FASTLIVO2_20261005.md)へ整理。位置RMSEは採用方式1.239/0.865m、FAST0.0353/0.373m。姿勢RMSEは採用方式0.358/0.538°、FAST0.662/3.711°。
- 図は `/root/uas_native/logs/zab-offline/adopted_drpm_vs_fastlivo2/plots/index.html`。保存済みデータの再集計で、推定器は再実行していない。採用方針の記録であり、通常simulation launchは未変更。


## 最新追記: 2026-10-05 センサ除外比較
- 採用DRPMからROVIOを外したLIOを2飛行で測定。位置RMSE12.108/33.395m、姿勢1.411/3.265°。採用したROVIO融合構成は維持し、今回のLIOは別実験。
- FAST-LIVO2標準画像なしは既存fast_lioを再利用（位置12.58km/7.44kmの発散）。ROVIO単体も再利用。
- **FAST-LIVO2 LiDARなしは単体VIO未対応。** 画像処理の三次元点がLiDARに依存し、独立VIOモードがない。lidar_en=0をVIO条件として比較しない。新規精度値なし、未対応として記録。
- 図：`/root/uas_native/logs/zab-offline/sensor_ablation/plots/index.html`。詳細：[SENSOR_ABLATION_20261005.md](resources/SENSOR_ABLATION_20261005.md)。元時刻維持・元結果保全。新規ROS検証プロセス停止済み。


## 最新追記: 2026-10-05 FAST-LIVO2カメラ比較
- link1/link4を既存link3と同じ2飛行で評価。3条件とも現在の同じ実行ファイルで各3回、計18回。カメラ画像と取り付け変換だけ変更。
- run2a_endの平均位置RMSE: link1 0.0924m、link3 0.0351m、link4 0.1265m。
- run1_tail_a2_prefix: link1は3/3発散（位置RMSE5365–7752m）、link4は2/3発散（6803/7200m、残る1回0.502m）、link3は3回とも0.353–0.370m。発散試行は追従開始後128–137秒に位置誤差10m超。
- 同一入力・同一実行ファイルでも数値にばらつきあり。過去のlink3結果は混ぜない。OpenMP 1スレッド制限の補助試験でも解消せず、原因未特定。カメラ差・環境差と実装の再現性を区別すること。
- 成果物: native/logs/zab-offline/fast_camera_comparison/plots/index.html。主数値はrepeat_evaluation.json（evaluation.jsonは最初の1試行）。詳細は[FASTLIVO2_CAMERA_COMPARISON_20261005.md](resources/FASTLIVO2_CAMERA_COMPARISON_20261005.md)。元データ/旧結果保持、起動したROS masterと推定器は停止済み、GUIなし。


## 最新追記: 2026-10-05 FAST-LIVO2の複数カメラ対応
- comparison_wsのFAST-LIVO2へ独自拡張。カメラ別校正/画像地図を持ち、共通状態・共分散へ設定順に逐次画像更新。LiDAR/IMUは1回。multicamera/cameras省略で旧単眼経路。
- 今回は同期画像・固定露出・固定取付変換が対象。複数台では露出推定OFFを必須とし、露出状態をカメラ間で共有しない。関節変形/非同期/独立自動露出は未対応。
- 3台で2飛行各3回、発散なし。位置RMSEは短0.083–0.092m、長0.366–0.402m。姿勢は短0.395–0.464°、長0.672–1.180°。過去の単眼比較と露出設定が違うため、直接の優位性比較にはしない。
- 最終版11試行で時刻・LiDAR/IMU回数一致、全カメラの更新寄与と共分散引継ぎを確認。ROS topic経由も98個の撮影時刻付き/multicamera/odometry出力を確認。従来/aft_mapped_to_initはLiDAR更新時のまま。
- 使い方native/comparison/MULTICAMERA.md。実行native/run_fast_multicamera.py。新パッチをbuild_livo_comparison.pyへ組込み。詳細[FASTLIVO2_MULTICAMERA_20261005.md](resources/FASTLIVO2_MULTICAMERA_20261005.md)。通常ROS2 launchは未変更、検証プロセス停止済み。

## 完了: 2026-10-05 FAST-LIVO2 3カメラ・全14飛行
- 全14飛行のデータを取得。12飛行を新規計算、run2a_end/run1_tail_a2_prefixは事前指定の既存trial1。link1→link3→link4、固定露出・固定取付変換。前回3カメラ実験と同じ推定器/条件。
- 成果物: `/root/uas_native/logs/zab-offline/fast_multicamera_all/plots/index.html`、同ディレクトリall_flights.pdf（14ページ）。各runに真値/推定/六軸誤差CSV。詳細は[FASTLIVO2_ALL_RUNS_20261005.md](resources/FASTLIVO2_ALL_RUNS_20261005.md)。
- **5飛行で発散**: run1_prefix、run1_middle、run1_tail_a1、run1_tail_b1、run2b。失敗値も保持。残り9飛行の位置RMSEは0.041–2.015m。各1試行で、成功率や平均性能の推定ではない。
- 初期化には前段センサを使うが評価は軌道追従区間のみ。開始時の位置/姿勢を1回整合。元header stampの整数nsを保存、誤差は10Hz補間。全runの内部時刻連続性・有限姿勢・3カメラ寄与/共分散引継ぎ・設定/校正・入力チェックサム/推定ライブラリSHAを照合。
- run1_middleは最後の1323.2秒のみ、次のLiDARが記録されておらず画像更新を解放できないため欠ける（最終1323.15秒）。外挿なし。run2bは全推定終了後の親中断から復旧し、OS終了コードと逐次入力SHAが未取得。制限はsummary/詳細資料に記録。
- ユーザー指示で一時停止後、同じプロセスから再開して完了。過去の入力切断試行はinterrupted_attemptsへ保存し本結果に混ぜない。
- **未完了の処理はない。** 所有ROS/監視/推定/取得/集計は終了。一時swap2ファイルとzram0/1を解除/片付け、元/dev/sdcのみへ復元。入力キャッシュ削除、元クラウドbag/旧結果は保持。runtime.jsonはcomplete/cleanup_complete=true。

## 完了: 発散5飛行で単眼も崩れるかの確認（2026-10-05）
- **対象5件すべて、単眼でも発散。** link3・露出推定OFF。3台構成からmulticameraだけ削除し、同じバイナリの従来単眼経路を実行。その他設定/校正/入力・推定ライブラリを維持。
- 発散判定は事前固定した位置誤差100m超が5秒連続。確認後にチャンク境界で終了。開始時刻（共通追従開始から秒、単眼/3台）: prefix167.5/76.6、middle106.2/145.7、tail_a1 126.1/124.2、tail_b1 78.4/56.9、run2b121.7/115.3。
- 出力native/logs/zab-offline/fast_monocular_failures/plots/index.html、comparison.pdf（5ページ）。真値・推定値は元ns時刻。単眼/3台/MIMOSAの六軸比較は同じ評価区間。早期終了RMSEを旧全区間RMSEと混ぜない。3台の発散時刻だけは旧全区間から算出。
- 詳細[FASTLIVO2_MONOCULAR_FAILURE_CHECK_20261005.md](resources/FASTLIVO2_MONOCULAR_FAILURE_CHECK_20261005.md)。各1試行で、複数カメラ拡張が正しいことや平均性能を証明した結果ではない。共通部分/環境のどこが原因かは未特定。
- 全5件正常終了、設定/入力MD5/バイナリ/出力連続性/有限姿勢を照合。早期停止判定の合成試験、短い既存bagの出力時刻・更新回数一致も成功。全所有プロセス停止、入力キャッシュ削除、元bag/旧結果保持。未完了処理なし。

## 実行中: 2026-10-06 無改変FAST-LIVO2単眼の再検証
- ユーザーは公式標準実装・元画像で数例を再検証し、改善なら標準処理へ複数カメラ版も整合して全14runを再取得するよう指示。改善なしなら報告して終了。
- 独立WS `/root/multilink_autonomy_ws/fast_livo2_official_ws`。公式revision0d2c034の75tracked files全一致、CMake/mainも変更なし。公式ノードへ通常ROS経由で入力。元画像raw RGB、標準scale0.5、露出推定ON、標準点群処理設定。機体の校正/32線点形式/時刻差0/topic/pose保存だけ変更。
- コード native/{run_fast_official_raw.py,official_fast_publisher.py,official_fast_ros1.sh,audit_fast_official.py,evaluate_fast_official.py}。出力 native/logs/zab-offline/fast_official_raw、進行ログ native/logs/fast_official_campaign.log。control run2a_end → run1_tail_b1 → run1_tail_a1 → run1_prefix。100m超5秒連続で早期停止、GTは推定器入力に含めない。
- ROS master11378。現在実行中、結果未確定。旧実験/元bagは保全。終了後に所有master/検証プロセスとキャッシュを片付ける。

## 現在実行中: 2026-10-06 PCL互換設定後の公式単眼再測定
- 以前の「本測定」と呼んだnativeビルド3件は最終結果に使用しない。終了時のPointCloud解放SIGSEGVをgdbと独立VoxelGridテストで再現し、EIGEN_MAX_ALIGN_BYTES=16 + EIGEN_MALLOC_ALREADY_ALIGNED=1で解消。前試行はnative/logs/zab-offline/fast_official_raw/preliminary_pcl_abi_mismatchへ保全。
- 原本ソース/CMakeへの変更はゼロ。CMAKE_PROJECT_INCLUDE=.../comparison/official_pcl_abi.cmakeでビルド定義を追加。FAST-LIVO2とvikitを再ビルドし、元75tracked files + vikit全tracked files一致を再照合。
- 現在の実行ログはnative/logs/fast_official_abi_campaign.log。3飛行prefix→tail_a1→tail_b1。結果出力の親ディレクトリはfast_official_raw。正常終了を必須とし、前段の終了異常試行の数値を採用しない。
- 集計はfinalize_fast_official.py（現時点では3件発散を仮定してassertしているため、改善/完走があれば実測に合わせて分岐と報告を見直す）。公式ログから画像補正後状態も抽出し、公式evoの時刻・補正前位置と照合。
- 条件付き依頼は引き続き有効：公式単眼で発散が改善/解消したら、複数カメラも標準入力/処理に整合し全14runを再計算。3件とも発散が残る場合は、発散時期の変化を明記して報告。
- ROS master11378は所有プロセス。終了後に停止し、作業記録と本項の状態を完了へ更新する。

## 2026-10-06 公式FAST-LIVO2単眼 再検証完了（最新状態）
- 本測定は完了。全3例で画像補正後も発散、全プロセスexit0。以前の単眼→公式単眼の100m超5秒継続の開始時刻（追従開始から秒）：prefix 167.5→133.8、tail_a1 126.1→114.4、tail_b1 78.4→84.2。安定完走は0/3のため、条件付き依頼の複数カメラ全14run再実行には進めていない。
- 最終結果はnative/logs/zab-offline/fast_official_raw直下の3run。preliminary_*の試行は集計対象外。plots/index.html、plots/comparison.pdfとresources/FASTLIVO2_OFFICIAL_RAW_VALIDATION_20261006.mdを参照。
- 公式ソース0d2c034/rpg_vikit6c886c8は無改変。公開avia推定設定、scale0.5、露出推定ON、元RGB、link3単眼。機体校正・入力形式のみ適合。PCL互換性のビルド定義2件はcomparison/official_pcl_abi.cmake。詳細はsource_audit.json。
- 全検証プロセス/master11378停止済み、専用cache削除済み。残作業・実行中の収録なし。元bag・旧結果保持。

## 2026-10-06 DRPM＋visionを公式FAST発散3runへ適用（進行中）
- 新依頼：run1_prefix/run1_tail_a1/run1_tail_b1を、採用済みDRPM＋link3単眼ROVIOで評価。全追従区間を処理し、保存済み公式FASTの画像補正後結果と共通区間で比較する。
- 出力native/logs/zab-offline/drpm_fast_failures、主ログnative/logs/drpm_failure_campaign.log、短い2本の先行準備・実行はdrpm_failure_prepare_{a1,b1}.log / drpm_failure_flight_{a1,b1}.log。所有master11376。推定器は以前の採用バイナリとSHA一致。
- 新3runの重みは、過去run2a_end/run1_tail_a2_prefixの保存済み50ms相対誤差を併合した固定共分散。今回の真値では調整しない。ROVIOは既採用の白黒補正480×360。FASTと同じ物理カメラだが前処理を同一化する実験ではない。
- 約40秒のintegration_previewでROVIO→DRPM付きMIMOSAの入出力・正常終了を確認（384 LiDAR処理、50ms推定連続）。本評価とは別。元bagは約73GBで、現在は主にクラウド転送待ち。完了後、比較図・検証・作業記録を確定し、所有処理と作業用sensor cacheを片付ける。
- 先行確認：drpm_fast_failures/common_preview/run1_prefix相当のfusion配下で共通147.2秒のみ評価済み。DRPM位置RMSE0.58065m/姿勢0.39595°（FAST210.13769m/16.57919°）、最大位置誤差2.646m。全区間の結果ではない。全3run本測定は継続中。
- 取得の最新ログはdrpm_failure_campaign_resume.log（旧campaign.logは容量対応で未完了downloadを停止した時の終了記録）。prefix39分割保持で再開。専用cacheは/dev/shm/drpm_fast_failures_cacheへのsymlink。Windows空き約6.4GiBのため、元MCAPの一時コピーをRAMに置く方式へ変更、データや推定設定は不変。a1全区間完了：5.01997m/0.53561°、最大11.47460m。b1融合継続中。所有master11376はそのまま。
- b1全区間も完了：DRPM＋visionは発散（位置RMSE983.333m、最大2408.108m、100m超5秒継続の開始95.2秒）。ROVIO単体も最大2539.195mまで破綻。FAST共通区間ではDRPM58.572m/1.896°対FAST189.297m/22.708°だが、直後にDRPMも発散する。a1/b1一時入力は削除済み。残りはprefix全区間のみ。prefix用ROS1bagは専用/dev/shmへのsymlinkで作成し、最終cleanupで実体も削除する。

## 完了: 2026-10-06 DRPM＋単眼visionで公式FAST発散3飛行を比較（最新状態）
- 全3飛行の全追従区間を処理済み。link3単眼、採用済み公式DRPM更新＋距離方向ノイズ＋ROVIO。重みは過去2飛行の相対誤差を併合して固定し、今回の真値で調整していない。
- 全区間DRPM位置RMSE/最大位置誤差[m]：run1_prefix47.96271/56.48900、run1_tail_a1 5.01997/11.47460、run1_tail_b1 983.33324/2408.10808。b1は発散、prefixにも大きなドリフト。共通短区間だけの改善を全区間の安定性と混同しない。prefixはROVIO単体15.84085mより融合47.96271mの方が悪い。
- 共通区間のDRPM対FAST位置RMSE[m]：prefix0.58065対210.13769、a1 3.48923対82.42032、b1 58.57201対189.29717。FASTは早期終了なので、これと全区間DRPMのRMSEは別扱い。前処理は各方式の既存設定を維持し、同じ物理カメラでも同一画像前処理ではない。
- 最終成果物native/logs/zab-offline/drpm_fast_failures/plots/index.htmlとcomparison.pdf（6ページ）。元ns時刻の真値/ROVIO/融合結果と六軸誤差CSV、監査結果を保存。詳細はWORKLOG末尾。全センサ周期欠落なし、出力50ms連続、全プロセス正常終了、バイナリ/公式DRPMヘッダ/入力/設定一致を確認。prefix先行短区間との位置差0。
- 所有master11376・取得/推定処理は全停止。一時NPZ/ROS1bagと専用RAMキャッシュ削除済み。元bag/旧結果保全。runtime/validationはcomplete、cleanup_complete=true。未完了の処理なし。

## 2026-10-06 元rosbag3runをローカルへ常設する作業（取得中）
- ユーザーがWindows空きを増やし、3runのローカル保存を指示。native/cache_zab_bags.pyを実行中、進捗native/logs/zab-local-download.log、保存先native/logs/zab-local-bags/<run label>/rosbag。対象prefix/tail_a1/tail_b1、全127MCAP約73.44GB。元の全3カメラ/IMU/LiDAR等を保持。全取得・サイズ/MD5・rosbag読込検証後に完了更新する。
- 常設データを実験cleanupで消さないこと。DRPM入力準備と公式FAST再生はlocal_zab_bags.py経由で常設データを優先し、一時cacheのsymlinkだけ削除する。
- 速度改善：eduroamへ追加接続し、Windows版rcloneをWi-Fi IP10.24.131.14へbind。実bag取得が転送中約32MB/s（接続を含む平均23.49MB/s）へ改善。有線も維持。現在はcache_zab_bags.py --wifi-ip 10.24.131.14（Windows Wi-Fi4並列＋Linux有線2並列）、ログzab-local-download-fast.log。元進行中の全転送を終えてからスケジューラを切替、既存データは保持。取得後にWi-Fiを元の切断状態へ戻す。

## 完了: 元rosbag3runのローカル常設と転送高速化（2026-10-06 最新状態）
- `/root/uas_native/logs/zab-local-bags/{run1_prefix,run1_tail_a1,run1_tail_b1}/rosbag` に全元データを保存。80/22/25分割、合計73.44GB。metadata.yaml付き、各ディレクトリを `ros2 bag play <dir>` で指定できる。
- 全127MCAPの元MD5・サイズ一致、全ファイルのROS2 reader読込、3bagディレクトリからセンサ実メッセージ読込成功。validation.json/reuse_validation.json/progress.jsonを参照。全センサと全記録期間を保持し、元時刻を変更していない。
- prepare_livo_comparison.pyとrun_fast_official_raw.pyはlocal_zab_bags.pyを通じて常設データを優先。cacheにはsymlinkだけ置き、終了処理で元bagを消さない。**この常設先を実験の一時cacheと一緒に削除しないこと。**
- 転送を有線のみ約12MB/sから、Windows rcloneをWi-Fiへbind＋有線の併用へ変更し、実測合計42.7MB/s（3.52倍）に改善。切替後の残作業は約21分26秒で終了。保存済みeduroamへの追加接続は取得後に解除し、Wi-Fiは元の切断状態に戻した。有線は維持。
- 取得・検証プロセスは全終了、一時MCAP/partialなし。Windows空き約49GiB。クラウド元bag・旧結果保持。未完了の取得なし。

## 実行中: DRPM＋3カメラROVIOで同じ3runを評価（2026-10-06）
- ユーザーはカメラ3台で同じ3runのDRPM検証を指示。run1_tail_a1→run1_prefix→run1_tail_b1の順、全追従区間を処理し前回単眼と比較。DRPM/融合共分散は前回固定、ROVIOの総特徴点上限25・設定を維持。3台向け重み再校正は行わない。
- native/run_drpm_three_camera.py campaign、ログnative/logs/drpm_three_campaign.log、出力logs/zab-offline/drpm_three_camera。prefix入力を並行準備（drpm_three_prepare_prefix.log）。phaseごとにファイルロック/完了markerで二重処理を防ぐ。
- 3台既存ROVIOハーネスに共分散CSV出力を追加した独立binaryをbuild済み。初回buildのパスalias置換漏れで既存3台binaryを再ビルドしたが、既存vio_recomputed/manifest.jsonのSHAと完全一致（45605f...9bd）を確認。修正後は新しい出力先だけに独立ビルド済み。過去の推定結果/単眼binaryは不変。
- master11376（roscore PID26598）を所有して起動中。終了後に停止する。常設元bagは必ず保持、派生NPZ/ROS1bagは各runの評価後削除。finalize_drpm_three_camera.pyで比較PDF6ページと検証結果を作成し、完了時に本項の状態を更新する。

## 完了: DRPM＋3カメラROVIOで同じ3runを比較（2026-10-06 最新状態）
- 前項の実行は全完了。全追従区間、link1/link3/link4、合計特徴点25、前回単眼の融合重み/DRPM設定を固定。今回の真値で再調整なし。
- 位置RMSE[m]（単眼融合→3カメラ融合、ROVIO3台単体）：prefix47.96271→5.05157、単体2.65872。a1 5.01997→4.38222、単体0.90523。b1 983.33324→0.37417、単体0.26632。b1最大誤差0.61663m、prefix/a1最大8.15634/8.29686m。3台化で全run改善するが、全区間の融合位置RMSEは3カメラROVIO単体より悪い。DRPM単独の改善効果を示した比較ではない。
- 成果物native/logs/zab-offline/drpm_three_camera/plots/index.html、comparison.pdf（9ページ）。単眼対3台全区間、公式FASTの短い共通区間、3台融合対ROVIO単体詳細の六軸図。comparison.json/validation.jsonと元時刻の推定/真値CSVを保持。
- 全入力周期/画像同期/特徴点3台利用、出力50ms連続、前回と同じ真値/区間/バイナリ/融合設定を検証。DRPMは全LiDAR時刻で実行。詳しい測定値と制約はWORKLOG末尾。
- 全所有処理/master11376停止、派生NPZ/ROS1bag/cache削除。常設元127MCAP73.44GBおよび旧結果は保持。runtime/validationはcomplete・cleanup_complete=true。未完了処理なし。

## 2026-10-06 3カメラMIMOSA＋ROVIOのDRPMなし対照：開始
- ユーザー指示により同じprefix/tail_a1/tail_b1でDRPM OFFを追加測定。ROVIOは直前の3カメラCSVを再利用し、融合共分散/バイナリ/全追従区間を固定。変更はdrpm_enabledだけ。既存project_on_degneneracyも前回と同じfalse。通常MIMOSA点面因子と採用DRPMのICP姿勢因子の比較であり、同一解法で確率抑制のみを除いた比較ではない。
- run_three_camera_no_drpm.pyでセンサだけを常設元bagから再抽出し、前回と入力bag SHA256一致を検証してから測定する。画像/ROVIOは再計算しない。結果native/logs/zab-offline/three_camera_no_drpm、主ログnative/logs/three_camera_no_drpm.log。plot_three_camera_no_drpm.pyで六軸ON/OFF/ROVIO図を出力予定。所有master11376 PID38285は完了時に停止。元bag/前回結果は保持。

## 完了: 3カメラ MIMOSA＋ROVIOのDRPM ON/OFF対照（2026-10-06 最新状態）
- 前項のOFF測定は全3run完了。前回3カメラROVIO出力/融合重み/バイナリ/全追従区間を固定し、入力bag SHA256も一致。変更はdrpm_enabledだけ（ログ先除く）。既存MIMOSA退化投影も両条件OFF。
- OFF / ON 位置RMSE[m]：prefix3.147873/5.051568、a1 0.191751/4.382224、b1 0.196655/0.374166。全3runでOFFが小さい。姿勢RMSE[deg]：prefix1.094874/1.503656、a1 0.098940/0.424795、b1 0.562995/0.420671（b1のみONが小さい）。OFF最大位置誤差5.44656/0.75132/0.36335m、全3run発散なし。
- OFFは通常MIMOSA点面因子、ONは採用DRPMの独立ICP姿勢因子のため、退化確率だけの因果比較ではない。各1試行。3カメラROVIO単体と比べるとOFF融合はa1/b1の位置を改善、prefixでは悪化。DRPM導入が必要だったとは言えない。
- 成果物native/logs/zab-offline/three_camera_no_drpm/plots/index.html、comparison.pdf（3ページ）、元時刻CSV、comparison.json/validation.json。六軸ON/OFF/ROVIO、全区間、縦軸省略なし。詳細はWORKLOG末尾。
- 全入力/出力時刻/設定差・正常終了を検証。全所有処理/master11376停止、派生センサ入力/cache削除、常設元127MCAPと旧結果保持。runtime/validation complete、cleanup_complete=true。未完了処理なし。

## 2026-10-06 run1_tail_b1 Vision原因解析（完了、追加推定なし）
- 詳細native/logs/zab-offline/b1_vision_diagnosis/index.html。実経路の進行時刻で区間分け、保存ログ/コード解析のみ、推定器変更なし。
- 過去単眼ROVIOは静止10秒時点から速度0.546m/s・深度中央値1.8km、3台は正常。単眼MIMOSAの不調は既に破綻したVIO入力の融合。現在3台MIMOSAは位置を改善、姿勢のみROVIO単体より悪い。相対姿勢制約＋以前単眼の固定重みなのでROVIO絶対姿勢精度を保持する構造ではない。どの他制約が主因か未分離。
- FAST単眼は主通路に入った約66秒後、70〜80秒にロールが急増。78.95秒LiDAR変更2.35deg/画像変更0.0106deg、誤差28.4deg・対応19点。対応ゼロは81.05秒以降。画像停止が先ではなく、LiDAR/IMU姿勢悪化を画像が戻せずその後対応消失。画像重みと共有地図誤りの分離には内部ログ追加が必要。表/図/CSV/source SHA保存、WORKLOGに詳細。稼働プロセスなし。

## 2026-10-07 単眼ROVIO初期化修正（完了）
- 現在の依頼：単眼の初期化失敗を先に解消。ローカル3runで検証済み、オンラインsimulation不要、GUIなし。融合/FAST/3カメラの再計算はしていない。
- rovio_initialization.pyで記録URDFの加速度バイアス仕様から初期分散を生成。従来0.0004→0.010001、ゼロ平均は保持。replay_monocular_comparison.pyの今後の単眼実行に適用済み。ライブラリ/バイナリや3カメラ・live launchには変更なし。
- 全3runで離陸前の偽速度/特徴深度の崩壊を解消。b1の10秒時点速度0.546→0.000583m/s、深度中央値1806→9.945m（距離初期値を維持、真の距離を測定した意味ではない）。真値未使用、初期3分散のみの対照。
- 全追従位置RMSE旧→修正：prefix15.841→22.609m（悪化）、a1 8.313→7.011m、b1 1040.097→0.560m。初期化は改善しても飛行中の単眼ドリフトは残る。全14単眼安定化やactive perceptionの効果を確認した結果ではない。ゼロ速度更新は実験したが採用せずOFF維持。
- native/logs/zab-offline/monocular_startup/plots/index.html、comparison.pdf、evaluation.json。test_monocular_startup.py/report_monocular_startup.pyで再現。b1旧設定のCSV完全再現、3run同バイナリ/校正/件数/時刻/全区間完走確認。旧結果・元bag保持。詳細WORKLOG末尾。
- 最終確認：PDF9ページオブジェクト・HTML参照・3run検証記録に成功。所有ROS master11376（PID152172）と推定器を停止。validation.json complete。元bag/過去結果を保持、未完了処理なし。

## 2026-10-07 修正後単眼ROVIO＋MIMOSA＋DRPM（完了・最新）
- ユーザー依頼の同じ3runでlink3単眼融合を再計算。ROVIOはmonocular_startup/bias CSVをそのまま再利用。MIMOSA/DRPMバイナリ・融合設定・旧2run由来の固定共分散を保持し、再チューニングなし。
- 位置RMSE[m]前回融合→今回融合（修正後ROVIO単体）：prefix47.963→13.175（22.609）、a1 5.020→3.090（7.011）、b1 983.333→6.524（0.560）。旧融合より全3run良いが、b1はROVIO単体より悪い。今回融合姿勢RMSE2.874/0.770/0.454degで単体より全3run悪く、a1は旧融合よりも悪い。
- native/logs/zab-offline/drpm_corrected_monocular/plots/index.html、comparison.pdf（6ページ）、comparison.json/validation.json、各fusion/<label>/metrics.json・元時刻CSV。六軸図は修正後融合/ROVIO/旧3カメラ対照と、旧単眼融合との対照。全追従評価、時刻欠落なし、不確かさによるVIO棄却0、DRPM全スキャン実行確認。
- run_corrected_mono_drpm.py/plot_corrected_mono_drpm.pyで再現。全処理完了、所有master11376/11377停止、派生入力bag/NPZ削除、元127MCAPと過去結果保持。ライブsimulation/GUIなし。詳細WORKLOG末尾。

## 2026-10-08 修正後単眼ROVIO＋MIMOSAのDRPM ON/OFF（完了・最新）
- 最新ユーザー依頼のDRPM OFFを同じ3runで完了。link3単眼ROVIOの初期化修正済みCSVを再利用。入力bag全体のSHA256が前回ONと一致、融合重み/バイナリ固定、DRPM有効フラグのみ変更（ログ先除く）。通常MIMOSAの退化投影は両条件OFF。
- 位置RMSE[m] ON/OFF：prefix13.174592/23.457815、a1 3.090375/5.344842、b1 6.523832/0.264193。姿勢RMSE[deg] ON/OFF：prefix2.874468/1.149464、a1 0.769588/0.333373、b1 0.453674/0.603085。位置はprefix/a1でON、b1でOFFが良く、姿勢では逆。単眼ROVIO位置RMSE22.609260/7.010673/0.559758。
- native/logs/zab-offline/corrected_monocular_no_drpm/plots/index.html、comparison.pdf、comparison.json/validation.json。3ページ6軸でOFF/ON/ROVIO単体比較。run_corrected_mono_no_drpm.pyとplot_corrected_mono_no_drpm.pyで再現。全追従20Hz欠落なし、VIO棄却0、DRPM分岐未実行を確認。
- OFF通常MIMOSA点面制約とON採用DRPM独立ICP姿勢制約の比較。同じ解法の確率処理だけの差ではない。既存ON/3カメラ結果は保持。
- 全処理完了、所有master11376/11377停止、一時bag/NPZ削除、元bag保持。GUI/ライブsimulationなし。未完了処理なし。詳細WORKLOG末尾。


## 2026-10-08 修正後単眼ROVIO＋MIMOSA：標準並進退化処理ON（完了・最新）
- 最新依頼の同じ3run、DRPM OFF・標準退化処理ON・並進のみを測定完了。しきい値は並進40/回転0を保持、単眼link3 ROVIO（初期化修正済み）出力と固定融合重み・バイナリも保持。前回標準OFFと入力bag全体SHA256一致。有効フラグとログ先以外の設定差なし。
- 全3runで退化処理分岐呼出し0回、全時刻の位置・姿勢が標準OFFと完全一致。現在のしきい値で発動しなかった結果であり、補正を発動したときの有効性は未検証。勝手にしきい値を上げないこと。
- 位置RMSE[m] / 姿勢RMSE[deg]：prefix23.457815/1.149464、a1 5.344842/0.333373、b1 0.264193/0.603085。全追従20Hz欠落なし、VIO棄却0。
- native/logs/zab-offline/corrected_monocular_native_degen/plots/index.html、comparison.pdf（3ページ）とcomparison.json/validation.json。標準ON/OFF・採用DRPM・ROVIO単体の六軸図、ON/OFFは一致するので線が重なる。再現run_corrected_mono_native_degen.py、plot_corrected_mono_native_degen.py。
- 全所有処理/master11376/11377停止、一時bag/NPZ削除、元127MCAP73.44GB・過去結果保持。ライブsimulation/GUIなし、未完了処理なし。詳細WORKLOG末尾。


## 2026-10-08 最新方針：DRPM＋MIMOSAと単眼ジンバルの先行研究
- ユーザーはDRPM＋MIMOSAを一旦採用。次に試すのは単眼カメラのジンバル回転で、LiDARの退化方向を視覚で補うこと。今回の依頼は先行研究調査のみで、実装・simulationは依頼されていない。
- 調査完了。公開本文を確認した範囲では、LIVO＋LiDAR退化方向検出＋その方向を補うカメラ視野の能動制御を全て満たす論文は未発見。SKF（方向選択的融合）、IGLOV（ジンバル視野計画）、DC-VINS（可動カメラVIO）が主要参考。可動カメラとLiDAR・IMUを併用した先行例自体はある。ETH RSLの研究紹介にもパン・チルトによる退化対策があり、世界初や未着手とは断定しない。
- resources/README.md末尾に本文の確認箇所付き比較と未確認点、resources/active_gimbal_20261008/にPDF10本と取得記録。既存推定設定・データ・実験結果は保持。カメラ回転を実装する際は時変のカメラ対IMU変換を扱う必要がある。未完了プロセスなし。

## 2026-10-13 最新：単眼ジンバルの能動視覚を実装・接続検証済み
- `native/active_perception/`と`launch/active_gimbal.launch.py`を追加。DRAGON固定四角形、link3単眼の光学中心で交差するyaw/pitchジンバル、link2 LiDAR、採用DRPM＋MIMOSA＋修正済み単眼ROVIO。既存3カメラlaunchの既定動作と過去結果・元bagは保持。
- 公式FIFリポジトリ76d7b834の特徴点ベースの情報計算を無改変で利用。保存したROVIO視覚点の6×6情報と、その時点のDRPMの有効LiDAR情報を共通の機体座標で組み合わせ、弱い方向の予測不確かさ減少＋進行方向＋角度変更量で選択する。85候補＋現在方向、実測合成角速度上限20度/秒。全FIFのボクセル近似を移植したわけではない。地図や退化情報が足りないときは保持し、未知の特徴は推測しない。
- 画像時刻の実測関節角を補間し、ROVIOの画像更新前に特徴点・パッチ・共分散を回転変換。光学中心の並進は非対応として拒否、機体関節の四角形からの逸脱も検出。推定器の基準バイナリは書き換えず`build/active_perception`へ独立ビルド。固定融合共分散/DRPM設定は既採用値。
- 初回`python3 /root/uas_native/active_perception/build.py`、必要ならbringupをcolconビルド。起動例：`ros2 launch multilink_uas_bringup active_gimbal.launch.py world_file:=/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf spawn_x:=0 spawn_y:=0 takeoff:=true`。対照は`mode:=fixed`。GUI/自動rosbagなし、takeoff既定false。起動端末Ctrl+Cで全所有処理終了（既存manager.py stopの対象ではない）。
- 検証：数値テスト10件＋ROVIO座標/共分散変換テスト成功。元bag2.003〜20秒を固定時に再生し、既存ROVIOの位置・姿勢・共分散・バイアスがbyte単位一致、特徴点データ行一致。`logs/active-static-regression/validation.json`。
- ZaBの固定/可動とも離陸＋30秒hover成功。最終可動`logs/active-gimbal-20261013-144001/validation.json`：ROVIO20Hz欠落なし、MIMOSA最大間隔100ms、評価p95=23.01ms、yaw24.80度/pitch30度の角度範囲、合成20度/秒以下。全プロセスexit0で停止、master11379/11380も停止済み。
- 判断ログ`decisions.jsonl`、地図`visual_map.npz`、設定/ソースsnapshot/バイナリSHAを各runへ保存。地図は起動ごとのROVIO座標系なので別起動への無整合再利用は不可。評価は近似で、地図/姿勢/共有IMU相関や未観測遮蔽・追跡成功を厳密には扱わない。長距離での精度改善を証明した段階ではない。次は同条件の固定／能動比較。WORKLOG末尾とREADMEの起動手順を参照。

## 2026-10-13 固定／偏向3軌道比較（進行中）
- 出力`native/logs/active-fixed-three-20261013`。`active_perception/compare_routes.py --output <同ディレクトリ>`で完了済みを再利用して再開。master11379、専有supervisor lock、GUI/bagなし。推定DRPM OFF、視野方向用の診断だけON、単眼link3。重み・速度0.5m/s固定。
- a1固定／偏向完了、b1固定完了。b1偏向初回はsim3秒からROVIO初期化が崩れ、カメラ偏向評価0回。`excluded_startup/run1_tail_b1_active_attempt1`に移動済み。偏向による発散と扱わない。`excluded_runs.json`に根拠保存。再試行が必要。
- 現在prefix固定1070秒を飛行中。その後prefix偏向を自動実行予定。現在のcampaignプロセスは起動時にb1初回をスキップしたため、prefix両条件の終了後に`compare_routes.py --output <同じ出力> --labels run1_tail_b1`を実行してb1偏向を取り直すこと。既存b1固定は再実行しない。
- 新規試行はsim5秒までのROVIO初期位置変化1m未満を離陸前適格条件とし、既存正常試行も同基準で確認。推定器を再調整したわけではない。条件を満たさない試行は本比較から除外・保持し、停止理由を監査。
- `report_routes.py <出力>`で完了分のmetrics/validation/PDF/HTML生成。現在の途中結果：a1 MIMOSA位置4.899→3.310m、姿勢0.286→0.363deg、b1固定位置0.429m。未完了条件を完走結果と混同しない。最終的に3組を揃え、全プロセス停止・記録更新まで行う。

## 2026-10-13 固定／偏向3軌道比較（完了・最新）
- 上記の進行中タスクは全6有効飛行の取得・評価まで完了。native/logs/active-fixed-three-20261013/plots/index.html、comparison.pdf（6ページ）。DRPM推定補正OFF・標準退化投影OFF、カメラ方向判断用の診断のみON。link3単眼、0.5m/s、同じルート/seed/推定設定、重み再調整なし。
- MIMOSA位置RMSE固定→偏向[m]：prefix18.753519→46.477887、a1 4.899126→3.309576、b1 0.428570→11.891206。回転[deg]：prefix1.645536→4.553411、a1 0.285864→0.363372、b1 0.760641→2.353881。位置は1/3改善・2/3悪化、回転は3/3悪化。ROVIO単体も位置はa1だけ改善。現在の方策で一貫した改善は得られなかった。各条件1試行。
- b1偏向初回は偏向開始前の初期化失敗なので除外・保持し、同設定で取り直して完走済み。元CSVとstartup判定を保存。根本的な初期化安定性の解決とは別。全有効試行は共通の離陸前適格条件に合格。
- 全追従カバー率100%、ROVIO50ms/MIMOSA100ms/LiDAR100msの最大間隔。全設定・ルート・校正・バイナリ一致を確認。全プロセス停止、GUI/bagなし。prefixの制御だけ完走後cleanupのSIGINTでexit-2（推定器exit0・データ完全）；自然終了待ち修正後の最終b1は全exit0。詳細WORKLOG末尾。未完了測定なし。

## 2026-10-14: 現在の画角調整対象はUniPilot＋単眼2軸ジンバル

ユーザーはDRAGONを使う手間を避け、標準UniPilot機体でカメラだけを偏向する方針へ変更。手動画角確認の既定起動は `ros2 launch /root/uas_native/gimbal_inspect.launch.py`（ZaB、Gazebo、rqt画像と角度スライダー）。カメラ位置・基準姿勢の編集元は `native/unipilot_gimbal.py` の `MOUNT_XYZ/MOUNT_RPY`。前方単眼480x270/20Hz、元モデルの画角・画像ノイズを保持。離陸・着陸は `/unipilot/takeoff`・`/unipilot/land` のTriggerサービス。ユーザー端末からCtrl+Cで全終了。

離陸→偏向中hover→着陸を実測確認し停止済み。標準垂直gainでは未離陸のため、既存UniPilotデモと同じvelocityGain=4 4 10を使用。詳細WORKLOG末尾と `native/logs/unipilot-gimbal-check-gain/validation.json`。自動偏向/ROVIO/MIMOSAのUniPilot移植は今回行っていない。既存 `active_gimbal.launch.py` はDRAGON用のまま。旧手動画角確認は `robot:=dragon` で利用可能。

2026-10-14 GUI追記: UniPilotのGUIがWSLg/D3D12によるOpenGL初期化失敗で終了する現象を再現。起動をserver/guiに分離、WSLではgui_software_rendering:=trueを既定として表示だけCPU描画へ変更。通常コマンドは同じ。物理/センサはGPU設定不変。GUIはZaBで約2fpsになる場合あり。詳細はWORKLOG末尾。

2026-10-14 更新（直前のCPU描画既定を置換）: GUIをGPU描画へ戻し、WSL互換GL3.3/GLSL330/xcb設定を追加。2回起動でクラッシュなし、読み込み後約25〜41fpsを確認。CPU描画は明示オプションのみ。UniPilotジンバル範囲はyaw±50deg/pitch±90deg、取り付け位置は既存のまま。SDF/rqtでGIMBAL_LIMITS_DEGを共有。WORKLOG末尾とlogs/unipilot-gimbal-gpu-limits-checkを参照。


## 2026-10-14 最新：UniPilot単眼のIMU・初期化修正と検証を完了
- UniPilotへ移植済みの自動偏向／ROVIO＋MIMOSAについて、IMUバイアスの時間変化をSDFに合わせ、world重力9.8と3秒平均初期姿勢を使う修正を採用。`active_perception/unipilot_run.py` / `unipilot_campaign.py` の既定は `--imu-model sensor --stationary-mode mean`。airborne-init時はMIMOSAもhover待ち。旧再現は `--imu-model legacy --stationary-mode legacy --legacy-slam-start`。
- a1/b1固定・偏向の同一入力再生4条件でROVIO位置誤差は改善。全長オンラインの融合後位置RMSE旧→新はa1固定156.51→13.10m、偏向150.96→45.79m、b1固定449.29→29.86m、偏向23.16→31.96m。改善3/4、大きなドリフトは未解決。DRPM融合OFF。真値は評価と機体制御だけ、センサ設定・配置・融合重みは保持。
- 結果 `native/logs/unipilot-imu-fix-comparison/index.html`、再生 `unipilot-sensor-bias-a1-v2` / `unipilot-sensor-bias-b1`。各runの実装・設定・入力監査を保存。無効な初回再生2件はINVALID_EXPERIMENT.txtで明示、v2だけ使用。旧DRAGON結果を上書きしない。
- 残件：UniPilot単眼の位置スケール・姿勢ドリフト、偏向の一貫した有効性、IMUのまれな1周期欠落。配置2要因だけが原因とは断定できない（IMU/画像仕様や機体制御も異なる）。MIMOSAのbias因子・融合重みは従来のまま。far60/gyro prior縮小は採用なし。
- 全オンライン／再生／master／keepalive停止済み、GUI/bagなし。詳細はWORKLOG末尾。単眼診断用input_recordは任意で既定OFF。新しい依頼が来るまで追加実験を自動起動しない。


## 2026-10-15 最新：a1 yaw−50deg / pitch0deg固定を完走・評価済み
- ユーザー指定a1のみ。修正済みUniPilot、DRPM融合OFF、単眼カメラyaw−50/pitch0を初期化前から固定。`--modes fixed --fixed-angles-deg -50 0` をcampaignへ追加、既定動作は不変。
- 前回0度固定→今回−50度固定：ROVIO位置RMSE69.373→155.536m、回転6.543→3.257deg。MIMOSA＋ROVIO位置13.095→114.103m、回転6.239→4.306deg。位置は悪化、回転は改善。別飛行各1試行、統計的な方向優劣は未確定。
- 全経路228.493秒を完走、raw IMU/LiDAR欠落0。追従中yaw最大誤差0.001995deg、pitch誤差3.51e-12deg。起動sim3.737秒で角度整定、VIO開始27.650秒、初期化中も角度固定済み。全所有処理停止、GUI/bagなし。
- 結果 `native/logs/unipilot-yaw-minus50-a1-20261015/plots/index.html`、PDF/数値/実測角度検証。詳細WORKLOG末尾。追加実験は稼働していない。


## 2026-10-15 最新：a1 yaw＋50deg / pitch0deg固定の検証完了
- ＋50degを追加測定、修正済みUniPilot/DRPM融合OFF/単眼、同じa1全228.493秒を完走。今回ROVIO位置RMSE92.472m/回転9.441deg、MIMOSA＋ROVIO位置40.249m/回転7.702deg。
- 位置は−50deg（155.536/114.103m）より小さいが0deg（69.373/13.095m）より大きい。回転は3方向で最大。各1試行・別飛行、統計的結論は出さない。
- 角度は初期化前に整定、追従中yaw最大誤差0.001817deg/pitch3.70e-12deg。raw IMU/LiDAR欠落0。全所有プロセス停止、GUI/bagなし。
- 3方向の六軸図と評価結果：`native/logs/unipilot-yaw-plus50-a1-20261015/plots/index.html`。report_fixed_view.pyは実際の指定角度からラベルを生成し、--extra-referenceで追加方向を比較する。過去の出力は上書きしていない。

## 2026-10-15 UniPilotのセンサ条件をDRAGONに合わせたa1比較
- ユーザー依頼でa1のみ、単眼yaw/pitch0、DRPM融合OFF。`--sensor-profile dragon` で物理カメラ/IMU/LiDAR仕様をDRAGONと一致させる（取付位置/姿勢、機体、制御はUniPilotのまま）。標準nativeは不変。参照仕様はnative/sensor_profilesに保存済み。
- 物理仕様だけ変更した試行はROVIO位置RMSE7125m、ROVIO誤差設定もDRAGONにした試行も3499mと発散。ホバリング中の実回転を3秒平均してgyro biasへ代入する初期化が、ほぼ無雑音のgyro仮定と整合しないことを事後の真値監査で確認。
- 最終条件は `--sensor-profile dragon --rovio-noise-profile dragon --stationary-duration 0 --modes fixed --fixed-angles-deg 0 0`。初回IMU前の画像を捨てる起動guardも追加（初期化待ち中のqueue overflow対策、IMU受信後の推定は変更なし）。物理仕様はcamera120deg/no image noise/raw720x540→480x360/20Hz、IMU1000Hz/DRAGON noise/bias、LiDAR360x32/30m。詳細WORKLOG。
- 最終a1: ROVIO位置RMSE7.044938m/回転.387546deg、MIMOSA＋ROVIO5.053299m/.515453deg。旧UniPilot69.373m/6.543deg、13.095m/6.239degから改善。DRAGON参考7.956m/.058589deg、4.899m/.285864degと位置は同程度、回転はまだDRAGONが良い。
- 出力 `native/logs/unipilot-dragon-sensors-init-v2-a1-20261015/index.html`（全3試行と旧UniPilot/DRAGONの表）、plots/comparison.pdf（最終条件の六軸）。各条件1回・別飛行、要因ごとの独立寄与は未検証。全区間完走152.181s wall、raw IMU/LiDAR欠落0、全所有プロセス停止済み、GUI/bagなし。


## 2026-10-15 最新：UniPilot ROVIOドリフト原因診断・完了
- ユーザー依頼によりa1/b1の同一入力対照と、3回の診断用固定飛行を実施。主因の強い証拠は**GazeboのIMUを平均せず200Hz瞬時値で読むこと**。同一a1飛行/画像/無雑音IMUの200Hz→1000Hzで位置RMSE41.609→7.387m。1000Hzの直前5ms区間平均を200Hzで入力すると6.020m。更新回数を増やさず改善。実機200Hz一般の欠陥ではない。
- 距離/移動量のスケールが大きく崩れている。別の診断a1/b1で整合する特徴点の推定距離が真値三角測量の中央値2.296/2.634倍。b1の経路は事後の縮尺補正だけで49.719→3.303m（主要精度表には補正を使わない）。画像ノイズ除去・広角・far60だけでは全区間の改善が一貫しない。
- 1000Hz無雑音でもbias推定を動かすと回転9.292degが残る。診断上の正しいbiasゼロ固定なら0.137deg、位置6.478m。実運用でbiasを固定する提案ではない。nativeの雑音あり条件の1000Hz/区間平均全系統再実験はまだ行っていない。b1の更新頻度比較も未実施。通常の既定値は変更していない。
- 出力 `native/logs/unipilot-drift-cause-20261015/index.html` / `plots/diagnosis.pdf` / `summary.json`。設定・入力SHA・原条件byte一致・各対照結果・未確認点を保存。更新頻度比較のnativeオンライン503.954mも除外せず記載。異なる物理飛行の基準値を混ぜない。高レート参照は1標本欠損（最大2ms）、元画像/外部変換は全再生でSHA一致。
- 診断道具はactive_perception/{diagnose_unipilot_drift,audit_recorded_imu,audit_feature_geometry,audit_imu_integration,average_reference_imu,report_unipilot_drift}.py等。`--diagnostic-sensors`は既定OFF。全検証プロセス停止済み。詳細と全条件の数値はWORKLOG末尾。


## 2026-10-15 最新：IMU修正を通常経路へ採用、hover静止仮定を撤去
- ユーザー目的はgimbal比較の前提不具合除去。UniPilot run/campaign/gimbal生成の既定はIMU1000Hz（`imu_sampling=resolved`）。雑音stdと量子化幅を√5倍にし時間当たり雑音密度を保持、bias分布/時間変化は保持。旧200Hzは明示`--imu-sampling legacy`。DRAGON profileは不変。
- UniPilot既定初期化は`--stationary-mode wait --stationary-duration 3`。3秒待って標準ROVIOの加速度初期化を行い、動いているhoverの角速度平均をbiasへ代入しない。旧meanは明示時だけ。biasをゼロ固定するわけではない。
- 1000Hzのnative雑音ありa1/b1全飛行を取得し、同じ入力・初期化時刻でmean→waitを評価。位置8.796826→6.787755m / 18.778818→6.764554m、回転2.741588→3.251510deg / 2.637104→5.029648deg。位置は改善、回転は悪化。成績合わせではなく成立しない静止仮定を除く修正。最終2区間の数値は同一入力再生。オンラインmean出力は再生とbyte一致。
- 最終wait設定の可動60秒も別途オンライン完走、IMU/画像欠損0、画像時刻の実測外部変換一致。モデル座標/距離/非等方相関共分散のgimbal変換検証も成功。
- 尺度残差はa1特徴距離比中央値1.196、b1 .987で区間依存。K/画像サイズ/外部変換/距離・逆距離の単位に不整合は未発見。深度初期値や重みの最適化、真値による倍率補正は未採用。残る誤差が全て手法の限界とは断定しない。
- 出力`native/logs/unipilot-imu-scale-audit-20261015/index.html` / `plots/comparison.pdf` / `summary.json`。入力は`unipilot-imu-production-fix-20261015`、最終可動確認は`unipilot-imu-final-gimbal-check-20261015`。次の固定/可動比較は両方resolved+waitを共通使用。WORKLOG末尾参照。検証用シミュレーションは停止済み。

## 2026-10-16 推定器の前提検証・最終prefix再取得中
- ユーザー依頼は残件（初期化後の回転誤差、独立した生画像/距離検証、代表3run固定vs可動比較）をすべて完了すること。a1/b1の原因切り分け・全長同時比較は完了。集約`native/logs/unipilot-estimator-validation-20261016/`。通常はresolved1000Hz+wait3秒、DRPM融合OFF、性能合わせの重み/深度調整なし。
- 公式視覚更新/ジンバルhook無効との固定再生はa1/b1で全出力byte一致。画像対応を独立追跡した結果、一部の誤対応と、整合する点にも距離推定誤差。参照無雑音IMU+真のbiasゼロ固定の診断で回転約.09〜.10degへ減るため、画像更新とbias推定が結び付く経路を確認。通常経路にbias固定/真値入力は導入していない。合成画像の機体静止・ジンバル回転試験も合格。
- paired a1: ROVIO位置固定10.292→可動32.181m、MIMOSA+ROVIO2.156→27.009m。b1:14.793→2.223m、22.317→9.242m。全長同一物理飛行/同一IMU、追従区間の周期欠落0。a1は悪化、b1は改善。各1回の比較。
- 初回prefix全長はIMU35標本欠落で除外。2回目は既知の急な曲がり角route327.835sで追従.182041mが上限.18mを超え停止。2試行の設定/CSVは`unipilot-validation-paired-20261016/rejected_prefix_input_loss`と`rejected_prefix_tracking`。raw入力は後述の明示削除で除去。
- 通信の高頻度IMU/encoder履歴を2000件へ拡大し、メッセージ内容が元adapterと一致する試験は合格。3回目prefixだけ既存`--corner-speed-limit`を有効化（経路形状・上限不変、角だけ減速）。現在`unipilot-validation-paired-20261016/run1_prefix_active`へ再取得中。固定/可動は同じ実飛行を共有。全入力自動監査→固定公式処理との再生一致→全3run報告の自動検証が待機している。
- 容量不足を受けユーザーが余計なファイル削除を明示。元Driveの29MCAPのMD5を再照合後、再生成できる古い比較.bag/npz、画像cache、今回不採用prefixのraw入力を計66件19.263GB削除。原本ローカルbag69GiB・元MCAP・結果CSV/図/設定・コードは保持。Windows実空き.6→9.7GB。削除台帳`unipilot-estimator-validation-20261016/cleanup_manifest.json`。
- 最終prefixは初期化/approach中sim26.166→26.168秒のIMU1標本欠落を記録。追従評価の開始は約42sなので評価対象外だが、隠さずfull_stream_imu_gapsへ記録する。現時点の追従中欠落0。残件はこのprefix完走/最終監査/報告更新と全プロセス停止。部分結果を全3run完了や全条件の無欠陥と断定しない。

### 2026-10-16 続報：再送可能な受信へ修正してprefix取得中
- 上記3回目prefixは追従中にも1標本欠落を検出し停止・不採用、rejected_prefix_best_effortに保存。rawを明示整理、削除合計23.423GB。保存が必要な旧診断画像は内容のSHA一致を確認して可逆圧縮し、追加7.723GB削減（storage_compression.json、各フォルダに復元手順）。
- ros1_bridge既存parameter_bridgeはYAML QoS指定が送信側だけに効き、ROS2受信はbest-effortのままだった。公式変換factoryとloop防止を使うreliable_bridgeを独立ビルドし、送受信両方をreliableに変更。build_reliable_bridge.py、performance/reliable_bridge.cpp。往復各2万標本の完全一致/欠落・重複ゼロ、実飛行8系統の送受信QoS一致を確認。
- 現在run1_prefix_activeをこの通信修正で取得中。全3run/全6系統の最終報告は初期化前からのIMU欠落ゼロも必須とする。a1/b1の過去rawを再監査し、4系統とも全時間欠落0を確認済み。最終prefix完走後に監査・公式視覚更新固定版との再生一致・図更新を実施する。

## 2026-10-16 最新：残件検証・3run比較完了、全プロセス停止
- 上記「取得中」の続報は終了。最終prefixは信頼性を指定した通信で全1070 route秒完走（wall867.977s）。固定/可動を同一飛行/同一IMUで比較。全3run/6条件でIMU全時間欠落0、画像・外部変換監査合格。固定公式視覚更新の出力もprefixを含めbyte一致。
- 位置RMSE固定→可動：a1 ROVIO10.292→32.181m / 融合2.156→27.009m、b1 14.793→2.223m / 22.317→9.242m、prefix58.788→37.856m / 69.464→18.558m。prefixは曲がり角だけ既存先読み減速。各経路1試行。DRPM OFF、resolved1000Hz+wait3秒を共通適用。初回の位置姿勢整合だけ、尺度fitなし。
- 入力・初期化・座標・ジンバル補償の今回の検証は完了。残る誤差は公式視覚更新でも再現し、画像対応/距離推定/IMU bias推定の結び付きの影響を診断した。全上流実装・未知条件の不具合不在を数学的に証明したという意味ではない。性能合わせの重み変更や真値入力は通常系へ入れていない。
- 結果入口 native/logs/unipilot-estimator-validation-20261016/index.html、六軸6ページplots/comparison.pdf、原因切り分け4ページplots/diagnostics.pdf。completed状態はcompletion_status.json。現状の固定/可動比較はこの結果を使用する。
- prefix診断参照記録器だけ評価終了後のshutdown例外で終了1。推定器とシミュレーション等は全て0、補助記録器を通常入力に接続していない。独立参照imu.csvは区間末尾まで欠落0。imu_fastは3欠落で使用せず。診断記録器の終了競合を修正し、受信中SIGINT/SIGTERMの3試験成功。例外/元runtimeは消さず報告。
- 不要ファイル23.423GB削除、保持データ可逆圧縮7.723GB削減。原本MCAP/ローカルbag69GiB、精度結果/図/コードは保持。一部旧診断rawは.bin.zst、各フォルダのRESTORE_COMPRESSED_INPUTS.txtで復元（展開SHA検証済み）。全所有シミュレーション/ROS master/再生/GPU keepaliveは停止済み。

## 2026-10-17 進行中：ジンバルの安定的改善
- 現ユーザーはROVIO+MIMOSAで固定より安定して改善するまで自走を依頼。未達、終了しない。サブエージェントはdeveloperにより不可。GUI/bag/連続画像保存なし（画角診断PNG数枚のみ）。Windows /mnt/c実空き約10GB、2GB停止guard。
- 全候補/台帳: native/logs/gimbal-improvement-20261017。通常設定は未変更、tracking_aware_v1〜v4.yamlで実験。v1は成功追跡/鮮度/25特徴上限/現在特徴保持、v2は0.8秒先の可視性予測、v3は意図的走査だけ5deg/s・特徴不足1秒で前方復帰、v4は機体回転を2関節の角速度へ射影して補償。最大関節速度20deg/s、物理yaw±50/pitch±90。
- 旧DRAGON融合共分散が残っていた。過去の有効native固定a1/b1だけから50ms相対誤差を等重み校正したnative_fusion_calibration.jsonを--fusion-calibrationで両側へ共通適用。今回の評価出力は校正に不使用。MIMOSA位置は1m未満の例が多い。全時間相関/共有IMU相関は厳密には未処理。
- 通信2件修正: reliable_bridge ROS1名を公式Factoryのloop除外名/ros_bridgeへ一致（各方向20000標本+echo検査成功）。Gazebo Transportのzmq_msg_sendがEINTRで生成済みIMUを捨てる件は、simulationだけLD_PRELOADで同一part再試行。実ソケット/OSシグナル2000multipart試験で欠落重複0。HWM増加は効かず撤回済み。EINTR修正後の全飛行入力監査は欠落0。
- 正式監査合格の開発結果（固定→可動、位置RMSE）: v2 b1はROVIO11.457→2.937m、融合.641→.479m（回転は.184→.245degへ悪化）。v3 a1は8.327→6.475m、.617→.587m。v4 a1は6.873→8.994m、.540→.477m（ROVIO回転2.953→13.980degへ悪化）。過去の欠落あり試行はresults.json valid=falseで保持、成功扱いしない。
- 現在v2-seed17 prefix/domain84とv4-seed17 b1/domain85を実行中。v2 prefixは途中で固定より大きく悪化、完走待ち。v4はb1→prefixを順に行う。候補凍結後の未使用seed18/19・3経路反復は未着手。事前基準validation_protocol.json。今の単発結果だけで安定改善を主張しない。
- core.py候補評価を一括計算へ高速化。50ケースで全候補の評価差最大1.58e-14/操作一致、14試験成功。単体81.3→8.84ms、実飛行中央値約17ms（旧約170ms）。v4 a1は旧計算、b1から新計算。元コードは各flight/implementation/core.pyへ保存。vectorization_validation.jsonに検証とSHA。
- 現実行のcommandはcampaign.jsonに保存。完走後report_gimbal_campaign.py CAMPAIGN --plotsで検証/6軸図。省略・縮尺合わせなし。終了/中断時は所有シミュレーション/master/GPU keepalive全停止。詳細WORKLOG末尾。

### 2026-10-17 続行時の更新（上の「現在実行中」を置き換える）
- 依頼はまだ未達。v1〜v6の単発試験を繰り返しても安定改善を証明できていない。v4はa1融合.540→.477m/b1 .510→.683m/prefix9.373→8.841m、v5は全体不採用（prefix9.074→111.280m）。v6旧a1はRAM guardで末尾1.6秒前に中断した部分結果、成功扱いしない。以後sim最大2並列、長いprefix2本は時間をずらす。全失敗CSV/設定は保持。
- 比較の原因分離を追加。両カメラを同じ向きに固定しても別画像の微小画素差/初期化でROVIO差が出る。--shared-initializationは同じ実測視野の間だけ元の固定画像/外部変換をそのまま両ROVIOへ送り、最初の偏向時から不可逆に可動の元画像へ切替。真値/画素変更/時刻変更なし。通常系には影響しない。3単体試験成功。
- 同一画像でROVIOがbit一致してもMIMOSAだけ差が残った。初期IMUの平均区間と因子到着順の差が原因。experimental --ordered-fusionをIMU/LiDAR/VIO全ての取得時刻に揃える方式へ修正。IMUは各観測時刻までだけ元callbackへ渡す。同期時刻/offset=0の現在simulation専用、通常nodeは変更しない。60 route秒のnull対照でROVIO1559標本/MIMOSA1568標本の全位置姿勢がbit一致（ordered-imu-null-a1-seed17/null_reproducibility.json）。因子/標本/重みは変更しない。以降の固定/可動比較は共通初期状態とこの配送方式で統一。
- --initialization-excitation（実験専用）追加。評価前26秒にX/Y±25cm、Z±10cmの滑らかな往復。固定/可動が同じ運動・同じ正面視野を共有し、評価区間は以前と同じ。尺度の初期推定に必要な加減速を与える。機体+追従誤差上限が既存経路の認証包絡内と検査。推定器への真値/指令入力なし。
- 現在 v6-ordered-excited-seed17でa1/domain85、b1/domain86（その後a1側でprefix）の開発比較。前半では両方改善しているが全長結果未確定。未使用seed18/19は未着手。候補を凍結しvalidation_protocol.jsonの事前基準で検証すること。planner_nodeの診断列だけ途中追加、既存a1/b1動作は不変、次のprefixにはbody速度/角速度/弱方向を数値保存する。
- Windows実空き約12GB、当タスクログ約1.3GB、連続画像/点群/bagなし。再生成可能な合成raw約146MBは再生SHA確認後削除、結果/生成コード保持。容量guard継続。WORKLOGが詳細台帳。

### 2026-10-17 最新：v6不採用、v7を開発比較中
- **目標未達。終了しない。** ユーザーは安定改善まで自走を依頼。v6を固定してseed18/19反復を行ったが、seed18 a1融合6.300→.793mは改善、seed18 b1 4.399→4.525mは悪化、seed19 a1 3.433→6.253mは悪化。最悪比1.821>1.5、a1平均比.974>.9、残り全勝でも5/6勝不可のため早期不採用。v6_early_rejection.json、root index.htmlに明記。実行中seed19 b1/seed18 prefixはscheduler SIGINTで正常停止し部分結果を保持、seed19 prefix未開始。continue_gimbal_validation.pyは終了、勝手に再開しない。
- v6開発prefixも完走・監査合格だが、融合16.119→30.427mで悪化（ROVIO75.438→58.360m/29.209→15.817deg）。3経路を通じた安定改善は得られていない。全ソース/校正/binaryはfrozen_v6_sources.zipに元SHA一致で保存。
- **現在v7-seed17のa1/domain85,b1/domain86を実行中。** CLIは各campaign.jsonに保存。共通画像初期化、26秒準備往復、native融合校正、時刻順IMU/LiDAR/VIO、DRPM融合OFFを維持。core.pyのv7フラグ transition_aware=trueのみ制御変更。
- v7の根拠: 従来の90%特徴保持制約が「最終目標角」に掛かり、速度制限後の次回1degで全特徴を保てる場合まで37.5deg目標として拒否していた。現在は次の到達可能角で保持制約、将来保持も到達可能な0.8秒後の角度で評価。保持不能なら最も保持する候補を選び診断へ記録。情報の式/重みはv6のまま。20試験合格、command単体/一括2000ケース最大差0。
- 次: v7 a1/b1全長結果→必要ならprefix、うまくいく候補を凍結して新しい未使用seed20/21で反復。18/19は既に見たため、新候補の独立検証として再利用しない。事前の基準自体は維持。失敗を隠す/評価区間を切り詰める/基準を緩めることはしない。
- Windows実空き約12GB。ログは画像連続/点群/bagなし。完了26encoder CSVを可逆gzipにし全展開SHA一致後に元コピーを削除、402MB削減。report_routes.loadは.csvがなければ.csv.gzを読む。各フォルダRESTORE_ENCODERS.txt。入力元bag69GiB/結果CSV/設定は保持。終了時は全所有sim/master/GPUを停止。詳細WORKLOG。

## 2026-10-17 ジンバル改善・進行中（v8）
- ユーザー目標は安定的な改善まで自走。未達なので完了扱いしない。v6/v7は未使用seed反復で不採用、元設定と全結果を保持。v7 seed20 a1は固定融合3.395m→可動29.859m、入力/共通初期状態/コード一致監査合格。v7_early_rejection.json。seed21はまだ未使用。
- 現在v8-seed17のa1/b1をdomains85/86で同時比較中。tracking_aware_v8.yamlは実験用。画像角点180の追跡+LiDAR深度（対応がある場合）+時系列三角測量で偏向用の地図を作る。地図の姿勢はMIMOSA、取得時刻の回転はIMU積分、未来の角速度は0.3秒平均。ROVIO/MIMOSAへの因子/センサ入力/重みはv7と同じ。通常既定値は変更していない。
- UniPilotのLiDARは後方半球、前方カメラとの重複が乏しく、v8-smokeの同時刻LiDAR深度対応は0。三角測量の準備運動で82/116地図点を確認。短いsmokeは機能確認だけ。精度改善は未確認。
- 成功基準は従来通り3経路×2未使用seed、6中5勝、経路平均比≤.9、全体≤.8、最大比≤1.5。新候補では既に調査したseed18/19/20を未使用条件に再利用しない。汎用化したreport_gimbal_improvement.py --candidate vNで凍結したコード/校正/実行ファイルを各飛行の保存物と照合。
- 結果root: native/logs/gimbal-improvement-20261017。新規bag/連続画像点群なし。Windows /mnt/c 空き約9.36GiB、2GB停止guard。完了encoder圧縮36件、約549MB削減。既存bagは保持。

### 更新: v8の制御間隔問題を修正しv8b再取得中
- v8の完了3試行は偏向指令間隔1.4〜9.15秒の途切れがあり、採用用の有効比較にはしない。既存の入力/初期状態監査とは別の不具合。report_gimbal_campaign.pyは全候補共通で追従中の指令間隔≤planner.max_dt(.5秒)と例外なしを検査する。
- v8bで1000Hz IMU/encoder受信lockを画像地図・探索から分離。方向評価式は同じ。現在campaign v8b-seed17がa1/b1/prefixをjobs2で実行中。旧v8のprefixとseed20 b1は停止済み。v8bの結果と指令間隔をまず確認する。成功目標は未達。

### 最新: v8c-seed17を実行中
- v8bも姿勢等の受信が画像計算に阻害され、指令間隔1秒超になったため中断。現行v8cは画像地図を専用worker、最新1組待機、IMU/encoder独立受信、三角測量バッチ化。コアの向き評価式/重みはv8と同じ。
- campaign v8c-seed17（domains85/86、jobs2、a1/b1→prefix）が現在の所有シミュレーション。旧v8/v8bはすべて停止済み。センサ入力と推定器入力は間引かず、偏向用地図だけ最新フレームで追いつく。geometry_coalesced/geometry_failureを監査。
- 目標はまだ未達。3経路の有効完走と、未使用seed21/22等で事前基準を満たすまで安定改善とは報告しない。seed20は既知開発条件。

### 最新: v8d-seed17を実行中
- v8cはa1/b1完走したがb1で指令間隔.539秒が1回あり、基準.5秒は緩めず無効。prefixは初期化中に停止。
- 現行v8dは同じ方向評価式に、厳密に同値な遮蔽探索高速化とplanner完了待ちsimulation pacingを加えたもの。core.py indexed_occlusion、occlusion_queries.py、pace_simulation.py --planner、unipilot_run.pyのpreset pace_planner連携。物理dt/センサ頻度不変、処理待ちは実時間側。
- 今動いているのはv8d-seed17（a1/b1→prefix、domains85/86）。v8c以前は停止済み。未使用seed21/22の独立検証は未実施、目標未達。

### 最新: v8e-seed17を実行中
- v8dはplannerのROSタイマーとシミュレーション停止の相互待ちで失敗。全停止済み。v8eで停止中にも未完計算を実時間側から再試行し、成功指令の頻度のみsim時間5Hzに制限。last_planは成功時のみ更新。停止中再試行/重複なしを含む6試験成功。
- 現在のcampaignはv8e-seed17、domains85/86、jobs2、a1/b1→prefix。方向選択/推定器条件はv8dと同じ。目標未達、未使用seed21/22検証は未着手。

### v8e進捗
- seed17 a1/b1全監査合格。融合位置RMSEは5.715→2.195m、6.282→1.874m。a1融合回転.416→.572deg、b1 ROVIO13.291m/16.706degは悪化なので全指標改善ではない。
- 現在v8e-seed17のprefix/domain85、v8e-seed20のa1→b1/domain86。どちらもjobs1相当、全体2simまで。prefixは1070 route秒。候補はまだ凍結していない。方向評価式・共分散を動かさず、この結果を待つ。

### v8e候補固定（独立検証はこれから）
- seed20 a1も有効完走、融合2.543→1.781m/.422→.390deg。開発段階の3短試行は全て融合位置改善。
- frozen_candidate_v8e.json / frozen_v8e_sources.zipに24ファイルを固定。未使用seed21/22、3経路×2、元の基準。制御/融合/モデル生成元をこれ以上変更しない。変更が必要なら候補を不採用とし、新しい未使用seedで次候補を検証する。
- v8e-seed17 prefix/domain85とv8e-seed20 b1/domain86は進行中。次はseed20 b1終了後にvalidation-v8e-seed21のa1→b1をdomain86で開始し、continue_gimbal_validation.py --candidate v8e --seeds 20261021 20261022 --development-prefix v8e-seed17で残りを最大2sim・長い2本はずらして進める。prefix開発途中の数字を完走結果と扱わない。目標未達。

### 未使用seed21/22反復を開始
- 現在: dev v8e-seed17 prefix/domain85、validation-v8e-seed21 a1→b1/domain86。
- scheduler continue_gimbal_validation.py --candidate v8e --seeds 20261021 20261022 --development-prefix v8e-seed17 --separate-prefix が稼働中。dev prefix終了後にvalidation-v8e-seed21-prefix/domain85、seed21短2本終了後にseed22短2本/domain86、seed21長区間750秒以降かつseed22短終了後にvalidation-v8e-seed22-prefix/domain86を開始する。計2sim以下。手動で追加起動しない。
- v8e-seed20はa1/b1とも完走済み。a1数値は上記、b1の全結果集計中。反復開始時に固定24ファイルSHA全一致を確認。目標未達なので結果確認・必要な改善を継続する。

### 開発prefix悪化、独立検証継続中
- v8e-seed17 prefix1070秒完走/全監査合格だが融合12.361→17.317m、3.869→4.636degで悪化。ROVIO78.172→63.418mは改善。まだ安定改善未達。
- validation-v8e-seed21 a1は完走/全監査/凍結コード一致、融合3.232→2.924m（比.90469）/.396→.437deg。独立6組のうち1組のみ完了、1勝。設定は凍結のまま。
- 現在validation-v8e-seed21 b1/domain86（campaign所有）、validation-v8e-seed21-prefix/domain85（scheduler所有）。schedulerは次にseed22短2本、seed22長区間を自動起動する。元の成功基準は緩めない。開発prefixの失敗原因は診断してよいが、凍結候補の挙動は変えない。

### 最新: v8eを制御入力の修正で中止、v9開発中
- **目標未達・自走継続。** v8eの保持制約は画像地図更新の遅れで追跡点を除外し、prefixで86%の判断が空の保持入力だった。v8e_track_freshness_diagnosis.json。v8e反復は2短区間成功後に修正目的で中止（性能の閾値失敗と偽らない）。v8e_early_rejection.json、全結果/設定保持。
- 全v8e/scheduler停止済み。seed21 a1/b1完走、prefix途中、seed22は未起動。未使用候補は22/23。古いschedulerを再開しない。
- 現在v9-seed17のみ、prefix/domain85とa1→b1/domain86の2sim。tracking_aware_v9.yamlはdirect_track_input=true。最新ROVIO追跡視線を直接受信し、実測回転/融合速度で現在へ移す。地図workerの時刻と混同しない。古い/未受信ならhold、実際に特徴0なら0として区別。8試験成功。方向スコア/重み・融合設定は同じ。まず保持入力のage/countと連続制御を検証し、3経路の精度を確認する。

### 最新: v9とセンサ由来IMU設定の追加比較
- 現在v9-seed17 prefix/domain85（残り約400秒）、v9-sensorimu-seed17 a1→b1→prefix/domain86（起動直後）。schedulerは稼働していない。この2campaignだけ所有。
- v9 a1は全監査合格だが融合5.297→7.078mで悪化。v9 b1は完走、集計中。v9 prefix625秒途中は融合7.834→12.135m、未完なので確定値ではない。v9の保持入力修正自体は8試験/実際の入力監査で確認済み。
- MIMOSA白色IMU雑音の既定.8/.01は実SDF由来.00148842758/.000636416563より振幅537/15.7倍。原因と断定せず、--fusion-imu-noise sensorでこの2値だけ両推定器へ共通適用して比較。bias過程/初期prior/VIO重み/実センサは不変。legacy既定で既存試験の挙動を維持。4試験成功、fusion_imu_noise_audit.json。現在の通常設定には未採用。
- v8eは中止/凍結保存済み。seed21は使用済み、seed22/23は未使用。新候補はまだ凍結しない。全失敗も保存し、安定改善が得られるまで継続。Windows空き約10.7GB、2GBguard、GUI/bag/連続画像なし。

### 最新: v10開発比較を開始（以前の実行中記述を置換）
- 現所有campaignはv10-seed17のみ。session83245、prefix/domain85とa1→b1/domain86。最大2sim、GUI/bag/連続画像点群なし。古いcampaign/schedulerは停止済み。
- v9は3区間完走/全監査合格だが、融合a1 5.297→7.078m、b1 4.472→.743m、prefix12.681→21.858mと混在。安定改善未達。sensor白色IMU設定はa1途中で融合が固定86.678m/可動56.372mへ悪化したため採用しない。legacyを維持。
- sensor試行はpacer再開条件の永久停止も発見: 5Hz判断の正常時刻差.136sに対し再開閾値.1sでは追いつけない。再開1.25周期/停止1.5周期へ修正。2再現試験成功、v10で実走確認する。
- v10はv9の式をlog(追加前投影分散/追加後投影分散)へ変更するopt-inのみ。他の重み/保持/制約/センサ/融合は固定。新旧式と23関連試験成功。共通初期画像/26秒準備/取得時刻順融合/同一真値経路/欠落監査を維持。
- 次に3区間の全長結果、保持入力とpacer復帰を確認。候補が有望なら凍結し未使用seed22/23を使う。v6/v7/v8e全失敗を保持、成功基準を緩めない。Windows約10.1GiB、encoder圧縮60files/926MB削減。

### 最新：描画距離打切りの人工輪郭を切り分け中
- 現在2campaignのみ: v10-seed17 prefix/domain85（session83245、route約850/1070s）、v10-far200-seed17 a1→b1/domain86（session73423、a1約70/228s）。他のsim/schedulerなし。20mのv10はa1融合3.904→5.242mで悪化、b1 9.531→2.363mで改善。全監査合格。prefix未完。
- 保存した生画像とGT+encoderの幾何は偏向中も一致（median .0247px、11候補の時間offsetで0msが最良）。audit_camera_motion.py/camera_motion_audit.json。gimbal対応そのものの大きな時刻ずれはこの検査でなし。
- 新しい根拠: 独立画像追跡で静止構造に合わない観測のうち、a1 90.09%、b1 85.47%が白い20m描画打切り境界8px以内。一貫した観測では34%/8.5%。audit_clip_tracks.py、clip_boundary_track_audit{,_b1}.json。ROVIO単独の距離・姿勢誤差に人工境界が寄与する経路を調査中。
- v10-far200は描画距離だけ200mへ変更する既存CLIを使用（それから導出するplanner最大距離も20→200）。固定/偏向とも同じ。実画像は残す数枚だけ、連続収録なし。解像度/画角/雑音/IMU/初期化/融合は不変。通常既定は20mのまま、結果前に採用しない。途中70秒では固定融合1.330m→可動.793m、ROVIO回転.377→.871deg。未完なので成功判定には不使用。
- 20m prefix終了後は新しいfar200 prefixを別folder/domain85で開始する候補。同じcampaign.jsonへ同時書込み禁止。新規2つの長飛行を同時初期化しない。既知seed17は開発用、未使用seed22/23の凍結検証は未着手。報告器にprotocol_fields照合を追加したので、次の凍結にはcamera_far=200/fusion_imu_noise=legacy等の実行条件も含める。
- 目標未達。Windows実空き約10GB、RAM available約4GB。所有simを最大2に維持。終了/中断時全所有process停止。詳細WORKLOG。

### 現在の実行（v11、描画範囲1000m）
- 所有はv11-prefix-seed17/domain85/session35141、v11-seed17 a1→b1/domain86/session10767のみ。far200/session73423と20m/session83245は全完了停止済み。追加のsimを勝手に起動しない（最大2）。
- v10(20m)3区間全監査合格、融合位置固定→可動a1 3.904→5.242m、b1 9.531→2.363m、prefix16.223→17.801m。不採用。
- v10-far200はa1/b1両方完走合格、融合a1 4.768→3.571m、b1 15.966→.253m。ROVIOはa1位置12.870→10.868m/回転.644→3.716deg、b1位置27.236→7.244m/回転3.956→3.583deg。全結果保持。
- 正式な次条件v11は世界全体の対角636.37mを越えるcamera_far1000。31モデルのvisual/collision幾何/姿勢一致、world_render_extent.json。通常既定camera_farは未変更。制御式はv10のlog投影分散比と同一。未使用seed22/23の独立検証はまだ開始しない。
- v11 prefix途中183秒の融合固定.274/可動.832mと悪化、未完のまま採否を決めない。全1070秒を確認。v11短区間の結果もまだ未確定。
- 方向確率を一律縮小しても正規化で相殺される設計の問題を診断ファイルへ保存したが、制御の変更は保留。候補を同時に変更して因果を混ぜない。feature_lifetimes_early.jsonはsim145.4〜205.35秒に両方1200frame、保持98%（詳細WORKLOG）。
- Windows実空き約10.7GiB、完了encoder圧縮64件で約1.018GB削減。新bag/連続画像点群なし、今回の画像保存は数枚のみ。目標未達なので完了扱いせず継続。

### 最新: v11は2短区間改善・長区間悪化、v12実走中
- ユーザー目標未達。全v11終了/全監査合格。a1融合2.995→2.257m、b1 23.031→19.875m、prefix4.220→7.567m。ROVIO単体a1 8.465→10.623m、b1 37.047→33.776m、prefix39.127→63.353m。安定改善ではない。
- 現所有simulationはv12-seed17 a1→b1/domain86/session44865、v12-prefix-seed17/domain85/session73548のみ。最大2sim、GUI/bag/連続画像点群なし。旧v11セッション10767/35141は終了済み。schedulerは起動していない。
- v12はcoreの方向別分散比logに退化確率を後から掛ける方式。退化が小さい時も利得が不変となる正規化の性質を改めた。tracking_aware_v12.yaml、uncertainty_gain=directional_log。他の重み/推定器条件は同一、camera_far1000、legacy白色IMU設定。26関連試験成功。current_view診断も追加、現視野と目標の評価を比較可能。
- 未使用seed22/23は未使用。v12全長結果を確認し、有望なら凍結して反復する。成功基準は変更しない。
- planning_map_visibility_audit.jsonでは失った視野内特徴の減衰重み比中央値.009、90%点.151で、これが主要因とする証拠には不足。photometric_scoring_diagnosis.jsonでも2枚の新規角点の画素雑音は現2px床値未満。未確認の仮説を修正済みとしない。
- v12-seed17のlive read-only track_range_audit.jsonも取得予定。8mの保持視線仮定と実際のROVIO推定距離の違いを調査。推定器への入力なし。
- Windows実空き約9.8GB。完了encoder追加gzip中/成功時のみ元CSV削除。元bag69GiBは保持。


### 2026-10-17 19:34 UTC 自走改善の最新状態（継続中・未達）
v27短2本終了、a1融合2.481→3.089m/b1 29.479→28.641m。a1終了Gazebo -9、b1 0。後続停止。固定旧入力のROVIO25→75点/点間隔/直接補正/最下段画像の切り分けは両区間の安定改善を確認できず、既定ROVIO変更なし。結果はlogs/gimbal-improvement-20261017/rovio-*-calibration。
現在v28は低遅延成熟点注視でpitchを0に限定、yawだけ追従する比較。48関連試験成功、オンライン精度未確認。firstshort session40432(domain86)、scheduler55744、開発6組(seed24/25×a1,b1,prefix)、独立26/27未使用。停止時はfirstshortとscheduler両方を停止。実験sim最大1、GUI/生bagなし、Windows実空き8GB停止。unipilot_run終了は/server_control stop:trueを先に送り、別thread停止を試す変更（飛行検証未了）。D3D12保持後の新Windows coreなし。全失敗/設定/数値は圧縮保存。

### 2026-10-17 19:51 UTC 最新: v28s再検証中（目標未達）
- v28短seed24は融合a1 1.425→1.084m/b1 25.961→.469m、ROVIO位置も改善だが回転は悪化。Gazebo終了失敗のため完成6組とは数えない。v28後続は起動前停止。
- v28sはv28と同じyaw-only成熟点注視、pitch0。停止だけ独立Gazebo8.15 Stop mutexライブラリ＋Windows core生成防止に変更。静止3回、全子process0、元画像と完全一致。飛行後終了は未検証。停止監査は橋渡し子processにも拡大。
- 現所有: firstshort v28s-short-seed24 session87753/domain86、scheduler session1903。schedulerは残りseed25短2本とseed24/25のprefixを最大1simで実行。firstshort/scheduler両方の停止が必要。新seed26/27独立検証は未着手。
- build_serial_stop.py/build_no_crash_dump.pyで独立build。新--safe-shutdownをcampaign/scheduler/runへ伝播。通常既定では無効。追加1s融合較正は未使用、元較正を維持。
- GUI/bag/連続画像点群なし。Windows空き19.8GB、停止閾値8GB、全失敗数値保持。詳細WORKLOGとnative/logs/gimbal-improvement-20261017。

### 2026-10-17 20:08 UTC 最新（目標未達・v29実験中）
- v28sはseed24の2短区間で改善、seed25 a1で比1.97158悪化のため不採用。3完成組＋b1準備中断を保持。全部正常停止、old scheduler1903終了。
- 現在v29-short-seed24 firstshort session26158/domain86、scheduler50630。v29はyaw-onlyのままnative_fusion_window1s_calibration.jsonを使う（両側同一）。誤差相関の1秒蓄積量に合わせた実測近似。元native_fusion_calibrationは不変。
- schedulerは残りseed25短・24/25prefix、最大1sim。未使用26/27はまだ開始しない。3経路2反復の元基準を満たすまで凍結しない。終了時はfirstshortとscheduler両方停止。Windows空き19.8GB、8GB停止、GUI/bagなし。

### 2026-10-17 20:28 UTC 現在v29長経路（継続・目標未達）
- 現所有はscheduler session50630、その子first_prefix campaign PID1351652、v29-prefix-seed24/domain85。短firstshort26158完了、seed25shortも完了。次にschedulerがseed25prefixを起動。最大1sim。
- v29はv28sと同じyaw-only成熟点注視、融合だけnative_fusion_window1s_calibration.json（以前の固定記録1秒誤差/20の近似）へ。DRPM OFF、全センサ/推定器設定は両側同一。
- 短4比較全監査合格。seed24 a1融合.30359→.34042m、b1 5.14861→.45038m。seed25 a1 .33805→.32307m、b1 .31836→.27899m。3勝、平均比.76020、最大比1.12132、絶対最大悪化.03683m。a1経路平均比1.0385で元の全経路≤.9は不合格と明示済み。長経路まで調べる価値あり継続、元基準を通ったと偽らない。
- 未使用26/27独立検証未開始。今後の採用判定は低誤差区間のcm単位差と大ドリフト改善を区別する必要があるが、基準の変更を結果から隠さない。長区間結果待ち、成功は未確定。
- safe-shutdown（Gazebo8.15 Stop mutex独立core、D3D12保持、process-local no core dump）はv28s/v29の完走全て全子process0。Windows実空き19.8GB、8GB停止。生bag/連続画像点群なし、数値圧縮済み。

### 2026-10-17 20:49 UTC 最新: v30 DRPM ON 切り分け中、目標未達
- v29はprefix seed24完走/全監査合格、ROVIO73.595→15.290m/8.053→2.092deg改善、融合6.457→7.694m/3.608→4.317deg悪化。v29不採用、完成5組保持。seed25prefixは途中停止/全子process0、scheduler50630終了。v29_early_rejection.jsonとv29_progress.htmlに明示。
- **現所有:** v30-short-seed24 firstshort session28212/PID1356458(domain86)、scheduler session6106/PID1356592。最大1sim。v30はv29と同じyaw-only制御・native_fusion_window1s_calibration、DRPMを固定/可動両方ON（radial/official更新）にするだけ。最初のa1実行中、精度未確認。
- schedulerはseed25短と24/25prefixを順次実行。停止する時はfirstshortとscheduler両方。report_gimbal_campaign.pyは必ず--drpm-fusionを付ける。未使用26/27は未使用。DRPM OFF固定の絶対精度も比較し、相対改善のために悪い基準を採用しない。
- 新しい事後診断diagnose_view_covariance.pyでは、カメラ方向とともにnominal covarianceを回す仮説は短4組全て悪化し、不採用。固定側は完全同値。view_covariance_diagnosis_v29.json。真値は診断だけ、融合へ入力なし。vio_fusion_disagreement_v29.jsonはVIO対融合の連続相対姿勢のずれ（真値なし）、大外れ5.5〜11.8%で保存済み。これだけでrobust kernel導入を決めていない。
- report_gimbal_partial.pyは書込み中CSVの完全行だけ読んで途中値表示、採否に使用しない。report_gimbal_development.pyは元基準・中断・未達を明示する開発HTML/JSON生成、DRPM ON/OFFを宣言から表示。
- Windows実空き約19.8GB、8GB停止。own simulationのみcore dump防止、全最近試行で正常終了。画像/点群連続保存なし、数値と小診断だけ保持・完了ログ圧縮中。

### 2026-10-17 v31継続中
`logs/gimbal-improvement-20261017/development_v31.json`の6組を検証中。v29 yaw-only/1秒融合較正+ROVIO重力方向1Hzを両融合へ適用、DRPM OFF。v30 DRPM ONは悪い固定baselineのため不採用。安定改善はまだ未達。v31_firstshort.log/v31_scheduler.logを確認。独立26/27は未使用。Windows空き19.84GB、最大1sim、8GB保護、raw保存なし。詳細WORKLOG末尾。

### 2026-10-17 21:27 UTC 更新 — v32実行中
v31は短区間1勝1敗（b1比2.94）で不採用、長区間も約715秒で傾き増大を確認して診断停止。全子終了0、partialは完走扱いしない。v32は同じyaw-only制御、ROVIO移動量＋重力方向へ融合変更、DRPM OFF。旧固定2飛行から重力方向sigma .11853deg、移動量1秒/20較正を使用。`development_v32.json`が条件、`v32_firstshort.log`/`v32_scheduler.log`が進行。firstshort session59728、scheduler89715。最大1sim、Windows空き19.84GB、8GB保護、画像/点群保存なし。安定改善は未達、独立26/27は未使用。

### 2026-10-17 v32s開始
v32短区間の融合位置はa1 .445→.340m、b1 5.316→.242m。ただしa1のROS launch親のみ終了-9（実Gazebo/bridge全0）で元監査不合格。自動OnProcessExit Shutdownと親SIGINTの重複を避ける修正をunipilot_runへ追加し、親5回/子10個の小さいlaunch終了試験合格。推定/制御は変えずv32sで6比較24/25を再宣言。`development_v32s.json`、`v32s_firstshort.log`、`v32s_scheduler.log`参照。安定改善はまだ未達。未使用26/27を保持、Windows19.84GB/max1sim/8GB保護/rawなし。

### 2026-10-17 最新 v33s（目標未達・自走継続）
- 重要訂正: MIMOSA use_estimated_gravity=falseは元実装で未使用。常に可変gravity因子/予測。過去absolute_tiltの数値はworld-Z診断であり内部推定重力の誤差とは違う。JSONの数値を残してinterpretation_correctionを追記済み。位置/姿勢RMSEは変更なし。
- v32s a1は全長/全監査/実Gazeboとlaunch全0、融合.425675→.387069m。しかし上記frame前提の不整合修正を優先しb1準備中停止。v32/31/30/29は採用していない。
- 新build/mimosa-fixed-gravity/libmimosa_lib.soはfalse時の標準ImuFactor・2予測経路で固定gravityを使用、trueは元通り。30例の残差/予測/Jacobian比較PASS。元lib保持、同じprefetch mapobjects。
- 現所有: v33s-short-seed24 firstshort session23037/domain86、scheduler26191。最大1sim、開発6組24/25、独立26/27未使用。v32と同じyaw-only、visual displacement+実測重力傾き1Hz、DRPM OFF。新fixed-gravity libを両側へ同時適用。
- record_gravity_state.pyが全受信でgravity固定を監視し1Hz CSV保持。最初v33はfloat32の9.8丸めを監査異常とした初期化失敗で停止、軌道データなし。期待値をC++floatと同じにしたv33sで再試行、元失敗保持。
- ROOT logs/gimbal-improvement-20261017、native_fusion_displacement_gravity_calibration.json。停止時firstshortとscheduler両方。Windows実空き19.84GB、8GB下限、raw画像/点群/bag保存なし。完了ログのみSHA検証圧縮。次は実gravity固定/入力連続性、短区間精度、長区間精度。

### 最新 v33s長区間中、追加の不確かさ診断
- v33s短4組全監査/終了合格。融合固定→可動: a1/24 .391733→.372343m、b1/24 5.696168→.291546m、a1/25 .579776→.340155m、b1/25 .328→.530m（比1.61537）。3勝1敗、元の最大比1.5不合格。勝敗/閾値を書換えない。
- scheduler26191がv33s-prefix-seed24/domain85を実行中。その後seed25prefix。firstshort23037完了。最大1sim。独立26/27未使用。
- prefixの追加read-only ROVIO covariance probe session44114が1Hz CSVを記録、owner runtime作成で自動終了。最初のtopic誤記分はstartup別名保持、現在active/fixed両方記録。推定/制御には渡さない。
- logging-only別ROVIO build session63952 (build_rovio_uncertainty.py)。完成後 replay_rovio_capacity.py --uncertainty-log --features 25 --source logs/unipilot-validation-paired-20261016 --output 新規folder で元の固定較正2飛行再生。元16列は文字単位一致必須、追加列はbody velocity/pose variance/velocity covariance。生入力コピーなし、通常binary不変。まだ再生未開始。
- v33s_b1_seed25_axes_diagnosis.json: 融合悪化は主に通路x、VIO自身の局所1秒移動量誤差は可動側で全軸改善。短時間pose差をVIO移動量とするモデルと公表速度/不確かさを調べる計画。新しいvelocity factorなどは未実装、原因断定なし。

### 最新 v34（目標未達、速度を直接融合する方式を開始）
- v33sは短4組3勝1敗/最大比1.615>1.5で不採用。prefix24は710.681秒で診断停止、全子正常終了。prefix25未開始。旧scheduler26191/firstshort23037/probe44114/replay57318は全停止。独立26/27は未使用。
- 現所有v34-short-seed24 firstshort session93826/domain86、scheduler13559。a1→b1、その後25短と24/25prefix、最大1sim。停止する時両方止める。
- 同じyaw-only制御、同じ物理/センサ、固定gravity、DRPM OFF。--tilt-fusion --velocity-fusion --fixed-gravity-imu、--tilt-translation は付けない。calibration native_fusion_velocity_gravity_calibration.json。
- build/mimosa-velocity/mimosa_velocity_nodeはROVIOのbody速度と公表6x6twist covarianceを固定外部変換/取付位置でbody原点へ変換し、X,V状態に1Hzの速度因子。世界位置・yaw・pose差分は渡さない。ROVIO絶対重力方向1Hzは維持（sigma.00206877rad）。状態自体は元と同じ20Hzで作るが、観測なし時はMIMOSA graphがpublishしないため出力は通常LiDARの10Hzと追加観測時。監査上限.10001sは既存のまま。
- 旧固定較正2飛行のlogging-only再計算は元16列全行文字一致。公表速度covの保守的共通分散倍率7.908699（飛行等重みの白色化誤差2次moment最大固有値、最低1）。結果を見て重み探索なし。実際に全方向/全場面で統計的保証する値ではない。1Hzでも共有IMU/時系列/速度-傾きの相関は完全には処理しない。
- body_velocity_factor.hpp / velocity_odometry_manager.hpp、40座標/取付位置/gauge例、360状態Jacobian列+240cov変換列PASS。build_velocity_fusion.pyはordered dispatcherを生成コピーだけ変更、元source不変。別nodeビルド済。runner/campaign/schedulers/reportにflag/較正/1Hz因子周期/covariance正定/保存照合を追加。まだ実飛行効果未確認。
- Windows実空き19.87GB、8GB停止保護、画像/点群/bagなし。ROOT約3.7GB。v33s完了分を圧縮予定。

### 最新 v35成分比較中、v36の準備（目標未達）
- v34短2組全監査/終了合格、a1 .541→.516m、b1 .283→.321m。不採用ではなくモデル切り分け優先で追加反復停止、元6組基準合格ではない。
- MIMOSA IMUのbias random walkをGazeboと同じ平均回帰へ変更した別lib build/mimosa-sensor-biasを追加。SDFからacc tau175/driving.000375、gyro tau400/driving.00002。40遷移/480微分列PASS、1Hz実因子のphi/sigma監査、実ロードlibrary SHA一致を確認。固定gravity/libと同じmap objects、元lib保持。
- 現所有v35-short-seed24 firstshort session59478/domain86のみ、b1実行中。a1完了全合格、融合.482→.450m、回転.341→.361deg。制御・velocity/tilt融合はv34と同じ、biasモデルだけ変更。
- v35 scheduler22291/PID1396523は子起動前SIGINT済み。b1まで完走後、センサに比べて過大なwhite-noise設定の切り分けv36を予定。全6組基準を通ったとしない。v35_component_comparison.json。
- 次候補v36: v35と同じ--tilt-fusion --velocity-fusion --fixed-gravity-imu --sensor-bias-model、native_fusion_velocity_gravity_calibration.json。追加--fusion-imu-noise sensorでacc density .8→.00148843/gyro .01→.000636417（SDF stddev/precisionと1000Hzから変換）。物理センサ不変。v9/v15は旧モデルでこの変更に失敗済み、今回はbias/gravity/融合が違うため未確認。tracking_aware_v36.yaml準備済み、まだ飛行/宣言未開始。
- schedulerにwhite-noise flag伝播、reportにSDFとの設定一致監査を追加。独立26/27は未使用、最大1sim、Windows実空き19.87GB/8GB下限、画像/点群/bag保存なし。停止は現在firstshortだけ。詳細WORKLOG末尾。

### 2026-10-17 23時台 最新: v35残り4比較を再開（目標未達）
- v35短seed24完成全監査/終了合格: a1融合.482→.450m、b1 .223→.261m（1勝1敗）、ROVIO a1 7.630→5.994m、b1 45.623→6.823m。成功と扱わない。v35_short結果を再利用し残り4組を完了させる。
- 間にv36（実センサwhite noiseへ）a1を完走したが融合17.703→19.197mで両方悪化、不採用。v37（さらに標準LiDAR4DoF）も初期化後0.1秒から姿勢発散、追従12秒で>1km、途中停止・全process0。不採用。全結果保持。新しい4DoF実headerテスト15姿勢は投影数学に合格だがオンライン安定性はなかった。
- v35復帰: unipilot_run/campaignのv37版はROOT/source_after_v37へ保存、v35 flight implementationから2ファイルだけ戻し、development_v35.jsonの全SHA一致を確認。したがって現runner/campaignには--lidar-four-dofはない。scheduler/validation/reportにはオプションの伝播を残すが既定false、v35に使わない。
- 現所有scheduler session57816、development_v35_resume.log。second_short seed25/domain86から開始、その後prefix seed24/25。firstshort59478終了。v36scheduler70059、v37scheduler89948/firstshort67569も停止済み。最大1sim。現v35を停止するならscheduler57816へSIGINT（子を止める）。
- v35構成: yaw-only、velocity+tilt1Hz、fixed gravity、sensor bias mean-reversion、legacy white-noise、DRPM OFF、元較正native_fusion_velocity_gravity_calibration.json。センサ本体はnative。未使用26/27保持、元成功基準不変。Windows19.87GB、8GB下限、raw保存なし。

### 最新 v38: 元のLiDAR 4DoF設定を安定したIMU重みで比較中（目標未達）
- v35は短4組2勝2敗で不採用。長prefix24は約250秒時点の直近傾き約1.85degを確認して停止、全子0。独立26/27未使用。v36 sensor白色雑音は17m誤差、v37 sensor白色雑音+4DoFは初期化直後発散、両方不採用。
- 現在v38: v35と同じ制御/速度+傾き1Hz/固定gravity/センサbias遷移/legacy白色雑音、標準reg_4_dofだけON。runner/campaignはsource_after_v37の版へ復帰済み。固定/偏向に同じ変更。実験中のsource/条件を変更しない。
- firstshort session27876/PID1412386、scheduler session64638/PID1412374。最大1sim、24/25×a1,b1,prefixの6組。v38_firstshort.log/development_v38_scheduler.log。停止時は両方停止。a1途中110秒時点は融合.514→.432mで発散なし、完走成績ではない。
- Windows実空き約19.87GB、8GB停止、生bag/画像/点群保存なし。ROOT logs/gimbal-improvement-20261017 約3.8GB、停止済みログだけ圧縮。元成功基準/未使用独立26/27を維持。

### 最新 v39待機、v38不採用（目標未達）
- v38短4比較は全監査合格/全子0、a1/24 .556→.463m、b1/24 .332→.500、a1/25 .607→.572、b1/25 .401→1.019。2勝2敗、最大比2.542、元基準不合格。v38 scheduler64638/PID1412374は現在終了済み。firstshort27876終了。v38_progress.html/early_rejection保存。
- 現simはv38-gravity-diagnostic24/session89720/domain85、prefix最初300秒だけ。元1070秒試験と混同しない。206秒時点の重力傾き融合.126/.122degで、以前v35の大きな傾きドリフトを抑制。ただし位置.699→1.033mで可動悪化。完走確認待ち。
- v39 scheduler72454が宣言済みで待機、firstshort64123は上記診断のruntime全0を待ちsource SHAを照合後、short24 a1→b1/domain86を開始。最大1sim。停止するならこの2つと診断89720をそれぞれ確認。旧helper47376/cleanup94847は終了済み。
- v39はv38融合のまま視野だけ変更。parallax_fixation.pyをopt-inで使用。成熟特徴不足時も回転除去済みの観測視差を使い特徴を保持、十分な証拠がなければ元の復帰。新43テストPASS、飛行効果未検証。planner_nodeのimport選択、schedulerの新module SHA保存追加済み。v38診断は変更前に既にplanner稼働済みで旧読込み/snapshotのまま。
- 新資料はtracking_aware_v39.yaml/development_v39.json。融合tilt+velocity1Hz/fixedgravity/sensorbias/legacywhite/LiDAR4DoF/DRPMoff不変、nativeセンサ不変。基準変更なし/独立26,27未使用。Windows実空き19.87GB、8GB下限。raw記録なし、今回画像2枚だけsnapshot_v38_a1_seed25（別stamp）保持。ROOT3.8GB、停止済み数値圧縮完了。

### 最新 v39s実験中（目標未達・継続）
- v39は実験開始前に待機条件が停止、飛行0本。旧scheduler72454/PID1422841、待機64123は終了済み。v38診断89720も終了。現在はfirstshort v39s session12865/domain86（a1→b1）、scheduler98898（残り25短/24,25prefix）。最大1sim、停止時は両方を確認。
- v38最初300秒prefix診断は飛行終了/入力・設定監査合格だがpacerだけexit1（Gazebo停止後のpause service失敗）。元criterionではvalid=false、成功試験へ数えない。ROVIO位置12.588→2.880m、融合1.232→1.707m。重力傾き融合.13085/.12671degで、後半100秒も増大せず。これだけで全長安定とは言わない。
- simulation_shutdown.stop_world_clientsを追加。unipilot_runはpacerの最後のresume通信/終了を待ってからGazeboを停止し、結果をshutdown_request.world_clientsへ記録。既存exit1やtimeoutを隠さず、終了済みprocessへ再signalしない。実childを使う3試験+視野/既存43試験=46PASS。飛行後終了は現v39sで検証する。
- v39sは同じparallax視野制御とv38融合で、停止処理だけ追加。tracking_aware_v39s.yaml/development_v39s.json。a1途中114秒の融合.532→.445m、ROVIO9.971→9.513m、完走未確認。新制御の観測視差保持が動作し、正面復帰は大幅減。source凍結済みで変更しない。
- v38_initial_alignment_audit.json: 4短+prefix診断のMIMOSAは最初の評価時刻の固定/可動pose差がすべて厳密0。評価開始の座標合わせ差が融合悪化の原因ではない。ROVIOには最大.00236m/.01325degの既に生じた差があるが採点方法は変えない。
- report_gimbal_partial.pyは固定gravity時だけ絶対傾きRMSE/直近値を表示（初期姿勢合わせなし）。既存位置・姿勢スコアは変更なし。これらは診断のみ/完走判定へ使わない。
- Windows空き約19.85GB、8GB下限、ROOT約3.8GB。cleanup94847終了、完了数値は圧縮、raw画像/点群/bag保存なし。独立26/27未使用。まだ安定改善を達成していない。

### 2026-10-18T07:03:23.355160+00:00 再開時点
- v39s seed24 a1/b1完走・全監査合格/全子0、融合.556559→.465108m、.439014→.359585m。firstshort12865完了。scheduler98898/PID1423593はseed25短を実行、その後24/25 prefix。最大1sim、55凍結source一致、条件不変。
- 現時点2勝2比較だが安定改善は未達。独立26/27未使用。Windows空き19.19GB、raw保存なし。WORKLOG末尾とdevelopment_v39s_progress.jsonを確認。

### 最新v40: 二軸の既観測特徴追跡を検証中、目標未達
- v39sは4短比較2勝2敗、全監査/全子正常終了。固定→可動: a1/24 .556559→.465108、b1/24 .439014→.359585、a1/25 .510780→.561963、b1/25 .284763→.295725m。a1平均比.967945>元基準.9、不採用。prefix未開始、全記録保持。old scheduler98898/PID1423593、firstshort12865、stophelper52279は終了。
- 現所有v40 firstshort session5294/domain86（a1→b1/seed24）、scheduler53214（seed25短と24/25prefix）、最大1sim。v40_firstshort.log/development_v40_progress.json/development_v40_scheduler.logを確認。停止時両方必要。
- v40はv39s融合/較正/センサ/視差保持閾値を維持しyaw-onlyを解除、pitch±30degまで既観測点に追従。新spatial_parallax_fixation.py、planner_node opt-in分岐、schedulerの新module/test SHA列挙。元parallax module不変。yaw-only互換・上下追従・未観測点排除・尺度不変・成熟時同値を含む56試験PASS。実飛行効果未確認。
- 57ファイル凍結。native_fusion_velocity_gravity_calibration、velocity+tilt1Hz、fixed gravity、sensor mean-reverting bias、legacy white noise、LiDAR4DoF、DRPM OFF不変。元基準を維持、独立26/27未使用。Windows実空き19.18GB/8GB保護、raw保存なし。

### 最新v40不採用、長prefix24を融合診断に利用する段階
- v40短seed24 a1融合.479→.454m/ROVIO8.833→6.906m、b1融合.316→.439m/ROVIO29.100→3.182m。seed25 a1融合.431→.591m/ROVIO8.960→5.514m。全監査/全子0。短3比較1勝2敗で元基準不合格確定。b1/25現在終盤。v40_early_rejection.jsonへ不採用、条件/基準を変えない。
- 主firstshort5294終了、scheduler53214/PID1432636稼働。seed25 b1→prefix24だけ診断完走する。helper55803がprefix24 runtime作成直後にschedulerを止めてprefix25を省く。最大1sim。独立26/27未使用。
- 追加待機61431はcompare_fusion_sidecar.py、output ROOT/v40_fusion_sidecar_prefix24、対象v40-prefix-seed24/run1_prefix_active、port11485。前待機45050はnode起動前に停止、waiting_v1へ保持。追加3ノード: velocity active完全複製、既存displacement+tilt active/fixed。原モデルのカメラ運動を条件とした比較、制御には全く入力しない。全topic/TF隔離、直接C++childを所有しreference終了でSIGINT、RAM2.5GB/待機30分保護。生データ保存なし。実動作はこれから確認。
- 終了後 report_fusion_sidecar.py ROOT/v40_fusion_sidecar_prefix24。duplicate共通pose完全一致、LiDAR/VIO stamps/IMU prefix同値、全child0/全長出力を確認してから解釈する。MIMOSA loggerのgraph_manager_odometry.tumを既存evaluateで採点、truthはposthocのみ、変換CSVはtempで削除。
- v40 b1/24 VIO速度RMSE .39810→.03574m/sなのに融合悪化。1秒相対移動量RMSEはVIO .45249→.08647m、融合 .03174→.03098m。融合平均x誤差-1.84→-2.46mm/秒窓。これだけで融合bug/重み原因と断定しない。diagnose_fusion_relative_motion.py/JSON追加、WORKLOG参照。
- Windows19.18GB/8GB停止。cleanup26582終了、v39sまで全数値lossless圧縮。v40主source57件は凍結中、変更しない。安定改善目標は未達。

### 最新 v41実走開始（目標未達）
- v40は短4組1勝3敗で不採用。prefix24はturn中断で991.128/1070秒で停止、全main/sidecar/Gazebo正常停止。途中990.9秒の融合velocity固定6.96457→可動7.87912m、displacement5.51323→8.01835m、ROVIO70.25695→22.04680m。全長成果ではない。追加複製は全10483pose/入力列完全一致、3sidecar連続性合格。旧scheduler53214/helper55803/sidecar61431等は終了。
- 現在v41 firstshort session23172（seed24 a1→b1/domain86）、scheduler67428（seed25短/24,25prefix）。最大1sim。停止時は両方必要。ROOT/development_v41.jsonに59ファイル凍結、独立seed26/27未使用。成功基準不変。
- v41はv40と同じROVIO/velocity+tilt1Hz/較正/fixedgravity/sensorbias/legacywhite/LiDAR4DoF/DRPM OFF/nativeセンサ。変更は成熟点の視野選択のみ。joint_information_view.pyで全予測保持特徴のpose情報logdet比を評価、hold比3%以上改善時のみ偏向、90%追跡保持/0.8秒予測/角速度限界を維持。成熟点不足では元v40 fallback。新9+既存試験計69成功、精度効果は未確認。
- 自己遮蔽診断: 実mesh+撮影時encoderの投影で可動画像の.386–.446%に機体が入る。457.6秒のROVIO7点は半径16pxまで重なり0。この2画像だけで原因とは言わない。audit_self_visibility.py/ROOT/v40_prefix_self_track_snapshot、WORKLOG参照。画像計4PNGと7点だけ。
- Windows実空き20.56GB/8GB停止。生bag/連続画像点群なし。v40停止済み数値ログは圧縮可能だがまだ実施していない。全失敗保存、成功を主張しない。

### 最新 v41不採用、短区間で速度融合の原因を切分ける
- v41 a1/24 .465147→.374332m、b1/24 .332→.385m、a1/25 .300→.354m、短3組1勝2敗で不採用（全監査合格）。現在b1/25を完走させている。成功基準不変・独立26/27未使用。ROOT/v41_early_rejection.json。
- 現主scheduler67428/PID1446492、second_short PID1450846。firstshort23172とobserver4447は終了。新owner待機session73007がb1/25 runtime後にschedulerを止め、全sim解放と59SHA一致を確認して、別診断v41-velocity-factor-diagnostic24（b1全263.385秒/seed24/domain85）だけを起動。長prefix2本は未開始取消、基準の試行数へ診断を足さない。v41_velocity_factor_diagnostic_plan.json。
- 別補助待機session78786はv41_tilt_only_sidecar_b1_24（port11485）。duplicate_active/tilt_only_active/tilt_only_fixedの3nodeで、速度因子だけを除いた効果を同じセンサで切分け。制御には未使用。build/mimosa-tilt-only-diagnostic/以下の独立node。tilt-onlyはROVIO傾きを含み純LIOではない。全出力隔離、RAM2.5GB保護/親終了で自動停止。
- 旧prefix補助待機99215/PID1452886、旧stophelper78361/PID1454732は起動前に停止。build84717完了。stopped v40 sidecarの全10483pose/入力完全一致、未完なのでfull accuracyなし。Windows約21.2GB/8GB停止、raw保存なし。

### 最新: v42 開発比較を開始（2026-10-18）
- ユーザーの自走改善は未完了。v41は1勝3敗で不採用。別のb1速度融合診断は有効、複製3198姿勢完全一致。速度制約を外すと両方式とも悪化。現行の速度＋傾斜融合を維持。
- v42は「成熟した有用な特徴を保持できる間はジンバルを止める」変更のみ。`stable_view.py`、`tracking_aware_v42.yaml`。44テスト実行合格。開発6比較・同じ事前基準、独立seed26/27未使用。
- scheduler session43342、最初の短区間キャンペーンを別sessionで起動。`development_v42_scheduler.log` / `development_v42_first_short.log` / `development_v42_progress.json`を参照。1シミュレーション限定。`--early-stop`により、完了キャンペーン間で基準達成が不可能と分かったら未開始飛行を打ち切る。
- v41診断旧session73007/78786は全終了、Gazeboも正常終了。新規連続画像・点群・rosbagなし。Windows空き21.2 GB、最低8 GB。

### 最新 v42不採用確定、DRPM4診断を準備中（目標未達）
- 現主所有: scheduler session43342、second_short PID1464033/domain86。first_short session87797は終了。v42 a1/24 .440→.507m、b1/24 .440→.332m、a1/25 .347→.388m、全監査/終了合格。1勝2敗で不採用確定。b1/25を走行中、完了後schedulerの--early-stopが残りprefixを始めず終了する。61凍結ファイルを走行中変更しない。
- 別の情報重み付けmode0診断は偏向前の初期化で数十mずれ/処理遅延。supervisor4036/PID1460707をSIGINTし3子すべて0終了。ROOT/v42_information_weight_sidecar_b1_24にstartup_input_audit: 処理済み列は全3モデルとも元入力の厳密な接頭部一致。重み付け2モデルは停止時に同一時刻VIO1行だけ未処理、完全入力網羅ではない。全長結果へ数えない。待機partial reporter71894は対象が追従開始まで届かなかったため終了1、精度値なし。
- 次の別診断を準備: performance/drpm_four_dof_kernel.hpp + build_drpm_four_dof.py。公式Eq.24更新式を鉛直方位+並進の4変数に制限、除外傾き情報0。公式6DoFそのままではない。40計算例PASS、追加の雑音支配トンネル軸テストを加え再build中(session48334、ROOT/build_drpm_four_dof.log)。別build/mimosa-drpm-four-dof/libmimosa_lib.soでDRPMオブジェクトのみ差替え、現行libは不変。
- compare_fusion_sidecar.py/report_fusion_sidecar.pyに--comparison drpm_four_dof対応を追加済み。通常の完全複製1＋別lib使用の固定/可動DRPM4を出力隔離。実ロードlibのvariant別SHA確認、ROVIO速度/傾斜観測一致、入力接頭部と全長/正常終了を必須。まだ起動しない。
- 次に必要: v42全停止を確認してから、pace_simulation.py / unipilot_run.py / unipilot_campaign.pyに診断ノードのLiDAR完了topicを追加監視するopt-in引数を追加する。現v42凍結中なのでまだ編集していない。追加推定器の遅れも待つ単一pacerで全長b1別診断を行う。カメラ制御はv41/通常融合のまま、診断結果は制御に入力せず成功比較数へ足さない。予定ROOT/v43_drpm4_diagnostic_design.json。
- Windows空き約21.2GB、8GB停止、rawセンサ保存なし。今回の数値は停止分のみ可逆圧縮済み。独立26/27は未使用、全6事前基準は未達。ユーザーには安定改善未達を伝え、作業継続中。

### v42完了・4自由度DRPMの同一入力診断（2026-10-18T18:22:35.819213+00:00）
- v42短4比較は全監査・全子正常終了。融合位置RMSE固定→可動: a1/24 約.440→.507m、b1/24 .440→.332m、a1/25 .347→.388m、b1/25 .337→.637m。1勝3敗で不採用。元基準/未使用seed26,27を維持し、長prefixは開始せず。61実装ハッシュの最終一致を保存後に次診断の変更を行った。
- 4自由度DRPMの独立ライブラリが完成。40幾何例・公式式一致・傾き保持・雑音に支配されたトンネル方向の抑制・階数不足拒否の試験に合格。元の通常ライブラリは変更なし。
- 追加診断推定器もLiDAR処理完了まで待つ任意のpacing指定をrunner/campaignへ追加。既定は空。既存終了処理を含む5試験に合格。
- 現在v43-drpm4-diagnostic24でb1全263.385秒/seed24を実行。通常融合＋v41制御の主計算と、完全複製、4自由度DRPMの固定/可動を同じ物理飛行で比較。追加推定器はカメラ制御へ入力しない。主campaign PID1468562、補助supervisor PID1468549。新library実ロードSHA一致確認済み。全長の入力/出力一致と正常終了は実験後に検査する。
- Windows実空き約21GB、8GB停止保護。生bag/連続画像/点群保存なし。目標は未達。

### v43終了と初速度の整合性診断（2026-10-18T18:29:36.393387+00:00）
- 4自由度DRPMのb1全長診断は改善なし。通常融合固定.292205→可動.300196mに対し、DRPM固定17.591983→可動24.830422m。全main/追加子0、元13ファイルのSHA一致。追加複製3199共通姿勢は厳密一致。
- strict auditはfalse: 追加ノードの終了が主より0.6秒早く、飛行完了後の6出力を取り逃した。元結果を合格へ変更しない。補助auditで、初期化から事前定義した追従完了344.58秒までの全入力/IMU prefix/VIO観測は全追加ノードで同値、追従出力は全長存在を確認。退化対策は不採用。追加ノード終了は固定2秒待ちを廃し、主runtime確定まで待つよう修正した。
- diagnose_integrated_drift.pyで位置誤差を機体相対移動量と世界座標へ回す姿勢の2成分へ厳密分解。v41/v42の8組で再構成誤差<1e-8m。移動量成分が主に大きく、単なる方位誤差だけでは説明できない。因果効果の独立分解とは解釈しない。
- 初期速度sigma=1e-6m/sというほぼ厳密な静止priorを確認。事後の真値差分では5飛行の初期速度ノルム.0078–.0148m/sで、実ホバリングと不整合。これが後続ドリフトの原因かは未確認。
- v44-init-diagnostic24: a1の事前宣言45秒で、同じactive入力を使い初速sigma1e-6/1m/s × IMU白色雑音legacy/sensorの4条件を比較。1m/sは既存ROVIO初速度priorから採用し、真値や精度を使った調整なし。主は変更せず追加計算は制御へ戻さない。supervisor session31005、main campaign50390。初期化から全入力同値と全終了を監査する。生データ保存なし、Windows約21GB/8GB保護。

### 初期速度の診断完了、v45の比較開始（2026-10-18T18:32:38.890130+00:00）
- v44の事前宣言45秒を完走し、全main/追加子正常終了。通常設定の複製は全pose厳密一致。初速度sigmaだけ1e-6→1m/sにしたvariantは全主入力/IMU prefix/速度・傾き観測同値、設定差分監査合格。位置RMSE .098662→.041569m、回転.380918→.244208deg。真値は事後評価のみ。初速平均0は変えず、不確かさを既存ROVIO初期設定と揃えた。
- センサ由来IMU白色雑音＋初速sigma1は5.191623mで未採用。旧sigma＋同雑音は3053.93mまで発散し、追従後の最後.35秒の処理尾部が不足したためそのvariantはstrict監査false。reportにvariantごとの有効性を明示、全体falseを合格へ書換えない。通常雑音＋初速修正の比較は独立に全監査合格。
- runner/campaign/development/validationへ任意--fusion-initial-velocity-sigmaを追加。未指定なら既存値保持、指定時は有限正値検証、protocolと固定/可動両configへ保存。reportで事前指定値と両config一致を検査。構文/CLI検査と終了処理3試験成功。
- v45はv41の視野制御/物理センサ/融合較正/legacy雑音/4DoF/DRPM OFFを維持し、固定・可動双方の初速sigmaを1m/sにする。3経路×24/25の6比較と元成功基準を事前固定。59ファイルSHA固定、独立26/27未使用。firstshortはv45_firstshort.logを参照、scheduler40893。最大1sim、途中不合格確定時は未開始prefixを省略するが既取得結果を残す。
- Windows実空き21.39GB、8GB保護。数値のみ、生bag/連続画像点群なし。安定したジンバル改善の目標は未達。

### v45最初の全長比較と未接続の次案（2026-10-18T18:37:59.281855+00:00）
- v45 firstshort session28807、scheduler40893。a1/24全228.493秒を完走、設定/入力/出力/終了監査すべて合格。融合位置固定.439151→可動.397022m（9.59%減）、回転.379221→.407239deg。ROVIO位置8.214882→9.209751m/回転.425048→3.606838deg。現在b1/24を同条件で継続。1組の結果で安定改善としない。
- performance/test_imu_factor_derivatives.cppで固定重力IMU因子の全5入力について40状態×24列=960有限差分列を照合、最大差1.86182e-10で合格。現場のGTSAM実libraryを使用、実行中ソース/バイナリを変更せず。正しい測定雑音設定で精度が悪い理由を微分式の不一致では説明できない。
- 未接続のtranslation_information_gain.pyと6試験を追加。視野の評価を位置3成分の不確かさの減少へ絞り、回転との相関は全6次元逆情報から周辺化して残す。位置と回転を独立に扱わず、単位/座標回転不変・明示逆行列との30例一致・回転だけの不要情報を評価しないことを検査。v45の視野制御へは接続していない。8比較の誤差分解から次案を準備しただけで、精度改善の主張なし。

### 同時刻の融合・地図更新の整合性診断（2026-10-18T18:54:21.440720+00:00）
- v45短4比較は全監査/全子正常終了、融合位置RMSE固定→偏向: a1/24 .439151→.397022m、b1/24 .342→.352m、a1/25 .381→.439m、b1/25 .316→.350m。1勝3敗で元基準不合格。prefixは開始せず、59凍結ファイルの最終SHA一致を保存。独立seed26/27未使用、安定改善の目標は未達。
- 同時刻に別センサ因子を追加した際、MIMOSA graph内部stateは更新されるが返却用optimized_values_を更新しない箇所を確認。実ライブラリを使うtest_same_stamp_estimate.cppで旧版の返却速度と内部速度の差.990099m/sを再現、独立修正版では差<=1e-12。別ROS master11595を使用し終了済み。これは実装不整合の再現であり飛行ドリフトの主因と断定しない。
- build_batched_fusion.pyは独立build/mimosa-batched-visionを生成。初期化後の同時刻入力を視覚→LiDARの順に処理し、両更新後の推定値を地図に利用、graph出力を両更新完了後へ延期。SAME_KEYでも推定値を再取得する。センサ時刻/サンプル/融合重み/初期化順序は変更せず、通常ライブラリ/バイナリは保持。
- compare_fusion_sidecar/reportにbatched_fusion診断を追加。通常の完全複製1＋別libの固定/偏向2を同時入力で比較する。監査は同時刻内の順序だけを許容し、時刻ごとのセンサ多重集合・IMU総数/接頭部を厳密比較。順序交換は合格、時刻・センサ種・IMU数・接頭部・重複・欠落は不合格の7ケースに成功。視覚速度/傾き観測とconfigの同値、全評価区間、全子正常終了も必要。
- 現所有: v46診断supervisor session7801/PID1484406、main1484421、sidecar1484420。b1全263.384754秒/seed24、v45カメラ制御と通常融合を主とし、修正版の出力は制御へ返さない。ROOT/v46_batched_fusion_diagnostic_design.jsonに12ファイルSHAと評価範囲を事前固定。完了後同一入力監査/性能比較、効果があれば実制御の比較へ進む。
- Windows実空き約21GB、8GB下限。連続画像/点群/bagは記録しない。数値・設定・ソースだけ保持。最大1sim。

### 同時刻融合の修正確認と位置情報を優先する視野選択（2026-10-18T19:05:28.461584+00:00）
- v46 b1全長同一入力診断は全監査合格。通常融合の完全複製3208pose厳密一致、全入力stamp/IMU prefix・速度/傾き観測同値、全子正常終了。旧→同時刻一括更新の位置RMSE: 固定 .344886→.137010m、偏向 .425205→.338323m。改善は両方式に認めるが、修正後の偏向は固定より悪い。ジンバル改善成功とはしない。12凍結SHA一致。
- 実graphの追加publication試験に成功: 途中の状態は配信0、両更新完了後だけ配信1、元の時刻・返却値・内部値・配信速度が一致。別ROS master11595は停止済み。
- runner/campaign/development/validationに--batched-fusionを追加。対応する独立node/libを固定・偏向双方へ使用し、manifest/生成ソースも保存、reportでbinary SHAを照合。既定falseなので旧実験再現は維持。
- v47: 融合は同時刻修正版、初速sigma1、velocity+tilt1Hz、legacy白色雑音、sensor bias、LiDAR4DoF、DRPM OFF。視野だけを位置3成分の周辺共分散logdet減少で選ぶopt-inにした。回転との相関は残し、回転だけの観測改善を目的へ数えない。既知・現在見える成熟特徴だけ利用、未観測の画像情報や真値を利用しない。角度/速度/特徴保持制限/未成熟時fallbackは維持。
- 並進情報・統合視野・終了処理の29試験合格、旧設定の100視野判断は旧実装と出力/診断が厳密一致。構文/3CLI検査合格。実飛行性能は未確認。
- 現所有: v47 firstshort session75412、scheduler21702、最大1sim。67ファイルSHAを固定し3経路×seed24/25の6比較、元基準不変・独立26/27未使用。不合格確定なら未開始長区間を省略する。v46全終了、v45数値ログ可逆圧縮完了。
- Windows空き 21.39 GB、8GB保護、raw記録なし。安定したジンバル改善の目標は未達で作業継続中。

### v47最初の全区間比較（2026-10-18T19:11:00.421425+00:00）
- a1/24全228.493秒が完了、全入力/連続性/共通初期化/設定/終了監査と67ソース保存照合が合格。融合位置RMSE固定0.343473706→偏向0.289533234m、回転0.282886236→0.275163473deg。ROVIO位置6.828→4.677m、回転1.915→3.335deg。1組だけで安定改善としない。firstshort75412はb1/24を継続、scheduler21702は残りを待機。
- v46同時刻順序の監査: 同時刻3198組中、通常複製は3198組LiDAR先、修正版は初期化時の1組だけ旧順序、残る3197組が視覚先。同時刻の意味を取り違えた監査ではない。全原入力・出力照合結果は維持。


## 2026-10-20 — run2a_prefixの固定優位を方向別に診断
- 最新結果: `/root/uas_native/logs/run2a-direction-diagnosis-20261020/index.html`、4ページPDF、summary.json。元240秒の前半120秒を2回新飛行、採用UniPilot v57/DRPM OFFは無変更。固定画角の方向別拘束が強かったという仮説を、ROVIO全体RMSEだけで否定しないこと。
- 同じLiDAR行列と特徴点配置で計算した条件付き指標では、1回目239時刻の94.6%で偏向が強く、固定優位を支持しない。推定深度依存の指標で、真の特徴点位置や全240秒の検証ではない。
- 1回目固定/偏向位置RMSE .886/1.831m。速度/傾き交換で、差は速度+信頼度の側へ移った。純LIOは進行方向に−29.58mの終端ずれ、固定ROVIO速度は平均+.569m/sの過大評価で、相殺の向き。
- 2回目は全て偏向傾きで、速度/共分散を2×2交換。固定速度+固定共分散2.578m、固定速度+偏向共分散.483m、偏向速度+固定共分散8.018m、偏向速度+偏向共分散2.888m。実速度誤差の大きい固定値が同じ重みでも融合を改善し、誤差相殺を支持。重みも大きく影響。診断用の交換でありそのまま採用しない。
- 2回目の通常固定3.975m/偏向2.888mで順位逆転。同じ要求seedでも2回の実観測は別。各回内は同一入力、48/48・51/51監査成功、複製軌跡厳密一致。元run全240秒の主因確定や固定画角の恒常的優位と称さない。
- plannerは最終融合共分散を直接予測していない。今回地図のLiDAR直接深度は0、画像視差とMIMOSA移動に基づく三角測量のみ。この移動誤差の影響も追加確認対象だが未切分け。共通IMU相関・地図更新も今回単独未検証。
- 実験は終了、シミュレーション継続なし。採用推定器/偏向規則/融合重み81SHAは維持。ログ最小化のため数値を可逆圧縮。今回も新たな設定を採用していない。

## 2026-10-21 — ZaB・二ツ小屋の短窓比較完了
- 最新結果: `/root/uas_native/logs/weak-direction-windows-20261021/window_report/index.html`、comparison.pdf、summary.json。ZaB a1/b1・二ツ小屋interiorを各120秒×2回、計6飛行。各60秒に分割し、共通純LIO診断の最弱並進方向へ5/10/20秒の移動誤差を投影。固定5方向は独立単眼＋偏向＋純LIO、v57/DRPM OFFを変更せず。新しい偏向方式は採用していない。
- 主10秒の固定5方向平均→偏向[m]: a1前半 .01515→.01436、後半 .08500→.06877、b1前半 .01870→.01689、後半 .02011→.01394、二ツ小屋前半 .55346→1.16550、後半 .40475→.22582。ZaB後半2区間/二ツ小屋後半は2試行とも改善、二ツ小屋前半は2試行とも悪化。これらは5/20秒窓でも同符号。ZaB前半は試行で順位逆転。
- 重要: 二ツ小屋後半の視覚速度誤差RMS正面.8332/上.0096/偏向.1389m/sに対し融合方向誤差.2222/.5230/.2258mで順位不一致。純LIO−4.981m/10秒に対し正面速度偏り+.698m/s、偏向も+.1225m/sで相殺し得る方向。ZaB b1後半も偏向は視覚速度が最も不正確なのに融合が最良。短窓でも正しい視覚拘束の効果と相殺を因果分離していない。
- 診断は回転固定の並進ブロックで、ドリフトした純LIO地図に依存。事前の強異方性閾値.05以下は主10秒72窓中1窓、二ツ小屋0。「退化を検出した窓で普遍的に改善」と言わない。全窓/部分集合と全5/10/20秒を保存。2独立試行で、窓数を試行数とみなさない。
- 実装: record_common_lidar.py、run_window_validation.py、window_metrics.py/test_window_metrics.py、report_window_validation.py。既存observer/runnerに任意診断記録追加、推定器変更なし。4数値試験、6飛行の各65/65検査、採用/実験/評価SHA不変・全所有停止を確認。約196MiB、Windows空き176GB、bag/GUIなし。再生成はreport_window_validation.py <campaign root>→ログ内finalize_findings.py。全終了済み、再開待ち作業なし。

## 2026-10-24 — 特徴の偏在方向が変わる人工トンネルの検証完了
- 最新結果: `/root/uas_native/logs/switching-tunnel-20261024/validation/report_sparse/index.html`。PDF/全数値/入力監査/実偏向角図を同ディレクトリへ保存。環境は `/root/multilink_autonomy_ws/assets/switching_tunnel_20261024`、再生成器は `active_perception/build_switching_tunnel.py`。
- 幅6m・高さ5mの一定アーチ断面、形状/照明共通。特徴の多い面が32mごとに左→天井→右→床、逆順、全周豊富の3配置。130m/.5m/s、各3回、同一実飛行で固定5単眼と偏向単眼を比較。DRPM OFF、直前候補v10を無変更で使用。標準採用v57は変更していない。真値/正解模様配置は視野判断や推定へ入力しない。
- 偏在2配置の全6飛行で、VIO10秒軸移動・融合10秒軸移動・融合全位置の全指標が全固定より改善。3回平均のVIO10秒は最良固定→偏向で2.306→1.065m / 5.307→1.969m、融合全位置は33.612→11.174m / 44.707→12.473m。主軸速度（全拘束利用）は .270→.130m/s / .459→.223m/s。
- 全周豊富な対照は全3飛行で良い固定が優位。VIO10秒は最良下固定 .232m / 偏向1.139m、融合全位置は最良上固定6.042m / 偏向10.828m。固定平均と最良固定を混同しない。偏在でも絶対位置RMSE8.6〜13.2mが残る。制御された人工配置での結果で、実環境や単純な特徴追従方式への優位は未検証。
- 上下固定の速度拘束利用率は偏在で約75%/50%。画像とROVIO出力は連続、不確かさにより融合が拒否。欠落を補間しないreport_switching_tunnel_sparse.pyを第一試行後に追加し評価変更を明記（evaluation_amendment.json）。全利用の主軸速度と利用時のみの速度を分離。全方法共通の10秒移動指標は補助指標で事前登録ではない。
- 全9飛行85検査/回、74凍結SHA、圧縮再集計一致、全所有者停止をfinal_audit.jsonで確認。uniform初回は処理待ち停止で未完走→invalid_trialsへ保持し同seed/条件で再取得。取得ownerは全完了後に旧完全factor報告器で例外となるが取得はcomplete。再取得せず `report_switching_tunnel_sparse.py <validation>` → `audit_switching_tunnel.py <validation>` → `illustrate_switching_tunnel.py <validation>` で報告を再生成する。
- 全実験停止、待機/後続所有者なし。約0.5GiB数値/少数静止画像だけ保存、rawbagなし、Windows空き約156GiB。今回の依頼は完了、再開待ち実験なし。

## 2026-10-24 — Ostopoviceモデルを100mに拡張・比較完了
- ユーザー提供Downloads/hitler-highway-tunnel-at-ostopoviceのFBX/実画像から自動生成。生成器 `active_perception/generate_ostopovice_tunnel.py`、Assimp取込補助import_fbx_mesh.cpp。幅中央値5mと仮定、内壁凹凸を再構成・周期接続し100m。元の落書きを回転/反転/拡縮/色変更して壁・天井へランダム配置。独立seed24101/24102/24103、形状共通・配置全記録、結果による選別なし。環境assets/ostopovice_100m_20261024、元Downloads無変更。
- 表示は `ros2 launch multilink_uas_bringup ostopovice.launch.py` の一コマンド。seed:=24102/24103選択、機体/推定器なし、Ctrl+Cで終了。実GUI確認済み停止済み。新規生成のコマンドは結果HTMLへ記載、出力先上書き不可。
- 結果 `/root/uas_native/logs/ostopovice-20261024/validation/report/index.html`、PDF/summary/final_auditあり。3配置×2反復、各90m/.5m/s、固定5独立単眼と偏向、純LIO。DRPM OFF・v10候補・標準v57は不変。6本×85検査、139SHA一致、全所有停止、圧縮後集計一致。速度拘束利用率は全方法100%。
- VIO10秒軸移動RMSの2回平均[m] 固定平均→偏向：配置1 2.740→.602、配置2 4.118→.525、配置3 2.429→.301。最良上固定 .407/.358/.323。偏向は固定平均に6/6勝ち、全固定に1/6勝ち。
- 融合全位置RMSE[m] 固定平均→偏向：.790→1.005、.737→1.121、.765→1.409。固定平均に0/6勝ち。最良正面固定 .428/.404/.422、純LIO平均1.816/1.437/1.998。VIOの順位と融合の順位を区別し、今回融合が改善したとは説明しない。融合の相殺/重み原因の交換試験は今回未実施。
- 実偏向は序盤後主に天井を選ぶ。ランダム模様を配置した環境であり、最適視野が必ず場所ごとに切り替わるような作為的配置ではない。元凹凸があるため理想的な軸方向零拘束ではない。幅5mは実測復元ではなく仮定。2反復の結果を統計的有意と呼ばない。
- 再集計: report_ostopovice_trials.py <validation> → audit_ostopovice_trials.py <validation> → finalize_ostopovice_report.py <validation>。6飛行は完了、再実行不要。ログ273MiB/asset190MiB、Windows156GiB空き、連続rawなし、全所有プロセス停止、今回依頼完了。

## 2026-10-24 — 次の環境：自然なまとまりを持つ人工トンネル
- ユーザーはOstopoviceを離れ、完全人工かつ明確な偏在が場所で変わる環境を希望。`active_perception/build_weathered_tunnel.py`、assets/weathered_tunnel_20261024へ新環境を生成。100m×幅5m、一定アーチ断面高さ4.5m。外部画像なし。コンクリート色むら・染み・ひび・剥離・析出跡のランダムな集まり。約12/29/47/69/90mで豊富な向きが天井/左壁/右壁/天井/右壁へ変わる。境界は滑らか、領域長と幅は不規則。分布自体は人工条件である。
- 一コマンド表示 `ros2 launch multilink_uas_bringup weathered_tunnel.launch.py`。画像と概要 `/root/uas_native/logs/weathered-tunnel-20261024/index.html`。25実Gazebo画像と特徴数で偏在変化を確認、同seed再生成81asset一致。GUI描画39〜42fps確認後停止。新環境の飛行精度比較は未実施。旧比較/v10/標準v57無変更。
- 再生成 `python3 /root/uas_native/active_perception/build_weathered_tunnel.py <新しい出力先> --seed 24201 --length 100`。同seed再現、出力先上書き不可。新worldはviewerのworld_file:=<path>で表示可能。環境分布メタ情報はcontrollerへ渡さない。

## 2026-10-24 — 人工風化トンネルの固定/偏向比較完了
- 同一weathered24201環境、90m/.5m/sを3反復。結果 `/root/uas_native/logs/weathered-tunnel-20261024/validation/report/index.html`、PDF/summary/findings/angles付き。run_weathered_trials.py/report_weathered_trials.py/audit_weathered_trials.py/finalize_weathered_report.py。飛行は完了、再実行不要、全所有停止。
- 3回平均の融合位置RMSE：純LIO51.524m、固定5平均29.488m、最良固定上14.431m、偏向7.961m。偏向は固定平均へ3/3勝ち、各回全固定へ1/3勝ち。各回偏向4.360/14.381/5.143m、正面1.266/49.109/46.780m、上13.092/8.532/21.668m。試行ばらつきを省略しない。
- VIO10秒軸移動は最良固定正面3.475m→偏向3.287m（約5.4%減）。下固定はVIO発散、速度拘束採用0%、その他100%。固定5平均741.511mは下固定の発散に支配されるため、これだけを根拠に大幅改善と説明しない。軸速度は正面 .3742→偏向 .3304m/s。誤差相殺の因果分離は今回行っていない。
- シミュレーションの待ち合わせに修正あり。元pacerは新しい角度commandだけを完了扱いし、holdでreturnしたplannerを計算待ちと誤認して時計停止。未完了3回を保持。weathered_planner_node.py/scheduling_completion.pyで既存tickの正常return後に別topicへ処理完了を通知。推定器/選択規則/last_plan/command無変更、例外時は通知なし。run_weathered_view_array.pyで接続、元閾値.3/.25秒を維持。途中の閾値緩和だけの案は不採用。旧実験sourceは無変更、最終3回は共通修正。
- 元の厳密監査は全3回planner_continuous=falseを保持（最大更新間隔2.527/2.728/2.098秒）。weathered_trial_acceptance.pyで取得条件と手法の保持挙動を分離。後者とその参照失敗のみ許可、センサ/推定出力/同一入力/設定/正常終了は必須のまま。分類変更は最初の完走結果後に行ったとdesignへ記録。all_acquisitions_valid=true、strict valid=falseを混同しない。228SHA一致、圧縮再集計一致、所有PID消滅確認。ログ223MiB、Windows153GiB空き、連続rawなし。

## 2026-10-25 — 視覚だけの偏向とLiDAR情報を使う偏向の比較完了
- 最新結果 `/root/uas_native/logs/information-ablation-20261025/report/index.html`。weathered24201で各90m/.5m/s×3回、新規seed20262701/702/703。方式間は別飛行で同一センサ列ではない。既存v10の評価入力H/U/weakのみを除去する対照を追加し、予測・保持・復帰・制限・ROVIO/MIMOSA融合/DRPM OFFは共通。既定方式は変更していない。
- 視覚のみ→LiDARありの平均: 軸速度 .352952→.303592m/s、VIO10秒軸3.618391→3.117923m、融合10秒軸1.340792→1.072226m、全位置5.883566→4.952328m。各指標2/3勝ち。主評価区間の速度拘束100%。平均差には視覚のみ3回目の悪化が強く影響し、軸速度中央値はLiDARありが僅かに悪い。統計的有意や一般的優位とは呼ばない。
- 新規view_information_ablation.py / ablation_planner_node.py / run_information_ablation_array.py / run_information_ablation_trials.py、数値報告report_information_ablation.py、取得/判断ログ監査・説明finalize_information_ablation.py、test_view_information_ablation.py。再生成はreport → finalize。既存凍結コードは全て不変。
- 6本取得チェック合格/569凍結項目一致/全記録終了0/42PID消滅。元strict valid=falseは全試行保持（planner_continuous、最大角度更新間隔1.82〜4.27秒）。全シミュレーションとowner終了済み、再開待ちなし。ログ247MiB、Windows154GiB空き、連続rawなし。
