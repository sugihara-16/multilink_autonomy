#!/usr/bin/env python3
import json,html,hashlib,re
from pathlib import Path
ROOT=Path(__file__).resolve().parent;B=ROOT/'logs/zab-offline';OUT=B/'b1_vision_diagnosis';j=json.loads((OUT/'analysis.json').read_text());L='run1_tail_b1'
names={'side_out':'開始側支線へ進入','turn180_start':'開始側で180°旋回','side_return':'開始側支線から戻る','turn_to_main':'主通路へ向き合わせ','main_straight':'主通路を直進','turn_to_cross':'横坑へ向き合わせ','cross_out':'横坑へ進入','turn180_end':'横坑で180°旋回','cross_return':'横坑から戻る'}
rows=[]
for phase in j['phases']:
 cells=[]
 for n in ['rovio3','mimosa_off3','drpm3','fast1']:
  found=[x for x in j['estimators'][n]['phases'] if x['phase']==phase['name']]
  cells.append(f'{found[0]["position_rmse_m"]:.3f} / {found[0]["rotation_rmse_deg"]:.3f}' if found else '記録なし')
 rows.append(f'<tr><td>{names[phase["name"]]}</td><td>{phase["start_s"]:.1f}–{phase["end_s"]:.1f}</td>'+''.join('<td>'+x+'</td>' for x in cells)+'</tr>')
events=''.join(f'<tr><td>{v["elapsed_s"]:.2f}</td><td>{v["retrieved"]}</td><td>{v["lio_rotation_change_deg"]:.4f}</td><td>{v["vision_rotation_change_deg"]:.4f}</td><td>{v["rotation_error_after_vision_deg"]:.2f}</td></tr>' for v in j['fast_correction_events'])
s='''<!doctype html><meta charset="utf-8"><title>b1 Vision diagnosis</title><style>body{max-width:1350px;margin:30px auto;font:16px sans-serif;line-height:1.8}img{max-width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}h2{margin-top:35px}</style>
<h1>run1_tail_b1：Visionの不調を区間・更新順で切り分け</h1>
<p><b>以前の単眼ROVIO、現在の3カメラMIMOSA、単眼公式FAST-LIVO2は、別の現象として扱う必要がある。</b>既存出力・内部状態・更新ログ・ソースを分析。新しい推定器実験や設定変更はしていない。以下の時刻は特記しない限り評価開始61.8秒からの経過。</p>
<h2>1. 過去の単眼ROVIO＋MIMOSA：飛行前から入力VIOが破綻</h2>
<p>元記録10秒（追従開始の約52秒前）、真値速度は0に近いが単眼ROVIO速度0.546m/s。20秒で1.142m/s、30秒で1.853m/s。10秒の特徴点7個、推定距離の中央値1806m、最大4639m。3カメラでは同時刻20個、距離中央値9.68m、速度0.001m/s。単眼は特徴が完全消失したのではなく、残存特徴の距離・移動量の推定が崩れている。静止初期化での深度/移動量の不安定化を支持するが、画像のどの誤対応が起点だったかは保存ログだけでは確定しない。</p>
<p>追従開始後、単眼ROVIO位置誤差1m超0.6秒/10m超3.3秒、融合DRPM＋単眼は0.7秒/3.5秒。入力棄却なしで悪いROVIO移動量を融合している。トンネル中盤に入って初めてVisionが悪化したわけではない。3台化による改善は合計特徴上限25を維持した結果だが、実際の利用数・観測方向の両方が変わるため、それぞれの寄与は分離していない。</p>
<h2>2. 現在の3カメラMIMOSA：画像推定の破綻ではなく融合後の姿勢悪化</h2>
<p>全区間ROVIO単体0.266m/0.183°、通常MIMOSA＋ROVIO0.197m/0.563°、DRPM＋ROVIO0.374m/0.421°。通常MIMOSAは位置を改善している一方、姿勢はROVIO単体より悪い。3台ROVIOは全区間20個以上の特徴を維持、画像/推定周期の欠落なし。MIMOSA側の不確かさによる入力棄却も0件。</p>
<p>コードではROVIOの絶対姿勢をそのまま拘束せず、前時刻からの位置・姿勢の差だけを制約にする。LiDAR・IMUと同時に最適化するため、ROVIO単体の小さい絶対姿勢誤差が融合後もそのまま保証される構造ではない。最初の180°旋回で姿勢RMSEはROVIO0.124°、通常MIMOSA0.589°、DRPM0.411°。その後の支線からの帰路・主通路でも差が残る。</p>
<p>改善候補は、以前の別2飛行・単眼で校正した固定融合共分散を3台にも流用している点。例えば主通路でROVIO3台の50ms相対並進X誤差RMSEは1.17mmだが、重みの標準偏差は8.08mmで、良好な移動量を弱めに扱う設定。Yは実測0.47mm対設定0.33mmで逆方向。旋回/出入りでも誤差特性は変わる。これは設定の不適合を示す診断であり、同じ飛行で最適値を探した結果ではない。未処理の時間相関・共通IMU相関もあり、単純な統計的適合検定と解釈しない。LiDAR・IMU・重みのどれが姿勢劣化の主因かは追加の分離実験が必要。</p>
<h2>3. 公式FAST-LIVO2単眼：姿勢が先に崩れ、画像補正が戻せない</h2>
<p>約66秒に主通路の直進へ入り、約70秒の姿勢誤差0.49°から、75秒3.34°、77秒6.49°、79秒28.44°、80秒52.41°へ増大。主にロールが崩れる。最初の180°旋回は完了済みで、現在の暴走は旋回中ではない。</p>
<table><tr><th>経過秒</th><th>使えた画像地図点</th><th>その回のLiDAR姿勢変更[°]</th><th>画像姿勢変更[°]</th><th>画像更新後姿勢誤差[°]</th></tr>'''+events+'''</table>
<p>約79秒ではLiDAR更新2.35°に対してVision更新0.0106°。その時点で19点が使われており、特徴がゼロになるのは81.05秒から。79秒の画像更新は誤差を約0.0106°増やす方向でもある。したがって「カメラ停止/特徴ゼロ→姿勢破綻」という順番ではなく、「姿勢の悪化→画像更新で回復できない→画像対応も失う」という順番。全画像はコールバックまで届き、処理順の逆転も既存監査で除外済み。</p>
<p>FASTのVisionはLiDAR由来の3次元点と同じ推定姿勢・地図に画像を対応付ける。独立したROVIOの姿勢を採用する構造ではない。単調な通路でLiDAR/IMUの姿勢がずれると、画像の予測投影・参照パッチにも影響し、全体が一緒にずれるか、対応が失われる経路がある。コードは画面内判定・深度整合・パッチ誤差で候補を落とし、使える点0なら画像更新を実行しない。今回の時間順と整合するが、どの除外条件が支配したか、画像重みと地図の誤りのどちらが先に効いたかはログに内訳がなく未確定。</p>
<h2>区間別の誤差</h2><p>各セルは位置RMSE[m] / 姿勢RMSE[°]。すべて飛行開始時の同じ一度の整合を維持し、区間ごとの再整合なし。そのため後半の値には前区間からの誤差も含む。FASTは約94.3秒で発散判定による終了があり、主通路は途中まで、以降は記録なし。同じカメラ数の方式比較ではない（FAST1台、現MIMOSA3台）。</p><table><tr><th>区間</th><th>経過秒</th><th>ROVIO3</th><th>MIMOSA＋ROVIO3</th><th>DRPM＋ROVIO3</th><th>公式FAST単眼</th></tr>'''+''.join(rows)+'''</table><img src="diagnosis.png">
<h2>次の切り分け</h2><p>MIMOSAは「3カメラに対する重みの再校正を別の飛行で行う」「LiDAR制約とIMU制約のどちらが姿勢差を生むかを記録する」を優先。FASTは主通路に入る66〜82秒を対象に、画像の情報量・候補棄却理由・各センサ更新前後の状態/不確かさを記録し、画像重み不足と地図の誤りを分離する。現段階では“カメラが暗いから”や“画像が欠落したから”という説明は支持されない。</p><p><a href="analysis.json">全数値</a> / <a href="fast_updates.csv">FAST各画像更新の記録</a> / <a href="sources.json">入力ファイルと照合</a></p>'''
(OUT/'index.html').write_text(s)
files=[F for F in [B/'fast_official_raw'/L/'mat_pre.txt',B/'fast_official_raw'/L/'mat_out.txt',B/'fast_official_raw'/L/'estimator.log',ROOT/'logs/20261002-233545-95177/route_tracking.csv',ROOT/'logs/20261002-233545-95177/route/segments.json']]
for folder in ['drpm_three_camera','drpm_fast_failures']:
 for name in ['result_vio.csv','result_features.csv','result_state.csv']:files.append(B/folder/'rovio'/L/name)
for folder in ['drpm_three_camera','drpm_fast_failures','three_camera_no_drpm']:
 for name in ['odometry.csv','config.yaml']:files.append(B/folder/'fusion'/L/name)
files.append(B/'drpm_three_camera/calibration.json')
source={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};(OUT/'sources.json').write_text(json.dumps(dict(files=source,fast_visual_blocks_and_pose_pairs=1540,fast_timestamps_match_official_pose=True,phase_source='actual recorded route_t, not nominal simulation offset',estimator_rerun=False,estimator_modified=False),indent=2)+'\n')
for file in re.findall(r'(?:href|src)="([^"]+)"',s):assert (OUT/file).exists()
print('Saved report',OUT/'index.html')
