#!/usr/bin/env python3
"""Evidence report; does not modify estimators or recordings."""
import json,html,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from analyze_zab_runs import OUT,C,LABELS,pose
TEXT={
'run1_prefix':[
'入口付近から長い緩いカーブのトンネルを進み、北側の接続部へ至る約491m。LIO悪化地点は断面がほぼ一定の筒状区間。VIO悪化地点では画像の48〜63%が暗部で、壁の斑模様は残るが画面内の分布が偏る。',
'LIO: 進行方向の形状の手掛かり低下を強く支持。弱い方向は進行方向とほぼ一致し、位置誤差2mが先、姿勢誤差10度・60度が後に発生する。入口を離れて端面や接続部が見えなくなると前後移動を区別しにくくなる。後半の誤差には初期の破綻が持ち越されている。',
'VIO: 暗部と遠い開口への偏りによる距離・移動量の推定弱化が候補。短時間の画像追跡は残っており、単純な画像欠落や全面追跡不能ではない。18分程度の積算、深度初期化・少数特徴・IMUバイアスの寄与も未分離。'],
'run1_middle':[
'北側の長いトンネルを接続部・分岐を含めて約552m移動。開始付近は接続形状が見えるが、LIO悪化地点は長い均一な筒状区間。画像には斑模様と継ぎ目があり、VIO悪化地点の暗部は約30〜35%。',
'LIO: 進行方向の弱さに加え、姿勢が先に崩れる型。姿勢誤差10度が約144秒、位置誤差2mが約164秒。円弧状の壁・天井の対称性と、退化方向の保護が無い設定による誤対応/姿勢推定の不安定化が候補。実際の対応残差を保存していないため、姿勢破綻の直接原因は確定できない。',
'VIO: テクスチャは存在し、抽出点の短時間追跡も継続。低コントラスト・似た斑模様と長時間積算が候補だが、最大93mを暗さだけでは説明できない。特徴点の距離、尺度、ヨー、IMUバイアスを優先して確認すべき。'],
'run1_tail_a1':[
'北側の直線支線を往復する約101m。開始時は明るい接続部、LIO悪化時は非常に暗い筒状区間。暗部71〜85%、同じ検出条件で特徴点30〜71点/カメラ、画面の13〜21%の区画に集中。遠い明るい開口が目立つ。',
'LIO: 進行方向の形状情報が急減し、姿勢誤差が先に増えてほぼ180度まで崩れる。環境の退化を引き金とした姿勢推定破綻を疑う。4倍再生での異常終了とは別に、採用した等速再処理でもこの誤差が出ており、高速再生だけを原因にはできない。',
'VIO: 暗さと特徴の局在は悪条件だが、少数の残存点は短時間追跡でき、最大位置誤差は約3.5m。全面的な視覚推定の破綻ではない。往復後にもずれが残る点は積算誤差として説明可能だが、暗さの寄与を確定するには同経路の照明比較が必要。'],
'run1_tail_a2_prefix':[
'北側トンネルを約99mほぼ直進。LIO悪化地点の画像は明るく、190〜216点/カメラの模様の角が検出される。暗部0.4〜6%。形状は筒状で、進行方向の奥行きを区別する突起が少ない。',
'LIO: 画像の豊富さに関係なく幾何形状だけが退化する例。姿勢誤差10度が位置誤差2mより先で、最終的に姿勢がほぼ180度崩れる。単調な形状と姿勢推定の保護設定を組み合わせて疑う。',
'VIO: 「特徴が無い」が第一候補ではない。ほぼ一定速度・一定姿勢が長く続き、物体までの距離、移動量の尺度、バイアスの誤差が候補。約6.6mの誤差を生じるが、特徴追跡の破綻を示す内部記録は無い。'],
'run1_tail_b1':[
'約95mの接続部・横坑を含む経路。LIO悪化地点は明るく、横方向の開口/梁状の変化と壁の斑模様がある。398〜570点/カメラと特徴が多い。VIOが2mを超える地点はそれより前の筒状区間。',
'LIO: 形状の弱さは残るが、悪化地点の方向比0.070は他の重度退化地点より良い。「そこだけが最も単調だから壊れた」とは言えない。前区間での誤差蓄積、接続部での対応変更、地図との誤対応、姿勢/IMUの不整合が候補。最大7.5mから終端0.91mへ減るが、再拘束か誤差の相殺かは未確定。',
'VIO: 明るさや特徴数不足だけでは説明しにくい。VIO悪化開始地点を含む方向転換・停止の履歴、距離/ヨー/バイアスの積算誤差を候補とする。実際の追跡点数・更新残差が無く、どの更新が原因かは未確定。'],
'run1_tail_b2':[
'北側の幅のある緩いカーブを約158m移動。開口・設備・壁の斑模様が画像にあり、LIO悪化時でも384〜793点/カメラ、暗部0.1〜0.4%。一方、壁・天井の形状は長手方向に似ている。',
'LIO: 姿勢がほぼ反転する型ではなく、進行方向の移動を大幅に取り逃す型。開始〜終端の真値の移動156mに対し、同方向のLIO移動は約21m。方向情報比0.0062と進行方向の一致があり、長手方向の退化を強く支持する。',
'VIO: 特徴が豊富でも最大22.4mずれる。暗さよりも、繰り返し模様の対応、長い低加速度移動での尺度/ヨー/IMUバイアス、25点しか保持しない設定が候補。特徴数が多いことと正しい距離が分かることは別。'],
'run2a_prefix':[
'線路状の縦方向構造を持つ細い筒状区間を約119m直進。入口では端面が見え、内部へ進むと断面の繰り返しになる。LIO悪化時も237〜590点/カメラ、暗部11〜20%で画像特徴は残る。',
'LIO: 入口を離れた後の並進方向の弱化に加え、回転についても点群からの手掛かりが弱い。姿勢誤差10度が先に生じ、ほぼ180度へ崩れる。筒状構造の対称性、上寄りのLiDAR視野、退化対策OFFによる姿勢不安定化が候補。',
'VIO: 明確な全面暗転・特徴消失は見られない。ほぼ全区間が0.5m/sの一定速度で、尺度・ヨー・IMUバイアスの残りが有力候補。ただし3カメラのため単眼VIOの観測不能の理論をそのまま断定適用はできない。'],
'run2a_end':[
'同じ細い筒状トンネルの後半約23m。明るい壁と線路状の構造があり、183〜605点/カメラ、暗部約0.1%以下。LiDARでは長手方向に同じ断面が続く。',
'LIO: 今回最も切り分けやすい長手方向の退化例。方向情報比0.00185、弱い方向と進行方向がほぼ完全一致。真値が22.4m進む間、LIOの進行方向変位は約0m。姿勢誤差最大5.35度なので、姿勢反転よりも並進を観測できないことが主候補。',
'VIO: 最大位置誤差0.54m、姿勢誤差最大1.13度。今回のデータではVIOが大きく退化したrunではない。明るい模様と近い壁の見え方が有利だったという解釈に整合する。'],
'run2b':[
'細い筒状区間と接続通路を含む約125mの往復/方向転換経路。初期のLIO悪化地点は明るく、484〜541点/カメラ、暗部0.3〜2.6%。長手方向の形状は弱い。',
'LIO: 初期の位置誤差には幾何退化が整合するが、後半の約22.9kmの誤差は単純な軸方向ドリフトではなく推定器の発散。姿勢が約180度崩れた後にZが急減し、最後40秒のZ誤差は加速度約−13.1m/s²の二次曲線に近い。重力方向/IMUバイアス/誤った地図拘束の破綻が候補であり、環境だけを直接原因と断定しない。',
'VIO: 同じセンサー環境で最大約7.5m、姿勢誤差最大9.2度に留まる。完全な画像不良とは合わない。尺度・ヨー・バイアスの積算は候補だが、内部状態と残差を保存しておらず原因の一意特定はできない。']}

def main():
 profiles=json.loads((OUT/'profiles.json').read_text());mapping=json.loads((OUT/'environment_map.json').read_text());ev=json.loads((C/'error_plots/evaluation.json').read_text());rows=json.loads((OUT/'probe_summary.json').read_text())
 # Compact diagnostic curves have no axis breaks or hidden error intervals.
 for label in LABELS:
  fig,ax=plt.subplots(2,1,figsize=(10,5.5),sharex=True)
  for k,color in [('lio','#0072B2'),('vio','#D55E00')]:
   a=np.genfromtxt(C/'error_plots'/f'{label}_{k}_errors.csv',delimiter=',',names=True);xyz=np.column_stack([a[n] for n in ['ex_m','ey_m','ez_m']]);angle=Rotation.from_euler('xyz',np.column_stack([a[n] for n in ['eroll_deg','epitch_deg','eyaw_deg']]),degrees=True).magnitude()*180/np.pi
   ax[0].plot(a['elapsed_s'],np.linalg.norm(xyz,axis=1),color=color,label=k.upper());ax[1].plot(a['elapsed_s'],angle,color=color)
  anchor=ev[label]['anchor_ns']*1e-9
  for stage,t in profiles[label]['sample_times'].items():
   for sub in ax:sub.axvline(t-anchor,color='#999',ls=':',lw=.8)
  ax[0].legend();ax[0].set_ylabel('Position error [m]');ax[1].set_ylabel('Relative rotation error [deg]');ax[1].set_xlabel('Tracking elapsed time [s]');ax[0].set_title(label+' | Dotted lines: sampled sensor times')
  for sub in ax:sub.grid(alpha=.2)
  fig.tight_layout();fig.savefig(OUT/(label+'_diagnostic.png'),dpi=130);plt.close(fig)
 h=['<!doctype html><html lang="ja"><meta charset="utf-8"><title>ZaB run別・環境と推定誤差の調査</title><style>body{max-width:1250px;margin:35px auto;padding:0 20px;font-family:sans-serif;color:#202830;line-height:1.75}h1,h2{line-height:1.4}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #ccd3d9;padding:8px;vertical-align:top}th{background:#edf2f5}img{max-width:100%}.cameras{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.card{background:#f4f7fa;padding:18px;margin:18px 0}section{border-top:2px solid #cbd2d9;padding-top:20px;margin-top:40px}.small{font-size:13px;color:#52606b}</style>']
 h+=['<h1>ZaB：指定9飛行の環境とLIO/VIO誤差</h1><p><b>LIOの多くは、トンネルの長手方向を点群から判別しにくいことが強い原因候補です。VIOは暗い区間の影響もありますが、特徴が豊富な区間でもずれるため、環境だけでは説明できません。</b></p>', '<p>run2_tail_b2は確認済みのrun1_tail_b2として調査。元記録・推定結果・設定は変更していません。以下では観測事実と原因候補を区別します。原因確定のための設定変更比較実験は未実施です。</p>']
 h+=['<div class="card"><b>調べたもの</b><p>全対象飛行の真値/LIO/VIO、Gazeboの形状・配置、元センサー記録から抽出した'+str(len(rows))+'時点の点群と3カメラ画像（各時点0.2〜1.1秒の連続画像）、保存された校正・推定器設定・ソースコード。元MCAPはMD5照合済み。画像はROVIOと同じ校正・480×360の入力変換を使用。</p><p>点群の途中欠落は無く、LIO出力も10Hzで連続。VIOも20Hzで連続し、run1_middleの末尾2時刻だけ未出力。これは以前説明した記録終端の問題で、途中のドリフトを説明しません。</p></div>']
 h+=['<p><a href="#method">比較方法と制約</a> | <a href="#settings">環境以外の共通要因</a> | <a href="#next">次に原因を切り分ける実験</a></p><table><tr><th>run</th><th>実飛行距離</th><th>最大位置誤差 LIO / VIO</th><th>LIO悪化時の方向情報比</th><th>同時刻の画像特徴点数/カメラ</th></tr>']
 for label in LABELS:
  p=profiles[label];probe=next(r for r in rows if r['label']==label and r['stage']=='lio_onset');h.append(f'<tr><td><a href="#{label}">{label}</a></td><td>{p["length_m"]:.0f} m</td><td>{p["lio"]["max_position_m"]:.2f} / {p["vio"]["max_position_m"]:.2f} m</td><td>{probe["ratio"]:.5f}</td><td>{min(probe["corners"])}–{max(probe["corners"])}</td></tr>')
 h+=['</table><p class="small">誤差は開始時の位置・姿勢を一度だけ真値へ合わせた後の3次元位置誤差の最大値。方向情報比と画像特徴点は本調査の補助指標で、MIMOSA/ROVIO内部の指標ではありません。</p><img src="all_routes.png" alt="全飛行の真値軌道と環境形状">']
 for label in LABELS:
  p=profiles[label];anchor=ev[label]['anchor_ns']*1e-9;h.append(f'<section id="{label}"><h2>{label}</h2><p><b>観測した環境：</b>{TEXT[label][0]}</p><p><b>原因候補：</b>{TEXT[label][1]}</p><p>{TEXT[label][2]}</p>')
  h.append(f'<p class="small">開始後 {p["end_s"]-p["start_s"]:.1f}秒、実飛行距離{p["length_m"]:.1f}m。元run: {p["source_run"]}。主な近接メッシュ: '+', '.join(x[0].replace('model://','') for x in mapping[label]['nearest_models'][:3])+'。推定入力の再生速度上限: '+str(p['rate_cap'])+'倍。</p>')
  h.append(f'<img src="{label}_map.png" width="600"><img src="{label}_diagnostic.png"><table><tr><th>推定器</th><th>位置誤差2m超</th><th>姿勢誤差10°超</th><th>姿勢誤差60°超</th><th>終端位置誤差</th></tr>')
  for k in ['lio','vio']:
   def timing(key):return f'{p[k][key]-anchor:.1f}s' if p[k][key] is not None else 'なし'
   h.append(f'<tr><td>{k.upper()}</td><td>{timing("position_2m_at")}</td><td>{timing("rotation_10deg_at")}</td><td>{timing("rotation_60deg_at")}</td><td>{p[k]["end_position_m"]:.2f}m</td></tr>')
  h.append('</table><p class="small">表の時刻は追従開始の共通評価時刻からの経過時間。姿勢誤差はEuler角1軸の値ではなく、真値との相対回転角。</p>')
  for stage,t in sorted(p['sample_times'].items(),key=lambda x:x[1]):
   d=OUT/(label+'_'+stage);m=json.loads((d/'metrics.json').read_text());probe=next(x for x in rows if x['label']==label and x['stage']==stage);ld=m['lidar'];h.append(f'<details><summary>{stage} — 経過{t-anchor:.1f}s / 元時刻{t:.3f}s：画像と点群の数値</summary><p>点群有効点 {ld["valid_0p8_30m"]:,}/11,520、方向情報比 {ld["weak_to_strong_ratio"]:.5f}、弱い方向と進行方向の一致度 {ld["weak_alignment_to_motion"]:.4f}。回転の方向情報比 {ld["rotation_weak_ratio"]:.5f}。</p><div class="cameras">')
   for link in [1,3,4]:
    c=m['cameras'][str(link)];h.append(f'<div><img loading="lazy" src="{d.name}/cam_link{link}.png"><p class="small">link{link}：特徴点{c["corners"]}、分布範囲{c["corner_grid_coverage"]*100:.0f}%、暗部{c["dark_fraction"]*100:.1f}%、{c.get("tracked_duration_s",0):.2f}秒の追跡残存{c.get("lk_survival_fraction",0)*100:.1f}%</p></div>')
   h.append('</div></details>')
  h.append('</section>')
 h+=['''<section id="settings"><h2>環境以外の共通要因：確認済みの設定と未確定の因果</h2>
<p><b>LIO：</b>点群から動きを判別できない方向への更新を抑える設定 <code>project_on_degneneracy=false</code>、重力方向を固定した4自由度の位置合わせ <code>reg_4_dof=false</code>。したがって、今回のLIOにはこれらの保護は働いていません。形状の対称性によって姿勢や進行方向が誤った解へ動く危険を増やす候補です。ONにすれば今回直るという比較結果はまだありません。</p>
<p>LiDARは水平360点×垂直32本、距離30mまで、垂直視野−7°〜+52°で上寄りです。姿勢によりますが壁・天井に偏り、近い床の幾何を十分に使えない構成です。MIMOSAは点を2点に1点へ減らし、0.5mの空間単位で整理しています。環境の小さな形状差を弱める候補ですが、本調査の指標を実際の対応点集合と同一視していません。</p>
<p><b>VIO：</b>3カメラ使用、カメラ間の観測利用はON。ただしカメラ間視差による特徴点の距離の初期化 <code>doStereoInitialization=false</code>、追跡可能な特徴は全体で25点です。3カメラにしただけで、安定したステレオ距離が全点へ与えられる設定ではありません。実際の有効点数・距離分布・カメラ間の対応数を保存していないため、その効果は未確定です。</p>
<p><b>IMUと運動：</b>シミュレーションには加速度の固定偏り（bias_mean=0.1m/s²）が設定されています。単調な一定速度運動では、この偏り・姿勢・物体までの距離の誤差を分離しにくくなります。対象区間の多くは速度0.5m/s。特にrun1_tail_a2_prefix/run2a_prefixは1秒差分で約98%の時間が加速度0.02m/s²未満です。これは「一定速度だから必ずVIOが失敗する」という意味ではありません。3カメラの視差情報もあり、推定したバイアスを取得しないと原因の確定はできません。</p>
<p><b>否定できる説明／否定できない説明：</b>元点群・画像の途中欠落は確認されず、抽出時点の関節角も固定で変動0。LIO悪化の抽出時点は真値速度約0.5m/s、1秒平均の姿勢変化0.02°/s未満なので、急旋回が全runの共通トリガーという説明には合いません。高速再生はrun1_prefix/middleだけ4倍上限、残りは1倍上限でも発散しており、速度だけが原因ではありません。ただし処理順序・各推定器内部のIMU消費状況を完全に監査するログはなく、再生実装の寄与をゼロと断言はしません。</p>
<p>run2bの極端な発散は、環境が引き金でも、その後の状態推定の破綻を分離して扱うべきです。空の警告ログは正常性を保証しません。重力方向・バイアス・対応残差・推定の不確かさの内部ログが必要です。</p></section>
<section id="method"><h2>比較方法と制約</h2>
<p>各runの追従開始直後、LIOの位置誤差が2mまたは姿勢誤差が10°を初めて超える付近、必要に応じVIOの2m超・LIOの10°超の追加時点を選びました。この閾値は調査箇所を選ぶ目安であり、アルゴリズムの正式な退化判定ではありません。環境の途中全フレームを解析した結果ではなく、選択した時点の測定です。</p>
<p>点群は0.8〜30mの有効点を使い、0.3m単位で重複を減らし、近傍16点から平面の向きを求めました。平面法線nについて平均(n nᵀ)の最小/最大固有値を方向情報比とします。1に近いほど方向の偏りが少なく、0に近いほど特定方向の移動を判別しにくい指標です。弱い方向と実際の移動方向の一致度は絶対内積で、1が平行です。距離・対応の安定性・IMU拘束を含まない補助指標なので、MIMOSAの内部の退化判定そのものではありません。</p>
<p>画像の特徴点は全画像に同じShi–Tomasi条件（最大1000点、相対閾値0.01、間隔8px）を使いました。分布範囲は8×6の区画のうち点がある割合。暗部は8bit輝度10未満。短時間追跡は0.2〜1.1秒のLucas–Kanade法と往復1px以内の検査で、ROVIO内部の実際の採用点・追跡成功率ではありません。元分割ファイルの境界で短い窓があるため、残存率のrun間の単純比較はしません。斑模様の検出点が多くても正しい奥行きや長期の対応を保証しません。</p>
<p>誤差は初期の位置・姿勢を一度だけ真値へ合わせ、真値を推定の元取得時刻へ補間した値。尺度の補正・途中再整合はしていません。原本のCSV/MCAP/校正・推定パラメータを変更していません。</p></section>
<section id="next"><h2>次に行うと原因を切り分けやすい比較</h2>
<ol><li>run2a_endを同じ入力で再処理し、LIOの退化方向の保護だけをON/OFF比較。方向別の情報量・対応残差・IMUバイアスも保存する。</li>
<li>run1_tail_a1とrun2bで、姿勢が崩れる直前を同条件で再現し、重力方向を固定する位置合わせのON/OFFを別々に比較。入力順序も監査し、幾何退化と推定実装の問題を分離する。</li>
<li>VIOはrun1_tail_a2_prefix/run1_tail_b2で、まず実際の有効特徴点数・分布・深度・カメラ間対応数・IMUバイアスを記録。次に特徴数とステレオ距離初期化を一要因ずつ比較する。カメラ視野が重なる点があるかも確認する。</li>
<li>暗さの影響はrun1_tail_a1で同経路・同速度の照明変更比較、一定速度の影響は明るい同経路で緩い加減速だけを追加して比較する。元の今回の結果は比較基準として保持する。</li></ol>
<p>以上は提案であり、まだ再実験や設定変更はしていません。</p></section>
<section><h2>根拠データ・資料</h2>
<p><a href="profiles.json">誤差・運動集計</a> / <a href="probe_summary.json">画像・点群の測定値</a> / <a href="environment_map.json">通過場所とモデル</a>。各撮影箇所のmetrics.jsonには元run、分割ファイル名、取得時刻を保存しています。</p>
<p>ZaB自体が長い均質な構造のLiDAR評価用環境であることは<a href="https://zenodo.org/records/17407065">配布元の説明</a>にも明記されています。単眼VIOで低加速度時の尺度が弱くなることの理論的背景は<a href="https://arxiv.org/abs/2103.15215">Range–Visual–Inertial Odometry: Scale Observability Without Excitation</a>。この単眼の結果を今回の3カメラ構成へ直接断定適用してはいません。<a href="https://github.com/ethz-asl/rovio">ROVIO公式説明</a>もカメラ・IMU間の校正への感度を述べていますが、今回は校正不良を実証したわけではありません。</p>
<p class="small">ローカル設定: native/mimosa_dragon.yaml、各元run/rovio.info・dragon_slam.urdf、slam_ros1_ws/build/rovio/CMakeCache.txt。退化処理実装: mimosa/mimosa/include/mimosa/lidar/geometric_factor.hpp。画像変換: native/vio_images.py。</p></section></html>''']
 (OUT/'report.html').write_text('\n'.join(h))
 (OUT/'interpretation.json').write_text(json.dumps(TEXT,ensure_ascii=False,indent=2)+'\n')
 print('report complete',len(rows),'sensor times')
if __name__=='__main__':main()
