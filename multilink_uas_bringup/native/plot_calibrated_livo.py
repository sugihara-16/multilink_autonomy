#!/usr/bin/env python3
"""Original timestamps, common 10Hz evaluation, one initial rigid alignment."""
import csv,json,re,warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from plot_livo_comparison import read,interpolate,LABELS

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'logs/zab-offline/livo_comparison';OUT=ROOT/'logs/zab-offline/livo_calibrated'
NAMES={'mimosa_lio':'MIMOSA LIO','fast_lio':'FAST-LIVO2: LiDAR + IMU',
 'fast_tracking_no_visual':'FAST-LIVO2: vision off after tracking start',
 'fast_no_visual':'FAST-LIVO2: visual update disabled (same timing)','fast_full':'FAST-LIVO2: full',
 'mimosa_calibrated':'MIMOSA + calibrated ROVIO','mimosa_drpm_calibrated':'MIMOSA + DRPM + calibrated ROVIO',
 'old_drpm':'MIMOSA + DRPM (old weights)','rovio':'ROVIO alone'}
COLORS={'mimosa_lio':'#0072B2','fast_lio':'#D55E00','fast_no_visual':'#CC79A7','fast_full':'#009E73',
        'fast_tracking_no_visual':'#AA4499',
        'mimosa_calibrated':'#0072B2','mimosa_drpm_calibrated':'#D55E00','old_drpm':'#888888','rovio':'#56B4E9'}

def source(label,name):
    if name=='old_drpm':return BASE/'mimosa_drpm'/label/'odometry.csv'
    if name=='rovio':return BASE/'rovio'/label/'result_vio.csv'
    return OUT/name/label/'odometry.csv'

def evaluate(label):
    w=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window']
    truth=read(BASE/'inputs'/label/'truth.csv');start=w['start_ns'];end=w['end_ns']
    # LiDAR measurements occur at integer 100ms source timestamps in these bags.
    grid=np.arange(((start+99999999)//100000000)*100000000,end+1,100000000,dtype=np.int64)
    gp,gq=interpolate(truth,grid);g0,q0=interpolate(truth,[start]);data={};summary={}
    for name in NAMES:
        a=read(source(label,name));p0,r0=interpolate(a,[start]);R=q0[0]*r0[0].inv();T=g0[0]-R.apply(p0[0])
        p,q=interpolate(a,grid);ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore');ea=er.as_euler('xyz',degrees=True)
        errors=np.c_[ep,ea];angle=np.rad2deg(er.magnitude());native=a[0][(a[0]>=start)&(a[0]<=end)]
        m=dict(position_rmse_m=float(np.sqrt((ep**2).sum(axis=1).mean())),
               rotation_rmse_deg=float(np.sqrt((angle**2).mean())),position_max_m=float(np.linalg.norm(ep,axis=1).max()),
               rotation_max_deg=float(angle.max()),position_end_m=float(np.linalg.norm(ep[-1])),
               samples_on_common_grid=len(grid),native_tracking_samples=len(native),
               native_max_gap_s=float(np.diff(native).max()*1e-9),
               alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist())
        if name not in ['old_drpm','rovio']:
            dest=OUT/name/label;s=json.loads((dest/'summary.json').read_text());assert s['returncode']==0
            m['wall_s']=s['wall_s'];log=(dest/'run.log').read_text()
            if name.startswith('fast'):
                n=re.search(r'INPUT_COUNTS (\d+) (\d+) (\d+)',log);u=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',log)
                assert n and u;m['input_counts']=list(map(int,n.groups()));m['update_counts']=dict(zip(['lio','visual','skipped_visual'],map(int,u.groups())))
                if name=='fast_lio':assert m['input_counts'][2]==0 and m['update_counts']['visual']==0
                if name=='fast_no_visual':assert m['update_counts']['visual']==0 and m['update_counts']['skipped_visual']>0
                if name=='fast_full':assert m['update_counts']['visual']>0
                if name=='fast_tracking_no_visual':assert m['update_counts']['visual']>0 and m['update_counts']['skipped_visual']>0
            else:
                m['covariance_loaded']='Using calibrated full relative-pose covariance' in log
                assert m['covariance_loaded']==(name!='mimosa_lio')
                m['covariance_rejections']=log.count('is higher than threshold')
                m['drpm_logged_updates']=log.count('DRPM probabilities')
        # Original pose timestamps are preserved in native output. Evaluation grid is explicitly marked.
        with (OUT/'plots'/f'{label}_{name}_errors.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);writer.writerows(zip(grid,*errors.T))
        data[name]=errors;summary[name]=m
    return grid,data,dict(window=w,metrics=summary)

def failure_onsets():
    result={}
    for label in LABELS:
        g=read(BASE/'inputs'/label/'truth.csv');start=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window']['start_ns'];result[label]={}
        for method in ['fast_full','fast_lio','fast_no_visual','fast_tracking_no_visual']:
            t,p,r=read(source(label,method));keep=(t>=g[0][0])&(t<=g[0][-1]);t,p,r=t[keep],p[keep],r[keep]
            gp,gr=interpolate(g,t);R=gr[0]*r[0].inv();T=gp[0]-R.apply(p[0])
            error=np.linalg.norm(R.apply(p)+T-gp,axis=1);ids=np.flatnonzero(error>5);first=int(t[ids[0]]) if len(ids) else None
            log=(OUT/method/label/'run.log').read_text();matches=re.findall(r'effective feature num: (\d+)',log)
            result[label][method]=dict(first_position_error_above_5m_stamp_ns=first,
                relative_to_tracking_start_s=(first-start)*1e-9 if first else None,
                alignment='first available pose; diagnostic only, distinct from tracking-only RMSE alignment',
                zero_effective_feature_log_lines=sum(int(x)==0 for x in matches),effective_feature_log_lines=len(matches))
    (OUT/'failure_onset.json').write_text(json.dumps(result,indent=2))

def main():
    failure_onsets()
    plots=OUT/'plots';plots.mkdir(exist_ok=True);all_results={}
    groups={'vision_ablation':['mimosa_lio','fast_lio','fast_no_visual','fast_full'],
            'tracking_vision_ablation':['fast_full','fast_tracking_no_visual'],
            'weight_effect':['old_drpm','rovio','mimosa_calibrated','mimosa_drpm_calibrated'],
            'calibrated_fusion':['rovio','mimosa_calibrated','mimosa_drpm_calibrated']}
    with PdfPages(plots/'comparison.pdf') as pdf:
        for label in LABELS:
            grid,data,meta=evaluate(label);all_results[label]=meta
            for group,methods in groups.items():
                fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
                for j,ax in enumerate(axes):
                    for name in methods:
                        y=data[name][:,j].copy()
                        if j>=3:y[np.r_[False,abs(np.diff(y))>180]]=np.nan
                        ax.plot((grid-meta['window']['start_ns'])*1e-9,y,label=NAMES[name],color=COLORS[name],lw=1.15)
                    ax.grid(alpha=.25);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j])
                axes[0].legend(fontsize=9,ncol=2);axes[-1].set_xlabel('Tracking time [s]')
                fig.suptitle(label+' | '+group);fig.tight_layout(rect=(0,.015,1,.98));fig.savefig(plots/f'{label}_{group}.png',dpi=145);pdf.savefig(fig);plt.close(fig)
    (OUT/'evaluation.json').write_text(json.dumps(all_results,indent=2))
    rows=''
    for label,result in all_results.items():
        for method,m in result['metrics'].items():
            rows+=f'<tr><td>{label}</td><td>{NAMES[method]}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td></tr>'
    images=''.join(f'<h2>{label}: {group}</h2><img src="{label}_{group}.png">' for label in LABELS for group in groups)
    calibration=json.loads((OUT/'calibration.json').read_text());crows=''
    for label,fold in calibration['folds'].items():
        crows+=f'<tr><td>{label}</td><td>{fold["training_flight"]}</td><td>'+', '.join(f'{x:.6f}' for x in fold['sigma_rotation_deg'])+'</td><td>'+', '.join(f'{x*1000:.3f}' for x in fold['sigma_translation_m'])+'</td></tr>'
    html='''<!doctype html><meta charset="utf-8"><title>ROVIO校正・LIVO画像更新の比較</title>
<style>body{font:16px sans-serif;max-width:1450px;margin:30px auto;padding:0 16px}img{width:100%}td,th{border:1px solid #bbb;padding:8px}table{border-collapse:collapse}p{line-height:1.7}</style>
<h1>ROVIO校正と画像更新の効果</h1>
<p><strong>今回の2例では、FAST-LIVO2の画像更新が位置精度と推定の安定性に大きく寄与しました。</strong>画像なし条件のkm単位の誤差は推定の発散です。短いrunは追従開始前に発散したため、追従開始後だけ画像を止める追加対照も掲載しています。この追加対照の短いrunでも位置RMSEは約13.77m（画像あり約0.035m）になりました。</p>
<p>同じlink3カメラ1台・LiDAR・IMU、前回の2飛行。ROVIOは前回の単眼推定を再利用し、50msの相対姿勢誤差を測定。一方の飛行で校正した固定の6×6行列を他方へ使用。追従区間のみで校正・評価し、推定器には離陸前からの入力を与えています。</p>
<p>全方式を共通10Hz時刻で比較。追従開始時の剛体整合は一度だけ。縦軸の省略なし。元の推定時刻は各方式のodometry.csvに保持し、誤差CSVは補間評価時刻を明記しています。</p>
<p>DRPMは前回と同じ情報量の重み付けによる移植。原論文の独立ICP再現ではありません。共通IMU・時間相関は未補正。校正後も残る誤差や悪化を含めて表示します。</p>
<p><a href="comparison.pdf">全比較PDF</a> / <a href="../evaluation.json">評価・検証値</a> / <a href="../calibration.json">ROVIOの誤差特性と校正</a> / <a href="../README.md">解釈と実験条件</a></p>
<h2>基準の重み：標準偏差相当</h2><p>並びは回転X,Y,Z／並進X,Y,Z。実際には軸間成分も含む6×6行列を使用。飛行ごとに評価対象とは別の飛行で校正しています。</p>
<table><tr><th>評価飛行</th><th>校正飛行</th><th>回転 [deg]</th><th>並進 [mm]</th></tr>'''+crows+'''</table>
<h2>共通10Hzでの誤差</h2><table><tr><th>飛行</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'''+rows+'</table>'+images
    html+=''.join(f'<h2>{label}: ROVIO相対姿勢誤差</h2><img src="../calibration/{label}/relative_errors.png">' for label in LABELS)
    (plots/'index.html').write_text(html)
    print(json.dumps({l:{k:{v:m[v] for v in ['position_rmse_m','rotation_rmse_deg']} for k,m in r['metrics'].items()} for l,r in all_results.items()},indent=2))

if __name__=='__main__':main()
