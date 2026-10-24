#!/usr/bin/env python3
"""Compare actual monocular velocity constraints, keeping fusion error separate."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_direction_repeats import trial, ORDER, LABEL, COLORS
from report_routes import load, poses, interp
from window_metrics import stats


def aggregate(values):
    x=np.asarray(values,float)
    return dict(mean=float(x.mean()),sd=float(x.std(ddof=1)) if len(x)>1 else None,
                min=float(x.min()),max=float(x.max()),values=x.tolist())


def compare(records, fixed):
    """Paired flights, not correlated windows, are the replication unit."""
    a=np.array([r['methods']['active']['visual']['rmse'] for r in records])
    f=np.array([r['methods'][fixed]['visual']['rmse'] for r in records])
    return dict(direction=fixed,n=len(a),fixed_wins=int((f<a).sum()),
                active_wins=int((a<f).sum()),ties=int((a==f).sum()),
                fixed=aggregate(f),active=aggregate(a),
                fixed_minus_active=aggregate(f-a),
                reduction_of_mean_percent=float(100*(1-f.mean()/a.mean())))


def heldout(records):
    out=[]
    if len(records)<2:return out
    for i,test in enumerate(records):
        train=[r for j,r in enumerate(records) if j!=i]
        chosen=min(ORDER[:5],key=lambda n:np.mean([r['methods'][n]['visual']['rmse'] for r in train]))
        out.append(dict(test_trial=test['trial'],training_trials=[r['trial'] for r in train],
                        selected=chosen,fixed_mps=test['methods'][chosen]['visual']['rmse'],
                        active_mps=test['methods']['active']['visual']['rmse']))
    return out


def extra_samples(root,key,samples,rows):
    """Add full vector error and gravity tilt without changing the estimator."""
    f=root/key/'campaign'/(key+'_active');gt=poses(f/'truth.csv')
    enc=load(f/'encoders.csv');d=load(f/'route_tracking.csv')
    good=d['route_t']>0;t0=float(d['sim_t'][good][0]-d['route_t'][good][0])
    angles=np.column_stack([enc['yaw'],enc['pitch']])
    rate=np.rad2deg(np.linalg.norm(np.gradient(angles,enc['t'],axis=0),axis=1))
    paths={n:f/'views'/n for n in ORDER[:-1]};paths.update(front=f/'fixed',active=f)
    result=[]
    for n,p in paths.items():
        v=load(p/'mimosa/velocity_factors.csv');ts=v['stamp_ns'];_,q=interp(gt,ts)
        before,_=interp(gt,ts-1e7);after,_=interp(gt,ts+1e7)
        truth_velocity=(after-before)/.02
        estimated_velocity=q.apply(np.column_stack([v['v'+c] for c in 'xyz']))
        error=estimated_velocity-truth_velocity
        tilt=load(p/'mimosa/tilt_factors.csv');_,qt=interp(gt,tilt['stamp_ns'])
        up=np.column_stack([tilt['up_'+c] for c in 'xyz']);up/=np.linalg.norm(up,axis=1)[:,None]
        actual=qt.inv().apply(np.tile([0.,0.,1.],(len(qt),1)))
        tilt_error=np.rad2deg(np.arccos(np.clip(np.einsum('ni,ni->n',up,actual),-1,1)))
        features=load(p/'vio_diagnostic.csv')
        ix={round(float(t)*1e-9,6):i for i,t in enumerate(ts)}
        for s in samples:
            if s['method']!=n:continue
            j=ix[round(s['stamp_s'],6)];t=s['stamp_s'];e=error[j]
            r=dict(s,elapsed_s=t-t0,error_xyz_mps=e.tolist(),norm_error_mps=float(np.linalg.norm(e)),
                   truth_velocity_mps=truth_velocity[j].tolist(),vio_velocity_world_mps=estimated_velocity[j].tolist(),
                   true_speed_mps=float(np.linalg.norm(truth_velocity[j])),
                   gimbal_rate_deg_s=float(np.interp(t,enc['t'],rate)),
                   yaw_deg=float(np.rad2deg(np.interp(t,enc['t'],enc['yaw']))),
                   pitch_deg=float(np.rad2deg(np.interp(t,enc['t'],enc['pitch']))),
                   tilt_error_deg=float(np.interp(t,tilt['stamp_ns']*1e-9,tilt_error)),
                   features=float(np.interp(t,features['stamp_ns']*1e-9,features['features'])))
            result.append(r)
    return result


def reduce_trial(key,ss,rr):
    methods={}
    for n in ORDER[:-1]:
        s=[r for r in ss if r['method']==n];w=[r for r in rr if r['method']==n]
        e=np.array([r['error_mps'] for r in s]);sd=np.array([r['sigma_mps'] for r in s])
        vec=np.array([r['error_xyz_mps'] for r in s]);tilt=np.array([r['tilt_error_deg'] for r in s])
        normalized=e/sd
        methods[n]=dict(visual=stats(e),vector_rmse_mps=float(np.sqrt(np.mean(np.sum(vec*vec,axis=1)))),
                        world_bias_mps=vec.mean(axis=0).tolist(),sigma_median_mps=float(np.median(sd)),
                        normalized_rms=float(np.sqrt(np.mean(normalized**2))),
                        outside_2sigma_fraction=float(np.mean(abs(normalized)>2)),
                        tilt_rmse_deg=float(np.sqrt(np.mean(tilt**2))),
                        feature_median=float(np.median([r['features'] for r in s])),
                        fused_displacement=stats([r['error_m'] for r in w]),
                        integrated_velocity_error_rms_m=float(np.sqrt(np.mean([(r['velocity_bias_mps']*r['window_s'])**2 for r in w]))))
    unique=[r for r in rr if r['method']=='active']
    active=[r for r in ss if r['method']=='active']
    groups={}
    for name,check in [('moving',lambda r:r['gimbal_rate_deg_s']>=1),('still',lambda r:r['gimbal_rate_deg_s']<1)]:
        stamps={r['stamp_s'] for r in active if check(r)}
        groups[name]={n:stats([r['error_mps'] for r in ss if r['method']==n and r['stamp_s'] in stamps]) for n in ORDER[:-1]}
    return dict(trial=key,methods=methods,weak_windows=sum(r['weak'] for r in unique),windows=len(unique),
                gimbal_rate_median_deg_s=float(np.median([r['gimbal_rate_deg_s'] for r in active])),
                moving_fraction=float(np.mean([r['gimbal_rate_deg_s']>=1 for r in active])),motion_groups=groups)


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
    manifest=json.loads((root/'campaign.json').read_text());rows=[];samples=[]
    for key in manifest['order']:
        if manifest['runs'].get(key,{}).get('status')!='complete':continue
        route=key.rsplit('_seed',1)[0]
        rr,ss=trial(root,key,manifest['design']['segments_s'][route])
        samples.extend(extra_samples(root,key,ss,rr));rows.extend(rr)
    if not rows:raise RuntimeError('No completed trials')
    out=root/'report';out.mkdir(exist_ok=True)
    result=dict(complete=manifest['status']=='complete',design=manifest['design'],sections={})
    for route,bounds in manifest['design']['segments_s'].items():
        for seg,interval in enumerate(bounds):
            windows={}
            for W in [5,10,20]:
                rr=[r for r in rows if r['route']==route and r['segment']==seg and r['window_s']==W]
                ss=[r for r in samples if r['route']==route and r['segment']==seg and r['window_s']==W]
                keys=sorted(set(r['trial'] for r in rr))
                if not keys:continue
                records=[reduce_trial(k,[r for r in ss if r['trial']==k],[r for r in rr if r['trial']==k]) for k in keys]
                pairs={n:compare(records,n) for n in ORDER[:5]}
                windows[str(W)]=dict(trials=records,comparisons=pairs,heldout=heldout(records))
            if windows:result['sections'][f'{route}_{seg}']=dict(route=route,segment=seg,bounds_s=interval,windows=windows)
    for name,records in [('windows',rows),('visual_samples',samples)]:
        with (out/(name+'.csv')).open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in records for k in r)));w.writeheader();w.writerows(records)
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    render(out,result)
    sources=[Path(__file__),Path(__file__).with_name('report_direction_repeats.py'),Path(__file__).with_name('window_metrics.py'),Path(__file__).with_name('report_routes.py')]
    (out/'evaluation_sources.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},indent=2)+'\n')
    print(json.dumps(dict(complete=result['complete'],sections=len(result['sections']),report=str(out))))


def render(out,result):
    html='''<!doctype html><meta charset="utf-8"><title>固定視野と偏向：VIO拘束自体の精度</title>
<style>body{font:16px sans-serif;line-height:1.65;max-width:1500px;margin:30px auto;padding:0 15px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}img{max-width:100%}.good{background:#dff2df}</style>
<h1>固定視野と偏向：融合前の視覚速度の精度</h1>
<p>ZaB a1/b1・二ツ小屋、各3新規飛行。各固定方向は独立した単眼ROVIO。採用v57・DRPM OFF・融合重みを維持。区間・試行数は結果を見る前に固定し、全方向の成績と失敗を保存しています。主目的は固定優位のケースの収集であり、環境全体での発生割合の推定ではありません。</p>
<p>主指標はMIMOSAに入力したVIO速度の誤差[m/s]。共通のLiDAR弱方向へ投影し、10秒ごとに方向を更新。方向は純LIOの情報行列の並進部分（回転固定）の最小固有値方向です。ドリフトした地図に由来する診断基準であり、真の退化の保証ではありません。真値は採点にだけ使用。融合後の軌道誤差は別に記載します。</p>
<p>表は各飛行の誤差RMSの平均。勝数は独立飛行の対応比較。緑は固定が全試行で勝った比較で、統計的有意性を意味しません。7区間×5方向の探索結果です。区間内の窓を独立試行として数えません。固定方向の事後選択と、他の2試行だけで選ぶ評価を区別します。</p>
<p>σは実際の速度制約に渡した共分散から求めた方向別標準偏差で、融合後の影響力そのものではありません。傾き誤差は重力方向の誤差で、yawを含みません。機体真姿勢で速度を世界座標へ移し、速度そのものの誤差と姿勢の誤差を分離しています。</p>
<p>ZaB b1の180–240秒は直進と旋回・横坑への出入りが混在。60–120秒の冒頭にも旋回の末尾が含まれます。二ツ小屋とa1の直進区間とは条件が異なります。</p>
<p><a href="comparison.pdf">全比較PDF</a> / <a href="summary.json">全指標・全試行</a> / <a href="visual_samples.csv">時刻付き視覚速度誤差</a> / <a href="windows.csv">融合後の短区間誤差</a></p>'''
    with PdfPages(out/'comparison.pdf') as pdf:
        for key,section in result['sections'].items():
            d=section['windows']['10'];trials=d['trials'];N=len(trials)
            title=f"{section['route']} {section['bounds_s']} s"
            html+=f'<h2>{title} — {N}試行</h2><table><tr><th>固定(yaw,pitch)</th><th>固定速度誤差[m/s]</th><th>偏向速度誤差[m/s]</th><th>固定勝数</th><th>固定の低減率</th></tr>'
            for n,c in d['comparisons'].items():
                cls=' class="good"' if c['fixed_wins']==N else ''
                html+=f'<tr{cls}><td>{LABEL[n]}</td><td>{c["fixed"]["mean"]:.4f}</td><td>{c["active"]["mean"]:.4f}</td><td>{c["fixed_wins"]}/{N}</td><td>{c["reduction_of_mean_percent"]:.1f}%</td></tr>'
            html+='</table><p>他試行だけで固定方向を選択: '+', '.join(f'{t["test_trial"].split("seed")[-1]}: {LABEL[t["selected"]]} {t["fixed_mps"]:.4f} / 偏向 {t["active_mps"]:.4f}' for t in d['heldout'])+'</p>'
            html+='<details><summary>各試行の誤差・不確かさ・追跡特徴数</summary><table><tr><th>試行</th><th>方向</th><th>速度RMS</th><th>偏り</th><th>σ中央値</th><th>誤差/σ RMS</th><th>全方向速度RMS</th><th>傾きRMS[deg]</th><th>特徴数中央値</th><th>融合10秒誤差[m]</th></tr>'
            for t in trials:
                for n,v in t['methods'].items():
                    html+=f'<tr><td>{t["trial"].split("seed")[-1]}</td><td>{LABEL[n]}</td><td>{v["visual"]["rmse"]:.4f}</td><td>{v["visual"]["bias"]:.4f}</td><td>{v["sigma_median_mps"]:.4f}</td><td>{v["normalized_rms"]:.2f}</td><td>{v["vector_rmse_mps"]:.4f}</td><td>{v["tilt_rmse_deg"]:.3f}</td><td>{v["feature_median"]:.1f}</td><td>{v["fused_displacement"]["rmse"]:.4f}</td></tr>'
            html+='</table></details>'
            fig,axs=plt.subplots(2,2,figsize=(14,10))
            for ax,field,ylabel in [(axs[0,0],('visual','rmse'),'Actual VIO velocity RMS [m/s]'),(axs[0,1],('visual','bias'),'Signed VIO velocity bias [m/s]'),(axs[1,0],('fused_displacement','rmse'),'Fused 10 s displacement RMS [m]')]:
                for i,(n,color) in enumerate(zip(ORDER[:-1],COLORS[:-1])):
                    vals=[t['methods'][n][field[0]][field[1]] for t in trials]
                    ax.scatter(np.full(N,i)+np.linspace(-.15,.15,N),vals,color=color)
                    ax.plot([i-.2,i+.2],[np.mean(vals)]*2,color=color,lw=3)
                ax.set_xticks(range(6),[LABEL[n] for n in ORDER[:-1]],rotation=25,ha='right');ax.set_ylabel(ylabel);ax.grid(alpha=.25)
                if field[1]=='rmse':ax.set_yscale('log')
                else:ax.axhline(0,color='gray',lw=.7)
            for n,color in zip(ORDER[:-1],COLORS[:-1]):
                vals=[np.mean([t['methods'][n]['visual']['rmse'] for t in section['windows'][str(W)]['trials']]) for W in [5,10,20]]
                axs[1,1].plot([5,10,20],vals,'-o',color=color,label=LABEL[n])
            axs[1,1].set_xticks([5,10,20]);axs[1,1].set_xlabel('Duration holding weak direction [s]');axs[1,1].set_ylabel('VIO velocity RMS [m/s]');axs[1,1].set_yscale('log');axs[1,1].grid(alpha=.25);axs[1,1].legend(fontsize=8)
            fig.suptitle(title);fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(key+'.png'),dpi=130);plt.close(fig)
            html+=f'<p>誤差RMSは全試行を表示するため対数軸。偏りは線形軸。5/20秒は方向の更新周期を変えた確認です。</p><img src="{key}.png">'
    (out/'index.html').write_text(html)


if __name__=='__main__':main()
