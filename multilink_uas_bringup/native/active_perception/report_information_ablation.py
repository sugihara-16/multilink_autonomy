#!/usr/bin/env python3
"""Keep every trial and separate visual constraint quality from fused accuracy."""
import argparse,gzip,json,hashlib,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_switching_tunnel_sparse import evaluate
from report_routes import load

MODES=['lidar_conditioned','visual_only']
FIELDS=['coverage','complete_axial_velocity_rms_mps','vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m']
TITLES=['Accepted visual velocity coverage','Visual axial velocity RMS [m/s]','VIO axial displacement RMS / 10 s [m]','Fused axial displacement RMS / 10 s [m]','Full fused position RMSE [m]']

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root;m=json.loads((r/'campaign.json').read_text());out=r/'report';out.mkdir(exist_ok=True)
    result=dict(status=m['status'],design=m['design'],trials={},aggregates={},comparison={},source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());rows=[]
    for key in m['order']:
        if m['runs'].get(key,{}).get('status')!='complete':continue
        mode,seed=key.rsplit('_seed',1);record,samples=evaluate(r,key,m['design']['segments_s'])
        f=r/key/'campaign'/(key+'_active');enc=load(f/'encoders.csv');t0=record['tracking_start_s'];mask=(enc['t']>=t0)&(enc['t']<=t0+182)
        planners=list(f.glob('*plan*.jsonl*'))
        decisions={};labels={}
        for path in planners:
            if path.name not in ['planner.jsonl','planner.jsonl.gz']:continue
            op=gzip.open if path.suffix=='.gz' else open
            for line in op(path,'rt'):
                v=json.loads(line);reason=v.get('reason','unknown');decisions[reason]=decisions.get(reason,0)+1
                label=v.get('view_information_mode');labels[str(label)]=labels.get(str(label),0)+1
        metrics=dict(record['methods']['active']);metrics['full_fused_position_rmse_m']=record['full_fused']['active']['mimosa']['position_rmse_m']
        result['trials'][key]=dict(mode=mode,seed=int(seed),metrics=metrics,acceptance=m['runs'][key]['acceptance'],raw=record,decisions=decisions,mode_labels=labels)
        rows.extend(samples)
    for mode in MODES:
        trials=[v for v in result['trials'].values() if v['mode']==mode]
        result['aggregates'][mode]={field:dict(values=[t['metrics'][field] for t in trials],mean=float(np.mean([t['metrics'][field] for t in trials])) if trials and all(t['metrics'][field] is not None for t in trials) else None) for field in FIELDS}
    for field in FIELDS:
        pairs=[]
        for seed in m['design']['seeds']:
            a=result['trials'].get(f'lidar_conditioned_seed{seed}');b=result['trials'].get(f'visual_only_seed{seed}')
            if a and b:
                x,y=a['metrics'][field],b['metrics'][field]
                pairs.append(dict(seed=seed,lidar=x,visual=y,difference=None if x is None or y is None else x-y))
        result['comparison'][field]=pairs
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    html='''<!doctype html><meta charset="utf-8"><title>LiDAR方向情報の効果</title><style>body{font:16px sans-serif;max-width:1350px;margin:30px auto;line-height:1.7}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px}img{max-width:100%}</style><h1>視覚のみの偏向 vs LiDAR情報を使う偏向</h1><p>自然な偏在トンネル24201。各90m・0.5m/s、各方式3回。LiDAR行列を向きの評価に使うかだけを変更し、推定器・融合重み・特徴予測・保持・回復・回転制限・準備運動は共通。視覚のみでもLIVOは継続する。IGLOVの再現ではなく、提案方式の要素を取り除く比較。</p><p>方式間は別飛行。同じ要求seedでも画像やセンサ雑音の完全一致は保証しない。順番を交互にし、3反復を全件表示。古い3試行は主比較へ混ぜない。真値・模様配置は偏向の評価に使用しない。</p><p>巡航10〜170秒の視覚速度誤差と利用率が主指標。10秒移動・全長融合位置誤差も併記。尺度補正なし。誤差相殺を単独で証明する試験ではない。角度保持による元のplanner_continuous=falseを別記し、取得合格と混同しない。</p><p><a href="comparison.pdf">PDF</a> / <a href="summary.json">全数値</a> / <a href="../campaign.json">事前条件・進捗</a></p>'''
    html+='<p>状態: '+m['status']+'</p><table><tr><th>方式</th>'+''.join('<th>'+s+'</th>' for s in TITLES)+'</tr>'
    def fmt(v):return '—' if v is None else f'{v:.5f}'
    for mode in MODES:
        html+='<tr><td>'+mode+'</td>'+''.join('<td>'+fmt(result['aggregates'][mode][f]['mean'])+'</td>' for f in FIELDS)+'</tr>'
    html+='</table><h2>全試行</h2><table><tr><th>試行</th>'+''.join('<th>'+s+'</th>' for s in TITLES)+'</tr>'
    for key,t in result['trials'].items():html+='<tr><td>'+key+'</td>'+''.join('<td>'+fmt(t['metrics'][f])+'</td>' for f in FIELDS)+'</tr>'
    html+='</table>'
    with PdfPages(out/'comparison.pdf') as pdf:
        fig,axs=plt.subplots(2,3,figsize=(16,9))
        for ax,field,title in zip(axs.ravel(),FIELDS,TITLES):
            for j,mode in enumerate(MODES):
                vs=[v for v in result['aggregates'][mode][field]['values'] if v is not None]
                if vs:ax.scatter(j+np.linspace(-.08,.08,len(vs)),vs);ax.plot([j-.2,j+.2],[np.mean(vs)]*2)
            ax.set_xticks([0,1],['LiDAR-conditioned','Visual-only']);ax.set_title(title);ax.grid(alpha=.3)
            if field=='coverage':ax.set_ylim(0,1.05)
        axs[-1,-1].axis('off');fig.tight_layout();pdf.savefig(fig);fig.savefig(out/'metrics.png',dpi=130);plt.close(fig)
        fig,axs=plt.subplots(3,2,figsize=(15,10),squeeze=False)
        for i,seed in enumerate(m['design']['seeds']):
            for mode in MODES:
                key=f'{mode}_seed{seed}'
                if key not in result['trials']:continue
                f=r/key/'campaign'/(key+'_active');enc=load(f/'encoders.csv');t0=result['trials'][key]['raw']['tracking_start_s'];mask=(enc['t']>=t0)&(enc['t']<=t0+182)
                for j,angle in enumerate(['yaw','pitch']):axs[i,j].plot(enc['t'][mask]-t0,np.degrees(enc[angle][mask]),label=mode)
            for j,angle in enumerate(['yaw','pitch']):axs[i,j].set(title=f'Seed {seed}: {angle}',xlabel='Tracking time [s]',ylabel='Angle [deg]');axs[i,j].grid(alpha=.3)
            if axs[i,0].lines:axs[i,0].legend()
        fig.tight_layout();pdf.savefig(fig);fig.savefig(out/'angles.png',dpi=130);plt.close(fig)
    html+='<img src="metrics.png"><h2>実際の偏向</h2><p>pitch正は下、負は上。</p><img src="angles.png"><h2>取得監査</h2>'
    for key,t in result['trials'].items():html+='<p>'+key+': '+str(t['acceptance'])+'</p>'
    (out/'index.html').write_text(html)
    with (out/'velocity_samples.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=['trial','method','section','elapsed_s','axial_velocity_error_mps','error_xyz_mps']);w.writeheader();w.writerows(rows)
    print(out/'index.html')

if __name__=='__main__':main()
