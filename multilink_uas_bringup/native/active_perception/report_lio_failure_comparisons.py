#!/usr/bin/env python3
"""Publish all prespecified routes, including failures and small new LIO errors."""
import argparse
import html
import json
import os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

LABELS=['run1_prefix','run1_middle','run1_tail_a1','run1_tail_a2_prefix',
        'run1_tail_b1','run1_tail_b2','run2a_prefix','run2a_end','run2b']
METHODS=['pure_lio','fusion_fixed','fusion_active']
TITLES={'pure_lio':'MIMOSA only','fusion_fixed':'MIMOSA + ROVIO, fixed','fusion_active':'MIMOSA + ROVIO, gimbal'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--partial',action='store_true')
    args=parser.parse_args();root=args.root.resolve()
    manifest=json.loads((root/'campaign.json').read_text())
    records={};rows=[];figures=[];pdf=None
    if not args.partial:pdf=PdfPages(root/'all_runs.pdf')
    try:
        for label in LABELS:
            entry=manifest['runs'].get(label,{})
            status=entry.get('status','pending')
            record=dict(status=status,legacy_selection_rmse_m=manifest['selection'][label]['legacy_lio_rmse_m'])
            path=Path(entry['path']) if 'path' in entry else None
            if status in ['complete','reused','invalid'] and path and (path/'comparison.json').exists():
                data=json.loads((path/'comparison.json').read_text());metrics=data['metrics']
                if data['route']!=label:raise RuntimeError('Mislabeled comparison: '+label)
                record.update(valid=data['valid'],metrics={key:metrics[key] for key in METHODS},
                    current_lio_large_error=metrics['pure_lio']['position_rmse_m']>manifest['threshold_m'],
                    fixed_position_ratio=metrics['fusion_fixed']['position_rmse_m']/metrics['pure_lio']['position_rmse_m'],
                    active_position_ratio=metrics['fusion_active']['position_rmse_m']/metrics['pure_lio']['position_rmse_m'],
                    gimbal_fixed_ratio=metrics['fusion_active']['position_rmse_m']/metrics['fusion_fixed']['position_rmse_m'],
                    report=os.path.relpath(path/'index.html',root),source_comparison=str(path/'comparison.json'))
                def value(key,metric):return f'{metrics[key][metric]:.3f}'
                rows.append('<tr><td><a href="'+html.escape(record['report'])+'">'+label+'</a></td><td>'+str(data['valid'])+'</td>'+''.join('<td>'+value(k,'position_rmse_m')+'</td>' for k in METHODS)+''.join('<td>'+value(k,'rotation_rmse_deg')+'</td>' for k in METHODS)+'</tr>')
                figures.append('<h2>'+label+'</h2><img src="'+html.escape(os.path.relpath(path/'comparison.png',root))+'">')
                if pdf is not None:
                    fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
                    for key,color in zip(METHODS,['#333333','#2474b5','#df6619']):
                        e=np.genfromtxt(path/(key+'_errors.csv'),delimiter=',',names=True)
                        for n,ax in enumerate([*axs[:,0],*axs[:,1]]):
                            field=['x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg'][n]
                            ax.plot(e['route_t'],e[field],color=color,lw=.9,label=TITLES[key]);ax.grid(alpha=.3)
                            ax.set_ylabel(['x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'][n])
                    axs[0,0].legend();axs[-1,0].set_xlabel('Trajectory time [s]');axs[-1,1].set_xlabel('Trajectory time [s]')
                    fig.suptitle(label+' / same physical flight / no scale fitting');fig.tight_layout();pdf.savefig(fig);plt.close(fig)
            else:
                record.update(error=entry.get('error'))
                rows.append('<tr><td>'+label+'</td><td colspan="7">'+html.escape(status+(': '+entry['error'] if 'error' in entry else ''))+'</td></tr>')
            records[label]=record
    finally:
        if pdf is not None:pdf.close()
    complete=all(v.get('valid',False) for v in records.values())
    summary=dict(complete=complete,threshold_m=manifest['threshold_m'],
        selection='Legacy DRAGON LIO position RMSE > 1 m, fixed before new testing. All nine selected routes are retained regardless of new outcomes.',
        comparison='Frozen UniPilot v57, each route one shared physical flight, identical LiDAR/IMU, all ROVIO factors disabled for pure LIO, DRPM OFF. Prefix reuses the preceding comparison. No parameter tuning in this campaign.',
        seed=manifest['seed'],runs=records)
    (root/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
    page='''<!doctype html><meta charset="utf-8"><title>MIMOSA高誤差runの三構成比較</title>
<style>body{font:16px sans-serif;line-height:1.7;max-width:1250px;margin:30px auto;padding:0 20px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccc;padding:6px;text-align:right}td:first-child{text-align:left}img{width:100%}.note{background:#fff4d4;padding:12px}</style>
<h1>MIMOSA単体・固定カメラ融合・偏向カメラ融合の比較</h1>
<p>RMSEは区間全体の誤差の大きさを表す指標です（誤差の二乗平均の平方根）。小さいほど高精度です。位置は3軸を合わせた距離、回転は真値と推定姿勢の間の回転角で評価しています。</p>
<p>過去のDRAGON全14runで、MIMOSA単体の位置RMSEが1mを超えた9runを事前に選定しました。残る5runは0.12m以下、選定9runは2.73m以上です。旧値は対象選定だけに使い、表は現在のUniPilot構成による比較です。</p>
<p>各runの3構成は同じ1回の飛行・LiDAR・IMUを共有。単体ではROVIOの速度・傾き拘束を初期化から両方外しています。採用v57の推定器・融合重み・ジンバル設定は固定、DRPM OFF。追従開始で位置・姿勢を一度だけ合わせ、尺度補正なし。各run1試行で、prefixは直前の検証を再利用しました。</p>
<p class="note">過去の機体構成と現在の構成は異なります。現在のMIMOSA単体で誤差が小さくなったrunも、事前に選んだ対象として表に残しています。結果を見て対象を除外していません。</p>'''
    page+='<p>全9runの精度比較検査合格：'+str(complete)+'</p>'
    if not args.partial:page+='<p><a href="all_runs.pdf">全runの六軸PDF</a> / <a href="comparison.json">数値・対象選定</a></p>'
    page+='<table><tr><th rowspan="2">run</th><th rowspan="2">検査</th><th colspan="3">位置RMSE [m]</th><th colspan="3">回転RMSE [deg]</th></tr><tr>'+''.join('<th>'+s+'</th>' for s in ['単体','固定融合','偏向融合']*2)+'</tr>'+''.join(rows)+'</table>'+''.join(figures)
    (root/'index.html').write_text(page)
    print(json.dumps(dict(complete=complete,available=sum('metrics' in v for v in records.values()),runs={k:v['status'] for k,v in records.items()})))


if __name__=='__main__':main()
