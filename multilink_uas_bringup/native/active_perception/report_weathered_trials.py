#!/usr/bin/env python3
"""Report all prespecified random tunnel layouts without filtering bad estimates."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_switching_tunnel_sparse import evaluate
from report_direction_repeats import ORDER, LABEL, COLORS
from report_routes import load

FIELDS=['coverage','complete_axial_velocity_rms_mps','accepted_axial_velocity_rms_mps',
        'vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m']


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root
    campaign=json.loads((r/'campaign.json').read_text());out=r/'report';out.mkdir(exist_ok=True)
    data=dict(status=campaign['status'],trials={},conditions={},source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    samples=[]
    for key in campaign['order']:
        if campaign['runs'].get(key,{}).get('status')!='complete':continue
        layout=key.rsplit('_seed',1)[0]
        record,ss=evaluate(r,key,campaign['design']['segments_s'][layout]);data['trials'][key]=record;samples.extend(ss)
    html='''<!doctype html><meta charset="utf-8"><title>人工風化トンネル：固定と偏向</title>
<style>body{font:16px sans-serif;line-height:1.7;max-width:1500px;margin:30px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:7px}img{max-width:100%}</style>
<h1>自然なまとまりを持つ人工偏在トンネル：固定5方向と偏向</h1>
<p>幅5m・高さ4.5m・長さ100mの一定アーチ断面。人工的なコンクリート、染み、ひび、剥離、析出跡を生成し、5つの不規則なまとまりへ偏在させた。配置seed24201は変更せず、同一環境で3回独立に90mを0.5m/sで飛行する。天井・左右壁の間で密度が滑らかに変化する。形状は全長一定、理想的な内壁の点対面拘束は軸方向に零。実センサによる観測・推定行列とは区別する。</p>
<p>各飛行で固定5単眼と偏向単眼を同時比較し、各推定器は自分のカメラだけを使う。固定は(yaw,pitch)=(-50,0),(0,0),(50,0),(0,90),(0,-90)度。偏向候補v10、ROVIO＋MIMOSA、DRPM OFF、共通準備・センサ・融合重みは前回から無変更。純LIOも同じLiDAR/IMU入力で記録。真値は機体制御と事後採点に限定し、推定器・視野制御へ入力しない。</p>
<p>主指標は巡航10〜170秒のトンネル軸X方向の実際のVIO速度拘束誤差と利用率。欠落は補間せず、利用時の値と全区間の値を分離。補助指標は同区間の重複しない10秒移動誤差と全90mの融合位置誤差。10秒移動は窓開始姿勢を剛体整合し、尺度は補正しない。全3試行・全方向を除外なく記載し、窓数を独立試行数と数えない。</p>
<p>固定平均は5方向を等確率で選んだ場合の試行誤差平均。最良固定は試験後に選んだ参考値。各方向の数値と個々の反復も併記する。画像の特徴数だけから位置推定性能を判断しない。</p>
<p><a href="comparison.pdf">比較PDF</a> / <a href="summary.json">全数値</a> / <a href="velocity_samples.csv">速度拘束</a> / <a href="final_audit.json">取得監査</a> / <a href="../../index.html">環境の画像・分布</a></p>'''
    with PdfPages(out/'comparison.pdf') as pdf:
        for layout,entry in campaign['entries'].items():
            keys=[k for k in data['trials'] if k.rsplit('_seed',1)[0]==layout]
            if not keys:continue
            methods={}
            for n in ORDER[:6]:
                values={f:[data['trials'][k]['methods'][n][f] for k in keys] for f in FIELDS[:-1]}
                values[FIELDS[-1]]=[data['trials'][k]['full_fused'][n]['mimosa']['position_rmse_m'] for k in keys]
                methods[n]={f:dict(values=v,mean=float(np.mean(v)) if all(x is not None for x in v) else None) for f,v in values.items()}
            condition=dict(trials=keys,methods=methods)
            for f in FIELDS[3:]:
                active=np.array(methods['active'][f]['values']);fixed=np.array([methods[n][f]['values'] for n in ORDER[:5]])
                best=min(ORDER[:5],key=lambda n:methods[n][f]['mean'])
                condition[f+'_comparison']=dict(best_fixed_of_mean=best,active_wins_all_fixed=int((active<fixed.min(0)).sum()),active_wins_fixed_mean=int((active<fixed.mean(0)).sum()),active_to_best_mean_ratio=float(active.mean()/fixed.mean(1).min()))
            condition['pure_lio_position_rmse_m']=[data['trials'][k]['pure_lio']['position_rmse_m'] for k in keys]
            data['conditions'][layout]=condition
            html+=f'<h2>{layout}：{len(keys)}飛行</h2><table><tr><th>方向</th><th>利用率</th><th>軸速度RMS：全区間 [m/s]</th><th>軸速度RMS：利用時のみ [m/s]</th><th>VIO 10秒移動 [m]</th><th>融合10秒移動 [m]</th><th>融合全位置 [m]</th></tr>'
            for n,values in methods.items():
                html+='<tr><td>'+LABEL[n]+'</td>'
                for f in FIELDS:
                    val=values[f]['mean'];html+='<td>'+('—' if val is None else f'{100*val:.1f}%' if f=='coverage' else f'{val:.4f}')+'</td>'
                html+='</tr>'
            html+=f"</table><p>純LIOの全位置RMSE平均：{np.mean(condition['pure_lio_position_rmse_m']):.3f}m。</p>"
            fig,axs=plt.subplots(2,3,figsize=(17,10))
            for ax,f in zip(axs.ravel(),FIELDS):
                for j,n in enumerate(ORDER[:6]):
                    vv=methods[n][f]['values'];valid=[v for v in vv if v is not None]
                    if valid:
                        ax.scatter(j+np.linspace(-.08,.08,len(valid)),valid,color=COLORS[j])
                        if len(valid)==len(vv):ax.plot([j-.2,j+.2],[np.mean(valid)]*2,color=COLORS[j])
                    if len(valid)<len(vv):ax.text(j,.03,'missing',ha='center',transform=ax.get_xaxis_transform(),fontsize=8)
                ax.set_xticks(range(6),[LABEL[n] for n in ORDER[:6]],rotation=30);ax.set_title(f.replace('_',' '));ax.grid(alpha=.3)
                if f=='coverage':ax.set_ylim(0,1.05)
                elif any(methods[n][f]['mean'] is not None for n in ORDER[:6]):ax.set_yscale('log')
            fig.suptitle(layout);fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(layout+'.png'),dpi=120);plt.close(fig)
            html+=f'<img src="{layout}.png">'
            preview=Path(entry['world']).parent/'texture_overview.jpg'
            shutil_target=out/(layout+'_texture.jpg');shutil_target.write_bytes(preview.read_bytes())
            html+=f'<details><summary>生成テクスチャ全長（横方向100m、縦方向は周方向）</summary><img src="{shutil_target.name}"></details>'
            html+=f'<details><summary>実Gazebo画像：5地点×固定5方向</summary><img src="../../optical_check/{layout}_contact_sheet.jpg"></details>'
            for key in keys:
                html+=f'<p>{key}: <a href="../{key}/six_axes.pdf">六軸誤差PDF</a></p>'
        fig,axs=plt.subplots(len(data['conditions']),2,figsize=(15,4*len(data['conditions'])),squeeze=False)
        for row,(layout,c) in enumerate(data['conditions'].items()):
            for key in c['trials']:
                f=r/key/'campaign'/(key+'_active');enc=load(f/'encoders.csv');route=load(f/'route_tracking.csv');t0=data['trials'][key]['tracking_start_s'];mask=(enc['t']>=t0)&(enc['t']<=t0+182)
                xx=np.interp(enc['t'][mask],route['sim_t'],route['ref_x'])
                for col,n in enumerate(['yaw','pitch']):axs[row,col].plot(xx,np.degrees(enc[n][mask]),label=key.rsplit('seed',1)[1])
            for col,n in enumerate(['yaw','pitch']):axs[row,col].set(title=layout+' '+n,xlabel='X [m]',ylabel='Angle [deg]');axs[row,col].grid(alpha=.3);axs[row,col].legend()
        fig.tight_layout();pdf.savefig(fig);fig.savefig(out/'angles.png',dpi=120);plt.close(fig)
    html+='<h2>実際の偏向角</h2><p>pitch +90は下、-90は上です。</p><img src="angles.png">'
    (out/'index.html').write_text(html);(out/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
    with (out/'velocity_samples.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['trial','method','section','elapsed_s','axial_velocity_error_mps','error_xyz_mps']);writer.writeheader();writer.writerows(samples)
    print(out/'index.html')


if __name__=='__main__':main()
