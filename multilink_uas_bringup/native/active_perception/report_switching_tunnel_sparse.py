#!/usr/bin/env python3
"""Account explicitly for unavailable VIO factors; never interpolate across them.

The original complete-factor evaluator cannot score a filter rejecting uncertain
VIO. Preserve its original metric where coverage is complete, report conditional
error and coverage otherwise, and add common-window pose displacement errors.
This changes evaluation only, not any estimator, control, or acquisition.
"""
import argparse,csv,gzip,hashlib,json,os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import load,poses,interp
from report_direction_repeats import ORDER,LABEL,COLORS
from report_fusion_sidecar import tum
from window_metrics import local_displacement_error

def rms(x):return float(np.sqrt(np.mean(np.square(x)))) if len(x) else None

def evaluate(root,key,bounds):
    f=root/key/'campaign'/(key+'_active');gt=poses(f/'truth.csv');rt=load(f/'route_tracking.csv');g=rt['route_t']>0;t0=float(rt['sim_t'][g][0]-rt['route_t'][g][0])
    paths={n:f/'views'/n for n in ORDER[:6]};paths.update(front=f/'fixed',active=f)
    result={};samples=[];sections={}
    for name,p in paths.items():
        v=load(p/'mimosa/velocity_factors.csv');ts=v['stamp_ns'];ep,q=interp(gt,ts);a,_=interp(gt,ts-1e7);b,_=interp(gt,ts+1e7)
        errors=q.apply(np.column_stack([v['v'+c] for c in 'xyz']))-(b-a)/.02;elapsed=ts*1e-9-t0
        vio=poses(p/'rovio.csv');fused=poses(p/'mimosa.csv');diag=load(p/'vio_diagnostic.csv');records=[];features=[];viod=[];fusedd=[];parts=[]
        for i,(lo,hi) in enumerate(bounds):
            mask=(elapsed>=lo-1e-9)&(elapsed<hi-1e-9);e=errors[mask];times=elapsed[mask];slots=np.floor(times-lo+1e-7).astype(int);covered=len(set(slots.tolist()))
            for t,error in zip(times,e):samples.append(dict(trial=key,method=name,section=i,elapsed_s=float(t),axial_velocity_error_mps=float(error[0]),error_xyz_mps=error.tolist()))
            local={n:[] for n in ['vio','fused']}
            for start in np.arange(lo,hi-10+.001,10):
                stamps=(t0+np.array([start,start+10]))*1e9;gp,gq=interp(gt,stamps)
                for kind,estimate in [('vio',vio),('fused',fused)]:
                    pp,qq=interp(estimate,stamps);delta=local_displacement_error(gp[0],gp[1],gq[0].as_matrix(),pp[0],pp[1],qq[0].as_matrix());local[kind].append(float(delta[0]))
            dm=(diag['stamp_ns']*1e-9>=t0+lo)&(diag['stamp_ns']*1e-9<t0+hi);features.extend(diag['features'][dm].tolist())
            part=dict(expected_slots=int(hi-lo),covered_slots=covered,accepted_samples=len(times),coverage=covered/(hi-lo),accepted_axial_velocity_rms_mps=rms(e[:,0]),complete_axial_velocity_rms_mps=rms(e[:,0]) if covered==int(hi-lo) else None,vio_local_10s_axial_rms_m=rms(local['vio']),fused_local_10s_axial_rms_m=rms(local['fused']),median_features=float(np.median(diag['features'][dm])))
            parts.append(part);records.extend(e[:,0].tolist());viod.extend(local['vio']);fusedd.extend(local['fused'])
        expected=sum(p['expected_slots'] for p in parts);covered=sum(p['covered_slots'] for p in parts)
        result[name]=dict(expected_slots=expected,covered_slots=covered,coverage=covered/expected,accepted_axial_velocity_rms_mps=rms(records),complete_axial_velocity_rms_mps=rms(records) if covered==expected else None,vio_local_10s_axial_rms_m=rms(viod),fused_local_10s_axial_rms_m=rms(fusedd),median_features=float(np.median(features)),sections=parts)
    geometry=np.array([json.loads(l) for l in gzip.open(root/key/'observers/common_lidar.jsonl.gz','rt')]);mask=(geometry[:,1]>=t0+bounds[0][0])&(geometry[:,1]<=t0+bounds[-1][-1]);e=np.linalg.eigvalsh(geometry[mask,18:54].reshape(-1,6,6)[:,3:,3:]);ratio=e[:,0]/np.maximum(e[:,-1],1e-15)
    array=json.loads((root/key/'array_comparison.json').read_text());comparison=json.loads((root/key/'comparison.json').read_text())
    return dict(methods=result,full_fused=array['metrics'],pure_lio=comparison['metrics']['pure_lio'],estimated_geometry=dict(median_min_max_ratio=float(np.median(ratio)),fraction_below_005=float(np.mean(ratio<.05))),tracking_start_s=t0),samples

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root;m=json.loads((r/'campaign.json').read_text());out=r/'report_sparse';out.mkdir(exist_ok=True);result=dict(status=m['status'],evaluation_note=__doc__,trials={},conditions={},source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());samples=[]
    for key in m['order']:
        if m['runs'].get(key,{}).get('status')!='complete':continue
        route=key.rsplit('_seed',1)[0];record,ss=evaluate(r,key,m['design']['segments_s'][route]);result['trials'][key]=record;samples.extend(ss)
    doc='''<!doctype html><meta charset="utf-8"><title>特徴偏在トンネル：固定と偏向</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1500px;margin:30px auto;padding:0 20px}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:7px}img{max-width:100%}</style><h1>特徴が偏在するトンネル：固定と偏向</h1><p>幅6m・高さ5mの一定アーチ断面。模様の多い面を32mごとに左壁→天井→右壁→床（alternating）、逆順（reversed）へ変更。全周に模様があるuniformを対照とし、各3飛行。形状・照明・センサ・飛行は共通の人工環境です。現実の環境での汎化検証とは区別します。</p><p>直前の偏向候補v10を無変更で使用。各固定方向は独立した単眼ROVIO＋MIMOSA。DRPM OFF、既存の推定器と融合重みを維持。真値は共通機体制御と事後評価に使用し、偏向と推定器には渡しません。模様配置の正解も偏向へ渡しません。130m、0.5m/s、中央40秒×4区間を評価します。</p><p>主指標はトンネル軸Xに沿う、実際に融合へ渡ったVIO速度の誤差です。固定の一部で拘束が欠落したため、欠落を補間・ゼロ埋めせず利用率と併記します。「利用時RMS」は利用可能な時間だけの値で、低利用率の方法を有利と判定する値ではありません。全方法共通の補助指標として、VIOと融合後の10秒相対移動誤差、全行程の融合位置誤差を計算します。速度と相対移動は別の指標です。</p><p>比較単位は独立した飛行です。4区間は同じ飛行内で相関があります。最良固定は結果を見た参考値で、途中で向きを選び直す固定ではありません。失敗も含めて全方向を表示します。</p><p><a href="comparison.pdf">比較PDF</a> / <a href="summary.json">全結果JSON</a> / <a href="velocity_samples.csv">利用された速度拘束</a></p>'''
    with PdfPages(out/'comparison.pdf') as pdf:
        for condition in m['entries']:
            keys=[k for k in result['trials'] if k.rsplit('_seed',1)[0]==condition]
            if not keys:continue
            methods={}
            for n in ORDER[:6]:
                values={field:[result['trials'][k]['methods'][n][field] for k in keys] for field in ['coverage','accepted_axial_velocity_rms_mps','complete_axial_velocity_rms_mps','vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m']}
                values['full_fused_position_rmse_m']=[result['trials'][k]['full_fused'][n]['mimosa']['position_rmse_m'] for k in keys]
                methods[n]={f:dict(values=v,mean=float(np.mean(v)) if all(x is not None for x in v) else None) for f,v in values.items()}
            result['conditions'][condition]=dict(trials=keys,methods=methods)
            doc+=f'<h2>{condition} — {len(keys)}飛行</h2><table><tr><th>方向</th><th>速度拘束の利用率</th><th>利用時の軸速度RMS [m/s]</th><th>VIOの10秒移動誤差 [m]</th><th>融合の10秒移動誤差 [m]</th><th>融合の全行程位置RMSE [m]</th></tr>'
            for n,vs in methods.items():
                doc+=f'<tr><td>{LABEL[n]}</td>'
                for field in ['coverage','accepted_axial_velocity_rms_mps','vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m']:
                    val=vs[field]['mean'];text='—' if val is None else f'{val*100:.1f}%' if field=='coverage' else f'{val:.4f}';doc+='<td>'+text+'</td>'
                doc+='</tr>'
            pure=[result['trials'][k]['pure_lio']['position_rmse_m'] for k in keys];doc+=f'</table><p>純LIOの全行程位置RMSE平均: {np.mean(pure):.3f}m。</p>'
            for field in ['vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m']:
                best=min(ORDER[:5],key=lambda n:methods[n][field]['mean']);a_vals=np.array(methods['active'][field]['values']);fixed=np.array([methods[n][field]['values'] for n in ORDER[:5]])
                comparison=dict(best_fixed_of_mean=best,active_wins_fixed_mean=int((a_vals<fixed.mean(axis=0)).sum()),active_wins_all_fixed=int((a_vals<fixed.min(axis=0)).sum()),active_to_best_mean_ratio=float(a_vals.mean()/fixed.mean(axis=1).min()))
                result['conditions'][condition][field+'_comparison']=comparison
            fig,axs=plt.subplots(2,2,figsize=(14,10))
            for ax,field,title in zip(axs.ravel(),['coverage','vio_local_10s_axial_rms_m','fused_local_10s_axial_rms_m','full_fused_position_rmse_m'],['VIO factor availability','VIO axial local displacement / 10 s [m]','Fused axial local displacement / 10 s [m]','Fused full-trajectory position RMSE [m]']):
                for j,n in enumerate(ORDER[:6]):
                    v=methods[n][field]['values'];ax.scatter(j+np.linspace(-.12,.12,len(v)),v,color=COLORS[j]);ax.plot([j-.2,j+.2],[np.mean(v)]*2,color=COLORS[j])
                ax.set_xticks(range(6),[LABEL[n] for n in ORDER[:6]],rotation=25);ax.set_title(title);ax.grid(alpha=.3)
                if field=='coverage':ax.set_ylim(0,1.05)
                else:ax.set_yscale('log')
            fig.suptitle(condition+' / n='+str(len(keys)));fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'{condition}.png',dpi=130);plt.close(fig);doc+=f'<img src="{condition}.png">'
            sheet=r.parent/'optical_check_v2'/f'{condition}_contact_sheet.jpg';doc+=f'<details><summary>Gazeboの静止画像と角検出数</summary><img src="{os.path.relpath(sheet,out)}"></details>'
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');(out/'index.html').write_text(doc)
    with (out/'velocity_samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=['trial','method','section','elapsed_s','axial_velocity_error_mps','error_xyz_mps']);writer.writeheader();writer.writerows(samples)
    print(out/'index.html')

if __name__=='__main__':main()
