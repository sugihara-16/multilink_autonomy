#!/usr/bin/env python3
"""Common weak-direction local displacement and actual visual velocity errors."""
import argparse,csv,gzip,json
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_routes import load,poses,interp
from report_fusion_sidecar import tum
from window_metrics import local_displacement_error,translation_axis,stats
ORDER=['yaw_m50','front','yaw_p50','pitch_p90','pitch_m90','active','pure_lio']
LABEL={'yaw_m50':'(-50,0)','front':'(0,0)','yaw_p50':'(+50,0)','pitch_p90':'(0,+90) down','pitch_m90':'(0,-90) up','active':'Gimbal','pure_lio':'LIO only'}
COLORS=['#8a4bb0','#2474b5','#3d954b','#b58a10','#c64d72','#111111','#888888']

def trial(root,key,segments):
 out=root/key;f=out/'campaign'/(key+'_active')
 a=json.loads((out/'array_comparison.json').read_text());assert a['valid']
 gt=poses(f/'truth.csv');rt=load(f/'route_tracking.csv');good=rt['route_t']>0
 t0=float(rt['sim_t'][good][0]-rt['route_t'][good][0]);tend=t0+max(end for start,end in segments)
 with gzip.open(out/'observers/common_lidar.jsonl.gz','rt') as stream:geo=np.array([json.loads(l) for l in stream])
 assert len(geo)>100 and np.all(np.diff(geo[:,1])>0)
 qconfig=yaml.safe_load((f/'mimosa.yaml').read_text());RBL=Rotation.from_quat(qconfig['lidar']['T_B_S'][3:]).as_matrix()
 paths={n:f/'views'/n for n in ORDER[:-1]};paths.update(front=f/'fixed',active=f)
 estimates={n:poses(p/'mimosa.csv') for n,p in paths.items()}
 pure=tum(out/'observers/pure_lio/mimosa/graph_manager_odometry.tum').copy();pure[:,0]*=1e9;estimates['pure_lio']=pure
 velocities={};track_features={}
 for n,p in paths.items():
  v=load(p/'mimosa/velocity_factors.csv');times=v['stamp_ns'];mask=(times>=t0*1e9)&(times<=tend*1e9);times=times[mask]
  body=np.column_stack([v['v'+c][mask] for c in 'xyz']);before,_=interp(gt,times-1e7);after,_=interp(gt,times+1e7);_,q=interp(gt,times)
  error=q.apply(body)-(after-before)/.02
  C=np.array([v[f'c{i}{j}'][mask] for i in range(3) for j in range(3)]).T.reshape(-1,3,3);R=q.as_matrix();C=R@C@R.transpose(0,2,1)
  velocities[n]=(times*1e-9,error,C)
  d=load(p/'vio_diagnostic.csv');track_features[n]=(d['stamp_ns']*1e-9,d['features'])
 enc=load(f/'encoders.csv');angles=np.column_stack([enc['yaw'],enc['pitch']])
 rows=[];samples=[];window_checks=[]
 for W in [5,10,20]:
  for seg,(lo,hi) in enumerate(segments):
   for start in np.arange(lo,hi-W+.001,W):
    t=np.array([t0+start,t0+start+W]);gp,gq=interp(gt,t*1e9)
    ix=np.searchsorted(geo[:,1],t[0]+1e-7,side='right')-1
    if ix<0 or not 0<=t[0]-geo[ix,1]<.25:raise ValueError('No timely common geometry '+key+' '+str(start))
    u,eig,ratio=translation_axis(geo[ix,18:54].reshape(6,6),RBL,gq[0].as_matrix(),gp[1]-gp[0])
    weak=bool(ratio<.05)
    ce=(enc['t']>=t[0])&(enc['t']<=t[1]);aa=angles[ce]
    travel=float(np.rad2deg(np.linalg.norm(np.diff(aa,axis=0),axis=1).sum()))
    window_checks.append(dict(window_s=W,segment=seg,start_s=float(start),geometry_age_s=float(t[0]-geo[ix,1]),eigenvalues=eig.tolist(),weak_ratio=float(ratio),weak=weak,direction_world=u.tolist(),gimbal_joint_travel_deg=travel))
    for n in ORDER:
     ep,eq=interp(estimates[n],t*1e9)
     e=local_displacement_error(gp[0],gp[1],gq[0].as_matrix(),ep[0],ep[1],eq[0].as_matrix())
     row=dict(trial=key,route=key.rsplit('_seed',1)[0],seed=int(key.rsplit('_seed',1)[1]),segment=seg,window_s=W,start_s=float(start),method=n,weak=weak,weak_ratio=float(ratio),error_m=float(u@e),all_direction_error_m=float(np.linalg.norm(e)),gimbal_travel_deg=travel)
     if n!='pure_lio':
      times,ve,C=velocities[n];m=(times>=t[0]-1e-9)&(times<t[1]-1e-9);proj=ve[m]@u;sigma=np.sqrt(np.einsum('i,nij,j->n',u,C[m],u))
      if len(proj)!=W:raise ValueError('Missing actual velocity factors '+key+' '+n+' '+str(start)+' '+str(len(proj)))
      row.update(velocity_bias_mps=float(proj.mean()),velocity_rmse_mps=float(np.sqrt(np.mean(proj**2))),velocity_sigma_mps=float(np.median(sigma)))
      ft,fc=track_features[n];fm=(ft>=t[0])&(ft<t[1]);row['median_feature_count']=float(np.median(fc[fm]))
      for stamp,error,sd in zip(times[m],proj,sigma):samples.append(dict(trial=key,route=row['route'],seed=row['seed'],segment=seg,window_s=W,method=n,weak=weak,stamp_s=float(stamp),error_mps=float(error),sigma_mps=float(sd)))
     rows.append(row)
 (out/'window_geometry.json').write_text(json.dumps(window_checks,indent=2)+'\n')
 return rows,samples

def summarize(rows,samples,manifest):
    fixed=ORDER[:5]; report={}
    for route,segments in manifest['design']['segments_s'].items():
        keys=[k for k in manifest['order'] if k.startswith(route+'_') and manifest['runs'].get(k,{}).get('status')=='complete']
        result={'trials':keys,'segments':{},'heldout':[]}
        for seg,bounds in enumerate(segments):
            result['segments'][str(seg)]={'bounds_s':bounds,'windows':{}}
            for W in [5,10,20]:
                methods={}
                for method in ORDER:
                    by_trial=[]
                    for key in keys:
                        rr=[r for r in rows if r['trial']==key and r['segment']==seg and r['window_s']==W and r['method']==method]
                        ss=[r for r in samples if r['trial']==key and r['segment']==seg and r['window_s']==W and r['method']==method]
                        if not rr:continue
                        e=np.array([r['error_m'] for r in rr]);v=np.array([r['error_mps'] for r in ss]);sd=np.array([r['sigma_mps'] for r in ss])
                        by_trial.append(dict(trial=key,directional=stats(e),all_direction_rms_m=float(np.sqrt(np.mean([r['all_direction_error_m']**2 for r in rr]))),visual=stats(v),sigma_median_mps=float(np.median(sd)) if len(sd) else None,normalized_visual_rms=float(np.sqrt(np.mean((v/sd)**2))) if len(sd) else None,weak_windows=sum(r['weak'] for r in rr),gimbal_travel_median_deg=float(np.median([r['gimbal_travel_deg'] for r in rr]))))
                    x=[t['directional']['rmse'] for t in by_trial]
                    methods[method]=dict(trials=by_trial,mean_rms_m=float(np.mean(x)) if x else None,sd_across_flights_m=float(np.std(x,ddof=1)) if len(x)>1 else None,min_rms_m=min(x) if x else None,max_rms_m=max(x) if x else None)
                result['segments'][str(seg)]['windows'][str(W)]=methods
            if keys:
                d=result['segments'][str(seg)]['windows']['10'];ranks=sorted(fixed,key=lambda n:d[n]['mean_rms_m'])
                result['segments'][str(seg)]['fixed_ranking']=ranks
                result['segments'][str(seg)]['fixed_mean_m']=float(np.mean([d[n]['mean_rms_m'] for n in fixed]))
                result['segments'][str(seg)]['active_vs_fixed_mean_per_trial']=[]
                for i,key in enumerate(keys):
                    vals={n:d[n]['trials'][i]['directional']['rmse'] for n in ORDER};avg=float(np.mean([vals[n] for n in fixed]))
                    result['segments'][str(seg)]['active_vs_fixed_mean_per_trial'].append(dict(trial=key,fixed_mean_m=avg,active_m=vals['active'],ratio=vals['active']/avg,fixed_ranking=sorted(fixed,key=vals.get)))
        if len(keys)>1:
            for hold in keys:
                training=[k for k in keys if k!=hold];train_scores={};per_segment={}
                for n in fixed:
                    train_scores[n]=np.mean([t['directional']['rmse'] for seg in result['segments'].values() for t in seg['windows']['10'][n]['trials'] if t['trial'] in training])
                shared=min(train_scores,key=train_scores.get)
                for seg,d in result['segments'].items():
                    per_segment[seg]=min(fixed,key=lambda n:np.mean([t['directional']['rmse'] for t in d['windows']['10'][n]['trials'] if t['trial'] in training]))
                    get=lambda n:next(t['directional']['rmse'] for t in d['windows']['10'][n]['trials'] if t['trial']==hold)
                    result['heldout'].append(dict(test_trial=hold,segment=int(seg),shared_fixed=shared,segment_fixed=per_segment[seg],shared_fixed_rms_m=get(shared),segment_fixed_rms_m=get(per_segment[seg]),active_rms_m=get('active')))
            global_score={n:np.mean([d['windows']['10'][n]['mean_rms_m'] for d in result['segments'].values()]) for n in fixed}
            result['best_single_fixed_retrospective']=min(global_score,key=global_score.get)
            result['best_single_fixed_scores_m']=global_score
        report[route]=result
    return report

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;m=json.loads((root/'campaign.json').read_text());rows=[];samples=[]
    for key in m['order']:
        if m['runs'].get(key,{}).get('status')!='complete':continue
        route=key.rsplit('_seed',1)[0];rr,ss=trial(root,key,m['design']['segments_s'][route]);rows+=rr;samples+=ss
    if not rows:raise ValueError('No completed flights')
    out=root/'report';out.mkdir(exist_ok=True)
    for name,records in [('windows',rows),('visual_velocity',samples)]:
        fields=list(dict.fromkeys(k for r in records for k in r))
        with (out/(name+'.csv')).open('w') as f:
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(records)
    result=dict(complete=m['status']=='complete',design=m['design'],routes=summarize(rows,samples,m))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    html='''<!doctype html><meta charset="utf-8"><title>固定方向の順位と偏向・反復比較</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1400px;margin:30px auto;padding:0 15px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}img{width:100%}</style><h1>固定方向の順位は区間によって変わるか</h1><p>ZaB往路・復路、二ツ小屋の2区間を新規各3試行。固定5方向は独立した単眼推定器。v57・DRPM OFFを変更せず。過去結果は区間選択にのみ使用し、今回の集計には混ぜていません。</p><p>区間は追従開始からの経過シミュレーション時刻で指定。区間前の飛行・地図・特徴追跡履歴を保持。主評価は10秒移動量の共通最弱方向への誤差RMS、補助5/20秒、全3方向誤差と実視覚速度の誤差・偏り・不確かさをCSV/JSONに保存。方向は純LIO診断の回転固定並進ブロックから得た評価用の共通基準で、真の退化方向を保証しません。真値補正は評価座標の始点整合だけで、推定器や制御規則へ返しません。</p><p>表は各飛行のRMSの平均±試行間標準偏差。固定平均は5方向等確率の場合の参考値で、普遍的な方向分布ではありません。窓数を独立試行数と扱いません。区間別最良固定は事後選択で、実際の旋回コストを含まない参考値です。</p><p><a href="comparison.pdf">PDF</a> / <a href="summary.json">全試行・方向順位・別試行への適用結果</a> / <a href="windows.csv">移動誤差</a> / <a href="visual_velocity.csv">視覚速度</a></p>'''
    with PdfPages(out/'comparison.pdf') as pdf:
        for route,r in result['routes'].items():
            if not r['trials']:continue
            html+='<h2>'+route+'</h2><table><tr><th>評価区間[s]</th>'+''.join('<th>'+LABEL[n]+'</th>' for n in ORDER)+'<th>固定平均</th></tr>'
            fig,axs=plt.subplots(2,2,figsize=(14,9))
            for seg,d in r['segments'].items():
                primary=d['windows']['10'];html+='<tr><td>'+str(d['bounds_s'])+'</td>'
                for n in ORDER:
                    q=primary[n];sd=q['sd_across_flights_m'];html+=f'<td>{q["mean_rms_m"]:.4f} ± {sd:.4f}</td>' if sd is not None else f'<td>{q["mean_rms_m"]:.4f}</td>'
                html+=f'<td>{d["fixed_mean_m"]:.4f}</td></tr>'
                ax=axs[int(seg),0]
                for i,(n,c) in enumerate(zip(ORDER[:-1],COLORS[:-1])):
                    vals=[t['directional']['rmse'] for t in primary[n]['trials']];ax.scatter(np.full(len(vals),i)+np.linspace(-.12,.12,len(vals)),vals,color=c);ax.plot([i-.2,i+.2],[np.mean(vals)]*2,color=c,lw=3)
                ax.set_xticks(range(6),[LABEL[n] for n in ORDER[:-1]],rotation=25,ha='right');ax.set_ylabel('10 s weak-direction RMS [m]');ax.set_title(f'{route} {d["bounds_s"]} s');ax.grid(alpha=.25)
                ax=axs[int(seg),1]
                for n,c in zip(ORDER[:-1],COLORS[:-1]):
                    ax.plot([5,10,20],[d['windows'][str(W)][n]['mean_rms_m'] for W in [5,10,20]],'-o',label=LABEL[n],color=c)
                ax.set_xticks([5,10,20]);ax.set_xlabel('Window duration [s]');ax.set_ylabel('Directional RMS [m]');ax.grid(alpha=.25)
            axs[0,1].legend(fontsize=8,ncol=2);fig.tight_layout();pdf.savefig(fig);fig.savefig(out/(route+'.png'),dpi=130);plt.close(fig)
            html+='</table><p>各試行の固定方向順位: '+str({seg:[t['fixed_ranking'] for t in d['active_vs_fixed_mean_per_trial']] for seg,d in r['segments'].items()})+'</p>'
            html+='<img src="'+route+'.png">'
            if r.get('heldout'):
                html+='<p>別試行で選んだ固定方向の評価：残る試行で方向を選び、除外しておいた1試行で採点。各区間専用の固定方向と、両区間共通の固定方向を別々に比較します。</p><table><tr><th>試行/区間</th><th>共通固定方向/誤差[m]</th><th>区間別固定方向/誤差[m]</th><th>偏向[m]</th></tr>'
                for t in r['heldout']:html+=f'<tr><td>{t["test_trial"]}/{t["segment"]}</td><td>{LABEL[t["shared_fixed"]]} / {t["shared_fixed_rms_m"]:.4f}</td><td>{LABEL[t["segment_fixed"]]} / {t["segment_fixed_rms_m"]:.4f}</td><td>{t["active_rms_m"]:.4f}</td></tr>'
                html+='</table>'
    (out/'index.html').write_text(html)
    print(json.dumps(dict(complete=result['complete'],flights=sum(len(r['trials']) for r in result['routes'].values()),report=str(out)),indent=2))

if __name__=='__main__':main()
