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

def trial(root,key):
 out=root/key;f=out/'campaign'/(key+'_active')
 a=json.loads((out/'array_comparison.json').read_text());assert a['valid']
 gt=poses(f/'truth.csv');rt=load(f/'route_tracking.csv');good=rt['route_t']>0
 t0=float(rt['sim_t'][good][0]-rt['route_t'][good][0]);tend=t0+120
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
  for seg in range(2):
   for start in np.arange(seg*60,(seg+1)*60-W+.001,W):
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

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;m=json.loads((root/'campaign.json').read_text())
 rows=[];samples=[]
 for key in m['order']:
  if m['runs'].get(key,{}).get('status')!='complete':continue
  r,s=trial(root,key);rows.extend(r);samples.extend(s)
 if not rows:raise ValueError('No completed trials')
 output=root/'window_report';output.mkdir(exist_ok=True)
 for name,records in [('windows',rows),('visual_velocity',samples)]:
  fields=list(dict.fromkeys(k for r in records for k in r))
  with (output/(name+'.csv')).open('w') as f:
   w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(records)
 summary={};per_trial={}
 for route in m['entries']:
  summary[route]={}
  for seg in range(2):
   summary[route][str(seg)]={}
   for W in [5,10,20]:
    modes={}
    for n in ORDER:
     rr=[r for r in rows if r['route']==route and r['segment']==seg and r['window_s']==W and r['method']==n]
     ss=[s for s in samples if s['route']==route and s['segment']==seg and s['window_s']==W and s['method']==n]
     modes[n]=dict(all=stats([r['error_m'] for r in rr]),weak=stats([r['error_m'] for r in rr if r['weak']]),visual_all=stats([s['error_mps'] for s in ss]),visual_weak=stats([s['error_mps'] for s in ss if s['weak']]))
    summary[route][str(seg)][str(W)]=modes
 for key in m['order']:
  per_trial[key]={}
  for seg in range(2):
   per_trial[key][str(seg)]={n:stats([r['error_m'] for r in rows if r['trial']==key and r['segment']==seg and r['window_s']==10 and r['method']==n]) for n in ORDER}
 result=dict(design=m['design'],complete=m['status']=='complete',pooled=summary,per_trial_primary=per_trial,
  window_time_basis='Elapsed simulation time from tracking onset; route execution reaches at least 120 route seconds, so first 120 elapsed seconds are available even with corner slowing.',interpretation='Positive signed bias follows the chosen common weak direction, oriented with true travel where possible. RMS retains both bias and scatter. Windows within one flight are not independent repeats. Diagnostic weak ratio is not a calibrated degeneracy probability.')
 motion={}
 for route in m['entries']:
  motion[route]={}
  for W in [5,10,20]:
   rr=[r for r in rows if r['route']==route and r['window_s']==W and r['method']=='active']
   if rr:motion[route][str(W)]=dict(joint_travel_deg_median=float(np.median([r['gimbal_travel_deg'] for r in rr])),joint_travel_deg_max=float(max(r['gimbal_travel_deg'] for r in rr)),features_median=float(np.median([r['median_feature_count'] for r in rr])))
 result['gimbal_motion']=motion
 result['window_length_caveat']='All three fixed window lengths retained. Joint travel and feature counts are reported; feature stabilization time is not directly identified from these logs.'
 (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
 figures=[]
 with PdfPages(output/'comparison.pdf') as pdf:
  for route in summary:
   if not summary[route]['0']['10']['active']['all']['n']:continue
   fig,axs=plt.subplots(2,2,figsize=(13,8))
   for seg in range(2):
    for n,color in zip(ORDER,COLORS):
     axs[seg,0].plot([5,10,20],[summary[route][str(seg)][str(W)][n]['all']['rmse'] for W in [5,10,20]],'-o',label=LABEL[n],color=color)
    ax=axs[seg,1];ks=ORDER[:-1];sv=[summary[route][str(seg)]['10'][n]['visual_all'] for n in ks];x=np.arange(len(ks))
    ax.bar(x,[s['rmse'] for s in sv],color=COLORS[:-1],alpha=.7,label='Velocity error RMS');ax.plot(x,[s['bias'] for s in sv],'kx',label='Signed bias')
    ax.set_xticks(x, [LABEL[n] for n in ks],rotation=25,ha='right');ax.set_ylabel('Actual visual velocity error [m/s]');ax.grid(alpha=.25);ax.legend(fontsize=8)
    axs[seg,0].set_title(f'{route}: {seg*60}–{(seg+1)*60} s');axs[seg,0].set_xlabel('Window duration [s]');axs[seg,0].set_ylabel('Weak-direction displacement RMS [m]');axs[seg,0].set_xticks([5,10,20]);axs[seg,0].grid(alpha=.25)
   axs[0,0].legend(fontsize=8,ncol=2);fig.tight_layout();pdf.savefig(fig);fig.savefig(output/(route+'.png'),dpi=130);plt.close(fig);figures.append(route)
 table=''
 for route in figures:
  for seg in range(2):
   d=summary[route][str(seg)]['10'];base=d['pure_lio']['all']['rmse'];fixedmean=np.mean([d[n]['all']['rmse'] for n in ORDER[:5]])
   rr=[r for r in rows if r['route']==route and r['segment']==seg and r['window_s']==10 and r['method']=='active'];weakcount=sum(r['weak'] for r in rr)
   table+=f'<tr><td>{route} {seg*60}–{(seg+1)*60}s</td><td>{weakcount}/{len(rr)}</td>'+''.join(f'<td>{d[n]["all"]["rmse"]:.4f}</td>' for n in ORDER)+f'<td>{fixedmean:.4f}</td></tr>'
 head=''.join('<th>'+LABEL[n]+'</th>' for n in ORDER)
 html='''<!doctype html><meta charset="utf-8"><title>ZaB / Futatsugoya 退化方向の局所移動誤差</title><style>body{font:16px sans-serif;line-height:1.7;max-width:1400px;margin:30px auto;padding:0 15px}td,th{border:1px solid #ccc;padding:7px}table{border-collapse:collapse}img{width:100%}</style><h1>共通の弱い方向をどれだけ拘束できたか</h1>
<p>ZaB a1/b1・二ツ小屋隧道の各120秒を2回。評価区間は追従開始からのシミュレーション時刻で前半/後半60秒、主評価10秒、補助5/20秒。全方向を同じ飛行で独立した単眼推定器として比較。DRPM OFF、採用v57不変。</p>
<p>共通の純LIOの診断用情報行列から、回転固定時に最も弱い並進方向を取得し、各窓の始点で固定。実際の融合後共分散や真の退化方向ではなく共通診断基準です。最小/最大固有値比0.05未満を「強い異方性」と分類。全窓の結果を主表に掲載し、その部分集合の集計もJSONに保持します。</p>
<p>各窓の開始位置・姿勢だけを合わせ、終点までの移動誤差を共通方向へ投影。尺度誤差は補正しません。VIOは融合に実際に入った速度を真値姿勢で回し、真値速度との差を同じ方向に投影。平均の偏り・ばらつき・RMSを別々に算出。真値は事後評価のみ。</p>
<p>窓は重複させず、偏向中も除外しません。2試行のみで統計的有意差は主張しません。融合改善だけでは誤差相殺を排除できないため、視覚速度の正確さを併記します。5/10/20秒の選択は精度を見る前に固定済み。</p>
<h2>10秒間の弱方向移動誤差RMS [m]（2回を集計）</h2><table><tr><th>区間</th><th>強い異方性の窓</th>'''+head+'<th>固定5方向平均</th></tr>'+table+'''</table><p>固定平均は各方向RMSの算術平均。1方向を等確率で選ぶ場合の値です。強い異方性の窓数は各10秒窓で数え、独立試行数ではありません。</p><p><a href="comparison.pdf">比較PDF</a> / <a href="summary.json">全窓長・偏り・ばらつき・弱方向部分集合</a> / <a href="windows.csv">全窓CSV</a> / <a href="visual_velocity.csv">視覚速度誤差CSV</a></p>'''
 for route in figures:html+=f'<h2>{route}</h2><img src="{route}.png">'
 (output/'index.html').write_text(html)
 print(json.dumps(dict(complete=result['complete'],trials=len(set(r['trial'] for r in rows)),output=str(output)),indent=2))
if __name__=='__main__':main()
