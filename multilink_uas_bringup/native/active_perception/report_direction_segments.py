#!/usr/bin/env python3
"""Standard 60-second trajectory errors alongside local directional metrics."""
import argparse,json,re
from pathlib import Path
import numpy as np
from report_routes import load,poses,interp
from report_fusion_sidecar import tum
from report_direction_repeats import ORDER,LABEL

def main():
 p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root;m=json.loads((root/'campaign.json').read_text());records=[];coverage=[]
 for key in m['order']:
  if m['runs'].get(key,{}).get('status')!='complete':continue
  route=key.rsplit('_seed',1)[0];out=root/key;f=out/'campaign'/(key+'_active');gt=poses(f/'truth.csv');rt=load(f/'route_tracking.csv');good=rt['route_t']>0;t0=float(rt['sim_t'][good][0]-rt['route_t'][good][0])
  paths={n:f/'views'/n for n in ORDER[:-1]};paths.update(front=f/'fixed',active=f)
  estimates={n:poses(v/'mimosa.csv') for n,v in paths.items()};pure=tum(out/'observers/pure_lio/mimosa/graph_manager_odometry.tum').copy();pure[:,0]*=1e9;estimates['pure_lio']=pure
  for seg,(lo,hi) in enumerate(m['design']['segments_s'][route]):
   grid=np.arange(lo,hi+.001,.1);stamps=(t0+grid)*1e9;gp,gq=interp(gt,stamps)
   coverage.append(dict(trial=key,segment=seg,elapsed_bounds_s=[lo,hi],route_progress_bounds_s=np.interp(t0+np.array([lo,hi]),rt['sim_t'],rt['route_t']).tolist(),position_start=gp[0].tolist(),position_end=gp[-1].tolist(),yaw_start_end_deg=np.rad2deg(gq[[0,-1]].as_euler('xyz')[:,2]).tolist(),true_distance_m=float(np.linalg.norm(np.diff(gp,axis=0),axis=1).sum())))
   for n,e in estimates.items():
    ep,eq=interp(e,stamps);align=gq[0]*eq[0].inv();err=align.apply(ep-ep[0])+gp[0]-gp;angle=np.rad2deg((gq.inv()*align*eq).magnitude());norm=np.linalg.norm(err,axis=1)
    records.append(dict(trial=key,route=route,segment=seg,method=n,position_rmse_m=float(np.sqrt(np.mean(norm**2))),position_max_m=float(norm.max()),position_end_m=float(norm[-1]),rotation_rmse_deg=float(np.sqrt(np.mean(angle**2)))))
 result=dict(complete=m['status']=='complete',alignment='Segment-start rigid pose alignment, no scale fit, no estimator reset',records=records,coverage=coverage)
 (root/'report/segment_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
 html='<section id="segment-metrics"><h2>60秒区間の位置誤差</h2><p>区間始点の位置・姿勢だけを整合した位置RMSE[m]。各飛行の平均±試行間標準偏差。10秒ごとに始点を合わせる主指標とは異なり、区間内の蓄積を含みます。真値による尺度補正や推定器リセットはしていません。</p><table><tr><th>区間</th>'+''.join('<th>'+LABEL[n]+'</th>' for n in ORDER)+'</tr>'
 for route in m['entries']:
  for seg in range(2):
   html+='<tr><td>'+route+' / '+str(m['design']['segments_s'][route][seg])+'</td>'
   for n in ORDER:
    vals=[r['position_rmse_m'] for r in records if r['route']==route and r['segment']==seg and r['method']==n]
    if not vals:html+='<td>未取得</td>';continue
    html+=f'<td>{np.mean(vals):.4f} ± {np.std(vals,ddof=1):.4f}</td>' if len(vals)>1 else f'<td>{vals[0]:.4f}</td>'
   html+='</tr>'
 html+='</table><p><a href="segment_metrics.json">位置・回転・最大誤差、区間の実座標と実飛行距離</a></p></section>'
 path=root/'report/index.html';h=path.read_text();h=re.sub(r'<section id="segment-metrics">.*?</section>','',h,flags=re.S);path.write_text(h+html)
 print(json.dumps({'segment_records':len(records),'flights':len(set(r['trial'] for r in records))}))
if __name__=='__main__':main()
