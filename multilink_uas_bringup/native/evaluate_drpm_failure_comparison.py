#!/usr/bin/env python3
"""Full-flight DRPM evaluation and common-window official FAST comparison."""
import csv,json,re,subprocess
from collections import Counter
from pathlib import Path
import numpy as np
import yaml
from plot_fast_camera_comparison import read,errors
from run_drpm_failure_comparison import ROOT,BASE,OUT,WS,LABELS,save,sha

def onset(e,grid,anchor):
 n=np.linalg.norm(e[:,:3],axis=1);hit=np.flatnonzero(np.convolve((n>100).astype(int),np.ones(51,int),'valid')==51) if len(n)>=51 else []
 return float((grid[hit[0]]-anchor)*1e-9) if len(hit) else None

def export(p,grid,e):
 with p.open('w') as f:
  w=csv.writer(f);w.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);w.writerows(zip(grid,*e.T))

def evaluate(label):
 d=OUT/'fusion'/label;inp=OUT/'inputs'/label;s=json.loads((d/'summary.json').read_text());assert s['returncode']==0
 window=s['window'];start=(window['start_ns']+99999999)//100000000*100000000;end=window['end_ns'];grid=np.arange(start,end+1,100000000,dtype=np.int64)
 a=read(d/'odometry.csv');vio=read(OUT/'rovio'/label/'result_vio.csv');truth=read(inp/'truth.csv')
 expected=np.arange((window['start_ns']+49999999)//50000000*50000000,end+1,50000000,dtype=np.int64)
 cadence={}
 for name,v in [('drpm',a),('rovio',vio)]:
  native=v[0][(v[0]>=window['start_ns'])&(v[0]<=end)]
  assert np.isin(grid,native).all(),(name,'Incomplete 10 Hz evaluation coverage',len(native),len(grid))
  cadence[name]=dict(native_samples=len(native),native_max_gap_s=float(np.diff(native).max()*1e-9),missing_20hz_stamps=int(len(np.setdiff1d(expected,native))))
  assert v[0][0]<=start and v[0][-1]>=grid[-1]
  assert np.isfinite(v[1]).all()
 f=BASE/'fast_official_raw'/label;fast=read(f/'odometry_post_visual.csv');fm=json.loads((f/'metrics.json').read_text());assert start==fm['evaluation_start_ns']
 common=grid[grid<=fm['evaluation_end_ns']];full={};paired={}
 for name,v in [('drpm',a),('rovio',vio)]:
  e,m=errors(v,truth,start,grid);m.update(first_sustained_100m_s=onset(e,grid,start),native_max_gap_s=cadence[name]['native_max_gap_s'],common_grid_samples=len(grid),completed_tracking=True)
  export(d/f'{name}_full_errors.csv',grid,e);full[name]=m
 for name,v in [('drpm',a),('rovio',vio),('fast_official',fast)]:
  e,m=errors(v,truth,start,common);m.update(first_sustained_100m_s=onset(e,common,start),common_grid_samples=len(common));export(d/f'{name}_common_errors.csv',common,e);paired[name]=m
 assert abs(paired['fast_official']['position_rmse_m']-fm['post_visual']['position_rmse_m'])<1e-8
 log=(d/'run.log').read_text();assert 'Using calibrated full relative-pose covariance' in log
 vio_rejections=log.count('is higher than threshold')
 rows=list(csv.DictReader((d/'drpm_registration.csv').open()));rows=[r for r in rows if window['start_ns']<=int(r['stamp_ns'])<=end]
 assert all(int(r['update_mode'])==1 and int(r['noise_mode'])==2 for r in rows)
 ts=np.array([int(r['stamp_ns']) for r in rows]);assert np.isin(ts,grid).all()
 ok=[r for r in rows if r['status']=='ok'];probs=np.array([[float(r[f'p{k}']) for k in range(6)] for r in ok]);assert np.isfinite(probs).all() and (probs>=0).all() and (probs<=1).all()
 registration=dict(scans=len(rows),status_counts=dict(Counter(r['status'] for r in rows)),converged_scans=sum(int(r['converged']) for r in ok),scans_with_suppressed_direction=int(np.sum(np.min(probs,axis=1)<.99)) if len(ok) else 0,mean_iterations=float(np.mean([int(r['iterations']) for r in ok])) if ok else None,expected_scans=len(grid),scans_without_registration_record=len(grid)-len(rows),vio_uncertainty_rejections=vio_rejections)
 cfg=yaml.safe_load((d/'config.yaml').read_text());assert np.allclose(np.array(cfg['odometry']['manager']['relative_pose_covariance']).reshape(6,6),json.loads((OUT/'calibration.json').read_text())['covariance'],rtol=0,atol=0)
 prev=yaml.safe_load((BASE/'drpm_official/official_update/run2a_end/config.yaml').read_text());cfg['logs_directory']=prev['logs_directory'];cfg['odometry']['manager']['relative_pose_covariance']=prev['odometry']['manager']['relative_pose_covariance'];assert cfg==prev
 oldtruth=read(f/'truth_full.csv');overlap=np.intersect1d(oldtruth[0],truth[0]);i=np.searchsorted(oldtruth[0],overlap);j=np.searchsorted(truth[0],overlap);assert np.array_equal(oldtruth[1][i],truth[1][j]);assert np.allclose(oldtruth[2][i].as_quat(),truth[2][j].as_quat(),rtol=0,atol=1e-14)
 for src,dest in [(d/'odometry.csv',d/'odometry_tracking.csv'),(inp/'truth.csv',d/'truth_tracking.csv'),(OUT/'rovio'/label/'result_vio.csv',d/'rovio_tracking.csv')]:
  with src.open() as f1,dest.open('w') as f2:
   rr=csv.DictReader(f1);wr=csv.DictWriter(f2,fieldnames=rr.fieldnames);wr.writeheader()
   latest={int(row['stamp_ns']):row for row in rr if window['start_ns']<=int(row['stamp_ns'])<=end}
   wr.writerows(latest[t] for t in sorted(latest))
 metrics=dict(label=label,window=window,alignment_stamp_ns=start,full_grid_end_ns=int(grid[-1]),common_grid_end_ns=int(common[-1]),full=full,common=paired,registration=registration,checks=dict(returncode=0,all_10hz_evaluation_stamps_present=True,native_cadence=cadence,drpm_executed_every_lidar_stamp=bool(np.array_equal(ts,grid)),source_truth_matches_official_fast=True,only_covariance_and_log_path_changed=True,estimator_artifacts=s['artifacts'],odometry_sha256=sha(d/'odometry.csv'),truth_sha256=sha(inp/'truth.csv')))
 save(d/'metrics.json',metrics)
 print('RESULT',label,json.dumps(dict(full=full,common=paired,registration=registration)),flush=True)
 return metrics

def plot(labels=None, provisional=False):
 labels=LABELS if labels is None else labels
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.backends.backend_pdf import PdfPages
 p=OUT/('plots_partial' if provisional else 'plots');p.mkdir(exist_ok=True);html=[];allm={};table=[]
 with PdfPages(p/'comparison.pdf') as pdf:
  for label in labels:
   d=OUT/'fusion'/label;m=json.loads((d/'metrics.json').read_text());allm[label]=m
   for mode,names in [('common',['drpm','fast_official']),('full',['drpm'])]:
    fig,axes=plt.subplots(6,1,figsize=(13,15),sharex=True)
    for name in names:
     a=np.genfromtxt(d/f'{name}_{mode}_errors.csv',delimiter=',',names=True)
     for j,ax in enumerate(axes):ax.plot((a['evaluation_stamp_ns']-m['alignment_stamp_ns'])*1e-9,a[a.dtype.names[j+1]],label={'drpm':'MIMOSA + DRPM + ROVIO (link3)','fast_official':'Official FAST-LIVO2 (link3)','rovio':'ROVIO alone (link3)'}[name],lw=1)
    for j,ax in enumerate(axes):ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j]);ax.grid(alpha=.3)
    axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(label+' | '+('Common window: DRPM vs official FAST' if mode=='common' else 'Full tracking: MIMOSA + DRPM + ROVIO'))
    fig.tight_layout();fig.savefig(p/f'{label}_{mode}.png',dpi=115);pdf.savefig(fig);plt.close(fig)
    html.append(f'<h2>{label}: {mode}</h2><img src="{label}_{mode}.png">')
   for mode in ['common','full']:
    for name,k in m[mode].items():table.append(f'<tr><td>{label}</td><td>{mode}</td><td>{name}</td><td>{k["position_rmse_m"]:.4f}</td><td>{k["rotation_rmse_deg"]:.4f}</td></tr>')
 note=('処理中：ここには完了した2飛行のみ掲載。run1_prefix全区間は計算中です。 ' if provisional else '')+'同じlink3の1台。採用済みDRPM公式更新＋距離方向ノイズとROVIOを融合。重みは過去2飛行の相対誤差から固定し、今回の真値で調整していません。画像処理は各方式の設定を維持（ROVIOは既採用の白黒補正480×360、FASTは元RGBから公式処理）。離陸前から初期化し、追従区間のみ評価。共通10Hz、開始時に一度だけ位置姿勢整合。途中の再整合、外挿、軸省略なし。FASTは発散後に打ち切った既存結果なので比較は共通区間のみ。全区間図にはDRPM＋ROVIOの融合結果を表示。ROVIO単体の指標も表に掲載。位置RMSEは3次元、姿勢RMSEは相対回転角。図は6軸の誤差。'
 (p/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{max-width:100%}td,th{padding:8px;border:1px solid #bbb}table{border-collapse:collapse}</style><h1>DRPM＋vision：FAST-LIVO2が発散した3飛行</h1><p>'+note+'</p><a href="comparison.pdf">PDF</a><table><tr><th>飛行</th><th>区間</th><th>方式</th><th>位置RMSE [m]</th><th>姿勢RMSE [deg]</th></tr>'+''.join(table)+'</table>'+''.join(html))
 save(OUT/('evaluation_partial.json' if provisional else 'evaluation.json'),allm)
 ref=WS/'drpm';original=subprocess.check_output(['git','-C',str(ref),'show','b39a540c946f4059f5e3dfde625216c437eaf158:src/degeneracy.h']);assert original==(WS/'src/mimosa/include/mimosa/lidar/official_drpm/degeneracy.h').read_bytes()
 artifacts=[m['checks']['estimator_artifacts'] for m in allm.values()];assert all(a==artifacts[0] for a in artifacts)
 for path,h in artifacts[0].items():assert sha(path)==h
 save(OUT/('validation_partial.json' if provisional else 'validation.json'),dict(status='partial' if provisional else 'complete',runs=labels,official_drpm_header_identical=True,estimator_artifacts_identical=True,target_truth_used_for_tuning=False,cleanup_complete=False))
 print(json.dumps({k:{'full':v['full']['drpm'],'common':v['common']} for k,v in allm.items()},indent=2))
if __name__=='__main__':plot()
