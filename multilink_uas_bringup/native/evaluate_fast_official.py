import json,csv,re
from pathlib import Path
import numpy as np
from plot_fast_camera_comparison import read,errors
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'logs/zab-offline';OUT=BASE/'fast_official_raw'
def finalize(d):
 s=json.loads((d/'summary.json').read_text());w=s['window'];a=read(d/'odometry_full.csv');gt=read(d/'truth_full.csv')
 start=(w['start_ns']+99999999)//100000000*100000000;end=min(w['end_ns'],int(a[0][-1]),int(gt[0][-1]));grid=np.arange(start,end+1,100000000,dtype=np.int64)
 assert len(grid)>10
 e,m=errors(a,gt,start,grid)
 with (d/'errors.csv').open('w') as f:
  wr=csv.writer(f);wr.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);wr.writerows(zip(grid,*e.T))
 stamps=a[0][(a[0]>=start)&(a[0]<=end)];m.update(native_samples=len(stamps),max_gap_s=float(np.diff(stamps).max()*1e-9),evaluation_start_ns=int(start),evaluation_end_ns=int(grid[-1]),early_stop=s['early_stop'])
 text=(d/'estimator.log').read_text();m['input_images_logged']=len(re.findall('Get image, its header time:',text));m['visual_updates']=len(re.findall(r'\[ VIO \] Raw feature num:',text));m['loopback_errors']=len(re.findall('loop back',text))
 assert m['visual_updates']>0 and m['loopback_errors']==0
 # Verify every image up through the final evaluated state reached the official callback.
 from decimal import Decimal
 received=np.array([int(Decimal(v)*10**9) for v in re.findall(r'Get image, its header time: ([0-9.]+)',text)],dtype=np.int64)
 expected=np.arange(s['first_stamp_ns'][3],int(grid[-1])+1,50000000,dtype=np.int64)
 assert np.array_equal(received[received<=grid[-1]],expected),'Image callback gap before evaluation end'
 m['image_callback_coverage_verified_to_ns']=int(grid[-1]);m['pending_images_after_evaluation']=s['counts'][3]-len(received)
 assert m['max_gap_s']<=.101,'Odometry discontinuity in evaluation window'
 m['node_exit']=json.loads((d/'publisher_audit.json').read_text())['node_returncode']
 norm=np.linalg.norm(e[:,:3],axis=1);hit=np.flatnonzero(np.convolve((norm>100).astype(int),np.ones(51,int),'valid')==51) if len(norm)>=51 else []
 m['first_sustained_100m_s']=float((grid[hit[0]]-start)*1e-9) if len(hit) else None
 old=BASE/'fast_monocular_failures'/d.name/'odometry_full.csv'
 if old.exists():
  previous=read(old);common=grid[grid<=previous[0][-1]];_,before=errors(previous,gt,start,common);_,after=errors(a,gt,start,common);m['same_interval_comparison']=dict(start_ns=int(common[0]),end_ns=int(common[-1]),previous=before,official=after)
 # The unchanged node also logs the state after the visual update.
 from scipy.spatial.transform import Rotation
 rows=np.loadtxt(d/'mat_out.txt');pairs=[]
 i=0
 while i+1<len(rows):
  if rows[i,0]==rows[i+1,0]:pairs.append((rows[i],rows[i+1]));i+=2
  else:i+=1
 pre=np.array([v[0] for v in pairs]);post=np.array([v[1] for v in pairs]);n=min(len(post),len(a[0]))
 assert n>10
 offset=a[0][0]-int(round(pre[0,0]*1e9));ts=np.rint(pre[:n,0]*1e9).astype(np.int64)+offset
 assert np.array_equal(ts,a[0][:n]);assert np.allclose(pre[:n,4:7],a[1][:n],rtol=1e-5,atol=2e-6)
 pq=Rotation.from_euler('xyz',post[:n,1:4]/57.3);pg=grid[grid<=ts[-1]]
 pe,pm=errors((ts,post[:n,4:7],pq),gt,start,pg);pn=np.linalg.norm(pe[:,:3],axis=1);hit=np.flatnonzero(np.convolve((pn>100).astype(int),np.ones(51,int),'valid')==51) if len(pn)>=51 else []
 pm['first_sustained_100m_s']=float((pg[hit[0]]-start)*1e-9) if len(hit) else None;m['post_visual']=pm;m['post_visual_timestamp_offset_ns']=int(offset);m['post_visual_stamps_matched_to_official_pose']=True
 with (d/'odometry_post_visual.csv').open('w') as f:
  wr=csv.writer(f);wr.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw']);wr.writerows(zip(ts,*post[:n,4:7].T,*pq.as_quat().T))
 with (d/'post_visual_errors.csv').open('w') as f:
  wr=csv.writer(f);wr.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);wr.writerows(zip(pg,*pe.T))
 (d/'metrics.json').write_text(json.dumps(m,indent=2));print('METRICS',d.name,m,flush=True)
def plot():
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.backends.backend_pdf import PdfPages
 p=OUT/'plots';p.mkdir(exist_ok=True);html=[];allm={}
 with PdfPages(p/'comparison.pdf') as pdf:
  for d in OUT.iterdir():
   if not (d/'metrics.json').exists():continue
   m=json.loads((d/'metrics.json').read_text());allm[d.name]=m;a=np.genfromtxt(d/'post_visual_errors.csv',names=True,delimiter=',');fig,axes=plt.subplots(6,1,figsize=(13,15),sharex=True)
   for j,ax in enumerate(axes):ax.plot((a['evaluation_stamp_ns']-m['evaluation_start_ns'])*1e-9,a[a.dtype.names[j+1]],label='Official FAST-LIVO2 raw link3');ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j]);ax.grid(alpha=.3)
   old=BASE/'fast_monocular_failures'/d.name/'odometry_full.csv'
   if old.exists():
    previous=read(old);gt=read(d/'truth_full.csv');grid=a['evaluation_stamp_ns'].astype(np.int64);common=grid[grid<=previous[0][-1]];e,_=errors(previous,gt,m['evaluation_start_ns'],common)
    for j,ax in enumerate(axes):ax.plot((common-m['evaluation_start_ns'])*1e-9,e[:,j],label='Previous adapted monocular link3',lw=1,alpha=.8)
   axes[0].legend();axes[-1].set_xlabel('Tracking time [s]');fig.suptitle(d.name);fig.tight_layout();fig.savefig(p/(d.name+'.png'),dpi=110);pdf.savefig(fig);plt.close(fig)
   html.append(f'<h2>{d.name}</h2><pre>{json.dumps(m,indent=2)}</pre><img src="{d.name}.png">')
 (p/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>img{max-width:100%}</style><h1>Official FAST-LIVO2, raw monocular images</h1><p>Unmodified upstream estimator. Original avia algorithm settings; hardware calibration, sensor format, topics, image time offset and pose logging adapted. Original image pixels. Displayed state after visual update from unchanged official diagnostic log, with every timestamp matched to the official pose file. Pose output before visual update is retained separately. Both give the same divergence verdict. Each curve covers the available common time interval; no extrapolation. Tracking-only evaluation, one initial rigid alignment.</p><a href="comparison.pdf">PDF</a>'+''.join(html))
 (OUT/'evaluation.json').write_text(json.dumps(allm,indent=2))
if __name__=='__main__':plot()
