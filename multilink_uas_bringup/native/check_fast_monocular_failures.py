#!/usr/bin/env python3
"""Preregistered monocular divergence check and common-window comparison."""
import csv,io,json,re,hashlib
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from plot_fast_camera_comparison import read,errors
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';OUT=LOG/'fast_monocular_failures'
LABELS=['run1_prefix','run1_middle','run1_tail_a1','run1_tail_b1','run2b']
def write(p,s):
 t=Path(str(p)+'.tmp');t.write_text(json.dumps(s,indent=2));t.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def live_read(path):
 text=path.read_text();text=text[:text.rfind('\n')+1];rows=list(csv.DictReader(io.StringIO(text)))
 if not rows:return None
 return (np.array([int(r['stamp_ns']) for r in rows],np.int64),np.array([[float(r[k]) for k in ['x','y','z']] for r in rows]),Rotation.from_quat([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rows]))
def detect_divergence(d,w):
 a=live_read(d/'odometry_full.csv')
 if a is None or a[0][-1]<w['start_ns']+6000000000:return None
 gt=read(LOG/'fast_multicamera_all'/d.name/'truth_full.csv');end=min(int(a[0][-1]),w['end_ns'],int(gt[0][-1]));end=end//100000000*100000000
 grid=np.arange(end-5000000000,end+1,100000000,dtype=np.int64)
 if grid[0]<w['start_ns']:return None
 e,_=errors(a,gt,w['start_ns'],grid);norm=np.linalg.norm(e[:,:3],axis=1)
 if np.all(norm>100):return dict(reason='position error continuously exceeds 100 m for 5 s',threshold_m=100,sustained_s=5,start_ns=int(grid[0]),end_ns=int(grid[-1]),min_error_m=float(norm.min()),max_error_m=float(norm.max()))
 return None

def finalize(d):
 s=json.loads((d/'summary.json').read_text());assert s['returncode']==0;w=s['window'];a=read(d/'odometry_full.csv');gt=read(d/'truth_full.csv')
 cfg=yaml.safe_load((d/'config.yaml').read_text());base=yaml.safe_load((LOG/'fast_multicamera_all'/d.name/'config.yaml').read_text());base['parameters'].pop('multicamera');assert cfg==base,'Only removal of multicamera configuration is permitted'
 assert not cfg['parameters']['vio']['exposure_estimate_en']
 native=a[0][(a[0]>=w['start_ns'])&(a[0]<=w['end_ns'])];assert len(native)>0
 expected=np.arange((w['start_ns']+49999999)//50000000*50000000,w['end_ns']+1,50000000,dtype=np.int64)
 assert np.array_equal(native,expected[:len(native)]),'Tracking timestamp gap'
 if not s['early_stop']:assert np.array_equal(native,expected) or (d.name=='run1_middle' and np.array_equal(native,expected[:-1]))
 else:assert detect_divergence(d,w),'Early-stop criterion no longer holds in final output'
 raw=np.genfromtxt(d/'odometry_full.csv',delimiter=',',names=True);q=np.stack([raw[k] for k in ['qx','qy','qz','qw']],axis=-1);assert np.all(np.diff(raw['stamp_ns'])==50000000),'Noncontinuous raw output';assert np.allclose(np.linalg.norm(q,axis=1),1,rtol=0,atol=1e-6)
 text=(d/'run.log').read_text();counts=list(map(int,re.search(r'INPUT_COUNTS ([0-9 ]+)',text)[1].split()));assert counts==[s['counts'][i] for i in [0,1,3]];assert 'CAMERA_COUNTS' not in text and 'CAMERA_UPDATE' not in text,'Multicamera branch unexpectedly active'
 match=re.search(r'UPDATE_COUNTS lio=(\d+) visual=(\d+) skipped_visual=(\d+)',text);assert match and int(match[2])>0 and int(match[3])==0
 for name in ['odometry','truth']:
  with (d/f'{name}_full.csv').open() as f,(d/f'{name}.csv').open('w') as out:
   rows=csv.DictReader(f);writer=csv.DictWriter(out,fieldnames=rows.fieldnames);writer.writeheader();writer.writerows(r for r in rows if w['start_ns']<=int(r['stamp_ns'])<=min(w['end_ns'],int(native[-1])))
 old=read(LOG/'fast_multicamera_all'/d.name/'odometry_full.csv');lio=read(LOG/'results'/d.name/'lio.csv');start=max(w['start_ns'],int(lio[0][0]),int(gt[0][0]));start=(start+99999999)//100000000*100000000;end=min(w['end_ns'],int(native[-1]),int(old[0][-1]),int(lio[0][-1]),int(gt[0][-1]));grid=np.arange(start,end+1,100000000,dtype=np.int64)
 metrics={}
 for key,est in [('monocular',a),('three_camera',old),('mimosa',lio)]:
  e,m=errors(est,gt,start,grid);norm=np.linalg.norm(e[:,:3],axis=1);over=norm>100;streak=np.convolve(over.astype(int),np.ones(51,int),'valid') if len(over)>=51 else []
  hit=np.flatnonzero(np.asarray(streak)==51);m['first_sustained_100m_s']=float((grid[hit[0]]-start)*1e-9) if len(hit) else None;metrics[key]=m
  with (d/f'{key}_errors.csv').open('w') as f:
   wr=csv.writer(f);wr.writerow(['evaluation_stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);wr.writerows(zip(grid,*e.T))
 validation=dict(condition='upstream monocular branch: multicamera absent',camera='link3',exposure_estimate=False,configuration_only_multicamera_removed=True,input_counts=counts,updates=dict(lio=int(match[1]),visual=int(match[2])),native_samples=len(native),evaluation_start_ns=start,evaluation_end_ns=int(grid[-1]),full_source_replayed=s['full_source_replayed'],early_stop=s['early_stop'],metrics=metrics,odometry_sha256=sha(d/'odometry.csv'))
 write(d/'metrics.json',validation)

def plot():
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.backends.backend_pdf import PdfPages
 out=OUT/'plots';out.mkdir(exist_ok=True);allmetrics={};html=[]
 with PdfPages(out/'comparison.pdf') as pdf:
  for label in LABELS:
   d=OUT/label
   if not (d/'metrics.json').exists():continue
   m=json.loads((d/'metrics.json').read_text());allmetrics[label]=m;fig,axes=plt.subplots(6,1,figsize=(14,15),sharex=True)
   for key,name,color in [('monocular','FAST-LIVO2 link3','#0072B2'),('three_camera','FAST-LIVO2 3 cameras','#D55E00'),('mimosa','MIMOSA LIO','#777777')]:
    a=np.genfromtxt(d/f'{key}_errors.csv',delimiter=',',names=True);t=(a['evaluation_stamp_ns']-m['evaluation_start_ns'])*1e-9
    for j,k in enumerate(a.dtype.names[1:]):axes[j].plot(t,a[k],label=name,color=color,lw=1)
   for ax,name in zip(axes,['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(name);ax.grid(alpha=.3)
   axes[0].legend();axes[-1].set_xlabel('Common tracking time [s]');fig.suptitle(label+(' | stopped after sustained >100 m error' if m['early_stop'] else ' | full tracking window'));fig.tight_layout();pdf.savefig(fig);fig.savefig(out/f'{label}.png',dpi=120);plt.close(fig)
   html.append(f'<h2>{label}</h2><p>{"単眼の発散を確認して早期終了" if m["early_stop"] else "全区間を確認"}。共通評価区間: {m["evaluation_start_ns"]/1e9:.2f}–{m["evaluation_end_ns"]/1e9:.2f}秒</p><img src="{label}.png">')
 write(OUT/'evaluation.json',allmetrics)
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>単眼の発散確認</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{width:100%}</style><h1>FAST-LIVO2 単眼・3カメラの比較</h1><p>単眼はlink3、露出推定OFF。3カメラ版と他条件は同一。明確な発散を位置誤差100m超が5秒連続と事前定義。到達時はその区間までで終了、未到達なら全区間を確認。途中終了の誤差値を全区間の性能とは比較しない。3方式とも同じ開始時刻で位置・姿勢を一度だけ整合し、共通10Hz時刻・同じ終了時刻で評価。</p><p><a href="comparison.pdf">全図PDF</a> / <a href="../evaluation.json">数値と検証</a></p>'+''.join(html))
 print('Plots:',len(allmetrics),flush=True)
if __name__=='__main__':plot()
