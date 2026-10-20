#!/usr/bin/env python3
"""Compare fixed single-camera experiments at original measurement times."""
import csv,json,warnings
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parent;BASE=ROOT/'logs/zab-offline/livo_comparison';OUT=BASE/'plots'
LABELS=['run2a_end','run1_tail_a2_prefix']
METHODS={'mimosa_off':('MIMOSA OFF','#0072B2'),'mimosa_on':('MIMOSA ON','#E69F00'),'fast_livo2':('FAST-LIVO2','#009E73'),'mimosa_drpm':('MIMOSA + DRPM','#CC79A7')}
def read(path):
    if path.suffix=='.tum':
        a=np.loadtxt(path);t=np.rint(a[:,0]*1e9).astype(np.int64);p=a[:,1:4];q=a[:,4:8]
    else:
        rows=list(csv.DictReader(path.open()));t=np.array([int(r['stamp_ns']) for r in rows],np.int64)
        p=np.array([[float(r[k]) for k in ('x','y','z')] for r in rows]);q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows])
    assert np.isfinite(p).all() and np.isfinite(q).all()
    # Graph logger can publish multiple optimizations at one state; retain the last.
    order=np.argsort(t,kind='stable');t,p,q=t[order],p[order],q[order];keep=np.r_[t[1:]!=t[:-1],True]
    return t[keep],p[keep],Rotation.from_quat(q[keep])
def interpolate(a,t):
    ts,p,q=a;origin=int(ts[0]);x=(ts-origin)*1e-9;y=(np.asarray(t)-origin)*1e-9
    assert min(t)>=ts[0] and max(t)<=ts[-1]
    return np.column_stack([np.interp(y,x,p[:,j]) for j in range(3)]),Slerp(x,q)(y)
def evaluate(label,methods=METHODS):
    window=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'];truth=read(BASE/'inputs'/label/'truth.csv')
    estimates={}
    for name in methods:
        path=BASE/name/label/'odometry.csv'
        estimates[name]=read(path)
    anchor=max(window['start_ns'],int(truth[0][0]),*[int(a[0][0]) for a in estimates.values()]);end=min(window['end_ns'],int(truth[0][-1]))
    pg,qg=interpolate(truth,[anchor]);data={};metrics={}
    for name,a in estimates.items():
        pa,qa=interpolate(a,[anchor]);R=qg[0]*qa[0].inv();T=pg[0]-R.apply(pa[0]);t,p,q=a
        keep=(t>=anchor)&(t<=end);t,p,q=t[keep],p[keep],q[keep];gp,gq=interpolate(truth,t)
        ep=R.apply(p)+T-gp;er=gq.inv()*(R*q)
        with warnings.catch_warnings(record=True) as warnings_seen:ea=er.as_euler('xyz',degrees=True)
        e=np.c_[ep,ea];data[name]=(t,e)
        # Shared 20 Hz grid avoids rewarding different output frequencies.
        grid=np.arange(int(t[0]),min(end,int(t[-1]))+1,50000000,dtype=np.int64)
        pe=np.column_stack([np.interp(grid,t,ep[:,j]) for j in range(3)]);angles=np.interp(grid,t,np.rad2deg(er.magnitude()))
        metrics[name]=dict(position_rmse_m=float(np.sqrt(np.mean(np.sum(pe**2,axis=1)))),position_max_m=float(np.linalg.norm(ep,axis=1).max()),position_end_m=float(np.linalg.norm(ep[-1])),rotation_rmse_deg=float(np.sqrt(np.mean(angles**2))),rotation_max_deg=float(np.rad2deg(er.magnitude()).max()),samples=len(t),first_ns=int(t[0]),last_ns=int(t[-1]),max_gap_s=float(np.diff(t).max()*1e-9),coverage=(int(t[-1])-int(t[0]))/(end-anchor),alignment_rotation_xyzw=R.as_quat().tolist(),alignment_translation_m=T.tolist(),euler_singularity_warnings=len(warnings_seen))
    return data,dict(window=window,anchor_ns=anchor,end_ns=end,metrics=metrics)
def main():
    OUT.mkdir(exist_ok=True);summary={}
    with PdfPages(OUT/'comparison.pdf') as pdf:
        for label in LABELS:
            data,meta=evaluate(label);summary[label]=meta
            fig,axes=plt.subplots(6,1,figsize=(14,16),sharex=True)
            for j,ax in enumerate(axes):
                for name,(t,e) in data.items():
                    y=e[:,j].copy()
                    if j>=3:y[np.r_[False,np.abs(np.diff(y))>180]]=np.nan
                    ax.plot((t-meta['anchor_ns'])*1e-9,y,label=METHODS[name][0],color=METHODS[name][1],lw=2.0 if name=='mimosa_off' else 1.1)
                ax.grid(alpha=.22);ax.axhline(0,color='gray',lw=.5);ax.set_ylabel(['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]'][j]);ax.tick_params(labelbottom=True)
            axes[0].legend(ncol=4);axes[-1].set_xlabel('Time since tracking start [s]');fig.suptitle(label+' | Single-camera LIVO comparison',fontsize=17)
            fig.text(.1,.015,'Same link3 image, LiDAR and IMU. Tracking only; one rigid alignment. OFF/ON overlap (no detections). Full axes.',fontsize=10)
            fig.tight_layout(rect=(0,.035,1,.97));fig.savefig(OUT/(label+'.png'),dpi=160);fig.savefig(OUT/(label+'.pdf'));pdf.savefig(fig);plt.close(fig)
            for name,(t,e) in data.items():
                with (OUT/(label+'_'+name+'_errors.csv')).open('w') as f:
                    w=csv.writer(f);w.writerow(['stamp_ns','ex_m','ey_m','ez_m','eroll_deg','epitch_deg','eyaw_deg']);w.writerows([[int(ts),*row] for ts,row in zip(t,e)])
    (OUT/'evaluation.json').write_text(json.dumps(summary,indent=2))
    rows=''.join('<tr><td>'+label+'</td><td>'+METHODS[k][0]+'</td><td>'+f"{v['position_rmse_m']:.3f}"+'</td><td>'+f"{v['position_max_m']:.3f}"+'</td><td>'+f"{v['rotation_rmse_deg']:.3f}"+'</td></tr>' for label,m in summary.items() for k,v in m['metrics'].items())
    (OUT/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>単眼LIVO 4条件比較</title><style>body{font:16px sans-serif;max-width:1400px;margin:30px auto}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style><h1>単眼LIVO：4条件・2飛行</h1><p>link3カメラ1台。MIMOSAの3条件には同じ単眼ROVIOを入力。軌道追従区間のみ評価。開始時に1回だけ位置・姿勢を整合し、スケール補正や途中の再整合は行っていません。縦軸の省略はありません。</p><p>ONは標準実装の代入漏れを修正。DRPMは論文の確率的な情報量の重み付けをMIMOSAのLiDAR因子へ導入。詳細は<a href="../README.md">比較条件と制約</a>。</p><p><a href="comparison.pdf">全図PDF</a>・<a href="evaluation.json">数値と時刻の詳細</a></p><table><tr><th>飛行</th><th>手法</th><th>位置RMSE [m]</th><th>最大位置誤差 [m]</th><th>姿勢RMSE [deg]</th></tr>'+rows+'</table>'+''.join('<h2>'+k+'</h2><img src="'+k+'.png">' for k in LABELS))
    print(json.dumps({k:v['metrics'] for k,v in summary.items()},indent=2))
if __name__=='__main__':main()
