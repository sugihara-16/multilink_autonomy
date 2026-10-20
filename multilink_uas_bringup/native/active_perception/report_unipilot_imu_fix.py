#!/usr/bin/env python3
"""Tracking-only, uniform route-time comparison of complete UniPilot trials."""
import argparse,json,html
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from report_routes import load,poses,interp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def assess(folder,end):
    route=load(folder/'route_tracking.csv');mask=route['route_t']>0
    t=route['sim_t'][mask];r=route['route_t'][mask]
    keep=np.r_[True,np.diff(r)>1e-8];t=t[keep];r=r[keep]
    ref=np.arange(.1,end-.001,.1)
    if r[-1]<ref[-1]:raise ValueError('Incomplete route')
    stamps=np.rint(np.interp(ref,r,t)*1e9)
    truth=poses(folder/'truth.csv');out={};series={}
    for name in ['rovio','mimosa']:
        a=poses(folder/(name+'.csv'))
        if stamps[0]<max(truth[0,0],a[0,0]) or stamps[-1]>min(truth[-1,0],a[-1,0]):
            raise ValueError('Tracking window not covered: '+str(folder))
        p,q=interp(a,stamps);gt,gq=interp(truth,stamps);align=gq[0]*q[0].inv()
        pe=align.apply(p-p[0])+gt[0]-gt;qe=align*q
        euler=(qe.as_euler('xyz')-gq.as_euler('xyz')+np.pi)%(2*np.pi)-np.pi
        err=np.c_[pe,np.rad2deg(euler)];rot=np.rad2deg((gq.inv()*qe).magnitude())
        pos=np.linalg.norm(pe,axis=1);raw=a[(a[:,0]>=stamps[0])&(a[:,0]<=stamps[-1])]
        out[name]=dict(position_rmse_m=float(np.sqrt(np.mean(pos**2))),rotation_rmse_deg=float(np.sqrt(np.mean(rot**2))),position_max_m=float(pos.max()),samples=len(ref),max_output_gap_s=float(np.diff(raw[:,0]).max()*1e-9))
        series[name]=(ref,stamps,err)
    return out,series

def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--corrected',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    report=dict(scope='Separate online flights, same route/seed/sensor settings; not identical sensor inputs. DRPM fusion OFF. Tracking only; first-pose rigid alignment; no scale fitting; uniform 0.1 s route-time grid.',runs={})
    ends=[json.loads((d/'protocol.json').read_text())['route_end'] for d in [a.baseline,a.corrected]]
    if ends[0]!=ends[1]:raise ValueError('Different route lengths')
    series={}
    for condition,base in [('before',a.baseline),('corrected',a.corrected)]:
        runtime=json.loads((base/'runtime.json').read_text())
        if runtime['status']!='route_complete' or not runtime['stopped']:raise ValueError('Flight not complete/stopped')
        report[condition+'_input_audit']=json.loads((base/'input_audit.json').read_text())
        for view,d in [('fixed',base/'fixed'),('active',base)]:
            key=condition+'_'+view;report['runs'][key],series[key]=assess(d,ends[0])
            for name,(t,stamps,err) in series[key].items():
                np.savetxt(a.output/(key+'_'+name+'.csv'),np.c_[t,stamps,err],delimiter=',',header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',comments='',fmt=['%.9f','%.0f']+['%.12g']*6)
    (a.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    images=[]
    with PdfPages(a.output/'comparison.pdf') as pdf:
        for name in ['rovio','mimosa']:
            for view in ['fixed','active']:
                fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
                for condition,color in [('before','#b36b20'),('corrected','#2469a5')]:
                    t,_,err=series[condition+'_'+view][name]
                    for i,ax in enumerate([*axs[:,0],*axs[:,1]]):ax.plot(t,err[:,i],label=condition,color=color,lw=.9)
                for ax,label in zip([*axs[:,0],*axs[:,1]],['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(label);ax.grid(alpha=.25)
                for ax in axs[-1]:ax.set_xlabel('Route time [s]')
                axs[0,0].legend();fig.suptitle(f'{a.baseline.name} / {name} / {view}\nBefore vs corrected; separate online flights, full axes')
                fig.tight_layout();image=name+'_'+view+'.png';fig.savefig(a.output/image,dpi=130);pdf.savefig(fig);plt.close(fig);images.append(image)
    rows=[]
    for key,v in report['runs'].items():
        for name,m in v.items():rows.append(f'<tr><td>{key}</td><td>{name}</td><td>{m["position_rmse_m"]:.3f}</td><td>{m["rotation_rmse_deg"]:.3f}</td></tr>')
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>UniPilot IMU model correction</title><style>body{font-family:sans-serif;max-width:1150px;margin:25px auto}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style><h1>UniPilot IMU model correction</h1><p>'+html.escape(report['scope'])+'</p><p>One trial per condition. Remaining drift is not resolved. Input gap counts are recorded in comparison.json.</p><table><tr><th>Condition</th><th>Estimator</th><th>Position RMSE [m]</th><th>Rotation RMSE [deg]</th></tr>'+''.join(rows)+'</table><p><a href="comparison.pdf">PDF</a> / <a href="comparison.json">Metrics and input audit</a></p>'+''.join(f'<img src="{image}">' for image in images))
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
