#!/usr/bin/env python3
"""Paired live trajectory evaluation, first-pose alignment only, no scale fitting."""
import argparse,json,csv,html,hashlib
import yaml
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def csv_path(path):
    path=Path(path)
    return path if path.exists() else path.with_suffix(path.suffix+'.gz')

def load(path):
    a=np.genfromtxt(csv_path(path),delimiter=',',names=True)
    return {k:np.atleast_1d(a[k]) for k in a.dtype.names}

def poses(path):
    d=load(path);a=np.column_stack([d[k] for k in ['stamp_ns','x','y','z','qx','qy','qz','qw']])
    good=np.isfinite(a).all(axis=1)&(np.linalg.norm(a[:,4:8],axis=1)>.5);a=a[good]
    return a[np.r_[True,np.diff(a[:,0])>0]]

def interp(a,t):
    if min(t)<a[0,0] or max(t)>a[-1,0]:raise ValueError('No extrapolation permitted')
    p=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in range(1,4)])
    q=Slerp((a[:,0]-a[0,0])*1e-9,Rotation.from_quat(a[:,4:8]))((t-a[0,0])*1e-9)
    return p,q

def evaluate(out):
    protocol=json.loads((out/'protocol.json').read_text());runtime=json.loads((out/'runtime.json').read_text())
    if runtime['status']!='route_complete' and not (out/'failure_classification.json').exists():raise ValueError('Unclassified incomplete flight '+str(out))
    d=load(out/'route_tracking.csv');mask=d['route_t']>0
    t=d['sim_t'][mask];r=d['route_t'][mask]
    uniq=np.r_[True,np.diff(r)>1e-8];t=t[uniq];r=r[uniq]
    end=float(protocol['route_end']);requested=np.arange(.1,end-.001,.1);reference=requested[requested<=r[-1]]
    stamps=np.rint(np.interp(reference,r,t)*1e9)
    truth=poses(out/'truth.csv');arrays={k:poses(out/(k+'.csv')) for k in ('rovio','mimosa')}
    initial=arrays['rovio'][arrays['rovio'][:,0]<=5e9]
    preflight=float(np.linalg.norm(initial[:,1:4]-initial[0,1:4],axis=1).max())
    assert preflight<1.,('Invalid startup',str(out),preflight)
    result={'preflight_max_displacement_m':preflight,'status':runtime['status'],'route_end_s':end,'route_observed_end_s':float(r[-1]),'samples_requested':len(requested),'fusion_drpm_enabled':protocol['drpm_fusion'],
            'tracking_max_m':float(d['tracking_error_m'][mask].max()),'seed':protocol['seed'],'estimators':{}}
    assert protocol['drpm_fusion'] is False
    for name,a in arrays.items():
        valid=(stamps>=max(truth[0,0],a[0,0]))&(stamps<=min(truth[-1,0],a[-1,0]))
        ts=stamps[valid];rt=reference[valid]
        gt,gq=interp(truth,ts);est,eq=interp(a,ts);align=gq[0]*eq[0].inv()
        pe=align.apply(est-est[0])+gt[0]-gt;qe=align*eq
        euler=(qe.as_euler('xyz')-gq.as_euler('xyz')+np.pi)%(2*np.pi)-np.pi
        errors=np.column_stack([pe,np.rad2deg(euler)])
        norm=np.linalg.norm(pe,axis=1);angle=np.rad2deg((gq.inv()*qe).magnitude())
        subset=a[(a[:,0]>=ts[0])&(a[:,0]<=ts[-1])]
        m=dict(position_rmse_m=float(np.sqrt(np.mean(norm**2))),position_max_m=float(norm.max()),
            rotation_rmse_deg=float(np.sqrt(np.mean(angle**2))),rotation_max_deg=float(angle.max()),position_end_m=float(norm[-1]),
            first_100m_route_s=float(rt[np.flatnonzero(norm>100)[0]]) if np.any(norm>100) else None,
            count=len(ts),coverage=float(len(ts)/len(requested)),max_output_gap_s=float(np.diff(subset[:,0]).max()*1e-9),
            axis_rmse=np.sqrt(np.mean(errors**2,axis=0)).tolist())
        result['estimators'][name]=m
        np.savetxt(out/(name+'_tracking_errors.csv'),np.column_stack([rt,ts,errors]),delimiter=',',
                   header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',comments='',fmt=['%.9f','%.0f']+['%.12g']*6)
    lidar=np.loadtxt(out/'mimosa/lidar_manager_odometry.log',ndmin=2)[:,0]
    lidar=lidar[(lidar>=stamps[0]*1e-9)&(lidar<=stamps[-1]*1e-9)]
    result['lidar_updates']=len(lidar)
    result['max_lidar_update_gap_s']=float(np.diff(lidar).max()) if len(lidar)>1 else None
    enc=load(out/'encoders.csv');v=np.column_stack([enc['yaw'],enc['pitch']]);dt=np.diff(enc['t']);ok=dt>0
    result['encoder_span_deg']=np.rad2deg(np.ptp(v,axis=0)).tolist()
    result['max_angular_speed_deg_s']=float(np.rad2deg((np.linalg.norm(np.diff(v,axis=0),axis=1)[ok]/dt[ok]).max()))
    lines=[json.loads(l) for l in (out/'decisions.jsonl').read_text().splitlines()]
    result['decisions']=len(lines);result['evaluated']=sum(x['reason']=='evaluated' for x in lines)
    result['planning_p95_ms']=float(np.percentile([x['compute_ms'] for x in lines],95)) if lines else None
    assert result['max_angular_speed_deg_s']<20.01
    (out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();root=a.output
    manifest=json.loads((root/'campaign.json').read_text());plots=root/'plots';plots.mkdir(exist_ok=True)
    results={};labels=[]
    for key,v in manifest['runs'].items():
        out=Path(v['path'])
        if not (out/'runtime.json').exists():continue
        if json.loads((out/'runtime.json').read_text())['status']!='route_complete' and not (out/'failure_classification.json').exists():continue
        results[key]=evaluate(out);label=key.rsplit('_',1)[0]
        if label not in labels:labels.append(label)
    validation={}
    for label in labels:
        if not all(label+'_'+m in results for m in ('fixed','active')):continue
        paths=[root/(label+'_'+m) for m in ('fixed','active')]
        prot=[json.loads((p/'protocol.json').read_text()) for p in paths]
        assert prot[0]['binary_sha256']==prot[1]['binary_sha256']
        assert prot[0]['seed']==prot[1]['seed']
        configs=[yaml.safe_load((p/'mimosa.yaml').read_text()) for p in paths]
        for c in configs:c.pop('logs_directory')
        assert configs[0]==configs[1]
        assert not configs[0]['lidar']['geometric']['scan_to_map']['drpm_enabled']
        assert not configs[0]['lidar']['geometric']['scan_to_map']['project_on_degneneracy']
        for file in ('route/route.json','route/segments.json','cam0.yaml','rovio.info'):
            assert (paths[0]/file).read_bytes()==(paths[1]/file).read_bytes(),(label,file)
        assert all(not list((p/'mimosa').glob('*drpm_registration*')) for p in paths)
        a,b=[load(p/'route_tracking.csv') for p in paths]
        grid=np.arange(.1,min(results[label+'_fixed']['route_observed_end_s'],results[label+'_active']['route_observed_end_s'])-.001,.1)
        xyz=[]
        for d in (a,b):
            keep=np.r_[True,np.diff(d['route_t'])>1e-8]& (d['route_t']>0)
            xyz.append(np.column_stack([np.interp(grid,d['route_t'][keep],d[k][keep]) for k in ('gt_x','gt_y','gt_z')]))
        difference=np.linalg.norm(xyz[0]-xyz[1],axis=1)
        validation[label]={'same_binaries_settings_calibration_route_seed':True,
            'ordinary_mimosa_without_drpm_or_native_projection':True,
            'truth_trajectory_pair_max_difference_m':float(difference.max()),
            'truth_trajectory_pair_rmse_difference_m':float(np.sqrt(np.mean(difference**2))),
            'all_processes_exit_zero':all(all(c==0 for c in json.loads((p/'runtime.json').read_text())['processes'].values()) for p in paths)}
    for label,checks in validation.items():
        notes=[];estimator_exit=[]
        for mode in ('fixed','active'):
            run=root/(label+'_'+mode);runtime=json.loads((run/'runtime.json').read_text())
            estimator_exit.append(all(runtime['processes'][n]==0 for n in ('rovio','mimosa')))
            if runtime['processes'].get('route')==-2 and runtime['status']=='route_complete':
                notes.append(mode+': flight controller received SIGINT during cleanup after completing the route; full tracking data retained. Supervisor now waits for its natural exit.')
        checks['all_estimators_exit_zero']=all(estimator_exit)
        checks['shutdown_notes']=notes
        checks['full_tracking_coverage']=all(results[label+'_'+mode]['estimators'][name]['coverage']==1. for mode in ('fixed','active') for name in ('rovio','mimosa'))
    (root/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    figures=[]
    with PdfPages(plots/'comparison.pdf') as pdf:
        for label in labels:
            if not all(label+'_'+m in results for m in ('fixed','active')):continue
            for estimator in ('mimosa','rovio'):
                fig,axs=plt.subplots(3,2,figsize=(13,9),sharex=True)
                for mode,color in [('fixed','#2474b5'),('active','#df6619')]:
                    d=load(root/(label+'_'+mode)/(estimator+'_tracking_errors.csv'))
                    for ax,col in zip([*axs[:,0],*axs[:,1]],['x_m','y_m','z_m','roll_deg','pitch_deg','yaw_deg']):
                        ax.plot(d['route_t'],d[col],label=mode,color=color,lw=1);ax.set_ylabel(col);ax.grid(alpha=.3)
                for ax in axs[-1]:ax.set_xlabel('Trajectory time [s]')
                axs[0,0].legend();fig.suptitle(label+' / '+estimator+' / DRPM fusion OFF\nInitial pose alignment; full vertical axes; one new flight per condition')
                for ax in axs.flat:ax.set_xlim(0,results[label+'_fixed']['route_end_s'])
                fig.tight_layout();name=label+'_'+estimator+'.png';fig.savefig(plots/name,dpi=140);pdf.savefig(fig);plt.close(fig);figures.append(name)
    summary={'scope':'Single seeded trial per condition. New simulated flights; no proof of statistical superiority.',
             'alignment':'First tracking pose, no scale or subsequent fit','excluded_runs':json.loads((root/'excluded_runs.json').read_text()) if (root/'excluded_runs.json').exists() else [],'results':results}
    (root/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
    rows=[]
    for key,v in results.items():
        m=v['estimators']['mimosa'];rv=v['estimators']['rovio']
        rows.append(f'<tr><td>{key} — {v["status"]}; coverage {m["coverage"]:.1%}</td><td>{m["position_rmse_m"]:.4f}</td><td>{m["rotation_rmse_deg"]:.4f}</td><td>{rv["position_rmse_m"]:.4f}</td><td>{rv["rotation_rmse_deg"]:.4f}</td></tr>')
    text='''<!doctype html><meta charset="utf-8"><title>Fixed vs active single camera</title><style>body{font-family:sans-serif;max-width:1200px;margin:30px auto}img{width:100%}td,th{padding:8px;border:1px solid #ddd}table{border-collapse:collapse}</style><h1>Fixed vs active single camera — DRPM fusion OFF</h1><p>New online flights. Same route, speed, seed and estimator settings. Camera fixed until tracking begins. DRPM diagnostics only guide the camera; no DRPM pose correction. One trial per condition. No claim of statistical superiority.</p><p>RMSE: position [m], orientation [deg]. Tracking only, first-pose alignment, no scale fit.</p><table><tr><th>Flight</th><th>MIMOSA position (observed interval)</th><th>MIMOSA rotation</th><th>ROVIO position</th><th>ROVIO rotation</th></tr>'''+''.join(rows)+'</table><p><a href="comparison.pdf">PDF</a></p>'+''.join(f'<img src="{x}">' for x in figures)
    if summary['excluded_runs']:text+='<h2>Excluded initialization attempts</h2><p>run1_tail_b1 active attempt 1 diverged before takeoff and before any camera steering. No active view evaluations occurred. It is retained under excluded_startup and is not an active-versus-fixed result. All included trials must pass the same preflight displacement check (&lt;1 m by simulation time 5 s).</p>'
    (plots/'index.html').write_text(text)
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
