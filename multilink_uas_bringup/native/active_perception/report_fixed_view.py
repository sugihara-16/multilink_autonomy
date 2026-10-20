#!/usr/bin/env python3
"""Evaluate one constant gimbal view against an existing fixed-view trial."""
import argparse,json
from pathlib import Path
import numpy as np
import yaml
from report_unipilot_imu_fix import assess
from report_routes import load,poses
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def view_label(angles):
    def number(x):return format(float(x),'g').replace('-','minus').replace('.','p')
    return 'yaw_'+number(angles[0])+'_pitch_'+number(angles[1])

def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--extra-reference',type=Path,action='append',default=[]);a=p.parse_args()
    protocol=json.loads((a.flight/'protocol.json').read_text());runtime=json.loads((a.flight/'runtime.json').read_text())
    assert runtime['status']=='route_complete' and runtime['stopped']
    assert all(code==0 for code in runtime['processes'].values())
    assert protocol['mode']=='fixed' and not protocol['drpm_fusion']
    target=np.array(protocol['fixed_angles_deg']);end=protocol['route_end']
    reference=json.loads((a.reference/'protocol.json').read_text());assert reference['mode']=='fixed' and reference['route_end']==end
    equal={name:(a.flight/name).read_bytes()==(a.reference/name).read_bytes() for name in ['cam0.yaml','rovio.info','route/route.json','route/segments.json']}
    settings=[yaml.safe_load((d/'mimosa.yaml').read_text()) for d in [a.flight,a.reference]]
    for c in settings:c.pop('logs_directory')
    equal['fusion_weights_and_extrinsics']=settings[0]==settings[1]
    for key in ['imu_runtime_parameters','stationary_mode','slam_hover_initialization','drpm_fusion','binary_sha256','seed']:
        equal[key]=protocol[key]==reference[key]
    assert all(equal.values()),equal
    enc=load(a.flight/'encoders.csv');angles=np.rad2deg(np.c_[enc['yaw'],enc['pitch']]);route=load(a.flight/'route_tracking.csv');track=route['route_t']>0
    lo,hi=route['sim_t'][track][0],route['sim_t'][track][-1];keep=(enc['t']>=lo)&(enc['t']<=hi)
    errors=np.abs(angles[keep]-target);assert errors.max()<.05,errors.max()
    first_vio=poses(a.flight/'rovio.csv')[0,0]*1e-9
    init_angles=np.array([np.interp(first_vio,enc['t'],angles[:,i]) for i in range(2)])
    assert np.max(np.abs(init_angles-target))<.05
    settled=np.maximum.accumulate(np.max(np.abs(angles-target),axis=1)[::-1])[::-1]<.05
    first_settled=float(enc['t'][np.flatnonzero(settled)[0]])
    text=(a.flight/'rovio.log').read_text();import re
    m=re.search(r'\[\d+\.\d+, (\d+\.\d+)\]: Stationary IMU samples=',text)
    assert m,'Missing stationary initialization log'
    assert first_settled<float(m[1])-protocol['stationary_initialization_duration'],'Camera not settled before averaging'
    result=dict(flight=str(a.flight),reference=str(a.reference),scope='Single trial per orientation, separate physical flights. Same route/seed/estimator settings; not identical raw inputs. Tracking-only 0.1 s route grid, first-pose rigid alignment, no scale fit.',target_yaw_pitch_deg=target.tolist(),tracking_angle_min_deg=angles[keep].min(axis=0).tolist(),tracking_angle_max_deg=angles[keep].max(axis=0).tolist(),max_tracking_angle_error_deg=errors.max(axis=0).tolist(),settled_sim_s=first_settled,first_vio_sim_s=first_vio,first_vio_angles_deg=init_angles.tolist(),settings_equal=equal,input_audit=json.loads((a.flight/'input_audit.json').read_text()),runtime=runtime,metrics={})
    a.output.mkdir(parents=True,exist_ok=True);series={}
    trials=[(view_label(reference.get('fixed_angles_deg') or [0,0]),a.reference), (view_label(target),a.flight)]
    for extra in a.extra_reference:
        ep=json.loads((extra/'protocol.json').read_text());er=json.loads((extra/'runtime.json').read_text())
        assert ep['mode']=='fixed' and ep['route_end']==end and er['status']=='route_complete' and er['stopped']
        es=yaml.safe_load((extra/'mimosa.yaml').read_text());es.pop('logs_directory');assert es==settings[0]
        for key in ['imu_runtime_parameters','stationary_mode','slam_hover_initialization','drpm_fusion','binary_sha256','seed']:
            assert ep[key]==protocol[key],key
        for file in ['cam0.yaml','rovio.info','route/route.json','route/segments.json']:
            assert (extra/file).read_bytes()==(a.flight/file).read_bytes(),file
        trials.append((view_label(ep.get('fixed_angles_deg') or [0,0]),extra))
    assert len({label for label,_ in trials})==len(trials),'Duplicate orientation'
    result['trial_paths']={label:str(d) for label,d in trials}
    result['stationary_average_start_approx_sim_s']=float(m[1])-protocol['stationary_initialization_duration']
    for label,d in trials:
        result['metrics'][label],series[label]=assess(d,end)
        for name,(t,stamps,e) in series[label].items():np.savetxt(a.output/(label+'_'+name+'.csv'),np.c_[t,stamps,e],delimiter=',',header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',comments='',fmt=['%.9f','%.0f']+['%.12g']*6)
    (a.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n');images=[]
    with PdfPages(a.output/'comparison.pdf') as pdf:
        for name in ['rovio','mimosa']:
            fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
            for (label,_),color in zip(trials,['#a87632','#236aa1','#b34c55','#3e8c59']):
                t,_,e=series[label][name]
                for i,ax in enumerate([*axs[:,0],*axs[:,1]]):ax.plot(t,e[:,i],label=label,color=color,lw=1)
            for ax,label in zip([*axs[:,0],*axs[:,1]],['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(label);ax.grid(alpha=.25)
            for ax in axs[-1]:ax.set_xlabel('Route time [s]')
            axs[0,0].legend();fig.suptitle(f'run1_tail_a1 / {name}\nFixed camera directions; current yaw {target[0]:g}, pitch {target[1]:g}; DRPM OFF')
            fig.tight_layout();image=name+'.png';fig.savefig(a.output/image,dpi=135);pdf.savefig(fig);plt.close(fig);images.append(image)
    rows=[]
    for label,values in result['metrics'].items():
        for name,v in values.items():rows.append(f'<tr><td>{label}</td><td>{name}</td><td>{v["position_rmse_m"]:.3f}</td><td>{v["rotation_rmse_deg"]:.3f}</td><td>{v["position_max_m"]:.3f}</td></tr>')
    title=f'a1: yaw {target[0]:+g} / pitch {target[1]:g} fixed'
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>'+title+'</title><style>body{font-family:sans-serif;max-width:1150px;margin:25px auto;line-height:1.7}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style><h1>'+title+'</h1><p>'+result['scope']+'</p><p>Measured camera angles checked throughout tracking and before VIO initialization. Sensor noise and fusion weights unchanged. No GUI or rosbag.</p><table><tr><th>View</th><th>Estimator</th><th>Position RMSE [m]</th><th>Rotation RMSE [deg]</th><th>Max position error [m]</th></tr>'+''.join(rows)+'</table><p><a href="comparison.pdf">PDF</a> / <a href="comparison.json">Metrics, angle validation and input audit</a></p>'+''.join(f'<img src="{x}">' for x in images))
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
