#!/usr/bin/env python3
"""Compare a physical sensor profile change; never assert identical raw inputs."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
from report_unipilot_imu_fix import assess
from report_routes import load
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def main():
 p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--dragon',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 protocol=json.loads((a.flight/'protocol.json').read_text());runtime=json.loads((a.flight/'runtime.json').read_text())
 assert runtime['status']=='route_complete' and runtime['stopped'] and all(v==0 for v in runtime['processes'].values())
 assert protocol['sensor_profile']=='dragon' and protocol['fixed_angles_deg']==[0,0] and not protocol['drpm_fusion']
 end=protocol['route_end'];base_protocol=json.loads((a.baseline/'protocol.json').read_text())
 assert end==base_protocol['route_end']
 equal={n:(a.flight/n).read_bytes()==(a.baseline/n).read_bytes() for n in ['route/route.json','route/segments.json']}
 configs=[yaml.safe_load((d/'mimosa.yaml').read_text()) for d in [a.flight,a.baseline]]
 for c in configs:c.pop('logs_directory')
 equal['fusion_config']=configs[0]==configs[1]
 for k,v in protocol['binary_sha256'].items():
  if not k.startswith('native_'):
   if k=='rovio_gimbal' and v!=base_protocol['binary_sha256'].get(k):
    current=(a.flight/'implementation/rovio_gimbal.cpp').read_text()
    previous=(a.baseline.parent/'implementation/rovio_gimbal.cpp').read_text()
    equal['rovio_only_pre_imu_image_guard']=current.replace('if(!stationary_ready || imu_time==0)return;','if(!stationary_ready)return;')==previous
   else:equal['binary_'+k]=v==base_protocol['binary_sha256'].get(k)
 assert not protocol['truth_input_to_estimators']
 for k in ['stationary_mode','slam_hover_initialization','drpm_fusion','seed']:equal[k]=protocol[k]==base_protocol[k]
 # Body, controller, extrinsics and joint geometry must be identical.
 roots=[ET.parse(d/'unipilot_gimbal.sdf').getroot() for d in [a.flight,a.baseline.parent]]
 for name in ['imu','lidar','rgb_cam']:
  sensors=[r.find(f".//sensor[@name='{name}']") for r in roots]
  equal[name+'_pose']=sensors[0].findtext('pose')==sensors[1].findtext('pose')
 for r in roots:
  for parent in r.iter():
   for child in list(parent):
    if child.tag=='sensor':parent.remove(child)
 equal['body_controller_gimbal']=ET.tostring(roots[0])==ET.tostring(roots[1])
 assert all(equal.values()),equal
 enc=load(a.flight/'encoders.csv');track=load(a.flight/'route_tracking.csv');mask=track['route_t']>0
 take=(enc['t']>=track['sim_t'][mask][0])&(enc['t']<=track['sim_t'][mask][-1]);angles=np.rad2deg(np.c_[enc['yaw'],enc['pitch']][take]);assert np.abs(angles).max()<.05
 report=dict(scope='a1, fixed yaw/pitch 0, DRPM OFF. Different physical flights, one trial each. Sensor profile changes camera/IMU/LiDAR specification together; ROVIO noise profile and stationary initialization duration are recorded separately; does not identify an individual causal parameter. Dragon reference has different body, mounting and historic estimator calibration. Tracking only, first-pose rigid alignment, no scale fitting, common 0.1 s route grid.',unchanged=equal,intended_changes=['camera FOV/resolution/clipping/noise','IMU rate/white noise/static bias/dynamic bias/quantization','LiDAR scan/range/resolution','camera intrinsics and image resampling derived from physical sensor','ROVIO IMU noise profile and stationary initialization duration: explicitly recorded in estimator_condition'],estimator_condition=dict(rovio_noise_profile=protocol.get('rovio_noise_profile','sensor'),stationary_duration=protocol['stationary_initialization_duration'],baseline_stationary_duration=base_protocol['stationary_initialization_duration']),runtime=runtime,input_audit=json.loads((a.flight/'input_audit.json').read_text()),max_tracking_angle_error_deg=np.abs(angles).max(axis=0).tolist(),metrics={})
 series={};a.output.mkdir(parents=True,exist_ok=True)
 for label,d in [('UniPilot native',a.baseline),('UniPilot DRAGON sensors',a.flight),('DRAGON reference',a.dragon)]:
  report['metrics'][label],series[label]=assess(d,end)
  for estimator,(t,stamps,err) in series[label].items():
   np.savetxt(a.output/(label.replace(' ','_')+'_'+estimator+'.csv'),np.c_[t,stamps,err],delimiter=',',header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',comments='',fmt=['%.9f','%.0f']+['%.12g']*6)
 (a.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
 with PdfPages(a.output/'comparison.pdf') as pdf:
  for estimator in ['rovio','mimosa']:
   fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
   for label,color in zip(series,['#c56e22','#2266bb','#319452']):
    t,_,err=series[label][estimator]
    for i,ax in enumerate([*axs[:,0],*axs[:,1]]):ax.plot(t,err[:,i],label=label,color=color,lw=.9)
   for ax,label in zip([*axs[:,0],*axs[:,1]],['X [m]','Y [m]','Z [m]','Roll [deg]','Pitch [deg]','Yaw [deg]']):ax.set_ylabel(label);ax.grid(alpha=.25)
   for ax in axs[-1]:ax.set_xlabel('Route time [s]')
   axs[0,0].legend(fontsize=8);fig.suptitle('a1 / '+estimator+' / physical sensor profile comparison\nSeparate flights; full axes; DRAGON mounting differs')
   fig.tight_layout();fig.savefig(a.output/(estimator+'.png'),dpi=140);pdf.savefig(fig);plt.close(fig)
 rows=''.join(f'<tr><td>{label}</td><td>{e}</td><td>{m["position_rmse_m"]:.3f}</td><td>{m["rotation_rmse_deg"]:.3f}</td></tr>' for label,metrics in report['metrics'].items() for e,m in metrics.items())
 (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Sensor profile comparison</title><style>body{font-family:sans-serif;max-width:1150px;margin:25px auto}img{width:100%}td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}</style><h1>a1: sensor profile comparison</h1><p>'+report['scope']+'</p><p>ROVIO noise profile: '+str(report['estimator_condition']['rovio_noise_profile'])+'; stationary averaging: '+str(report['estimator_condition']['stationary_duration'])+' s.</p><table><tr><th>Condition</th><th>Estimator</th><th>Position RMSE [m]</th><th>Rotation RMSE [deg]</th></tr>'+rows+'</table><p><a href="comparison.pdf">PDF</a> / <a href="comparison.json">Metrics and validation</a></p><img src="rovio.png"><img src="mimosa.png">')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
