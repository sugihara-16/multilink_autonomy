#!/usr/bin/env python3
"""Controlled replay of recorded fixed-camera inputs. Oracle modes are diagnosis only."""
import argparse,hashlib,json,os,re,shlex,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from report_routes import load,poses,interp
from unipilot_calibration import imu_runtime_parameters
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]

def evaluate(csv,flight,end,out):
 route=load(flight/'route_tracking.csv');mask=route['route_t']>0;t=route['sim_t'][mask];r=route['route_t'][mask];keep=np.r_[True,np.diff(r)>1e-8];t=t[keep];r=r[keep]
 grid=np.arange(.1,end-.001,.1);stamps=np.rint(np.interp(grid,r,t)*1e9);a=poses(csv);gt=poses(flight/'truth.csv')
 assert r[-1]>=grid[-1] and stamps[0]>=max(a[0,0],gt[0,0]) and stamps[-1]<=min(a[-1,0],gt[-1,0])
 p,q=interp(a,stamps);g,gq=interp(gt,stamps);align=gq[0]*q[0].inv();pe=align.apply(p-p[0])+g[0]-g;qe=align*q;rot=np.rad2deg((gq.inv()*qe).magnitude());euler=np.rad2deg((qe.as_euler('xyz')-gq.as_euler('xyz')+np.pi)%(2*np.pi)-np.pi);pos=np.linalg.norm(pe,axis=1)
 result=dict(position_rmse_m=float(np.sqrt(np.mean(pos**2))),rotation_rmse_deg=float(np.sqrt(np.mean(rot**2))),position_max_m=float(pos.max()),samples=len(grid),first_exceeds_5m_s=float(grid[np.flatnonzero(pos>5)[0]]) if np.any(pos>5) else None)
 for lo,hi in [(0,30),(30,100),(100,end)]:
  m=(grid>=lo)&(grid<hi)
  if m.any():result[f'position_rmse_{lo}_{hi:g}s']=float(np.sqrt(np.mean(pos[m]**2)))
 diag=np.genfromtxt(csv,delimiter=',',names=True);mask=(diag['stamp_ns']>=stamps[0])&(diag['stamp_ns']<=stamps[-1]);diag=diag[mask]
 result.update(feature_count_percentiles=np.percentile(diag['features'],[0,10,50,90,100]).tolist(),median_depth_percentiles_m=np.percentile(diag['median_depth'],[0,10,50,90,100]).tolist(),max_output_gap_s=float(np.diff(diag['stamp_ns']).max()*1e-9))
 np.savez_compressed(out/'errors.npz',route_t=grid,stamp_ns=stamps,error=np.c_[pe,euler],position_error=pos,rotation_error=rot)
 return result

def main():
 p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=11501);p.add_argument('--binary',type=Path,default=WS/'build/active_perception/rovio_gimbal');p.add_argument('--feature-log',action='store_true');p.add_argument('--modes',nargs='+',default=['legacy_check','baseline','no_static_init','random_walk','dragon_prediction']);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 flight=a.flight;parent=flight.parent if flight.name=='fixed' else flight;protocol=json.loads((flight/'protocol.json').read_text());binary=a.binary;runtime=imu_runtime_parameters(parent/'unipilot_gimbal.sdf',parent/'world_serial.sdf','sensor')
 env={k:v for k,v in os.environ.items() if k in ['HOME','USER','PATH','LANG']};env['PATH']='/usr/local/bin:/usr/bin:/bin'
 prefix=f'source {NATIVE}/env_ros1.sh\nsource {WS}/slam_ros1_ws/devel/setup.bash\nexport ROS_MASTER_URI=http://127.0.0.1:{a.port}\nexport OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1\nexec '
 def cmd(v):return ['bash','--noprofile','--norc','-c',prefix+shlex.join(map(str,v))]
 with socket.socket() as s:
  if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('Master port busy')
 with (a.output/'master.log').open('w') as log:master=subprocess.Popen(cmd(['roscore','-p',str(a.port)]),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 report=dict(flight=str(flight),input_sha256=hashlib.file_digest((flight/'vio_input.bin').open('rb'),'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256((flight/'vio_input.bin').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),scope='Same callback-ordered images/IMU/extrinsics except explicitly labelled diagnostic oracle interventions. Tracking only, common .1s route grid, first-pose rigid alignment, no scale fitting.',runs={})
 if (a.output/'report.json').exists():report['runs']=json.loads((a.output/'report.json').read_text())['runs']
 try:
  proxy=xmlrpc.client.ServerProxy(f'http://127.0.0.1:{a.port}')
  for _ in range(100):
   try:proxy.getPid('/drift_review');break
   except OSError:time.sleep(.1)
  for mode in a.modes:
   if mode in report['runs']:continue
   out=a.output/mode;out.mkdir(exist_ok=True);config=(flight/'rovio.info').read_text();params=dict(runtime);duration=float(protocol.get('stationary_initialization_duration',3.));stationary=protocol.get('stationary_mode','mean');input_path=flight/'vio_input.bin';calibration=flight/'cam0.yaml';image_source=None;override=None;imu_source=None
   if mode=='legacy_check':params=dict(acc_bias_tau=[0.]*3,gyro_bias_tau=[0.]*3,gravity_magnitude=9.81);stationary='legacy'
   elif mode=='no_static_init':duration=0.
   elif mode=='wait_only':stationary='wait'
   elif mode.startswith('init_'):
    from audit_recorded_imu import imu_records
    from scipy.spatial.transform import Rotation as R
    u=np.array(list(imu_records(flight/'vio_input.bin')));idx=int(np.flatnonzero(u[:,0]*1e-9-u[0,0]*1e-9>=duration)[0]);window=u[:idx+1]
    avg=np.zeros(6)
    for row in window:avg+=row[1:]
    avg/=len(window);stationary='wait'
    bias=None;initial_acc=None
    if mode in ('init_mean_acc','init_mean_both'):initial_acc=avg[3:]
    if mode in ('init_mean_bias','init_mean_both'):bias=np.r_[avg[:3],avg[3:]*(1.-params['gravity_magnitude']/np.linalg.norm(avg[3:]))]
    if mode in ('init_true_gravity','init_true_gravity_bias'):
     _,q=interp(poses(flight/'truth.csv'),[u[idx,0]]);initial_acc=q.inv().apply([[0,0,params['gravity_magnitude']]])[0]
    if mode in ('init_true_bias','init_true_gravity_bias'):
     ref=np.loadtxt(parent/'diagnostic_sensors/imu.csv',delimiter=',',skiprows=1);ids=np.searchsorted(ref[:,0],window[:,0]);assert np.array_equal(ref[ids,0],window[:,0]);bias=np.mean(window[:,1:]-ref[ids,1:],axis=0)
    if mode not in ('init_mean_acc','init_mean_bias','init_mean_both','init_true_gravity','init_true_bias','init_true_gravity_bias'):raise ValueError(mode)
    if bias is not None:
     for key,v in zip(['gyb_0','gyb_1','gyb_2','acb_0','acb_1','acb_2'],bias):config=re.sub(rf'({key}\s+)[^;\s]+;',lambda m:m[1]+format(v,'.17g')+';',config,count=1)
    if initial_acc is not None:
     override=out/'initialization_override.csv';v=u[idx].copy();v[4:]=initial_acc;np.savetxt(override,v[None,:],delimiter=',',header='stamp_ns,gx,gy,gz,ax,ay,az',comments='',fmt=['%.0f']+['%.17g']*6);input_path=out/'input.fifo'
    (out/'initialization_intervention.json').write_text(json.dumps(dict(stamp_ns=int(u[idx,0]),samples=len(window),initial_bias=None if bias is None else bias.tolist(),initial_acc=None if initial_acc is None else initial_acc.tolist(),truth_assisted='true' in mode),indent=2))
   elif mode=='random_walk':params.update(acc_bias_tau=[0.]*3,gyro_bias_tau=[0.]*3)
   elif mode in ('depth_seed_3m','depth_seed_30m','depth_no_median','depth_cov_2'):
    changes={'depth_seed_3m':{'initDepth':1/3},'depth_seed_30m':{'initDepth':1/30},'depth_no_median':{'maxUncertaintyToDepthRatioForDepthInitialization':0},'depth_cov_2':{'initCovFeature_0':2}}[mode]
    for key,value in changes.items():config=re.sub(rf'({key}\s+)[^;\s]+;',lambda m:m[1]+format(value,'.12g')+';',config,count=1)
   elif mode in ('dragon_prediction','velocity_noise_dragon','gyro_noise_dragon'):
    ref=json.loads((NATIVE/'sensor_profiles/dragon_rovio_noise.json').read_text());start=config.index('Prediction\n');end=config.index('PoseUpdate\n',start);block=config[start:end]
    for key,values in ref['prediction'].items():
     if mode=='velocity_noise_dragon' and key!='vel':continue
     if mode=='gyro_noise_dragon' and key!='att':continue
     for i,value in enumerate(values):block=re.sub(rf'({key}_{i}\s+)[^;\s]+;',lambda m:m[1]+format(value,'.12g')+';',block,count=1)
    config=config[:start]+block+config[end:]
   elif mode in ('reference_clean','reference_clean_locked'):
    input_path=out/'input.fifo';imu_source=parent/'diagnostic_sensors/imu.csv';stationary='wait'
    assert imu_source.exists()
   elif mode.startswith('rate_clean_'):
    input_path=out/'input.fifo';duration=0.;imu_source=parent/'diagnostic_sensors'/('imu.csv' if '_200' in mode else 'imu_fast.csv')
    if '_average' in mode:imu_source=a.output/'imu_averaged_200.csv'
    assert imu_source.exists()
   elif mode.startswith('image_'):
    input_path=out/'input.fifo';image_source=parent/'diagnostic_sensors'/('wide.bin' if 'wide' in mode else ('far.bin' if 'far' in mode else 'clean.bin'))
    assert image_source.exists()
    if 'wide' in mode:
     cal=yaml.safe_load(calibration.read_text());cal['image_width']+=400;cal['camera_matrix']['data'][2]+=200
     calibration=out/'cam0.yaml';calibration.write_text(yaml.safe_dump(cal))
    if 'ideal_imu' in mode:override=a.output/'inputs/oracle_both.csv';duration=0.
   elif mode.startswith('oracle_'):
    input_path=out/'input.fifo'
    # Input intervention only: retain exactly the baseline initialization and noise settings.
    variant=mode.replace('_no_static','').replace('_locked_bias','');override=a.output/'inputs'/(variant+'.csv');assert override.exists()
    if mode.endswith('_no_static'):duration=0.
   elif mode not in ('baseline','online_check'):raise ValueError(mode)
   if mode.endswith('_locked_bias') or mode=='reference_clean_locked':
    if mode!='reference_clean_locked':duration=0.
    for section,finish,value in [('    Covariance\n','ImgUpdate\n',1e-16),('Prediction\n','PoseUpdate\n',0.)]:
     start=config.index(section);end=config.index(finish,start);block=config[start:end]
     for key in ('acb','gyb'):
      for i in range(3):block=re.sub(rf'({key}_{i}\s+)[^;\s]+;',lambda m:m[1]+format(value,'.12g')+';',block,count=1)
     config=config[:start]+block+config[end:]
   (out/'rovio.info').write_text(config);(out/'parameters.json').write_text(json.dumps(dict(runtime=params,stationary_duration=duration,stationary_mode=stationary,input=str(input_path),truth_assisted=(override is not None or imu_source is not None or mode.startswith('init_true')) and not mode.startswith('init_mean'),imu_source=str(imu_source),image_override=str(image_source),calibration=str(calibration)),indent=2)+'\n')
   for key,values in params.items():subprocess.run(cmd(['rosparam','set','/rovio_gimbal/'+key,json.dumps(values)]),env=env,check=True,stdout=subprocess.DEVNULL)
   args=[binary,f'_config:={out}/rovio.info',f'_calibration:={calibration}',f'_stationary_initialization_duration:={duration}',f'_stationary_mode:={stationary}',f'_input_replay:={input_path}',f'_diagnostic_csv:={out}/vio.csv']
   if a.feature_log:args.append(f'_feature_csv:={out}/features.csv')
   producer=None
   if override is not None or image_source is not None or imu_source is not None:
    if input_path.exists():input_path.unlink()
    os.mkfifo(input_path)
    producer=subprocess.Popen(['python3',str(HERE/'override_imu_stream.py'),str(flight/'vio_input.bin'),str(input_path),'--audit',str(out/'stream_audit.json'),*(['--imu-overrides',str(override)] if override else []),*(['--imu-source',str(imu_source)] if imu_source else []),*(['--image-source',str(image_source),'--minimum-image-end-ns',str(int(load(flight/'route_tracking.csv')['sim_t'][-1]*1e9)+100000000)] if image_source else [])])
   started=time.monotonic()
   try:
    with (out/'replay.log').open('w') as log:subprocess.run(cmd(args),env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=240)
   except BaseException:
    if producer and producer.poll() is None:producer.terminate();producer.wait(timeout=10)
    if input_path.suffix=='.fifo' and input_path.exists():input_path.unlink()
    raise
   if producer:
    producer.wait(timeout=10);assert producer.returncode==0;input_path.unlink()
   v=evaluate(out/'vio.csv',flight,protocol['route_end'],out);v['wall_s']=time.monotonic()-started
   if mode in ('legacy_check','online_check'):
    v['online_byte_identical']=(flight/'vio_diagnostic.csv').read_bytes()==(out/'vio.csv').read_bytes();assert v['online_byte_identical'],'Replay fidelity check failed'
   report['runs'][mode]=v;(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(mode,json.dumps(v),flush=True)
 finally:
  os.killpg(master.pid,signal.SIGINT)
  try:master.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
if __name__=='__main__':main()
