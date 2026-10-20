#!/usr/bin/env python3
"""Single-factor VIO replays, directly into ROVIO, with exact sensor stamps."""
import argparse,json,os,re,signal,struct,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'logs/zab-offline/vio_audit'
def run(label,variant):
 inputs=OUT/'inputs'/label;manifest=json.loads((inputs/'manifest.json').read_text());src=ROOT/'logs'/manifest['source']['run']
 dest=OUT/'experiments'/label/variant;dest.mkdir(parents=True,exist_ok=False)
 a=np.load(inputs/manifest['chunks'][0]);sel=(a['imu_t']>=2e9)&(a['imu_t']<5e9);q=Slerp(a['truth_t']*1e-9,Rotation.from_quat(a['truth'][:,3:7]))(a['imu_t'][sel]*1e-9)
 bias=np.mean(a['imu'][sel,:3]-q.inv().apply([0,0,9.81]),axis=0)
 mean=a['imu'][sel,:3].mean(0);config=(src/'rovio.info').read_text()
 if variant=='stereo':config=config.replace('doStereoInitialization false','doStereoInitialization true')
 if variant=='bias_cov':config=re.sub(r'(acb_[012]\s+)4e-4;',r'\g<1>0.04;',config)
 if variant=='bias_cov_matched':config=re.sub(r'(acb_[012]\s+)4e-4;',r'\g<1>0.01;',config)
 if variant in ('gyro_rw_static','gyro_model_static'):
  config=re.sub(r'(gyb_[012]\s+)3.8e-7;',r'\g<1>1e-12;',config)
 if variant=='gyro_model_static':
  config=re.sub(r'(gyb_[012]\s+)3e-4;',r'\g<1>1e-8;',config)
  config=re.sub(r'(att_[012]\s+)7.6e-7;',r'\g<1>4e-11;',config)
 if variant in ('gyro_model_matched','gyro_matched_depth10'):
  config=re.sub(r'(gyb_[012]\s+)3e-4;',r'\g<1>1e-10;',config)
  config=re.sub(r'(gyb_[012]\s+)3.8e-7;',r'\g<1>1e-16;',config)
  config=re.sub(r'(att_[012]\s+)7.6e-7;',r'\g<1>4e-11;',config)
 if variant in ('gyro_initial_only','gyro_bias_matched'):
  config=re.sub(r'(gyb_[012]\s+)3e-4;',r'\g<1>1e-10;',config)
 if variant=='gyro_bias_matched':config=re.sub(r'(gyb_[012]\s+)3.8e-7;',r'\g<1>1e-16;',config)
 if variant=='gyro_measurement_only':config=re.sub(r'(att_[012]\s+)7.6e-7;',r'\g<1>4e-11;',config)
 if variant=='bias_oracle':
  for j,b in enumerate(bias):config=re.sub(rf'(acb_{j}\s+)0;',rf'\g<1>{b:.15g};',config,count=1)
 if variant in ('depth10','gyro_matched_depth10'):config=config.replace('initDepth 0.5;','initDepth 0.1;')
 if variant=='direct':config=config.replace('useDirectMethod false','useDirectMethod true')
 (dest/'rovio.info').write_text(config)
 binary=OUT/('audit75' if variant=='features75' else 'audit25')
 initialization=re.findall(r'-- Filter: Initialized at t = ([0-9.]+)',(ROOT/'logs/zab-offline'/label/'rovio.log').read_text())
 assert len(initialization)==1
 initialization_ns=round(float(initialization[0])*1e9)
 counts=[0]*4;first_imu=None;first_sent=False;wall=time.monotonic()
 with (dest/'replay.log').open('w') as log:
  process=subprocess.Popen([str(binary),str(dest/'rovio.info'),str(src),str(dest/'result'),f'__name:=audit_{label}_{variant}'],stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT)
  try:
   for name in manifest['chunks']:
    with np.load(inputs/name) as a:
     arrays={k:a[k] for k in a.files if k in ['imu','imu_t','order'] or k.startswith('cam')}
    for stamp,kind,index,stored in arrays['order']:
     kind=int(kind);stamp=int(stamp)
     if kind==0:
      if first_imu is None:first_imu=stamp
      if stamp<initialization_ns:continue
      values=arrays['imu'][index].copy()
      if variant=='bias_removed':values[:3]-=bias
      if variant=='initial_mean' and not first_sent:values[:3]=mean
      first_sent=True;payload=values.astype('<f8').tobytes()
     else:payload=arrays[f'cam{kind-1}_im'][index].tobytes()
     process.stdin.write(struct.pack('<IQ',kind,stamp));process.stdin.write(payload);counts[kind]+=1
    print(label,variant,name,round(time.monotonic()-wall,1),flush=True)
   process.stdin.close();code=process.wait(timeout=60)
   if code:raise RuntimeError(f'replay exit {code}')
  finally:
   if process.poll() is None:process.terminate();process.wait(timeout=10)
 (dest/'run.json').write_text(json.dumps(dict(label=label,variant=variant,counts=counts,initialization_ns=initialization_ns,wall_s=time.monotonic()-wall,initial_mean=mean.tolist(),oracle_bias=bias.tolist(),source_manifest=manifest,method='Direct calls, original storage order, three images grouped by exact header stamps; same initialization stamp as original replay; no ROS queues, no GT correction. bias_oracle only uses initial stationary GT attitude as a diagnostic.'),indent=2))
 print('COMPLETE',label,variant,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('variants',nargs='+');a=p.parse_args()
 for variant in a.variants:run(a.label,variant)
