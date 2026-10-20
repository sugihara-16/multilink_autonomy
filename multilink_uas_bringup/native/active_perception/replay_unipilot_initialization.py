#!/usr/bin/env python3
"""Same callback-ordered inputs for initialization ablations; no truth to filter."""
import argparse,hashlib,json,os,re,shlex,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
HERE=Path(__file__).resolve().parent;NATIVE=HERE.parent;WS=NATIVE.parents[3]

def evaluate(csv_path,flight):
    d=np.genfromtxt(csv_path,delimiter=',',names=True);gt=np.genfromtxt(flight/'truth.csv',delimiter=',',names=True)
    route=np.genfromtxt(flight/'route_tracking.csv',delimiter=',',names=True);tracking=route[route['route_t']>0]
    lo,hi=tracking['sim_t'][0]*1e9,tracking['sim_t'][-1]*1e9
    d=d[(d['stamp_ns']>=max(lo,gt['stamp_ns'][0]))&(d['stamp_ns']<=min(hi,gt['stamp_ns'][-1]))]
    t=d['stamp_ns'];xyz=np.column_stack([d[k] for k in ('x','y','z')]);q=Rotation.from_quat(np.column_stack([d[k] for k in ('qx','qy','qz','qw')]))
    p=np.column_stack([np.interp(t,gt['stamp_ns'],gt[k]) for k in ('x','y','z')]);gq=Slerp((gt['stamp_ns']-gt['stamp_ns'][0])*1e-9,Rotation.from_quat(np.column_stack([gt[k] for k in ('qx','qy','qz','qw')])) )((t-gt['stamp_ns'][0])*1e-9)
    align=gq[0]*q[0].inv();ep=np.linalg.norm(align.apply(xyz-xyz[0])+p[0]-p,axis=1);er=np.rad2deg((gq.inv()*(align*q)).magnitude())
    result=dict(samples=len(d),position_rmse_m=float(np.sqrt(np.mean(ep**2))),position_max_m=float(ep.max()),rotation_rmse_deg=float(np.sqrt(np.mean(er**2))),rotation_max_deg=float(er.max()),scope='tracking window, first-pose rigid alignment, no scale adjustment')
    if 'median_depth' in d.dtype.names:result['median_depth_percentiles']=np.percentile(d['median_depth'],[0,50,100]).tolist()
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('flight',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--modes',nargs='+',default=['legacy','mean','gyro_prior','mean_gyro_prior']);p.add_argument('--port',type=int,default=11491);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    binary=WS/'build/active_perception/rovio_gimbal';env={k:v for k,v in os.environ.items() if k in ['HOME','USER','PATH','LANG']};env['PATH']='/usr/local/bin:/usr/bin:/bin'
    prefix=f'source {NATIVE}/env_ros1.sh\nsource {WS}/slam_ros1_ws/devel/setup.bash\nexport ROS_MASTER_URI=http://127.0.0.1:{a.port}\nexport OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1\nexec '
    def cmd(args):return ['bash','--noprofile','--norc','-c',prefix+shlex.join(map(str,args))]
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('Replay port in use')
    with (a.output/'master.log').open('w') as f:master=subprocess.Popen(cmd(['roscore','-p',str(a.port)]),env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),runs={})
    try:
        proxy=xmlrpc.client.ServerProxy(f'http://127.0.0.1:{a.port}')
        for _ in range(100):
            try:proxy.getPid('/replay');break
            except OSError:time.sleep(.1)
        for view,folder in [('active',a.flight),('fixed',a.flight/'fixed')]:
            if not folder.exists():continue
            for mode in a.modes:
                out=a.output/(view+'_'+mode);out.mkdir();config=(folder/'rovio.info').read_text()
                if 'gyro_prior' in mode:
                    # Variance of a 3 s gyro mean: white density/T plus
                    # bias driving density*T/3. No trajectory/error fitting.
                    block=config[config.index('Prediction\n'):config.index('PoseUpdate\n')]
                    prior=config[config.index('Init\n'):config.index('ImgUpdate\n')]
                    for axis in range(3):
                        white=float(re.search(rf'att_{axis}\s+(\S+);',block)[1]);drive=float(re.search(rf'gyb_{axis}\s+(\S+);',block)[1]);value=white/3+drive
                        prior=re.sub(rf'(gyb_{axis}\s+)[^;\s]+;',lambda m:m[1]+format(value,'.12g')+';',prior)
                    config=config[:config.index('Init\n')]+prior+config[config.index('ImgUpdate\n'):]
                (out/'rovio.info').write_text(config)
                args=[binary,f'_config:={out}/rovio.info',f'_calibration:={folder}/cam0.yaml','_stationary_initialization_duration:=3.0',f'_stationary_mode:={"legacy" if mode in ("legacy","gyro_prior","sensor","sensor_gravity") else "mean"}',f'_input_replay:={folder}/vio_input.bin',f'_diagnostic_csv:={out}/vio.csv']
                bias_parameters={'acc_bias_tau':[0.,0.,0.],'gyro_bias_tau':[0.,0.,0.],'gravity_magnitude':9.8 if 'gravity' in mode else 9.81}
                if mode.startswith('sensor'):
                    import xml.etree.ElementTree as ET
                    model=ET.parse(a.flight/'unipilot_gimbal.sdf').getroot()
                    for group,key in [('linear_acceleration','acc_bias_tau'),('angular_velocity','gyro_bias_tau')]:
                        taus=[]
                        for axis in 'xyz':
                            n=model.find(f".//sensor[@name='imu']/imu/{group}/{axis}/noise")
                            taus.append(float(n.findtext('dynamic_bias_correlation_time','0')) if float(n.findtext('dynamic_bias_stddev','0'))>0 else 0.)
                        bias_parameters[key]=taus
                    if 'gravity' in mode:args.append('_gravity_magnitude:=9.8')
                for key,values in bias_parameters.items():
                    subprocess.run(cmd(['rosparam','set','/rovio_gimbal/'+key,json.dumps(values)]),env=env,check=True,stdout=subprocess.DEVNULL)
                (out/'runtime_parameters.json').write_text(json.dumps(bias_parameters,indent=2)+'\n')
                started=time.monotonic()
                with (out/'replay.log').open('w') as f:subprocess.run(cmd(args),env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=240)
                v=evaluate(out/'vio.csv',folder);v['wall_s']=time.monotonic()-started
                if mode=='legacy':
                    original=folder/'vio_diagnostic.csv';v['online_byte_identical']=original.read_bytes()==(out/'vio.csv').read_bytes()
                    if not v['online_byte_identical']:raise RuntimeError('Legacy replay does not reproduce live filter')
                report['runs'][out.name]=v;(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(out.name,v,flush=True)
    finally:
        os.killpg(master.pid,signal.SIGINT)
        try:master.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
if __name__=='__main__':main()
