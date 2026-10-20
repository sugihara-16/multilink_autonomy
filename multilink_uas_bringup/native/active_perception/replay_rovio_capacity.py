#!/usr/bin/env python3
"""Capacity-only ROVIO comparison on previous fixed-camera calibration inputs.

First require byte-exact reproduction with the current 25-feature binary.
Original inputs, camera parameters and filter parameters are read-only. No raw
sensor stream is copied; only estimated poses and a small diagnostic are saved.
"""
import argparse,hashlib,json,os,re,shlex,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import yaml
from replay_unipilot_initialization import evaluate
from build import HERE,NATIVE,WS,BUILD


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=11492);p.add_argument('--features',type=int,nargs='+',choices=[25,75],default=[25,75]);p.add_argument('--penalty-distance',type=float);p.add_argument('--direct',action='store_true');p.add_argument('--end-level',type=int,choices=[0,1]);p.add_argument('--uncertainty-log',action='store_true');a=p.parse_args()
    if a.uncertainty_log and (a.features!=[25] or a.penalty_distance is not None or a.direct or a.end_level is not None):raise ValueError('Uncertainty replay changes logging only; requires 25 features and no parameter changes')
    if a.penalty_distance is not None and not 10<=a.penalty_distance<=100:raise ValueError('Spatial selection distance out of range')
    a.output.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('Master port already in use')
    import sys
    sys.path.insert(0,str(NATIVE));from manager import CLEAN
    prefix=f'source {NATIVE}/env_ros1.sh\nsource {WS}/slam_ros1_ws/devel/setup.bash\nexport ROS_MASTER_URI=http://127.0.0.1:{a.port}\nexport OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1\nexec '
    def command(args):return ['bash','--noprofile','--norc','-c',prefix+shlex.join(map(str,args))]
    with (a.output/'master.log').open('w') as f:
        master=subprocess.Popen(command(['roscore','-p',str(a.port)]),env=CLEAN,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    def stop(*unused):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    result=dict(scope=__doc__,flights={})
    try:
        proxy=xmlrpc.client.ServerProxy(f'http://127.0.0.1:{a.port}')
        for _ in range(100):
            try:proxy.getPid('/capacity_audit');break
            except OSError:time.sleep(.1)
        else:raise RuntimeError('Master did not start')
        for name in ['run1_tail_a1','run1_tail_b1']:
            base=a.source/(name+'_active');fixed=base/'fixed';protocol=json.loads((base/'protocol.json').read_text())
            runtime=yaml.safe_load((base/'rovio_runtime.yaml').read_text())['rovio_gimbal']
            params=a.output/(name+'_runtime.yaml');params.write_text(yaml.safe_dump({'rovio_gimbal':runtime}))
            subprocess.run(command(['rosparam','load',params]),env=CLEAN,check=True)
            result['flights'][name]={}
            for count,binary in [(25,BUILD/'rovio_gimbal'),(75,WS/'build/rovio-capacity-75/rovio_gimbal')]:
                if count not in a.features:continue
                if a.uncertainty_log:binary=WS/'build/rovio-uncertainty/rovio_gimbal'
                dest=a.output/f'{name}_{count}';dest.mkdir();beg=time.monotonic()
                config=fixed/'rovio.info'
                if a.penalty_distance is not None:
                    text,n=re.subn(r'(\bpenaltyDistance\s+)[^;\s]+;',lambda m:m[1]+str(a.penalty_distance)+';',config.read_text())
                    if n!=1:raise RuntimeError('Expected exactly one spatial selection setting')
                    config=dest/'rovio.info';config.write_text(text)
                if a.direct:
                    text,n=re.subn(r'(\buseDirectMethod\s+)false;',r'\g<1>true;',config.read_text())
                    if n!=1:raise RuntimeError('Expected one indirect visual-update setting')
                    config=dest/'rovio.info';config.write_text(text)
                if a.end_level is not None:
                    text,n=re.subn(r'(\bendLevel\s+)\d+;',lambda m:m[1]+str(a.end_level)+';',config.read_text())
                    if n!=1:raise RuntimeError('Expected one lowest image-pyramid level')
                    config=dest/'rovio.info';config.write_text(text)
                args=[binary,f'_config:={config}',f'_calibration:={fixed}/cam0.yaml',
                      '_stationary_initialization_duration:='+str(protocol['stationary_initialization_duration']),
                      '_stationary_mode:='+protocol['stationary_mode'],f'_input_replay:={fixed}/vio_input.bin',f'_diagnostic_csv:={dest}/vio.csv']
                with (dest/'replay.log').open('w') as log:
                    subprocess.run(command(args),env=CLEAN,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
                row=dict(features=count,penalty_distance=a.penalty_distance,direct=a.direct,end_level=a.end_level,wall_s=time.monotonic()-beg,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),metrics=evaluate(dest/'vio.csv',fixed))
                if count==25 and a.penalty_distance is None and not a.direct and a.end_level is None:
                    original=(fixed/'vio_diagnostic.csv').read_bytes();replay=(dest/'vio.csv').read_bytes()
                    if a.uncertainty_log:
                        columns=len(original.splitlines()[0].split(b','))
                        replay=b''.join(b','.join(line.split(b',')[:columns])+b'\n' for line in replay.splitlines())
                        row['uncertainty_logging_only']=True
                    row['null_byte_equal']=original==replay
                result['flights'][name][count]=row
                (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(name,count,json.dumps(row),flush=True)
                if count==25 and a.penalty_distance is None and not a.direct and a.end_level is None and not row['null_byte_equal']:raise RuntimeError('Current replay does not reproduce prior 25-feature output; do not interpret capacity comparison')
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        os.killpg(master.pid,signal.SIGINT)
        try:master.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()


if __name__=='__main__':main()
