#!/usr/bin/env python3
"""Same-flight ROVIO-only diagnostic; added outputs never drive fusion/control."""
import argparse,hashlib,json,os,signal,subprocess,time,yaml
from pathlib import Path
from compare_fusion_sidecar import owner_environment


def main():
    p=argparse.ArgumentParser();p.add_argument('--flight',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,required=True);a=p.parse_args()
    a.output.mkdir(exist_ok=False);start=time.monotonic();env=None
    while time.monotonic()-start<1800:
        if (a.flight/'rovio_runtime.yaml').exists() and (a.flight/'cam0.yaml').exists():
            env=owner_environment(a.port)
            if env:break
        time.sleep(.2)
    if not env:raise RuntimeError('No owner started')
    status=json.loads((a.flight/'route_status.json').read_text()) if (a.flight/'route_status.json').exists() else {}
    if status.get('sim_time',0)>15:raise RuntimeError('Too late to share initialization')
    protocol=json.loads((a.flight/'protocol.json').read_text());cfg=yaml.safe_load((a.flight/'mimosa.yaml').read_text())
    K=yaml.safe_load((a.flight/'cam0.yaml').read_text())['camera_matrix']['data'];T=cfg['lidar']['T_B_S']
    runtime=yaml.safe_load((a.flight/'rovio_runtime.yaml').read_text())['rovio_gimbal']
    binary=Path('/root/multilink_autonomy_ws/build/rovio-lidar-bootstrap/rovio_gimbal')
    manifest=json.loads(binary.with_name('manifest.json').read_text())
    for path,sha in manifest['sha256'].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
    (a.output/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    processes={};logs=[];commands={}
    try:
        for name,side,enabled in [('duplicate_active','active',False),('depth_active','active',True),('depth_fixed','fixed',True)]:
            node='bootstrap_'+name;folder=a.output/name;folder.mkdir();ref=a.flight/('fixed' if side=='fixed' else '')
            params=dict(runtime,lidar_depth_bootstrap=enabled,depth_intrinsics=[float(x) for x in K],depth_T_B_L=[float(x) for x in T],lidar_bootstrap_csv=str(folder/'initial_depth.csv'))
            parameter_path=folder/'parameters.yaml';parameter_path.write_text(yaml.safe_dump(params))
            subprocess.run(['rosparam','load',str(parameter_path),'/'+node],env=env,check=True,stdout=subprocess.DEVNULL)
            source='/fixed/active' if side=='fixed' else '/paired/active'
            command=[str(binary),'__name:='+node,'_config:='+str(ref/'rovio.info'),'_calibration:='+str(ref/'cam0.yaml'),
                '_stationary_initialization_duration:='+str(protocol.get('stationary_initialization_duration',3.)),
                '_stationary_mode:='+protocol.get('stationary_mode','wait'),'_diagnostic_csv:='+str(folder/'vio.csv'),
                '/slam/input/imu:=/vio/initialized_imu','/active/image:='+source+'/image','/active/extrinsic:='+source+'/extrinsic',
                '/rovio/odometry:=/'+node+'/odometry','/rovio/pcl:=/'+node+'/pcl']
            command.extend('/rovio/'+topic+':=/'+node+'/'+topic for topic in
                ['reset','reset_to_pose','transform','pose_with_covariance_stamped','patch','markers','T_G_W','extrinsics0','imu_biases'])
            command.append('/tf:=/'+node+'/tf')
            commands[name]=command;log=(folder/'node.log').open('w');logs.append(log)
            processes[name]=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (a.output/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        while not (a.flight/'runtime.json').exists() and time.monotonic()-start<2800:
            for name,proc in processes.items():
                if proc.poll() is not None:raise RuntimeError('Observer exited early: '+name)
            time.sleep(.2)
        if not (a.flight/'runtime.json').exists():raise TimeoutError('Reference did not finish')
    finally:
        for proc in processes.values():
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in processes.values():
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs:log.close()
        (a.output/'runtime.json').write_text(json.dumps(dict(scope=__doc__,stopped=True,processes={k:p.returncode for k,p in processes.items()},reference_finished=(a.flight/'runtime.json').exists()),indent=2)+'\n')
    print('Finished ROVIO-only depth diagnostic',flush=True)


if __name__=='__main__':main()
