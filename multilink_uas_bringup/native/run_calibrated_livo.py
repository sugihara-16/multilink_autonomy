#!/usr/bin/env python3
"""Held-out covariance calibration and controlled vision ablations; isolated ROS1."""
import hashlib,json,subprocess,time
from pathlib import Path
import numpy as np
import rospy,yaml

ROOT=Path(__file__).resolve().parent
BASE=ROOT/'logs/zab-offline/livo_comparison'
OUT=ROOT/'logs/zab-offline/livo_calibrated'
WS=Path('/root/multilink_autonomy_ws/comparison_ws')
LABELS=['run2a_end','run1_tail_a2_prefix']
METHODS=['mimosa_calibrated','mimosa_drpm_calibrated','mimosa_lio','fast_full','fast_no_visual','fast_lio','fast_tracking_no_visual']

def run(method,label):
    dest=OUT/method/label;dest.mkdir(parents=True,exist_ok=True)
    if (dest/'summary.json').exists():raise RuntimeError(f'Refusing to overwrite completed trial {dest}')
    w=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window']
    fold=json.loads((OUT/'calibration.json').read_text())['folds'][label]
    if method.startswith('mimosa'):
        config=yaml.safe_load((BASE/'mimosa_off'/label/'config.yaml').read_text())
        config['logs_directory']=str(dest)+'/'
        config['lidar']['geometric']['scan_to_map']['drpm_enabled']=method=='mimosa_drpm_calibrated'
        config['odometry']['manager']['enabled']=method!='mimosa_lio'
        if method!='mimosa_lio':
            config['odometry']['manager']['relative_pose_covariance']=np.array(fold['covariance']).ravel().tolist()
        (dest/'config.yaml').write_text(yaml.safe_dump(config))
        rospy.set_param('/mimosa',config)
        rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
        rospy.set_param('/mimosa/bag_name',str(BASE/'inputs'/f'{label}.bag'))
        cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa',
             '~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points',
             '~odometry/manager/odometry_in:=/comparison/vio']
    else:
        config=yaml.safe_load((BASE/'fast_livo2'/label/'config.yaml').read_text())
        config['parameters']['common']['img_en']=0 if method=='fast_lio' else 1
        config['disable_visual_update']=method=='fast_no_visual'
        if method=='fast_tracking_no_visual':config['disable_visual_after_s']=w['start_ns']*1e-9
        for key,value in config['parameters'].items():rospy.set_param('/'+key,value)
        rospy.set_param('/laserMapping',config['camera'])
        rospy.set_param('/comparison_disable_visual_update',config['disable_visual_update'])
        rospy.set_param('/comparison_disable_visual_after_s',config.get('disable_visual_after_s',-1.0))
        (dest/'config.yaml').write_text(yaml.safe_dump(config))
        cmd=[str(WS/'devel/lib/fast_livo/fastlivo_offline'),str(BASE/'inputs'/f'{label}.bag'),str(dest/'odometry.csv')]
    begin=time.monotonic()
    with (dest/'run.log').open('w') as log:
        proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=600)
    summary=dict(method=method,label=label,returncode=proc.returncode,wall_s=time.monotonic()-begin,window=w,
                 command=cmd,binary_sha256=hashlib.sha256(Path(cmd[0]).read_bytes()).hexdigest())
    if method in ['mimosa_calibrated','mimosa_drpm_calibrated']:summary['calibration_flight']=fold['training_flight']
    (dest/'summary.json').write_text(json.dumps(summary,indent=2))
    print(method,label,'exit',proc.returncode,'wall_s',summary['wall_s'],flush=True)
    if proc.returncode:raise RuntimeError(str(dest))

if __name__=='__main__':
    import sys
    rospy.init_node('calibrated_livo_driver',anonymous=True,disable_signals=True)
    methods=sys.argv[1:] or METHODS
    for method in methods:
        assert method in METHODS
        for label in LABELS:run(method,label)
