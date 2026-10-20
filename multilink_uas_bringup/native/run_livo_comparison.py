#!/usr/bin/env python3
"""Run isolated, sequential offline estimators; no simulation or GUI."""
import json,subprocess,time,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml,rospy
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/livo_comparison'
WS=Path('/root/multilink_autonomy_ws/comparison_ws')
def run(condition,label):
    dest=OUT/condition/label;dest.mkdir(parents=True,exist_ok=True)
    inp=OUT/'inputs'/label;manifest=json.loads((inp/'manifest.json').read_text())
    if condition.startswith('mimosa'):
        config=yaml.safe_load((ROOT/'mimosa_dragon_vio.yaml').read_text())
        config['logs_directory']=str(dest)+'/'
        reg=config['lidar']['geometric']['scan_to_map']
        reg['project_on_degneneracy']=condition=='mimosa_on';reg['drpm_enabled']=condition=='mimosa_drpm';reg['drpm_snr_factor']=10.0
        # Original common noise/factor settings remain unchanged across three MIMOSA runs.
        (dest/'config.yaml').write_text(yaml.safe_dump(config))
        rospy.set_param('/mimosa',config)
        rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
        rospy.set_param('/mimosa/bag_name',str(OUT/'inputs'/f'{label}.bag'))
        cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa',
            '~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points',
            '~odometry/manager/odometry_in:=/comparison/vio']
    elif condition=='fast_livo2':
        cfg=yaml.safe_load((WS/'src/FAST-LIVO2/config/avia.yaml').read_text())
        cfg['common'].update(img_topic='/comparison/image',lid_topic='/comparison/points',imu_topic='/comparison/imu')
        cfg['time_offset'].update(imu_time_offset=0.0,img_time_offset=0.0,lidar_time_offset=0.0)
        cfg['preprocess'].update(lidar_type=3,scan_line=16,point_filter_num=2,filter_size_surf=0.5,blind=0.8)
        cfg['lio'].update(dept_err=0.03,voxel_size=0.5)
        # No per-run tuning or truth-assisted initialization.
        txt=(OUT/'rovio'/label/'rovio.info').read_text();camera=re.search(r'Camera0\s*\{([^}]+)\}',txt).group(1)
        q=[float(re.search('qCM_'+k+r'\s+([^;]+);',camera).group(1)) for k in 'xyzw']
        tc=np.array([float(re.search('MrMC_'+k+r'\s+([^;]+);',camera).group(1)) for k in 'xyz'])
        R=Rotation.from_quat(q).as_matrix();tl=np.array([-.1925,.001,.049])
        cfg['extrin_calib']=dict(extrinsic_T=tl.tolist(),extrinsic_R=np.eye(3).ravel().tolist(),Rcl=R.ravel().tolist(),Pcl=(R@(tl-tc)).tolist())
        for k,v in cfg.items():rospy.set_param('/'+k,v)
        cam=yaml.safe_load((OUT/'rovio'/label/'cam0.yaml').read_text());K=np.array(cam['camera_matrix']['data']).reshape(3,3)
        calibration=dict(cam_model='Pinhole',cam_width=480,cam_height=360,scale=1.0,cam_fx=float(K[0,0]),cam_fy=float(K[1,1]),cam_cx=float(K[0,2]),cam_cy=float(K[1,2]),cam_d0=0.0,cam_d1=0.0,cam_d2=0.0,cam_d3=0.0)
        rospy.set_param('/laserMapping',calibration)
        (dest/'config.yaml').write_text(yaml.safe_dump(dict(parameters=cfg,camera=calibration)))
        cmd=[str(WS/'devel/lib/fast_livo/fastlivo_offline'),str(OUT/'inputs'/f'{label}.bag'),str(dest/'odometry.csv')]
    else:raise ValueError(condition)
    start=time.monotonic()
    with (dest/'run.log').open('w') as log:
        proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL)
    (dest/'summary.json').write_text(json.dumps(dict(condition=condition,label=label,returncode=proc.returncode,wall_s=time.monotonic()-start,window=manifest['window'],command=cmd),indent=2))
    if proc.returncode:raise RuntimeError(f'{condition} {label}: return code {proc.returncode}')
    print(condition,label,'completed',time.monotonic()-start,flush=True)
if __name__=='__main__':
    import sys
    rospy.init_node('comparison_driver',anonymous=True,disable_signals=True)
    for label in sys.argv[2:]:run(sys.argv[1],label)
