#!/usr/bin/env python3
"""Adopted radial DRPM with external ROVIO completely disabled, fixed prior conditions."""
import json,subprocess,time
from pathlib import Path
import rospy,yaml
from run_drpm_official import sha,ROOT,BASE,WS,LABELS
OUT=ROOT/'logs/zab-offline/sensor_ablation'
def run(label):
    source=ROOT/'logs/zab-offline/drpm_official/official_update'/label
    dest=OUT/'drpm_lio'/label;dest.mkdir(parents=True,exist_ok=True)
    if (dest/'summary.json').exists():raise RuntimeError(f'Refusing to overwrite {dest}')
    cfg=yaml.safe_load((source/'config.yaml').read_text());cfg['logs_directory']=str(dest)+'/'
    cfg['odometry']['manager']['enabled']=False
    assert cfg['lidar']['geometric']['scan_to_map']['drpm_registration_mode']==1
    assert cfg['lidar']['geometric']['scan_to_map']['drpm_noise_model']==2
    (dest/'config.yaml').write_text(yaml.safe_dump(cfg));rospy.set_param('/mimosa',cfg)
    rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
    bag=BASE/'inputs'/f'{label}.bag';rospy.set_param('/mimosa/bag_name',str(bag))
    cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa','~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points','~odometry/manager/odometry_in:=/comparison/disabled_vio']
    begin=time.monotonic()
    with (dest/'run.log').open('w') as log:p=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=1200)
    summary=dict(method='drpm_lio',label=label,returncode=p.returncode,wall_s=time.monotonic()-begin,command=cmd,input_sha256=sha(bag),
      artifacts={str(p):sha(p) for p in [Path(cmd[0]),WS/'devel/lib/libmimosa_lib.so']},source_config=str(source/'config.yaml'),window=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'])
    (dest/'summary.json').write_text(json.dumps(summary,indent=2));print(label,p.returncode,summary['wall_s'],flush=True)
    if p.returncode:raise RuntimeError(str(dest))
if __name__=='__main__':
    rospy.init_node('drpm_lio_driver',anonymous=True,disable_signals=True)
    for label in LABELS:run(label)
