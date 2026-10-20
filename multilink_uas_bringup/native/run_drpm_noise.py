#!/usr/bin/env python3
"""Same two saved inputs and held-out ROVIO weights; change only DRPM noise."""
import hashlib,json,subprocess,time,sys
from pathlib import Path
import rospy,yaml
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_noise'
BASE=ROOT/'logs/zab-offline/livo_comparison';CAL=ROOT/'logs/zab-offline/livo_calibrated'
WS=Path('/root/multilink_autonomy_ws/comparison_ws')
MODES={'legacy_isotropic':0,'propagated_isotropic':1,'radial':2}
LABELS=['run2a_end','run1_tail_a2_prefix']
def run(mode,label):
    dest=OUT/mode/label;dest.mkdir(parents=True,exist_ok=True)
    if (dest/'summary.json').exists():raise RuntimeError(f'Refusing to overwrite {dest}')
    assert json.loads((BASE/'inputs'/label/'bag_summary.json').read_text())['zero_point_time'], 'Radial implementation requires instantaneous scans'
    original=CAL/'mimosa_drpm_calibrated'/label/'config.yaml'
    cfg=yaml.safe_load(original.read_text());cfg['logs_directory']=str(dest)+'/'
    cfg['lidar']['geometric']['scan_to_map']['drpm_noise_model']=MODES[mode]
    (dest/'config.yaml').write_text(yaml.safe_dump(cfg))
    rospy.set_param('/mimosa',cfg)
    rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
    bag=BASE/'inputs'/f'{label}.bag';rospy.set_param('/mimosa/bag_name',str(bag))
    cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa','~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points','~odometry/manager/odometry_in:=/comparison/vio']
    begin=time.monotonic()
    with (dest/'run.log').open('w') as log:
        p=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=900)
    summary=dict(mode=mode,label=label,returncode=p.returncode,wall_s=time.monotonic()-begin,command=cmd,
      binary_sha256=hashlib.sha256(Path(cmd[0]).read_bytes()).hexdigest(),
      library_sha256=hashlib.sha256((WS/'devel/lib/libmimosa_lib.so').read_bytes()).hexdigest(),
      input_bag_sha256=hashlib.sha256(bag.read_bytes()).hexdigest(),
      original_config_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
      window=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'])
    (dest/'summary.json').write_text(json.dumps(summary,indent=2));print(mode,label,summary['wall_s'],p.returncode,flush=True)
    if p.returncode:raise RuntimeError(str(dest))
if __name__=='__main__':
    rospy.init_node('drpm_noise_driver',anonymous=True,disable_signals=True)
    for mode in sys.argv[1:] or MODES:
        assert mode in MODES
        for label in LABELS:run(mode,label)
