#!/usr/bin/env python3
"""Factorial DRPM update/noise comparison; unmodified reference kernels."""
import hashlib,json,subprocess,time,sys
from pathlib import Path
import rospy,yaml
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_official'
BASE=ROOT/'logs/zab-offline/livo_comparison';CAL=ROOT/'logs/zab-offline/livo_calibrated';WS=Path('/root/multilink_autonomy_ws/comparison_ws')
MODES={'official_both':(1,0),'official_update':(1,2),'official_noise':(2,0),'neither':(2,2),'official_noise_legacy_graph':(0,0)}
LABELS=['run2a_end','run1_tail_a2_prefix']
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()
def run(mode,label,iterations=10):
    dest=OUT/mode/label;dest.mkdir(parents=True,exist_ok=True)
    if (dest/'summary.json').exists():raise RuntimeError(f'Refusing to overwrite {dest}')
    assert json.loads((BASE/'inputs'/label/'bag_summary.json').read_text())['zero_point_time']
    cfg=yaml.safe_load((CAL/'mimosa_drpm_calibrated'/label/'config.yaml').read_text());cfg['logs_directory']=str(dest)+'/'
    reg=cfg['lidar']['geometric']['scan_to_map'];reg['drpm_registration_mode'],reg['drpm_noise_model']=MODES[mode];reg['drpm_icp_max_iterations']=iterations
    assert reg['drpm_enabled'] and not reg['project_on_degneneracy']
    (dest/'config.yaml').write_text(yaml.safe_dump(cfg));rospy.set_param('/mimosa',cfg)
    rospy.set_param('/mimosa/lidar/sensor',json.loads((WS/'src/mimosa/config/os_dummy.json').read_text()))
    bag=BASE/'inputs'/f'{label}.bag';rospy.set_param('/mimosa/bag_name',str(bag))
    cmd=[str(WS/'devel/lib/mimosa/mimosa_rosbag'),'__name:=mimosa','~imu/manager/imu_in:=/comparison/imu','~lidar/manager/lidar_in:=/comparison/points','~odometry/manager/odometry_in:=/comparison/vio']
    artifacts={str(p):sha(p) for p in [Path(cmd[0]),WS/'devel/lib/libmimosa_lib.so',WS/'src/mimosa/include/mimosa/lidar/official_drpm/degeneracy.h']}
    begin=time.monotonic()
    with (dest/'run.log').open('w') as log:p=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=1200)
    summary=dict(mode=mode,label=label,iterations=iterations,returncode=p.returncode,wall_s=time.monotonic()-begin,command=cmd,artifacts=artifacts,input_sha256=sha(bag),window=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window'])
    (dest/'summary.json').write_text(json.dumps(summary,indent=2));print(mode,label,summary['wall_s'],p.returncode,flush=True)
    if p.returncode:raise RuntimeError(str(dest))
if __name__=='__main__':
    rospy.init_node('drpm_official_driver',anonymous=True,disable_signals=True)
    for mode in sys.argv[1:] or MODES:
        assert mode in MODES
        for label in LABELS:run(mode,label)
