#!/usr/bin/env python3
"""Run synchronized FAST-LIVO2 cameras with common LiDAR/IMU; ROS1 environment."""
import argparse,copy,hashlib,json,subprocess,time
from pathlib import Path
import rospy,yaml
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';OUT=LOG/'fast_multicamera';WS=Path('/root/multilink_autonomy_ws/comparison_ws')
def main():
 p=argparse.ArgumentParser();p.add_argument('--label',choices=['run2a_end','run1_tail_a2_prefix'],required=True);p.add_argument('--cameras',nargs='+',choices=['link1','link3','link4'],default=['link1','link3','link4']);p.add_argument('--trial',default='trial1');p.add_argument('--legacy',action='store_true');p.add_argument('--legacy-exposure',action='store_true');p.add_argument('--config-only',action='store_true');a=p.parse_args()
 assert len(a.cameras)==len(set(a.cameras));assert not a.legacy or a.cameras==['link3'];assert not a.legacy_exposure or a.legacy
 name='legacy_exposure' if a.legacy_exposure else 'legacy_fixed' if a.legacy else '_'.join(a.cameras)
 d=OUT/name/a.label/a.trial;d.mkdir(parents=True,exist_ok=True)
 if (d/'summary.json').exists():raise RuntimeError('Completed trial exists')
 old=LOG/'fast_camera_comparison';cfg=yaml.safe_load((old/'link3'/a.label/'config.yaml').read_text());cfg['parameters']['vio']['exposure_estimate_en']=a.legacy_exposure
 rig={'cameras':a.cameras,'max_pending_frames':100}
 for camera in a.cameras:
  c=yaml.safe_load((old/camera/a.label/'config.yaml').read_text());ext=c['parameters']['extrin_calib'];rig[camera]=dict(topic=f'/comparison/image/{camera}',camera=c['camera'],Rcl=ext['Rcl'],Pcl=ext['Pcl'])
 if not a.legacy:cfg['parameters']['multicamera']=rig
 (d/'config.yaml').write_text(yaml.safe_dump(cfg));(d/'ros_parameters.yaml').write_text(yaml.safe_dump(dict(cfg['parameters'],laserMapping=cfg['camera'])))
 if a.config_only:print(d/'ros_parameters.yaml');return
 rospy.init_node('multicamera_trial',anonymous=True,disable_signals=True)
 if rospy.has_param('/multicamera'):rospy.delete_param('/multicamera')
 for k,v in cfg['parameters'].items():rospy.set_param('/'+k,v)
 rospy.set_param('/laserMapping',cfg['camera']);rospy.set_param('/comparison_disable_visual_update',False);rospy.set_param('/comparison_disable_visual_after_s',-1.0)
 bag=LOG/'livo_comparison/inputs'/f'{a.label}.bag' if a.legacy else OUT/'inputs'/f'{a.label}.bag'
 cmd=[str(WS/'devel/lib/fast_livo/fastlivo_offline'),str(bag),str(d/'odometry.csv')]
 start=time.monotonic()
 with (d/'run.log').open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=900)
 summary=dict(returncode=r.returncode,wall_s=time.monotonic()-start,cameras=a.cameras,legacy=a.legacy,exposure_estimate=a.legacy_exposure,command=cmd,binary_sha256=hashlib.sha256(Path(cmd[0]).read_bytes()).hexdigest(),window=json.loads((LOG/'livo_comparison/inputs'/a.label/'manifest.json').read_text())['window'])
 (d/'summary.json').write_text(json.dumps(summary,indent=2));print(name,a.label,a.trial,r.returncode,summary['wall_s'],flush=True)
 if r.returncode:raise RuntimeError('Estimator failed: '+str(d))
if __name__=='__main__':main()
