#!/usr/bin/env python3
"""Same FAST settings and binary; change camera image and calibrated extrinsic only."""
import hashlib,json,subprocess,time
from pathlib import Path
import rospy,yaml,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent;LOG=ROOT/'logs/zab-offline';BASE=LOG/'livo_comparison';OUT=LOG/'fast_camera_comparison';WS=Path('/root/multilink_autonomy_ws/comparison_ws');INPUTS=OUT/'inputs'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(label,link):
 dest=OUT/f'link{link}'/label;dest.mkdir(parents=True,exist_ok=True)
 if (dest/'summary.json').exists():raise RuntimeError('Completed trial exists '+str(dest))
 old=LOG/'livo_calibrated/fast_full'/label;cfg=yaml.safe_load((old/'config.yaml').read_text());manifest=json.loads((BASE/'inputs'/label/'manifest.json').read_text());src=ROOT/'logs'/manifest['source']['run'];cameras=json.loads((src/'camera_manifest.json').read_text())['cameras'];cam=next(c for c in cameras if c['link']==f'link{link}')
 R=Rotation.from_quat(cam['qCM']).as_matrix();tc=np.array(cam['MrMC']);tl=np.array(cfg['parameters']['extrin_calib']['extrinsic_T']);Rli=np.array(cfg['parameters']['extrin_calib']['extrinsic_R']).reshape(3,3)
 Rcl=R@Rli;Pcl=R@(tl-tc)
 if link==3:
  assert np.allclose(Rcl,np.array(cfg['parameters']['extrin_calib']['Rcl']).reshape(3,3),rtol=0,atol=1e-12)
  assert np.allclose(Pcl,cfg['parameters']['extrin_calib']['Pcl'],rtol=0,atol=1e-12)
 else:cfg['parameters']['extrin_calib'].update(Rcl=Rcl.ravel().tolist(),Pcl=Pcl.tolist())
 raw=yaml.safe_load((src/f'cam{cam["id"]}.yaml').read_text());K=np.array(raw['camera_matrix']['data']).reshape(3,3)
 assert np.allclose([K[0,0],K[1,1],K[0,2],K[1,2]],[cfg['camera'][k] for k in ['cam_fx','cam_fy','cam_cx','cam_cy']],rtol=0,atol=1e-12)
 assert cfg['parameters']['common']['img_en']==1 and not cfg['disable_visual_update']
 (dest/'config.yaml').write_text(yaml.safe_dump(cfg));(dest/'camera.json').write_text(json.dumps(cam,indent=2))
 for k,v in cfg['parameters'].items():rospy.set_param('/'+k,v)
 rospy.set_param('/laserMapping',cfg['camera']);rospy.set_param('/comparison_disable_visual_update',False);rospy.set_param('/comparison_disable_visual_after_s',-1.0)
 bag=BASE/'inputs'/f'{label}.bag' if link==3 else INPUTS/f'{label}_link{link}.bag'
 cmd=[str(WS/'devel/lib/fast_livo/fastlivo_offline'),str(bag),str(dest/'odometry.csv')];begin=time.monotonic()
 with (dest/'run.log').open('w') as log:p=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,timeout=600)
 s=dict(label=label,link=link,returncode=p.returncode,wall_s=time.monotonic()-begin,window=manifest['window'],command=cmd,binary_sha256=sha(Path(cmd[0])),camera_manifest_sha256=sha(src/'camera_manifest.json'))
 (dest/'summary.json').write_text(json.dumps(s,indent=2));print(label,'link',link,'exit',p.returncode,'wall',s['wall_s'],flush=True)
 if p.returncode:raise RuntimeError(str(dest))
if __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser();parser.add_argument('link',type=int,choices=[1,3,4]);parser.add_argument('labels',nargs='+');parser.add_argument('--repeat-dir',choices=['repeat_audit','repeat_b']);args=parser.parse_args()
 if args.repeat_dir:OUT=OUT/args.repeat_dir
 rospy.init_node('fast_camera_driver',anonymous=True,disable_signals=True)
 for label in args.labels:run(label,args.link)
