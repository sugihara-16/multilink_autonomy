#!/usr/bin/env python3
"""Reject invalid rigs instead of silently sharing exposure or duplicating a camera."""
import copy,json,subprocess,tempfile,resource
from pathlib import Path
import rospy,yaml
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_multicamera'
def main():
 resource.setrlimit(resource.RLIMIT_CORE,(0,0))
 rospy.init_node('rig_validation_test',anonymous=True,disable_signals=True)
 cfg=yaml.safe_load((OUT/'link1_link3_link4/run2a_end/trial1/config.yaml').read_text());results={}
 for case in ['shared_exposure','duplicate_topic','invalid_rotation']:
  c=copy.deepcopy(cfg)
  if case=='shared_exposure':c['parameters']['vio']['exposure_estimate_en']=True
  if case=='duplicate_topic':c['parameters']['multicamera']['link4']['topic']=c['parameters']['multicamera']['link1']['topic']
  if case=='invalid_rotation':c['parameters']['multicamera']['link4']['Rcl']=[0.0]*9
  for k,v in c['parameters'].items():rospy.set_param('/'+k,v)
  rospy.set_param('/laserMapping',c['camera'])
  with tempfile.TemporaryDirectory() as tmp:
   r=subprocess.run(['/root/multilink_autonomy_ws/comparison_ws/devel/lib/fast_livo/fastlivo_offline',str(OUT/'inputs/run2a_end.bag'),tmp+'/out.csv'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
  assert r.returncode!=0 and 'what()' in r.stdout,(case,r.returncode,r.stdout[-500:])
  results[case]={'returncode':r.returncode,'diagnostic':r.stdout[r.stdout.rfind('what()'):].strip()}
 for k,v in cfg['parameters'].items():rospy.set_param('/'+k,v)
 (OUT/'invalid_configuration_tests.json').write_text(json.dumps(results,indent=2));print('Invalid configurations rejected:',list(results))
if __name__=='__main__':main()
