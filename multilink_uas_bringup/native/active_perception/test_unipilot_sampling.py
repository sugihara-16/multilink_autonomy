#!/usr/bin/env python3
"""Sensor-rate change must not silently improve noise, bias or camera geometry."""
import json,math,sys,tempfile,unittest,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from unipilot_gimbal import prepare
from unipilot_calibration import prepare as calibration
from unipilot_calibration import fusion_white_noise
SOURCE=Path('/root/unified_autonomy_stack/workspaces/ws_sim/install/rmf_gz/share/rmf_gz/resources/robots/rmf_unipilot/model.sdf')
class Sampling(unittest.TestCase):
 def test_gyro_only_profile_keeps_acceleration_bias_and_other_factors(self):
  import copy
  with tempfile.TemporaryDirectory() as tmp:
   out=Path(tmp);model=prepare(SOURCE,out,imu_sampling='resolved');config,_=calibration(out);before=copy.deepcopy(config)
   report=fusion_white_noise(config,model,'sensor_gyro')
   expected=math.sqrt((.009**2+.00025**2/12)/200)
   self.assertAlmostEqual(config['imu']['preintegration']['gyro_noise_density'],expected,places=14)
   self.assertEqual(report['applied']['acc_noise_density'],report['previous']['acc_noise_density'])
   config['imu']['preintegration']['gyro_noise_density']=before['imu']['preintegration']['gyro_noise_density']
   self.assertEqual(config,before)
 def test_fusion_density_matches_sensor_and_preserves_other_factors(self):
  import copy
  with tempfile.TemporaryDirectory() as tmp:
   results=[]
   for mode in ['legacy','resolved']:
    out=Path(tmp)/mode;model=prepare(SOURCE,out,imu_sampling=mode);config,_=calibration(out);before=copy.deepcopy(config)
    legacy=fusion_white_noise(config,model,'legacy');self.assertEqual(config,before)
    report=fusion_white_noise(config,model,'sensor');results.append(report['applied'])
    for key in ['acc_noise_density','gyro_noise_density']:
     self.assertEqual(config['imu']['preintegration'][key],max(report['axes'][key]))
     config['imu']['preintegration'][key]=before['imu']['preintegration'][key]
    self.assertEqual(config,before)
   for key in results[0]:self.assertAlmostEqual(results[0][key],results[1][key],places=14)
   self.assertAlmostEqual(results[0]['acc_noise_density'],math.sqrt((.021**2+.005**2/12)/200),places=14)
   self.assertAlmostEqual(results[0]['gyro_noise_density'],math.sqrt((.009**2+.00025**2/12)/200),places=14)
 def test_native_noise_and_all_other_geometry(self):
  with tempfile.TemporaryDirectory() as tmp:
   roots=[];cfg=[]
   for mode in ['legacy','resolved']:
    out=Path(tmp)/mode;path=prepare(SOURCE,out,imu_sampling=mode);calibration(out);cfg.append((out/'rovio.info').read_text());roots.append(ET.parse(path).getroot())
   self.assertEqual(cfg[0],cfg[1])
   imus=[r.find(".//sensor[@name='imu']") for r in roots];rates=[float(i.findtext('update_rate')) for i in imus];self.assertEqual(rates,[200.,1000.])
   for na,nb in zip(imus[0].findall('.//noise'),imus[1].findall('.//noise')):
    for tag in ['stddev','precision']:
     va=na.find(tag);vb=nb.find(tag)
     if va is not None:
      self.assertAlmostEqual(float(va.text)**2/rates[0],float(vb.text)**2/rates[1],places=14);vb.text=va.text
    self.assertEqual(ET.tostring(na),ET.tostring(nb))
   imus[1].find('update_rate').text=imus[0].findtext('update_rate')
   self.assertEqual(ET.tostring(roots[0]),ET.tostring(roots[1]))
 def test_dragon_reference_is_unchanged(self):
  with tempfile.TemporaryDirectory() as tmp:
   paths=[prepare(SOURCE,Path(tmp)/m,sensor_profile='dragon',imu_sampling=m) for m in ['legacy','resolved']]
   self.assertEqual(paths[0].read_bytes(),paths[1].read_bytes())
 def test_invalid_mode(self):
  with tempfile.TemporaryDirectory() as tmp:
   with self.assertRaises(ValueError):prepare(SOURCE,tmp,imu_sampling='bad')
if __name__=='__main__':unittest.main()
