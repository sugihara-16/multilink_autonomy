#!/usr/bin/env python3
import hashlib,json,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from dragon_cameras import link_transforms,transform
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/fast_official_raw';WS=Path('/root/multilink_autonomy_ws/fast_livo2_official_ws');REF=WS.parent/'comparison_ws/src/FAST-LIVO2';REV='0d2c0346107b75b59934975adec9a6eeeb913c64'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 files=subprocess.check_output(['git','-C',str(REF),'ls-tree','-r','--name-only',REV],text=True).splitlines();matched={}
 for name in files:
  p=WS/'src/FAST-LIVO2'/name
  assert p.read_bytes()==subprocess.check_output(['git','-C',str(REF),'show',f'{REV}:{name}']),name
  matched[name]=sha(p)
 vikit={}
 vref=WS.parent/'comparison_ws/src/rpg_vikit';vrev='6c886c8e5d83997806e00294826d528cea3581dd'
 for name in subprocess.check_output(['git','-C',str(vref),'ls-tree','-r','--name-only',vrev],text=True).splitlines():
  p=WS/'src/rpg_vikit'/name
  assert p.read_bytes()==subprocess.check_output(['git','-C',str(vref),'show',f'{vrev}:{name}']),name
  vikit[name]=sha(p)
 audits={}
 for label in ['run2a_end','run1_tail_b1','run1_tail_a1','run1_prefix']:
  ent=next(e for e in json.loads((ROOT/'logs/zab-full-campaign/dataset_manifest.json').read_text())['valid_bags'] if e['label']==label);src=ROOT/'logs'/ent['run'];r=ET.parse(src/'dragon_slam.urdf').getroot();angles={j.get('name'):float(p.text) for j in r.findall('./ros2_control/joint') if (p:=j.find("state_interface[@name='position']/param[@name='initial_value']")) is not None};f=link_transforms(r,angles)
  T=np.linalg.inv(f['fc'])@f['cam_link3_optical'];g=next(g for g in r.findall('gazebo') if g.find("sensor[@name='base_lidar']") is not None);sensor=g.find("sensor[@name='base_lidar']");pose=list(map(float,sensor.find('pose').text.split()));L=np.linalg.inv(f['fc'])@f[g.get('reference')]@transform(pose[:3],pose[3:]);C=np.linalg.inv(T)@L
  manifest=json.loads((src/'camera_manifest.json').read_text());cam=manifest['cameras'][1];assert np.allclose(T[:3,3],cam['MrMC']);assert np.allclose(T[:3,:3].T,Rotation.from_quat(cam['qCM']).as_matrix())
  raw=yaml.safe_load((src/'raw_cam1.yaml').read_text());camera=r.find("gazebo/sensor[@name='cam_link3']/camera");fx=float(camera.find('image/width').text)/(2*np.tan(float(camera.find('horizontal_fov').text)/2));assert abs(fx-raw['camera_matrix']['data'][0])<1e-8
  audits[label]=dict(urdf_sha256=sha(src/'dragon_slam.urdf'),raw_calibration_sha256=sha(src/'raw_cam1.yaml'),imu_frame='fc',lidar_to_imu=L.tolist(),lidar_to_optical=C.tolist(),lidar_lines=int(sensor.find('lidar/scan/vertical/samples').text),camera_fx_from_simulation=fx,camera_manifest_consistent=True)
  assert audits[label]['lidar_lines']==32
 OUT.mkdir(exist_ok=True);(OUT/'source_audit.json').write_text(json.dumps(dict(revision=REV,abi_build_configuration_sha256=sha(ROOT/'comparison/official_pcl_abi.cmake'),abi_definitions=['EIGEN_MAX_ALIGN_BYTES=16','EIGEN_MALLOC_ALREADY_ALIGNED=1'],all_tracked_files_identical=True,files=matched,vikit_revision=vrev,vikit_files=vikit,vikit_unmodified=True,binary_sha256=sha(WS/'devel/lib/fast_livo/fastlivo_mapping'),libraries={p.name:sha(p) for p in (WS/'devel/lib').glob('*.so')},geometry=audits),indent=2));print('Exact upstream files:',len(matched),'Geometry verified:',list(audits))
if __name__=='__main__':main()
