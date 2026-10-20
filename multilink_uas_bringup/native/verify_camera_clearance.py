"""Offline discrete camera-housing interference check against DRAGON visual meshes."""
import argparse
import sys,json,struct,itertools
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory
from dragon_cameras import link_transforms,transform
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
out=args.output.resolve();r=E.parse(out/'dragon_slam.urdf').getroot()
manifest=json.loads((out/'camera_manifest.json').read_text());a=manifest['expected_joints'];meshes=[]
for link in r.findall('link'):
 for v in link.findall('visual'):
  m=v.find('geometry/mesh')
  if m is None:continue
  path=m.get('filename')
  if path.startswith('package://'):
   pkg,rel=path[10:].split('/',1);path=str(Path(get_package_share_directory(pkg))/rel)
  b=Path(path).read_bytes();n=struct.unpack_from('<I',b,80)[0]
  tris=np.frombuffer(b,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].astype(float)
  scale=np.array(list(map(float,m.get('scale','1 1 1').split())));tris*=scale
  o=v.find('origin');t=transform(list(map(float,o.get('xyz','0 0 0').split())),list(map(float,o.get('rpy','0 0 0').split()))) if o is not None else np.eye(4)
  tris=tris@t[:3,:3].T+t[:3,3];bounds=np.array(list(itertools.product(*zip(tris.min(axis=(0,1)),tris.max(axis=(0,1))))));meshes.append((link.get('name'),tris,bounds))
def intersects(tris):
 half=np.array([.0125,.015,.015]);p=tris-np.array([-.0125,0,0])
 mask=np.all(p.min(1)<=half,axis=1)&np.all(p.max(1)>=-half,axis=1);p=p[mask]
 if not len(p):return False
 edges=np.roll(p,-1,axis=1)-p
 axes=[np.cross(edges[:,0],edges[:,1])]
 axes += [np.cross(edges[:,i],np.eye(3)[j]) for i in range(3) for j in range(3)]
 valid=np.ones(len(p),bool)
 for axis in axes:
  proj=np.einsum('nji,ni->nj',p,axis);radius=np.abs(axis)@half
  valid &= (proj.min(1)<=radius+1e-9)&(proj.max(1)>=-radius-1e-9)
 return bool(valid.any())
checks=[('square',dict(a))]
for j in (1,2,3):
 for pitch,yaw in itertools.product(np.linspace(-np.pi/2,np.pi/2,7),repeat=2):
  b=dict(a);b.update({f'joint{j}_pitch':pitch,f'joint{j}_yaw':yaw});checks.append((f'joint{j} pitch={pitch:.3f} yaw={yaw:.3f}',b))
for j in (1,2,3,4):
 for roll,pitch in itertools.product(np.linspace(-np.pi/2,np.pi/2,7),repeat=2):
  b=dict(a);b.update({f'gimbal{j}_roll':roll,f'gimbal{j}_pitch':pitch});checks.append((f'gimbal{j} roll={roll:.3f} pitch={pitch:.3f}',b))
hits=[]
for label,angles in checks:
 f=link_transforms(r,angles)
 for c in manifest['cameras']:
  camera=c['link']
  inv=np.linalg.inv(f[camera]@transform(c['mount_xyz'],c['mount_rpy']))
  for link,tris,bounds in meshes:
   t=inv@f[link];corners=bounds@t[:3,:3].T+t[:3,3]
   if np.any(corners.min(0)>[0,.015,.015]) or np.any(corners.max(0)<[-.025,-.015,-.015]):continue
   p=tris@t[:3,:3].T+t[:3,3]
   if intersects(p):hits.append(dict(configuration=label,camera=camera,part=link))
report=dict(configurations=len(checks),body_joint_sweep='one pitch/yaw pair at a time, -90..90 deg in 30 deg steps; remaining body joints square',gimbal_sweep='one roll/pitch pair at a time, -90..90 deg in 30 deg steps',method='camera housing box vs visual mesh triangles; discrete poses, not continuous collision or arbitrary simultaneous deformation',hits=hits)
(out/'camera_clearance.json').write_text(json.dumps(report,indent=2));print('poses',len(checks),'hits',len(hits));print(json.dumps(hits[:15],indent=2))

if hits:
 raise SystemExit(1)
