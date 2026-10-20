#!/usr/bin/env python3
"""Read-only world/trajectory inspection for run1_tail_b1; writes derived figures."""
import json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from plan_cave_route import meshes
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[3];WORLD=WS/'assets/zab/zab_harmonic.sdf';SOURCE=ROOT/'logs/20261002-233545-95177';OUT=ROOT/'logs/zab-offline/b1_world_review';OUT.mkdir(parents=True,exist_ok=True)
record=json.loads((SOURCE/'world.json').read_text());assert hashlib.sha256(WORLD.read_bytes()).hexdigest()==record['sha256']
a=np.genfromtxt(ROOT/'logs/zab-offline/drpm_three_camera/fusion/run1_tail_b1/truth_tracking.csv',delimiter=',',names=True);t=a['stamp_ns']*1e-9;xyz=np.column_stack([a[k] for k in ['x','y','z']]);elapsed=t-t[0]
parts,sources=meshes(WORLD,WORLD.parent/'models',np.array([95,-85,-12]),np.array([220,-30,20]))
print('meshes',[(uri,len(m.faces)) for _,uri,m in parts],flush=True)
tri=np.concatenate([m.triangles for _,_,m in parts]);v0=tri[:,0];e1=tri[:,1]-v0;e2=tri[:,2]-v0
# Exact ray / triangle intersections, distances along unit rays.
def ray(o,d):
 h=np.cross(d,e2);det=np.einsum('ij,ij->i',e1,h);valid=np.abs(det)>1e-10;inv=np.zeros(len(det));inv[valid]=1/det[valid];s=o-v0;u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1);v=inv*(q@d);dist=inv*np.einsum('ij,ij->i',e2,q);valid &= (u>=0)&(v>=0)&(u+v<=1)&(dist>1e-5)
 return float(dist[valid].min()) if valid.any() else None
lights=[];w=ET.parse(WORLD).getroot().find('world')
for l in w.findall('light'):
 pos=np.array([float(v) for v in l.findtext('pose').split()][:3]);closest=float(np.linalg.norm(xyz-pos,axis=1).min())
 if closest<20:lights.append(dict(name=l.get('name'),type=l.get('type'),xyz=pos.tolist(),range=l.findtext('attenuation/range'),nearest_path_m=closest,cast_shadows=l.findtext('cast_shadows')))
sections=[];cross=np.array([.17,.985,0]);cross/=np.linalg.norm(cross)
for sec in [20,90,140,180]:
 i=np.argmin(abs(elapsed-sec));p=xyz[i];left=ray(p,cross);right=ray(p,-cross);up=ray(p,np.array([0,0,1]));down=ray(p,np.array([0,0,-1]));sections.append(dict(elapsed_s=float(elapsed[i]),xyz=p.tolist(),width_at_flight_height_m=left+right,height_at_flight_xy_m=up+down,ceiling_above_m=up,floor_below_m=down))
# Sample path every 20 seconds, ground-truth body orientation and camera viewing directions.
cams=json.loads((SOURCE/'camera_manifest.json').read_text())['cameras'];samples=[]
for sec in [0,20,40,60,90,120,160,200,220,240,260,272]:
 i=np.argmin(abs(elapsed-sec));R=Rotation.from_quat([a[k][i] for k in ['qx','qy','qz','qw']]);samples.append(dict(elapsed_s=float(elapsed[i]),xyz=xyz[i].tolist(),yaw_deg=float(R.as_euler('xyz',degrees=True)[2]),camera_azimuth_deg={c['link']:float(np.degrees(np.arctan2(*(R.as_matrix()@np.array(c['T_FC_camera'])[:3,0])[[1,0]]))) for c in cams}))
fig,ax=plt.subplots(figsize=(13,6))
for _,uri,m in parts:
 c=m.triangles_center;c=c[::max(1,len(c)//20000)];ax.scatter(c[:,0],c[:,1],s=.15,c='#bbbbbb',rasterized=True)
ax.plot(xyz[:,0],xyz[:,1],color='#0072B2',lw=2)
for s in samples[::2]:
 x,y,z=s['xyz'];ax.scatter(x,y,color='#D55E00',s=18);ax.annotate(f'{s["elapsed_s"]:.0f}s',(x,y),xytext=(0,10),textcoords='offset points',fontsize=9)
for l in lights:
 x,y,z=l['xyz'];ax.scatter(x,y,marker='*',c='#CC9900',s=100)
ax.scatter(*xyz[0,:2],c='green',marker='o',s=70,label='Start');ax.scatter(*xyz[-1,:2],c='black',marker='s',s=50,label='End');ax.set(xlim=(110,205),ylim=(-65,-40),xlabel='World X [m]',ylabel='World Y [m]',title='run1_tail_b1: recorded flight over world collision meshes (stars: nearby point lights)');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'route_world.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(3,3,figsize=(13,10))
for row,(stage,stamp) in enumerate([('early',66.7),('vio_onset',150.15),('lio_onset',233.7)]):
 for col,link in enumerate([1,3,4]):
  p=ROOT/f'logs/zab-offline/environment_analysis/run1_tail_b1_{stage}/cam_link{link}.png';axs[row,col].imshow(plt.imread(p),cmap='gray');axs[row,col].set_title(f'link{link}, tracking +{stamp-t[0]:.1f}s');axs[row,col].axis('off')
fig.suptitle('Recorded camera views (existing rectified 480 x 360 input images)');fig.tight_layout();fig.savefig(OUT/'camera_views.png',dpi=130);plt.close(fig)
result=dict(world=str(WORLD),recorded_world_sha256_matches=True,sources=sources,duration_s=float(elapsed[-1]),length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),xyz_start=xyz[0].tolist(),xyz_end=xyz[-1].tolist(),xyz_min=xyz.min(0).tolist(),xyz_max=xyz.max(0).tolist(),sections=sections,nearby_lights=lights,samples=samples)
(OUT/'world_analysis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='sources'},indent=2))
