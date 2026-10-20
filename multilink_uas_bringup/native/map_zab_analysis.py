#!/usr/bin/env python3
import json
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_zab_runs import ROOT,OUT,pose,LABELS
from plan_cave_route import meshes
from pathlib import Path
world=ROOT.parents[3]/'assets/zab'
parts,_=meshes(world/'zab_harmonic.sdf',world/'models',np.array([-100,-500,-50]),np.array([450,100,50]))
cloud=[];ids=[];names=[]
for i,(name,uri,m) in enumerate(parts):
 names.append((name,uri));v=m.triangles_center;v=v[::max(1,len(v)//30000)];cloud.append(v);ids.extend([i]*len(v))
cloud=np.vstack(cloud);ids=np.array(ids);tree=cKDTree(cloud)
profiles=json.loads((OUT/'profiles.json').read_text());mapping={}
fig,ax=plt.subplots(figsize=(13,12));ax.scatter(cloud[:,0],cloud[:,1],s=.08,color='#b6b6b6',rasterized=True)
for n,label in enumerate(LABELS):
 t,p,q=pose(label);dist,idx=tree.query(p[::100]);ni,ct=np.unique(ids[idx],return_counts=True);mapping[label]={'nearest_models':[(names[i][1],round(c/ct.sum(),3)) for i,c in sorted(zip(ni,ct),key=lambda x:-x[1])], 'samples':{}}
 ax.plot(p[:,0],p[:,1],lw=1.1,label=label);ax.text(*p[0,:2],str(n+1),fontsize=9)
 f,a=plt.subplots(figsize=(9,6));lo=p.min(0)-20;hi=p.max(0)+20;visible=np.all((cloud[:,:2]>=lo[:2])&(cloud[:,:2]<=hi[:2]),axis=1);a.scatter(cloud[visible,0],cloud[visible,1],s=.3,color='#aaa',rasterized=True);a.plot(p[:,0],p[:,1],color='#0072B2',lw=1.5);a.scatter(*p[0,:2],marker='o',color='green');a.scatter(*p[-1,:2],marker='s',color='black')
 for stage,st in profiles[label]['sample_times'].items():
  pos=np.array([np.interp(st,t,p[:,j]) for j in range(3)]);_,idx=tree.query(pos);mapping[label]['samples'][stage]={'model':names[ids[idx]][1],'xyz':pos.tolist()};a.scatter(*pos[:2],marker='x',color='#D55E00');a.annotate(stage,(pos[0],pos[1]),fontsize=8,xytext=(5,5),textcoords='offset points')
 a.set_title(label+' — actual flight over collision geometry');a.set_xlabel('World X [m]');a.set_ylabel('World Y [m]');a.axis('equal');a.grid(alpha=.2);f.tight_layout();f.savefig(OUT/(label+'_map.png'),dpi=140);plt.close(f)
ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlabel('World X [m]');ax.set_ylabel('World Y [m]');fig.tight_layout();fig.savefig(OUT/'all_routes.png',dpi=160);plt.close(fig)
(OUT/'environment_map.json').write_text(json.dumps(mapping,indent=2)+'\n');print(json.dumps(mapping,indent=2))
