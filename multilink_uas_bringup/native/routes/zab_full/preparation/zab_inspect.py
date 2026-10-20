import sys
sys.path.insert(0,'/root/uas_native')
from plan_cave_route import meshes
from pathlib import Path
import numpy as np,json,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parts,sources=meshes(Path('/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf'),Path('/root/multilink_autonomy_ws/assets/zab/models'),np.array([-100,-500,-50]),np.array([450,100,50]))
fig,ax=plt.subplots(figsize=(14,10))
for name,uri,m in parts:
 print(uri,len(m.faces),m.bounds.tolist(),flush=True)
 v=m.vertices[::max(1,len(m.vertices)//15000)];ax.scatter(v[:,0],v[:,1],s=.1)
 ax.text(*m.centroid[:2],uri[8:],fontsize=7)
mesh=trimesh.util.concatenate([m for _,_,m in parts]);mesh.export('/tmp/zab_collision.ply')
ax.axis('equal');ax.grid();fig.savefig('/root/uas_native/routes/zab_straight/overview.png')
Path('/root/uas_native/routes/zab_straight/mesh_sources.json').write_text(json.dumps(sources,indent=2))
