import sys
sys.path.insert(0,'/root/uas_native')
from plan_cave_route import astar
import numpy as np,trimesh,networkx as nx
from scipy import ndimage
from scipy.spatial import cKDTree
G=nx.read_gpickle('/tmp/zab_repaired.gpickle');mesh=trimesh.load('/tmp/zab_collision.ply',process=False);tri=mesh.triangles;mn=tri.min(1);mx=tri.max(1)
for index,(a,b,k,e) in enumerate(G.edges(keys=True,data=True)):
 if e['minimum_distance']>=1.25:continue
 p=np.array(e['xyz']);lo=np.floor(p.min(0)-[3,3,4]);hi=np.ceil(p.max(0)+[3,3,4]);h=.2
 ids=np.where(np.all(mx>=lo-2,1)&np.all(mn<=hi+2,1))[0];m=mesh.submesh([ids],append=True)
 v=m.voxelized(h,max_iter=None).points;shape=tuple((np.ceil((hi-lo)/h).astype(int)+1).tolist());occ=np.zeros(shape,bool);idx=np.rint((v-lo)/h).astype(int);idx=idx[np.all((idx>=0)&(idx<shape),1)];occ[tuple(idx.T)]=True
 d=ndimage.distance_transform_edt(~occ,sampling=[h,h,2*h]).astype('float32')
 # Remain near the intended road, not an exterior shortcut.
 xy=np.stack(np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij'),-1).reshape(-1,2)*h+lo[:2]
 tube=cKDTree(p[:,:2]).query(xy)[0].reshape(shape[:2])<2.5
 solved=False
 for threshold in [1.3,1.2,1.1]:
  free=(d>=threshold)&tube[:,:,None];labels,n=ndimage.label(free)
  start=np.rint((p[0]-lo)/h).astype(int);end=np.rint((p[-1]-lo)/h).astype(int)
  lab=labels[tuple(start)]
  if not lab or labels[tuple(end)]!=lab:
   print(index,'disconnected',threshold,'endpoint clearance',d[tuple(start)],d[tuple(end)],flush=True);continue
  cells=astar(free,d,start,end);path=lo+h*cells
  # Greedy line-of-sight simplification, tested at .05 m against voxel distances.
  simple=[path[0]];i=0
  def safe(a,b):
   q=a+np.linspace(0,1,max(2,int(np.linalg.norm(b-a)/.05)+1))[:,None]*(b-a)
   return np.min(ndimage.map_coordinates(d,((q-lo)/h).T,order=1,mode='constant'))>=threshold
  while i<len(path)-1:
   j=min(len(path)-1,i+100)
   while j>i+1 and not safe(path[i],path[j]):j-=1
   simple.append(path[j]);i=j
  simple[0]=p[0];simple[-1]=p[-1];simple=np.array(simple)
  dense=[]
  for x,y in zip(simple[:-1],simple[1:]):dense.extend(x+np.linspace(0,1,max(2,int(np.linalg.norm(y-x)/.05)+2),endpoint=False)[:,None]*(y-x))
  dense.append(simple[-1]);dense=np.array(dense);dist=[]
  scaled=m.copy();scaled.apply_scale([1,1,2])
  for ch in np.array_split(dense,max(1,len(dense)//80)):
   _,ds,_=trimesh.proximity.closest_point(scaled,ch*[1,1,2]);dist.extend(ds)
  clearance=float(min(dist)-.05)
  print(index,a,b,'threshold',threshold,'clearance',clearance,flush=True)
  if clearance>=1.15:
   e['xyz']=dense.tolist();e['weight']=float(np.linalg.norm(np.diff(dense,axis=0),axis=1).sum());e['minimum_distance']=clearance;e['envelope_z_scale']=2.;solved=True;break
 if not solved:print('UNSOLVED',index,a,b,p[0],p[-1],flush=True)
 nx.write_gpickle(G,'/tmp/zab_repaired.gpickle')
