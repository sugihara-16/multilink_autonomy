import numpy as np,networkx as nx,trimesh
from scipy.ndimage import gaussian_filter1d,map_coordinates
from pathlib import Path
v=np.load('/tmp/zab_voxels.npz');d=v['distance'];lo=v['lower'];h=.5
old=nx.read_gpickle('/tmp/zab_graph.gpickle');clusters=nx.Graph();clusters.add_nodes_from(old.nodes)
for a,b,e in old.edges(data=True):
 if e['weight']<6:clusters.add_edge(a,b)
mapping={};positions={}
for i,c in enumerate(nx.connected_components(clusters)):
 arr=np.array(list(c));score=np.array([d[x,y].max() for x,y in arr]);best=arr[score.argmax()];pos=np.r_[lo[:2]+h*best,lo[2]+h*d[tuple(best)].argmax()];positions[i]=pos
 for n in c:mapping[n]=i
G=nx.MultiGraph()
for a,b,e in old.edges(data=True):
 u,w=mapping[a],mapping[b]
 if u==w and e['weight']<15:continue
 G.add_edge(u,w,**e)
mesh=trimesh.load('/tmp/zab_collision.ply',process=False);tris=mesh.triangles;mins=tris.min(1);maxs=tris.max(1)
def getsub(p):
 bounds=np.array([p.min(0)-6,p.max(0)+6]);ids=np.where(np.all(maxs>=bounds[0],1)&np.all(mins<=bounds[1],1))[0];return mesh.submesh([ids],append=True)
def distance(m,p):
 result=[]
 for chunk in np.array_split(p,max(1,len(p)//100)):
  _,ds,_=trimesh.proximity.closest_point(m,chunk);result.extend(ds)
 return np.array(result)
def refine(m,p):
 original=distance(m,p);result=p.copy()
 for i in np.where(original<1.8)[0]:
  offsets=np.array([[x,y,z] for x in [-.75,-.5,0,.5,.75] for y in [-.75,-.5,0,.5,.75] for z in np.arange(-2.5,2.51,.5)])
  candidates=p[i]+offsets;grid=(candidates-lo)/h
  valid=map_coordinates(d,grid.T,order=1,mode='constant')>.1
  candidates=candidates[valid];offsets=offsets[valid]
  if not len(candidates):continue
  ds=distance(m,candidates);score=ds-.08*np.linalg.norm(offsets,axis=1);best=score.argmax()
  if ds[best]>original[i]:result[i]=candidates[best]
 return result
# junctions shared across incident edges
for i,pos in positions.items():
 sub=getsub(pos[None,:]);positions[i]=refine(sub,pos[None,:])[0]
for idx,(a,b,k,e) in enumerate(G.edges(keys=True,data=True)):
 path=e['path'];p=np.array([[*(lo[:2]+h*np.array(q)),lo[2]+h*d[q].argmax()] for q in path])
 if mapping[path[0]]!=a:p=p[::-1]
 p[0]=positions[a];p[-1]=positions[b]
 p=np.vstack([p[0],gaussian_filter1d(p,sigma=2,axis=0)[2:-2:2],p[-1]])
 sub=getsub(p);p=refine(sub,p);p[0]=positions[a];p[-1]=positions[b]
 ds=distance(sub,p);e['xyz']=p.tolist();e['minimum_distance']=float(ds.min())
 print(idx,a,b,round(e['weight'],1),'min',round(ds.min(),3),'at',np.round(p[ds.argmin()],2),flush=True)
 nx.write_gpickle(G,'/tmp/zab_refined.gpickle')
