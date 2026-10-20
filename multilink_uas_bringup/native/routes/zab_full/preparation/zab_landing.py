import networkx as nx,trimesh,numpy as np
G=nx.read_gpickle('/tmp/zab_safe.gpickle');m=trimesh.load('/tmp/zab_collision.ply',process=False)
for leaf in [n for n in G if G.degree(n)==1]:
 a,b,k,e=next(iter(G.edges(leaf,keys=True,data=True)));p=np.array(e['xyz'])
 # Global edge ordering determines stored point orientation.
 aa,bb=next((u,v) for u,v,kk in G.edges(keys=True) if kk==k and {u,v}=={a,b})
 reverse=leaf==bb;p=p[::-1] if reverse else p
 arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
 for cut in np.arange(0,min(12,arc[-1]/2),.5):
  i=np.searchsorted(arc,cut);q=p[i]
  xy=np.array([[q[0]+x,q[1]+y,q[2]+10] for x in [-.65,0,.65] for y in [-.65,0,.65]]+[[q[0]-.5382,q[1]-.3225,q[2]+10]])
  loc,ids,_=m.ray.intersects_location(xy,np.tile([0,0,-1],(len(xy),1)))
  if not len(loc):continue
  floors=[]
  for j in range(len(xy)):
   z=loc[(ids==j)&(loc[:,2]<q[2]-.5),2]
   if not len(z):break
   floors.append(max(z))
  if len(floors)==len(xy) and max(floors)-min(floors)<.35 and q[2]-max(floors)<6:
   p=p[i:];e['xyz']=(p[::-1] if reverse else p).tolist();e['weight']=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum());print(leaf,'trim',cut,'pos',q,'floor',min(floors),max(floors),flush=True);break
 else:raise RuntimeError(f'No safe floor for endpoint {leaf}')
nx.write_gpickle(G,'/tmp/zab_safe.gpickle')
