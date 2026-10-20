exec(open('/tmp/zab_repair.py').read().split('for index,')[0])
# Treat blocked joins as two safe dead ends; record disconnected components separately.
nextnode=max(G.nodes)+1
for a,b,k,e in list(G.edges(keys=True,data=True)):
 if e['minimum_distance']>=1.15:continue
 p=np.array(e['xyz']);lo=np.floor(p.min(0)-[3,3,4]);hi=np.ceil(p.max(0)+[3,3,4]);h=.2
 ids=np.where(np.all(mx>=lo-2,1)&np.all(mn<=hi+2,1))[0];m=mesh.submesh([ids],append=True)
 v=m.voxelized(h,max_iter=None).points;shape=tuple((np.ceil((hi-lo)/h).astype(int)+1).tolist());occ=np.zeros(shape,bool);idx=np.rint((v-lo)/h).astype(int);idx=idx[np.all((idx>=0)&(idx<shape),1)];occ[tuple(idx.T)]=True
 d=ndimage.distance_transform_edt(~occ,sampling=[h,h,2*h]).astype('float32')
 xy=np.stack(np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij'),-1).reshape(-1,2)*h+lo[:2]
 nearest=cKDTree(p[:,:2]).query(xy);tube=nearest[0].reshape(shape[:2])<2.5
 free=(d>=1.3)&tube[:,:,None];labels,n=ndimage.label(free);start=np.rint((p[0]-lo)/h).astype(int);end=np.rint((p[-1]-lo)/h).astype(int)
 G.remove_edge(a,b,k)
 requests=[(a,start,end,False),(b,end,start,True)]
 used_labels={labels[tuple(start)],labels[tuple(end)]}
 # Add isolated interiors of blocked connecting roads as separate recording flights.
 for labid in range(1,n+1):
  if labid in used_labels:continue
  possible=np.argwhere(labels==labid)
  if len(possible)<100:continue
  coordinates=lo+h*possible;dist,j=cKDTree(p).query(coordinates)
  select=(dist<1.5)&(j>0)&(j<len(p)-1)
  if not select.any():continue
  possible=possible[select];j=j[select];dist=dist[select]
  if (j.max()-j.min())*.1<3:continue
  start0=possible[np.argmin(j+.1*dist)]
  end0=possible[np.argmax(j-.1*dist)]
  requests.append((nextnode,start0,end0,False));nextnode+=1
 for node,initial,goal,reverse in requests:
  lab=labels[tuple(initial)]
  if not lab:raise RuntimeError('bad initial')
  possible=np.argwhere(labels==lab)
  # Stay within the longitudinal bounds of the original edge.
  coordinates=lo+h*possible;dist,j=cKDTree(p).query(coordinates)
  score=(len(p)-1-j if reverse else j)*.1 + .05*d[tuple(possible.T)]-.1*dist
  valid=(dist<2)&(j>0)&(j<len(p)-1)
  possible=possible[valid];score=score[valid]
  target=possible[score.argmax()]
  cells=astar(free,d,initial,target);path=lo+h*cells
  if np.linalg.norm(path[-1]-path[0])<2:continue
  simple=[path[0]];i=0
  while i<len(path)-1:
   j=min(len(path)-1,i+100)
   while j>i+1:
    q=path[i]+np.linspace(0,1,max(2,int(np.linalg.norm(path[j]-path[i])/.04)+1))[:,None]*(path[j]-path[i])
    if ndimage.map_coordinates(d,((q-lo)/h).T,order=1,mode='constant').min()>=1.3:break
    j-=1
   simple.append(path[j]);i=j
  if node in (a,b):simple[0]=p[-1] if reverse else p[0]
  simple=np.array(simple);dense=[]
  for x,y in zip(simple[:-1],simple[1:]):dense.extend(x+np.linspace(0,1,max(2,int(np.linalg.norm(y-x)/.05)+2),endpoint=False)[:,None]*(y-x))
  dense.append(simple[-1]);dense=np.array(dense);scaled=m.copy();scaled.apply_scale([1,1,2]);dist=[]
  for ch in np.array_split(dense,max(1,len(dense)//80)):
   _,ds,_=trimesh.proximity.closest_point(scaled,ch*[1,1,2]);dist.extend(ds)
  clearance=float(min(dist)-.05)
  print('partial',a,b,'from',node,'to',nextnode,'length',np.linalg.norm(np.diff(dense,axis=0),axis=1).sum(),'clearance',clearance,flush=True)
  assert clearance>=1.2
  G.add_edge(node,nextnode,xyz=dense.tolist(),weight=float(np.linalg.norm(np.diff(dense,axis=0),axis=1).sum()),minimum_distance=clearance,envelope_z_scale=2.,blocked_join=[a,b]);nextnode+=1
 nx.write_gpickle(G,'/tmp/zab_safe.gpickle')
