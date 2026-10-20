import numpy as np,networkx as nx
from scipy import ndimage
from skimage.morphology import skeletonize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
v=np.load('/tmp/zab_voxels.npz');d=v['distance'];lo=v['lower'];h=float(v['pitch'])
mask=d.max(2)>=.5
labels,num=ndimage.label(mask);counts=np.bincount(labels.ravel());counts[0]=0;mask=labels==counts.argmax()
s=skeletonize(mask);coords=set(map(tuple,np.argwhere(s)))
g=nx.Graph()
for x,y in coords:
 for dx,dy in [(0,1),(1,0),(1,1),(1,-1)]:
  q=(x+dx,y+dy)
  if q not in coords:continue
  if dx and dy and ((x+dx,y) in coords or (x,y+dy) in coords):continue
  g.add_edge((x,y),q,weight=h*np.hypot(dx,dy))
g=g.subgraph(max(nx.connected_components(g),key=len)).copy()
changed=True
while changed:
 changed=False
 for leaf in [n for n in g if g.degree(n)==1]:
  if leaf not in g:continue
  path=[leaf];length=0.;prev=None;u=leaf
  while True:
   ns=[n for n in g[u] if n!=prev]
   if not ns:break
   nxt=ns[0];length+=g[u][nxt]['weight'];path.append(nxt);prev,u=u,nxt
   if g.degree(u)!=2:break
  if length<8 and g.degree(u)>2:
   g.remove_nodes_from(path[:-1]);changed=True
# Compress each degree-2 chain.
G=nx.MultiGraph();anchors={n for n in g if g.degree(n)!=2};seen=set()
for a in anchors:
 for b in g[a]:
  if frozenset((a,b)) in seen:continue
  path=[a,b];length=g[a][b]['weight'];seen.add(frozenset((a,b)));u,cur=a,b
  while cur not in anchors:
   nxt=next(n for n in g[cur] if n!=u);length+=g[cur][nxt]['weight'];seen.add(frozenset((cur,nxt)));path.append(nxt);u,cur=cur,nxt
  G.add_edge(a,cur,weight=length,path=path)
print('components',sorted(counts[counts>0],reverse=True)[:15],flush=True)
print('Graph',len(G),G.number_of_edges(),'length',sum(x['weight'] for _,_,x in G.edges(data=True)),flush=True)
for n in G:print(n,'world',lo[:2]+h*np.array(n),'deg',G.degree(n),flush=True)
nx.write_gpickle(G,'/tmp/zab_graph.gpickle')
fig,ax=plt.subplots(figsize=(14,12))
ax.imshow(mask.T,origin='lower',extent=[lo[0],lo[0]+h*mask.shape[0],lo[1],lo[1]+h*mask.shape[1]],cmap='Greys',alpha=.3)
for a,b,e in G.edges(data=True):
 p=lo[:2]+h*np.array(e['path']);ax.plot(p[:,0],p[:,1]);mid=p[len(p)//2];ax.text(*mid,f'{e["weight"]:.0f}',fontsize=7)
for i,n in enumerate(G):ax.text(*(lo[:2]+h*np.array(n)),str(i),fontsize=8,color='red')
ax.axis('equal');fig.savefig('/root/uas_native/routes/zab_full/graph.png')
