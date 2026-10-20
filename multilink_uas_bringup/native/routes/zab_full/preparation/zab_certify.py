exec(open('/tmp/zab_refine.py').read().split('# junctions shared')[0])
from scipy.interpolate import splprep,splev
G=nx.read_gpickle('/tmp/zab_refined.gpickle')
# Improve narrow samples with a wider local search, then stop at safe ends.
for idx,(a,b,k,e) in enumerate(G.edges(keys=True,data=True)):
 p=np.array(e['xyz']);sub=getsub(p)
 for iteration in range(2):p=refine(sub,p)
 # Keep common junctions fixed to prevent edge disconnects.
 if G.degree(a)>1:p[0]=np.array(e['xyz'])[0]
 if G.degree(b)>1:p[-1]=np.array(e['xyz'])[-1]
 ds=distance(sub,p)
 if G.degree(a)==1:
  while len(p)>3 and ds[0]<1.4:p=p[1:];ds=ds[1:]
 if G.degree(b)==1:
  while len(p)>3 and ds[-1]<1.4:p=p[:-1];ds=ds[:-1]
 # Piecewise linear baseline is always certifiable; smooth only when safe.
 keep=np.r_[True,np.linalg.norm(np.diff(p,axis=0),axis=1)>.05];p=p[keep]
 arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
 t=np.linspace(0,arc[-1],int(arc[-1]/.1)+2)
 best=np.column_stack([np.interp(t,arc,p[:,i]) for i in range(3)])
 # Smooth limited geometric noise; explicitly preserve endpoints.
 for smoothing in [len(p)*.02,len(p)*.005,0]:
  try:
   tck,u=splprep(p.T,u=arc/arc[-1],s=smoothing,k=min(3,len(p)-1));q=np.array(splev(t/arc[-1],tck)).T
   q[0]=p[0];q[-1]=p[-1]
   distances=distance(sub,q)
   if distances.min()>=1.3:best=q;break
  except ValueError:pass
 ds=distance(sub,best)
 e['xyz']=best.tolist();e['weight']=float(np.linalg.norm(np.diff(best,axis=0),axis=1).sum());e['minimum_distance']=float(ds.min()-.05)
 print(idx,a,b,'length',round(e['weight'],1),'cert',round(e['minimum_distance'],3),'at',np.round(best[ds.argmin()],2),flush=True)
 nx.write_gpickle(G,'/tmp/zab_certified.gpickle')
