#!/usr/bin/env python3
"""Open route-inspection tours over mesh-certified ZaB corridors."""
import hashlib,itertools,json
from pathlib import Path
import numpy as np,networkx as nx,trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base=Path(__file__).resolve().parent/'routes/zab_full';base.mkdir(exist_ok=True)
world=Path('/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf')
G=nx.read_gpickle('/tmp/zab_safe.gpickle')
for a,b,k,e in G.edges(keys=True,data=True):e['origin']=a;e['end_node']=b
nx.write_gpickle(G,str(base/'corridors.gpickle'))
mesh=trimesh.load('/tmp/zab_collision.ply',process=False)
components=sorted(nx.connected_components(G),key=lambda c:sum(e['weight'] for _,_,e in G.subgraph(c).edges(data=True)),reverse=True)
fig,ax=plt.subplots(figsize=(12,12));runs=[]
for number,component in enumerate(components,1):
 g=G.subgraph(component).copy();odd=[n for n in g if g.degree(n)%2];dist=dict(nx.all_pairs_dijkstra_path_length(g,weight='weight'))
 best=None
 for start,end in itertools.combinations(odd,2) if odd else [(next(iter(g)),next(iter(g)))]:
  remaining=[n for n in odd if n not in (start,end)];complete=nx.Graph();complete.add_nodes_from(remaining)
  for a,b in itertools.combinations(remaining,2):complete.add_edge(a,b,weight=dist[a][b])
  pairs=nx.algorithms.matching.min_weight_matching(complete,maxcardinality=True,weight='weight');cost=sum(dist[a][b] for a,b in pairs)
  if best is None or cost<best[0]:best=(cost,start,end,pairs)
 cost,start,end,pairs=best
 tour=g.copy()
 for a,b in pairs:
  path=nx.shortest_path(g,a,b,weight='weight')
  for u,v in zip(path[:-1],path[1:]):
   e=min(g[u][v].values(),key=lambda e:e['weight']);tour.add_edge(u,v,**e,duplicate=True)
 walk=list(nx.eulerian_path(tour,source=start,keys=True));segments=[];t=0.;yaw=None;route_points=[];coverage=[];length=0.
 for u,v,k in walk:
  e=tour[u][v][k];p=np.array(e['xyz']);p=p if e['origin']==u else p[::-1]
  if route_points:assert np.linalg.norm(p[0]-route_points[-1])<.01,(u,v,p[0],route_points[-1])
  direction=p[-1]-p[0];heading=float(np.arctan2(direction[1],direction[0]))
  if yaw is None:yaw=heading
  delta=float(np.arctan2(np.sin(heading-yaw),np.cos(heading-yaw)))
  if abs(delta)>.01:
   duration=max(1.,1.875*abs(delta)/.25)
   segments.append(dict(kind='rotate',t0=t,duration=duration,start=p[0].tolist(),end=p[0].tolist(),yaw0=yaw,yaw1=yaw+delta));t+=duration;yaw+=delta
  arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))];L=float(arc[-1]);duration=L/.5+2.
  segments.append(dict(kind='polyline',t0=t,duration=duration,start=p[0].tolist(),end=p[-1].tolist(),yaw0=yaw,yaw1=yaw,speed_mps=.5,ramp_s=2.,points=p.tolist(),arc=arc.tolist(),edge=[e['origin'],e['end_node']]))
  t+=duration;length+=L;route_points.extend(p[::max(1,len(p)//100)].tolist());route_points.append(p[-1].tolist());coverage.append([e['origin'],e['end_node']]);ax.plot(p[:,0],p[:,1],lw=1,label=f'run {number}' if len(coverage)==1 else None)
 first=np.array(segments[0]['start']);rootxy=first[:2]-[.5382,.3225]
 locations,ids,_=mesh.ray.intersects_location([[*rootxy,first[2]+10]],[[0,0,-1]])
 print('start',number,start,first,rootxy,locations,flush=True)
 below=locations[locations[:,2]<first[2]-.4,2]
 assert len(below),'No start floor';floor=float(max(below));spawn=[*rootxy,floor+.5]
 directory=base/f'run{number}';directory.mkdir(exist_ok=True)
 plan=dict(world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),frame='world',reference='FC IMU',length_m=length,waypoints=route_points,spawn_xyz=spawn,cruise_speed_mps=.5,duration_s=t,tracking_slow_m=.08,tracking_stop_m=.18,unique_road_length_m=sum(e['weight'] for _,_,e in g.edges(data=True)),duplicate_length_m=cost,edge_visits=coverage,minimum_certified_clearance=min(e['minimum_distance'] for _,_,e in g.edges(data=True)),envelope='horizontal radius 1m, vertical half-height 0.5m for narrow connectors; sphere radius 1m elsewhere',component_nodes=list(component),sensor_only=True)
 (directory/'route.json').write_text(json.dumps(plan,indent=2)+'\n');(directory/'segments.json').write_text(json.dumps(dict(segments=segments),indent=2)+'\n');runs.append(dict(route=str(directory),length_m=length,sim_duration_s=t,unique_length_m=plan['unique_road_length_m'],duplicate_length_m=cost,spawn=spawn))
 print(number,'length',round(length,1),'unique',round(plan['unique_road_length_m'],1),'repeat',round(cost,1),'minutes',round(t/60,1),'spawn',spawn,flush=True)
ax.axis('equal');ax.grid();ax.legend();fig.savefig(base/'coverage.png')
(base/'campaign.json').write_text(json.dumps(dict(runs=runs,total_length_m=sum(r['length_m'] for r in runs),total_sim_duration_s=sum(r['sim_duration_s'] for r in runs),note='Four collision-disconnected accessible components; blocked joining surfaces excluded, interior connectors covered by separate takeoffs'),indent=2)+'\n')
