#!/usr/bin/env python3
"""Certify a 100 m straight path from the ZaB collision meshes."""
from pathlib import Path
import hashlib,json
import numpy as np
import trimesh
from plan_cave_route import meshes
from trajectory_math import segment_value

world=Path('/root/multilink_autonomy_ws/assets/zab/zab_harmonic.sdf')
out=Path(__file__).resolve().parent/'routes/zab_straight';out.mkdir(parents=True,exist_ok=True)
parts,sources=meshes(world,world.parent/'models',np.array([170,-80,-10]),np.array([350,-40,10]))
m=trimesh.util.concatenate([m for _,_,m in parts])
a=np.array([210.,-60.,-.5]);direction=np.array([100.,-2.,0]);direction/=np.linalg.norm(direction);b=a+100*direction
points=a+np.linspace(0,100,2001)[:,None]*direction
_,distance,_=trimesh.proximity.closest_point(m,points)
clearance=float(distance.min()-.025)
assert clearance>1.9,clearance
# Vertical intersections prove the path is below ceiling and above floor.
locations,ids,_=m.ray.intersects_location(points[::20]+[0,0,20],np.tile([0,0,-1],(101,1)))
for i in range(101):
 heights=locations[ids==i,2]
 assert np.any(heights<-.5) and np.any(heights>-.5),i
# Start spawn and landing footprint must lie on continuous, approximately flat floor.
rootxy=a[:2]-[.5382,.3225]
locations,ids,_=m.ray.intersects_location([[*rootxy,10],[*b[:2],10]],[[0,0,-1]]*2)
floors=[float(max(locations[(ids==i)&(locations[:,2]<-.5),2])) for i in range(2)]
spawn=[*rootxy,floors[0]+.5]
yaw=float(np.arctan2(direction[1],direction[0]))
segment=dict(t0=0.,duration=102.,start=a.tolist(),end=b.tolist(),yaw0=yaw,yaw1=yaw,kind='trapezoid',speed_mps=1.,ramp_s=2.)
for t in np.linspace(0,102,1021):
 p,v,_,_=segment_value(segment,t)
 assert np.linalg.norm(v)<=1.+1e-10
assert np.allclose(segment_value(segment,102)[0],b)
plan=dict(world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),frame='world',reference='FC IMU',length_m=100.,waypoints=[a.tolist(),b.tolist()],spawn_xyz=spawn,enclosing_radius_m=1.,conservative_center_surface_distance_m=clearance,validation='Collision triangle distances every 0.05 m minus 0.025 m; vertical floor/ceiling rays every 1 m',floor_at_spawn_and_finish=floors,cruise_speed_mps=1.,ramp_s=2.,duration_s=102.,note='Ascent and descent are not covered by the 1 m sphere clearance; ground contact expected')
(out/'route.json').write_text(json.dumps(plan,indent=2)+'\n');(out/'segments.json').write_text(json.dumps(dict(segments=[segment]),indent=2)+'\n');(out/'mesh_sources.json').write_text(json.dumps(sources,indent=2)+'\n')
print(json.dumps(plan,indent=2))
