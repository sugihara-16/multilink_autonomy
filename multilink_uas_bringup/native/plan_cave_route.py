#!/usr/bin/env python3
"""Offline, mesh-derived cave route. Does not command a robot or feed estimators."""
import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation
import trimesh


def pose(node):
    p=[float(v) for v in (node.findtext('pose') or '0 0 0 0 0 0').split()]
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',p[3:]).as_matrix();t[:3,3]=p[:3]
    return t


def meshes(world, models, lower, upper):
    cache={};result=[];sources={str(world):hashlib.sha256(world.read_bytes()).hexdigest()}
    def geometry(g):
        mesh=g.find('mesh')
        if mesh is not None:
            uri=mesh.findtext('uri');path=models/uri.removeprefix('model://')
            if path not in cache:
                loaded=trimesh.load(str(path),force='mesh',process=False)
                m=trimesh.Trimesh(vertices=loaded.vertices.copy(),faces=loaded.faces.copy(),process=False)
                del loaded
                if path.suffix.lower()=='.dae':
                    asset=ET.parse(path).getroot();ns={'c':asset.tag.split('}')[0].strip('{')}
                    unit=asset.find('c:asset/c:unit',ns)
                    m.apply_scale(float(unit.get('meter','1')) if unit is not None else 1.)
                    up=asset.findtext('c:asset/c:up_axis',default='Z_UP',namespaces=ns)
                    if up!='Z_UP':raise ValueError(f'Check COLLADA up axis before using: {path}: {up}')
                cache[path]=m
                sources[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
            m=cache[path].copy()
            m.apply_scale([float(x) for x in mesh.findtext('scale','1 1 1').split()])
            return m
        if g.find('box') is not None:return trimesh.creation.box([float(x) for x in g.findtext('box/size').split()])
        if g.find('sphere') is not None:return trimesh.creation.icosphere(subdivisions=2,radius=float(g.findtext('sphere/radius')))
        if g.find('cylinder') is not None:return trimesh.creation.cylinder(radius=float(g.findtext('cylinder/radius')),height=float(g.findtext('cylinder/length')),sections=32)
        if g.find('plane') is not None:return None # z=0 infinite ground separately enforced
        raise ValueError('Unhandled geometry '+ET.tostring(g,encoding='unicode'))
    w=ET.parse(world).getroot().find('world')
    entries=[]
    for inc in w.findall('include'):
        center=pose(inc)[:3,3]
        if np.any(center<lower-50) or np.any(center>upper+50):continue
        model_path=models/inc.findtext('uri').removeprefix('model://')/'model.sdf'
        sources[str(model_path)]=hashlib.sha256(model_path.read_bytes()).hexdigest()
        entries.append((inc.findtext('name'),inc.findtext('uri'),pose(inc),ET.parse(model_path).getroot().find('model')))
    entries += [(m.get('name'),'inline',np.eye(4),m) for m in w.findall('model')]
    for name,kind,outer,model in entries:
        tmodel=outer@pose(model)
        for link in model.findall('link'):
            for item in link.findall('collision'):
                m=geometry(item.find('geometry'))
                if m is None:continue
                m.apply_transform(tmodel@pose(link)@pose(item))
                if np.any(m.bounds[1]<lower) or np.any(m.bounds[0]>upper):continue
                result.append((name,kind,m))
    return result,sources


def astar(free, distance, start, goal):
    shape=free.shape
    def flat(p):return int(np.ravel_multi_index(tuple(p),shape))
    s,g=flat(start),flat(goal);cost={s:0.};parent={};queue=[(float(np.linalg.norm(start-goal)),0.,s)]
    closed=set();strides=(shape[1]*shape[2],shape[2],1)
    while queue:
        _,value,u=heapq.heappop(queue)
        if u in closed:continue
        if u==g:
            path=[u]
            while path[-1]!=s:path.append(parent[path[-1]])
            return np.array(np.unravel_index(path[::-1],shape)).T
        closed.add(u);p=np.array(np.unravel_index(u,shape))
        for axis in range(3):
            for sign in (-1,1):
                q=p.copy();q[axis]+=sign
                if q[axis]<0 or q[axis]>=shape[axis] or not free[tuple(q)]:continue
                v=u+sign*strides[axis]
                # Bias away from walls without claiming the shortest Euclidean path.
                nc=value+1.+.25/max(float(distance[tuple(q)]),.1)
                if nc<cost.get(v,float('inf')):
                    cost[v]=nc;parent[v]=u
                    heapq.heappush(queue,(nc+float(np.linalg.norm(q-goal)),nc,v))
    raise RuntimeError(f'No safe route between {start} and {goal}')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world',type=Path,default=Path('/root/unified_autonomy_stack/workspaces/ws_sim/src/subt_cave_sim/worlds/darpa_cave_01.sdf'))
    parser.add_argument('--pitch',type=float,default=.5)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    if args.pitch <= 0: parser.error('--pitch must be positive')
    h=args.pitch;lower=np.array([15.,-245.,-5.]);upper=np.array([205.,65.,50.])
    shape=tuple((np.ceil((upper-lower)/h).astype(int)+1).tolist())
    radius=1.0;threshold=1.95 # Candidate search only; exact mesh distances certify the final path.
    cache=out/'voxel_cache.npz'
    if cache.exists():
        data=np.load(cache);occupied=data['occupied'];distance=data['distance'];inside=data['inside']
        metadata=json.loads((out/'mesh_sources.json').read_text())
        assert metadata['pitch']==h and metadata['shape']==list(shape)
        assert metadata['lower']==lower.tolist() and str(args.world) in metadata['sha256']
        for name,sha in metadata['sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha,name
        print('Using checked voxel cache',flush=True)
    else:
        world_meshes,sources=meshes(args.world,args.world.parent.parent/'models',lower,upper)
        occupied=np.zeros(shape,bool);parts=[]
        for name,kind,m in world_meshes:
            points=m.voxelized(h, max_iter=None).points
            idx=np.rint((points-lower)/h).astype(int);idx=idx[np.all((idx>=0)&(idx<shape),axis=1)]
            occupied[tuple(idx.T)]=True
            parts.append(dict(name=name,kind=kind,bounds=m.bounds.tolist(),triangles=len(m.faces)))
            print(name,len(idx),'surface voxels',flush=True)
        # Restrict to columns with a floor and ceiling; outside empty space is not a shortcut.
        lo=occupied.argmax(axis=2);hi=shape[2]-1-occupied[:,:,::-1].argmax(axis=2)
        z=np.arange(shape[2])[None,None,:]
        inside=occupied.any(axis=2)[:,:,None]&(z>lo[:,:,None])&(z<hi[:,:,None])
        occupied[:,:,lower[2]+np.arange(shape[2])*h<=0]=True
        distance=ndimage.distance_transform_edt(~occupied,sampling=h).astype(np.float32)
        metadata=dict(pitch=h,shape=list(shape),lower=lower.tolist(),parts=parts,sha256=sources)
        (out/'mesh_sources.json').write_text(json.dumps(metadata,indent=2))
        np.savez_compressed(cache,occupied=occupied,distance=distance,inside=inside)
    free=inside & (distance>=threshold)
    # Junctions, east/north corridor and southern dogleg. Seeds are not trusted clearances.
    seeds=[('start',[40,5,1.5]),('junction',[100,0,3]),('east_corner',[175,0,3]),
           ('north_corridor',[175,40,3]),('junction_return',[100,0,3]),
           ('south_straight',[100,-75,3]),('dogleg',[125,-100,3]),
           ('home',[40,5,1.5])]
    def xyz(idx):return lower+np.asarray(idx)*h
    def cell(point):return np.rint((np.asarray(point)-lower)/h).astype(int)
    labels,n=ndimage.label(free)
    def nearest(point,label=None):
        c=cell(point);a=np.maximum(c-20,0);b=np.minimum(c+21,shape)
        sl=tuple(slice(x,y) for x,y in zip(a,b));candidates=np.argwhere(free[sl] if label is None else labels[sl]==label)+a
        if len(candidates)==0:raise RuntimeError(f'No clear voxel near {point}')
        return candidates[np.argmin(np.linalg.norm(xyz(candidates)-point,axis=1))]
    start=nearest(seeds[0][1]);component=labels[tuple(start)]
    targets=[(name,nearest(p,component)) for name,p in seeds]
    def safe_segment(a,b):
        points=np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/(h/4)))+1))
        indices=np.rint((points-lower)/h).astype(int)
        return bool(np.all(free[tuple(indices.T)]))
    route=[];stages=[]
    for (an,a),(bn,b) in zip(targets,targets[1:]):
        raw=xyz(astar(free,distance,a,b));short=[raw[0]];i=0
        while i<len(raw)-1:
            j=len(raw)-1
            while j>i+1 and not safe_segment(raw[i],raw[j]):j-=1
            short.append(raw[j]);i=j
        if route:short=short[1:]
        route.extend(short);stages.append(dict(name=bn,index=len(route)-1,position=xyz(b).tolist()))
        print(bn,'waypoints',len(route),flush=True)
    route=np.array(route)
    assert all(safe_segment(a,b) for a,b in zip(route,route[1:]))
    dense=np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.1))+1)) for a,b in zip(route,route[1:])])
    # Independent triangle-distance check. Distance to a surface is 1-Lipschitz;
    # subtract half the maximum sample interval to bound the entire polyline.
    world_meshes,sources=meshes(args.world,args.world.parent.parent/'models',lower,upper)
    exact=dense[:,2].copy() # infinite ground plane
    for name,kind,m in world_meshes:
        bound=np.maximum(np.maximum(m.bounds[0]-dense,dense-m.bounds[1]),0)
        selected=np.flatnonzero(np.linalg.norm(bound,axis=1)<exact)
        for chunk in np.array_split(selected,max(1,math.ceil(len(selected)/200))):
            if len(chunk):
                _,d,_=trimesh.proximity.closest_point(m,dense[chunk])
                exact[chunk]=np.minimum(exact[chunk],d)
    conservative=float(exact.min()-.05)
    if conservative < radius+.1:
        raise RuntimeError(f'Exact clearance insufficient: {conservative:.4f} m')
    length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum())
    plan=dict(world=str(args.world),world_sha256=metadata['sha256'][str(args.world)],frame='world',reference='FC IMU',
        start_note='Post-takeoff route; ascent from ground is not covered by this clearance certificate',
        voxel_pitch_m=h,validation='Exact collision-triangle distance at <=0.1m spacing, minus 0.05m Lipschitz margin; includes ground plane',enclosing_radius_m=radius,voxel_clearance_threshold_m=threshold,
        conservative_center_surface_distance_m=conservative,length_m=length,waypoints=route.tolist(),stages=stages,
        caveat='Offline geometry validation; tracking error, dynamic objects and actual flight not validated')
    (out/'route.json').write_text(json.dumps(plan,indent=2)+'\n')
    np.savetxt(out/'waypoints.csv',route,delimiter=',',header='x,y,z',comments='')
    np.savez_compressed(out/'overview.npz',occupied_xy=occupied.any(axis=2),free_xy=free.any(axis=2),lower=lower,pitch=h,route=route)
    print(json.dumps({k:v for k,v in plan.items() if k not in ('waypoints','stages')},indent=2),flush=True)

if __name__=='__main__':main()
