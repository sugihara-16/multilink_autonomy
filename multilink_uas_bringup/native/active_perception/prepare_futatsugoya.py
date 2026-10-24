#!/usr/bin/env python3
"""Import the licensed original OBJ/texture tiles without altering mesh geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import zipfile
import numpy as np
import trimesh
import collada
from scipy.ndimage import gaussian_filter1d

HERE=Path(__file__).resolve().parent
WS=HERE.parents[4]
SOURCE_URL='https://sketchfab.com/3d-models/futatsugoya-tunnel-abandoned-road-japan-4749ba1722774d3eb101199d51bd39b8'

def save(p,d):
    p.write_text(json.dumps(d,indent=2)+'\n')

def parse(source):
    archive=next((source/'source').glob('*.zip'))
    with zipfile.ZipFile(archive) as z:
        names=[n for n in z.namelist() if n.endswith('.obj')]
        assert len(names)==1
        raw=z.read(names[0])
    vertices=[];uv=[];groups={};current=None
    for line in raw.decode().splitlines():
        a=line.split()
        if not a:continue
        if a[0]=='v':vertices.append(list(map(float,a[1:4])))
        elif a[0]=='vt':uv.append(list(map(float,a[1:3])))
        elif a[0]=='usemtl':current=a[1];groups.setdefault(current,[])
        elif a[0]=='f':
            assert len(a)==4,'Only original triangles supported'
            groups[current].append([[int(x)-1 for x in t.split('/')[:2]] for t in a[1:]])
    return np.array(vertices),np.array(uv),{k:np.array(v) for k,v in groups.items()},hashlib.sha256(raw).hexdigest()

def export_dae(path,v,uv,groups,normals):
    doc=collada.Collada();doc.assetInfo.upaxis='Z_UP';doc.assetInfo.unitmeter=1.0;doc.assetInfo.unitname='meter'
    sources=[collada.source.FloatSource('positions',np.array(v,copy=True,order='C'),('X','Y','Z')),
             collada.source.FloatSource('normals',np.array(normals,copy=True,order='C'),('X','Y','Z')),
             collada.source.FloatSource('uv',uv-np.floor(uv),('S','T'))]
    geom=collada.geometry.Geometry(doc,'tunnel','tunnel',sources);bindings=[]
    for i,(name,faces) in enumerate(groups.items()):
        tile=np.array([int(x)-1 for x in re.search(r'_u(\d+)_v(\d+)$',name).groups()])
        coords=uv[faces[:,:,1]]-tile
        assert coords.min()>=0 and coords.max()<=1,'Incorrect texture tile assignment'
        image=collada.material.CImage('image'+str(i),'../textures/'+name+'.jpg')
        surface=collada.material.Surface('surface'+str(i),image)
        sampler=collada.material.Sampler2D('sampler'+str(i),surface)
        texture=collada.material.Map(sampler,'UVSET0')
        effect=collada.material.Effect('effect'+str(i),[surface,sampler],'lambert',diffuse=texture,ambient=(1,1,1,1),double_sided=True)
        mat=collada.material.Material('material'+str(i),name,effect)
        doc.images.append(image);doc.effects.append(effect);doc.materials.append(mat)
        inputs=collada.source.InputList();inputs.addInput(0,'VERTEX','#positions');inputs.addInput(1,'NORMAL','#normals');inputs.addInput(2,'TEXCOORD','#uv',set='0')
        indices=np.stack([faces[:,:,0],faces[:,:,0],faces[:,:,1]],axis=-1)
        symbol='tile'+str(i);geom.primitives.append(geom.createTriangleSet(indices.ravel(),inputs,symbol))
        bindings.append(collada.scene.MaterialNode(symbol,mat,inputs=[('UVSET0','TEXCOORD','0')]))
    doc.geometries.append(geom);node=collada.scene.Node('model',[collada.scene.GeometryNode(geom,bindings)])
    doc.scenes.append(collada.scene.Scene('scene',[node]));doc.scene=doc.scenes[0];doc.write(str(path))

def make_route(root,name,mesh,start_x,end_x):
    out=root/name;out.mkdir(parents=True,exist_ok=True)
    x=np.arange(start_x,end_x+.01,.25)
    # All rays start inside the reconstructed tunnel; no synthetic surfaces.
    origins=np.c_[x,np.zeros(len(x)),np.full(len(x),5.45)]
    def hits(orig,direction):
        p,i,_=mesh.ray.intersects_location(orig,np.tile(direction,(len(orig),1)),multiple_hits=False)
        assert len(i)==len(orig),'Missing route surface'
        result=np.empty_like(orig);result[i]=p;return result
    floors=hits(origins,[0,0,-1]);ceilings=hits(origins,[0,0,1])
    height=ceilings[:,2]-floors[:,2];assert height.min()>3.2
    centers=floors.copy();centers[:,2]+=1.8
    left=hits(centers,[0,1,0]);right=hits(centers,[0,-1,0]);centers[:,1]=(left[:,1]+right[:,1])/2
    centers[:,1:]=gaussian_filter1d(centers[:,1:],sigma=5,axis=0,mode='nearest')
    _,distance,_=trimesh.proximity.closest_point(mesh,centers)
    assert distance.min()>1.1,'Insufficient center clearance'
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(centers,axis=0),axis=1))];length=arc[-1];duration=length/.5+2
    world=root.parent/'futatsugoya.sdf'
    route=dict(world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),frame='world',reference='FC IMU',length_m=float(length),waypoints=centers.tolist())
    segment=dict(kind='polyline',t0=0.,duration=float(duration),start=centers[0].tolist(),end=centers[-1].tolist(),yaw0=0.,yaw1=0.,speed_mps=.5,ramp_s=2.,points=centers.tolist(),arc=arc.tolist())
    spawn=centers[0].copy();spawn[2]=floors[0,2]+.35
    save(out/'route.json',route);save(out/'segments.json',dict(segments=[segment]))
    entry=dict(route=str(out),world=str(world),spawn=spawn.tolist(),route_end_s=float(duration),length_m=float(length),minimum_center_clearance_m=float(distance.min()),ceiling_height_range_m=[float(height.min()),float(height.max())],validation='Mesh clearance at 0.25 m spacing; center margin includes vehicle, not a full swept-body collision proof.')
    save(out/'world.json',dict(world_file=str(world),world_name='zab',model_name='rmf_unipilot',spawn_xyz=spawn.tolist()))
    save(out/'validation.json',entry)
    return entry

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,default=WS/'assets/futatsugoya');args=p.parse_args()
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    model=root/'models/futatsugoya';(model/'meshes').mkdir(parents=True,exist_ok=True);(model/'textures').mkdir(exist_ok=True)
    v,uv,groups,source_sha=parse(args.source);raw_bounds=[v.min(0).tolist(),v.max(0).tolist()]
    # Approximate portal locations inferred from this scan, not surveyed markers.
    portals=[-11.27,13.335];scale=384./(portals[1]-portals[0])
    v=np.c_[(v[:,1]-portals[0])*scale,-v[:,0]*scale,v[:,2]*scale]
    faces=np.concatenate([f[:,:,0] for f in groups.values()]);mesh=trimesh.Trimesh(v,faces,process=False)
    textures=[]
    for name in groups:
        src=args.source/'textures'/(name+'.jpg');shutil.copy2(src,model/'textures'/src.name)
        textures.append(dict(name=src.name,sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
    export_dae(model/'meshes/tunnel.dae',v,uv,groups,mesh.vertex_normals)
    # No convex hull: collisions must retain the tunnel's hollow interior.
    mesh.export(model/'meshes/collision.stl')
    (model/'model.config').write_text('<model><name>Futatsugoya tunnel</name><version>1</version><sdf version="1.9">model.sdf</sdf><author><name>Fujihiko Matsumoto</name></author><description>CC BY 4.0 photogrammetry; see attribution.json</description></model>')
    (model/'model.sdf').write_text('''<sdf version="1.9"><model name="futatsugoya"><static>true</static><link name="tunnel"><visual name="scan"><geometry><mesh><uri>model://futatsugoya/meshes/tunnel.dae</uri></mesh></geometry><cast_shadows>false</cast_shadows></visual><collision name="scan_collision"><geometry><mesh><uri>model://futatsugoya/meshes/collision.stl</uri></mesh></geometry></collision></link></model></sdf>''')
    # Keep the world transport name 'zab' required by the frozen campaign runner;
    # this contains only the new model, no ZaB geometry or lights.
    (root/'futatsugoya.sdf').write_text('''<sdf version="1.9"><world name="zab"><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.8</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><scene><ambient>1 1 1 1</ambient><background>0.1 0.1 0.1 1</background><shadows>false</shadows><grid>false</grid></scene><include><uri>model://futatsugoya</uri></include></world></sdf>''')
    save(root/'attribution.json',dict(title='Futatsugoya Tunnel “Abandoned road” japan',author='Fujihiko Matsumoto (fujisan)',source_url=SOURCE_URL,license='CC BY 4.0',license_url='https://creativecommons.org/licenses/by/4.0/',source_obj_sha256=source_sha,textures=textures,changes=['Uniform scale and rigid transform','Reconstructed missing material bindings from existing OBJ group and image names','Converted tiled UV coordinates to per-material 0..1 coordinates','Generated smooth normals; no geometry simplification','DAE visual and identical-triangle STL collision'],scale_assumption=dict(meters_per_source_unit=scale,approximate_portals_source_y=portals,assumed_portal_separation_m=384,reference='https://ootaki.xsrv.jp/26akitan2.pdf page 5, reproduces historical construction drawing',survey_calibrated=False),lighting='Uniform ambient illumination of photo textures; not a simulation of real tunnel darkness or headlights',raw_bounds=raw_bounds,world_bounds=mesh.bounds.tolist(),faces=len(faces),watertight=bool(mesh.is_watertight)))
    routes=root/'routes';routes.mkdir(exist_ok=True)
    entries={name:make_route(routes,name,mesh,a,b) for name,a,b in [('smoke',140,160),('interior',60,350)]}
    save(root/'routes.json',entries)
    print(json.dumps(entries,indent=2))

if __name__=='__main__':main()
