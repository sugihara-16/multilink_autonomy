#!/usr/bin/env python3
"""Deterministic synthetic tunnel: identical LiDAR geometry, spatially varying texture.

No geometry or texture-layout metadata is supplied to the view controller.
This is a controlled mechanism test, not a scanned real-world benchmark.
"""
import argparse, hashlib, json, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

SURFACES = ['left', 'roof', 'right', 'floor']
LAYOUTS = {'alternating': SURFACES, 'reversed': SURFACES[::-1], 'uniform': ['all']*4}
XMIN, XMAX = -40., 210.
EDGES = [24., 56., 88., 120., 152.]
SEGMENTS = [[25., 65.], [89., 129.], [153., 193.], [217., 257.]]

def save(p, obj):
    p.write_text(json.dumps(obj, indent=2)+'\n')

def strip_points(surface):
    if surface == 'left': return np.array([[3., 0.], [3., 3.]])
    if surface == 'right': return np.array([[-3., 3.], [-3., 0.]])
    if surface == 'floor': return np.array([[-3., 0.], [3., 0.]])
    t = np.linspace(0, np.pi, 25)
    return np.column_stack([3*np.cos(t), 3+2*np.sin(t)])

def write_mesh(path, yz):
    # Clockwise cavity boundary gives inward-facing triangles with this ordering.
    vertices=[];uv=[];faces=[]
    length=np.r_[0., np.cumsum(np.linalg.norm(np.diff(yz, axis=0), axis=1))]
    for k in range(len(yz)-1):
        n=len(vertices)
        vertices.extend([[XMIN,*yz[k]], [XMAX,*yz[k]], [XMAX,*yz[k+1]], [XMIN,*yz[k+1]]])
        uv.extend([[0,length[k]/length[-1]], [1,length[k]/length[-1]], [1,length[k+1]/length[-1]], [0,length[k+1]/length[-1]]])
        faces.extend([[n,n+1,n+2], [n,n+2,n+3]])
    vertices=np.array(vertices);uv=np.array(uv);faces=np.array(faces)
    # Verify inward normals rather than relying on double-sided rendering.
    tri=vertices[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    center=tri.mean(axis=1);toward=np.array([0,0,2.5])[None]-center;toward[:,0]=0
    if np.any(np.sum(normal*toward,axis=1)<0): faces=faces[:,::-1]
    normals=np.zeros_like(vertices)
    for face in faces:
        a,b,c=vertices[face];n=np.cross(b-a,c-a);n/=np.linalg.norm(n)
        normals[face]=n
    positions=' '.join(map(str,vertices.ravel()));tex=' '.join(map(str,uv.ravel()));normal_text=' '.join(map(str,normals.ravel()))
    indices=' '.join(str(int(v)) for f in faces for i in f for v in [i,i,i])
    path.write_text(f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="surface"><mesh>
<source id="p"><float_array id="pa" count="{vertices.size}">{positions}</float_array><technique_common><accessor source="#pa" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uva" count="{uv.size}">{tex}</float_array><technique_common><accessor source="#uva" count="{len(uv)}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<source id="n"><float_array id="na" count="{normals.size}">{normal_text}</float_array><technique_common><accessor source="#na" count="{len(normals)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
<triangles count="{len(faces)}"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><input semantic="NORMAL" source="#n" offset="2"/><p>{indices}</p></triangles>
</mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node><instance_geometry url="#surface"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')
    return vertices,faces

def texture(surface, layout, output):
    # Unique, non-repeating stains; same underlying marks in every layout.
    seed=240001+SURFACES.index(surface)
    rng=np.random.default_rng(seed);w,h=16384,512
    image=Image.new('L',(w,h),145);draw=ImageDraw.Draw(image)
    for _ in range(12500):
        x=int(rng.integers(w));y=int(rng.integers(h));dx=int(rng.integers(3,32));dy=int(rng.integers(3,35))
        color=int(rng.choice([50,65,90,110,175,195,220,235]))
        if rng.random()<.5:draw.rectangle((x,y,x+dx,y+dy),fill=color)
        else:draw.polygon([(x,y),(x+dx,y+int(dy*.3)),(x+int(dx*.7),y+dy),(x-int(dx*.2),y+int(dy*.6))],fill=color)
    rich=np.array(image.filter(ImageFilter.GaussianBlur(.6)),dtype=np.float32)
    x=np.linspace(XMIN,XMAX,w);strength=np.ones(w)
    if layout!='uniform':
        # Smoothly cross-fade over 4 m; no artificial transverse geometric seam.
        strength=np.ones(w)*.025
        start=np.clip((EDGES[0]+2-x)/4,0,1);strength=np.maximum(strength,start)
        for i,name in enumerate(LAYOUTS[layout]):
            if name==surface:
                a=np.clip((x-EDGES[i]+2)/4,0,1);b=np.clip((EDGES[i+1]+2-x)/4,0,1)
                strength=np.maximum(strength,np.minimum(a,b))
        strength=np.maximum(strength,np.clip((x-EDGES[-1]+2)/4,0,1))
    img=np.uint8(np.clip(145+(rich-145)*strength[None],0,255))
    Image.fromarray(img).convert('RGB').save(output)
    return dict(seed=seed,texture_size=[w,h],low_contrast_fraction=.025,transition_width_m=4.)

def build(root):
    root.mkdir(parents=True,exist_ok=False);(root/'meshes').mkdir();(root/'textures').mkdir()
    geometry=[]
    for s in SURFACES:
        v,f=write_mesh(root/'meshes'/f'{s}.dae',strip_points(s));geometry.append((s,v,f))
    base=ET.fromstring('''<sdf version="1.9"><world name="zab"><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.8</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><scene><ambient>1 1 1 1</ambient><background>0.1 0.1 0.1 1</background><shadows>false</shadows><grid>false</grid></scene></world></sdf>''')
    worlds={};texture_info={}
    for layout in LAYOUTS:
        tree=ET.fromstring(ET.tostring(base));world=tree.find('world');model=ET.SubElement(world,'model',name='controlled_tunnel');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='shell')
        for s in SURFACES:
            tex=root/'textures'/f'{layout}_{s}.png';texture_info[f'{layout}_{s}']=texture(s,layout,tex)
            for kind in ['collision','visual']:
                obj=ET.SubElement(link,kind,name=s);geo=ET.SubElement(obj,'geometry');mesh=ET.SubElement(geo,'mesh');ET.SubElement(mesh,'uri').text=str(root/'meshes'/f'{s}.dae')
                if kind=='visual':
                    mat=ET.SubElement(obj,'material');ET.SubElement(mat,'ambient').text='1 1 1 1';ET.SubElement(mat,'diffuse').text='1 1 1 1';pbr=ET.SubElement(mat,'pbr');metal=ET.SubElement(pbr,'metal');ET.SubElement(metal,'albedo_map').text=str(tex);ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='1'
        path=root/f'{layout}.sdf';ET.indent(tree);ET.ElementTree(tree).write(path,encoding='unicode');worlds[layout]=str(path)
    route=root/'route';route.mkdir();start=[20.,0.,2.5];end=[150.,0.,2.5];duration=262.
    save(route/'route.json',dict(frame='world',reference='FC IMU',length_m=130.,waypoints=[start,end],purpose='Same straight 0.5 m/s flight across four texture zones'))
    save(route/'segments.json',dict(segments=[dict(kind='polyline',t0=0.,duration=duration,start=start,end=end,yaw0=0.,yaw1=0.,speed_mps=.5,ramp_s=2.,points=[start,end],arc=[0.,130.])]))
    entries={name:dict(route=str(route),world=path,spawn=[20.,0.,.4],route_end_s=duration) for name,path in worlds.items()}
    save(root/'catalog.json',entries)
    manifest=dict(purpose='Controlled texture-switching tunnel; not ZaB or Futatsugoya scan',geometry=dict(x_range_m=[XMIN,XMAX],width_m=6,wall_height_m=3,ceiling_height_m=5,roof_facets=24,constant_cross_section=True,end_caps=False),layouts=LAYOUTS,zone_edges_x_m=EDGES,transition_width_m=4,trajectory_length_m=130,trajectory_speed_mps=.5,primary_sections_s=SEGMENTS,texture_info=texture_info,controller_receives_layout=False,geometry_shared=True)
    save(root/'environment.json',manifest)
    # Analytic noiseless point-to-plane information has exactly zero axial term.
    rows=[]
    for _,v,f in geometry:
        for face in f:
            a,b,c=v[face];n=np.cross(b-a,c-a);n/=np.linalg.norm(n);p=(a+b+c)/3-np.array([80,0,2.5]);rows.append(np.r_[np.cross(p,n),n])
    J=np.array(rows);H=J.T@J
    assert np.max(np.abs(J[:,3]))==0
    save(root/'geometry_check.json',dict(point_plane_information=H.tolist(),translation_eigenvalues=np.linalg.eigvalsh(H[3:,3:]).tolist(),axial_null_direction=[1,0,0],route_center_clearance_lower_bound_m=2.49,full_body_swept_collision_check=False))
    save(root/'hashes.json',{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file()})
    print(root)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();build(a.output.resolve())
