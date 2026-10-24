#!/usr/bin/env python3
"""Extend a scanned tunnel and redistribute its real wall textures reproducibly.

Only the source import profile is specific to this FBX: Y up, Z tunnel axis,
interior centre (X,Y)=(0,65), longitudinal interior -125..125 source units.
All random placements and transformations are saved; no SLAM result is read.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import RegularGridInterpolator
import trimesh
import collada

HERE = Path(__file__).resolve().parent


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cast(mesh, uv, image, xs, theta):
    xx, tt = np.meshgrid(xs, theta, indexing='ij')
    origins = np.c_[xx.ravel(), np.zeros(xx.size), np.full(xx.size, 65.)]
    directions = np.c_[np.zeros(xx.size), np.sin(tt.ravel()), np.cos(tt.ravel())]
    radii = np.empty(xx.size)
    rgb = np.empty((xx.size, 3), np.uint8)
    for start in range(0, len(origins), 16384):
        oo, dd = origins[start:start+16384], directions[start:start+16384]
        points, indices, triangles = mesh.ray.intersects_location(oo, dd, multiple_hits=False)
        if len(indices) != len(oo):
            raise ValueError(f'Open interior surface: {len(oo)-len(indices)} rays missing')
        bary = trimesh.triangles.points_to_barycentric(mesh.triangles[triangles], points)
        tex = np.einsum('ij,ijk->ik', bary, uv[mesh.faces[triangles]])
        colors = cv2.remap(image, (tex[:, 0]*(image.shape[1]-1)).astype('float32')[:, None],
                           ((1-tex[:, 1])*(image.shape[0]-1)).astype('float32')[:, None], cv2.INTER_LINEAR)
        radii[start+indices] = np.linalg.norm(points-oo[indices], axis=1)
        rgb[start+indices] = colors[:, 0]
    return radii.reshape(xx.shape), rgb.reshape(*xx.shape, 3)


def prepare(source, root, width):
    root.mkdir(parents=True, exist_ok=False)
    archive = source / 'source/most_brno_tunel.zip'
    with zipfile.ZipFile(archive) as z:
        for name in ['most_brno_tunel.FBX', 'most_brno_tunel_DiffuseMap.jpg']:
            (root/name).write_bytes(z.read(name))
    binary = root/'import_fbx_mesh'
    subprocess.run(['g++', '-O2', str(HERE/'import_fbx_mesh.cpp'), '-lassimp', '-o', str(binary)], check=True)
    subprocess.run([str(binary), str(root/'most_brno_tunel.FBX'), str(root/'source.obj')], check=True)
    raw = trimesh.load(root/'source.obj', force='mesh', process=False)
    mesh = trimesh.Trimesh(raw.vertices[:, [2, 0, 1]], raw.faces, process=False)
    image = np.asarray(Image.open(root/'most_brno_tunel_DiffuseMap.jpg').convert('RGB'))
    xs = np.linspace(-125., 125., 160)
    theta = np.linspace(-np.pi, np.pi, 384, endpoint=False)
    radius, _ = cast(mesh, raw.visual.uv, image, xs, theta)
    source_width = float(np.median(radius[:, 96] + radius[:, 288]))
    scale = width/source_width
    floor = float(np.median(65-radius[:, 0]))
    mean = radius.mean(axis=0)*scale
    yz = np.c_[mean*np.sin(theta), mean*np.cos(theta)]
    distances = np.linalg.norm(np.diff(np.vstack([yz, yz[0]]), axis=0), axis=1)
    arc = np.r_[0, np.cumsum(distances)]
    # Uniform physical arc spacing prevents angular stretching of texture patches.
    tex_x = np.linspace(xs[0], xs[-1], 768)
    tex_s = np.linspace(0, arc[-1], 1536, endpoint=False)
    tex_theta = np.interp(tex_s, arc, np.r_[theta, np.pi])
    print('Unwrapping original interior texture...', flush=True)
    _, colors = cast(mesh, raw.visual.uv, image, tex_x, tex_theta)
    # Image rows are surface arc; columns are distance along the original tunnel.
    unwrapped = colors.transpose(1, 0, 2)
    Image.fromarray(unwrapped).save(root/'interior_unwrapped.png')
    np.savez_compressed(root/'interior_geometry.npz', xs=xs*scale, theta=theta,
                        radius=radius*scale, center_z=(65-floor)*scale,
                        arc=arc, scale=scale)
    save(root/'source_manifest.json', dict(source_archive=str(archive), archive_sha256=sha(archive),
         fbx_sha256=sha(root/'most_brno_tunel.FBX'), diffuse_sha256=sha(root/'most_brno_tunel_DiffuseMap.jpg'),
         original_faces=len(mesh.faces), coordinate_transform='new (X,Y,Z) = source (Z,X,Y)',
         interior_source_axis_range=[-125,125], interior_source_center_yz=[0,65],
         measured_width_source_units=source_width, meters_per_source_unit=scale,
         requested_width_m=width, calibrated_from_survey=False, floor_source_y=floor,
         unwrap_shape=list(unwrapped.shape), shell_circumference_m=float(arc[-1]),
         note='Source diffuse image only. Normal/AO maps are not used to invent additional collision geometry.'))
    return unwrapped


def feather(height, width, margin):
    d = np.minimum(np.minimum(np.arange(height)[:,None], np.arange(height)[::-1,None]),
                   np.minimum(np.arange(width)[None,:], np.arange(width)[None,::-1]))
    t = np.clip(d/max(margin,1), 0, 1)
    return (t*t*(3-2*t)).astype('float32')


def textures(root, source, seed, length, geom):
    rng = np.random.default_rng(seed)
    im = np.asarray(Image.open(source/'interior_unwrapped.png').convert('RGB'))
    circumference = float(geom['arc'][-1])
    density = 128
    w, h = int(round(length*density)), int(round(circumference*density))
    # Choose crops from actual concrete/stone/paint surfaces; never flatten them.
    patches = []
    src_w_m = float(geom['xs'][-1]-geom['xs'][0])
    sy, sx = im.shape[0]/circumference, im.shape[1]/src_w_m
    for i in range(160):
        pm = float(rng.uniform(.65,1.8)); pw,ph = int(pm*sx),int(pm*sy)
        y = int(rng.integers(int(.15*im.shape[0]), int(.85*im.shape[0])-ph))
        x = int(rng.integers(0,im.shape[1]-pw))
        crop = im[y:y+ph,x:x+pw]
        hsv = cv2.cvtColor(crop,cv2.COLOR_RGB2HSV)
        patches.append(dict(x=x,y=y,w=pw,h=ph,color_fraction=float(np.mean(hsv[:,:,1]>65))))
    plain = sorted(patches,key=lambda p:p['color_fraction'])[:60]
    painted = [p for p in patches if p['color_fraction']>.10]
    if len(painted)<8:raise ValueError('Insufficient painted source crops')
    canvas = np.zeros((h,w,3),np.float32);weights=np.zeros((h,w),np.float32)
    tile = 256;step=192
    base_records=[]
    for yy in range(-64,h,step):
        for xx in range(-64,w,step):
            p = plain[int(rng.integers(len(plain)))];crop=im[p['y']:p['y']+p['h'],p['x']:p['x']+p['w']]
            flip = bool(rng.integers(2))
            if flip:crop=crop[:,::-1]
            patch=cv2.resize(crop,(tile,tile),interpolation=cv2.INTER_LINEAR).astype('float32')
            alpha=feather(tile,tile,48)
            x0,x1=max(0,xx),min(w,xx+tile);y0,y1=max(0,yy),min(h,yy+tile)
            sub=alpha[y0-yy:y1-yy,x0-xx:x1-xx]
            canvas[y0:y1,x0:x1]+=patch[y0-yy:y1-yy,x0-xx:x1-xx]*sub[:,:,None]
            weights[y0:y1,x0:x1]+=sub
            base_records.append(dict(x=xx,y=yy,source=p,flip=flip))
    canvas/=np.maximum(weights[:,:,None],1e-6)
    # The floor retains its real gravel/concrete texture and is not assigned graffiti.
    s=np.linspace(0,circumference,h,endpoint=False)
    theta=np.interp(s,geom['arc'],np.r_[geom['theta'],np.pi])
    floor=np.abs(theta)>2.35
    for yy in np.flatnonzero(floor):
        row=int(yy/h*im.shape[0]);u=np.mod(np.arange(w)/density,src_w_m)/src_w_m*(im.shape[1]-1)
        canvas[yy]=im[row,np.floor(u).astype(int)]
    # Homogeneous random cluster centres: no preselected left/roof/right sequence.
    clusters=[];placements=[]
    eligible=np.flatnonzero(~floor)
    for i in range(int(round(length/7))):
        cx=float(rng.uniform(0,length));cs=float(rng.uniform(s[eligible[0]],s[eligible[-1]]))
        count=int(rng.integers(5,12));clusters.append(dict(x_m=cx,arc_m=cs,count=count))
        for j in range(count):
            px=float(np.clip(rng.normal(cx,2.0),0,length));ps=float(np.clip(rng.normal(cs,.65),s[eligible[0]],s[eligible[-1]]))
            p=painted[int(rng.integers(len(painted)))];crop=im[p['y']:p['y']+p['h'],p['x']:p['x']+p['w']]
            size_m=float(rng.uniform(.65,2.0));size=int(size_m*density)
            patch=cv2.resize(crop,(size,size));flip=bool(rng.integers(2))
            if flip:patch=patch[:,::-1]
            angle=float(rng.uniform(-60,60));gain=float(rng.uniform(.85,1.15));hue=int(rng.integers(-12,13))
            hsv=cv2.cvtColor(patch,cv2.COLOR_RGB2HSV);hsv[:,:,0]=((hsv[:,:,0].astype(int)+hue)%180).astype('uint8')
            patch=np.clip(cv2.cvtColor(hsv,cv2.COLOR_HSV2RGB).astype(float)*gain,0,255).astype('uint8')
            alpha=feather(size,size,int(.14*size))
            matrix=cv2.getRotationMatrix2D((size/2,size/2),angle,1)
            patch=cv2.warpAffine(patch,matrix,(size,size),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
            alpha=cv2.warpAffine(alpha,matrix,(size,size),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            xx,yy=int(px*density-size/2),int(ps*density-size/2)
            x0,x1=max(0,xx),min(w,xx+size);y0,y1=max(0,yy),min(h,yy+size)
            aa=alpha[y0-yy:y1-yy,x0-xx:x1-xx].copy();aa[floor[y0:y1]]=0
            canvas[y0:y1,x0:x1]=canvas[y0:y1,x0:x1]*(1-aa[:,:,None])+patch[y0-yy:y1-yy,x0-xx:x1-xx]*aa[:,:,None]
            placements.append(dict(cluster=i,x_m=px,arc_m=ps,source=p,size_m=size_m,rotation_deg=angle,flip=flip,intensity_gain=gain,hue_shift=hue))
    result=np.uint8(np.clip(canvas,0,255))
    # Split into 10m textures, avoiding a single oversized graphics allocation.
    paths=[]
    for i,start in enumerate(range(0,w,10*density)):
        path=root/'textures'/f'wall_{i:02d}.png';Image.fromarray(result[:,start:start+10*density]).save(path);paths.append(path)
    Image.fromarray(result).resize((2000, max(200, int(2000*h/w)))).save(root/'texture_overview.jpg')
    save(root/'texture_placements.json',dict(seed=seed,pixels_per_meter=density,clusters=clusters,
         placements=placements,base_tiles=base_records,source_candidates=patches,
         rule='Uniform random cluster centres over longitudinal position and wall/ceiling arc; Gaussian scatter within each cluster. No prescribed direction sequence or estimator feedback.'))
    return paths


def mesh_chunk(path, xs, theta, radius, center_z, arc):
    xx,tt=np.meshgrid(xs,theta,indexing='ij')
    v=np.stack([xx,radius*np.sin(tt),center_z+radius*np.cos(tt)],axis=-1).reshape(-1,3)
    u=(xx-xs[0])/(xs[-1]-xs[0]);vv=np.broadcast_to(1-arc/arc[-1],xx.shape)
    uv=np.stack([u,vv],axis=-1).reshape(-1,2)
    n=len(theta);i=np.arange((len(xs)-1)*n).reshape(-1,n)[:,:-1].ravel()
    f=np.vstack([np.c_[i,i+n,i+n+1],np.c_[i,i+n+1,i+1]])
    tri=v[f];norm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);target=np.c_[np.zeros(len(f)),-tri.mean(1)[:,1],center_z-tri.mean(1)[:,2]]
    if np.median(np.einsum('ij,ij->i',norm,target))<0:f=f[:,::-1]
    mesh=trimesh.Trimesh(v,f,process=False)
    doc=collada.Collada();doc.assetInfo.upaxis='Z_UP';doc.assetInfo.unitmeter=1
    # A fixed metadata epoch makes regenerated geometry byte-reproducible.
    doc.assetInfo.created=doc.assetInfo.modified=datetime.datetime(2000,1,1)
    sources=[collada.source.FloatSource('p',np.array(v,copy=True,order='C'),('X','Y','Z')),collada.source.FloatSource('n',np.array(mesh.vertex_normals,copy=True,order='C'),('X','Y','Z')),collada.source.FloatSource('uv',np.array(uv,copy=True,order='C'),('S','T'))]
    g=collada.geometry.Geometry(doc,'tunnel','tunnel',sources);inp=collada.source.InputList();inp.addInput(0,'VERTEX','#p');inp.addInput(1,'NORMAL','#n');inp.addInput(2,'TEXCOORD','#uv',set='0')
    g.primitives.append(g.createTriangleSet(np.repeat(f[:,:,None],3,axis=2).ravel(),inp,'surface'))
    doc.geometries.append(g);doc.scenes.append(collada.scene.Scene('scene',[collada.scene.Node('shell',[collada.scene.GeometryNode(g)])]));doc.scene=doc.scenes[0];doc.write(str(path))
    return mesh


def generate(root, source, seed, length):
    root.mkdir(exist_ok=False);(root/'meshes').mkdir();(root/'textures').mkdir()
    g=dict(np.load(source/'interior_geometry.npz'));period=float(g['xs'][-1]-g['xs'][0]-1.)
    theta=np.linspace(-np.pi,np.pi,129);arc=np.interp(theta,np.r_[g['theta'],np.pi],g['arc'])
    interpolation=RegularGridInterpolator((g['xs'],np.r_[g['theta'],np.pi]),np.c_[g['radius'],g['radius'][:,0]])
    worlds=ET.fromstring('''<sdf version="1.9"><world name="zab"><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.8</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><scene><ambient>1 1 1 1</ambient><background>0.1 0.1 0.1 1</background><shadows>false</shadows><grid>false</grid></scene></world></sdf>''')
    model=ET.SubElement(worlds.find('world'),'model',name='ostopovice_extended');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='shell')
    tex=textures(root,source,seed,length,g);meshes=[]
    for j,start in enumerate(np.arange(0,length,10)):
        end=min(start+10,length);xs=np.linspace(start,end,round((end-start)/.1)+1)
        phase=np.mod(xs,period);ss,tt=np.meshgrid(g['xs'][0]+phase,theta,indexing='ij')
        rr=interpolation(np.c_[ss.ravel(),tt.ravel()]).reshape(ss.shape)
        head=phase<1.;ss2=ss[head]+period;tail=interpolation(np.c_[ss2.ravel(),tt[head].ravel()]).reshape(ss2.shape)
        weight=(.5-.5*np.cos(np.pi*phase[head]))[:,None];rr[head]=weight*rr[head]+(1-weight)*tail
        path=root/'meshes'/f'shell_{j:02d}.dae';meshes.append(mesh_chunk(path,xs,theta,rr,float(g['center_z']),arc))
        for kind in ['collision','visual']:
            obj=ET.SubElement(link,kind,name=f'shell_{j:02d}');geo=ET.SubElement(obj,'geometry');m=ET.SubElement(geo,'mesh');ET.SubElement(m,'uri').text=str(path)
            if kind=='visual':
                mat=ET.SubElement(obj,'material');ET.SubElement(mat,'ambient').text='1 1 1 1';ET.SubElement(mat,'diffuse').text='1 1 1 1';pbr=ET.SubElement(mat,'pbr');metal=ET.SubElement(pbr,'metal');ET.SubElement(metal,'albedo_map').text=str(tex[j]);ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='1'
    ET.indent(worlds);world=root/'world.sdf';ET.ElementTree(worlds).write(world,encoding='unicode')
    mesh=trimesh.util.concatenate(meshes)
    points=np.c_[np.arange(5,length-5+.01,.25),np.zeros(round((length-10)/.25)+1),np.full(round((length-10)/.25)+1,2.)]
    _,clearance,_=trimesh.proximity.closest_point(mesh,points)
    assert clearance.min()>1.25,'Insufficient flight clearance'
    floors,ids,_=mesh.ray.intersects_location(points[:1],np.array([[0,0,-1.]]),multiple_hits=False);assert len(ids)==1
    route=root/'route';route.mkdir();distance=length-10;duration=distance/.5+2
    save(route/'route.json',dict(frame='world',reference='FC IMU',length_m=distance,waypoints=[points[0].tolist(),points[-1].tolist()]))
    save(route/'segments.json',dict(segments=[dict(kind='polyline',t0=0.,duration=duration,start=points[0].tolist(),end=points[-1].tolist(),yaw0=0.,yaw1=0.,speed_mps=.5,ramp_s=2.,points=[points[0].tolist(),points[-1].tolist()],arc=[0.,distance])]))
    entry=dict(route=str(route),world=str(world),spawn=[5.,0.,float(floors[0,2]+.35)],route_end_s=duration)
    save(root/'environment.json',dict(seed=seed,length_m=length,source_manifest=str(source/'source_manifest.json'),
         geometry='Ray-sampled original interior, repeated axially with a 1m cosine overlap. Source roughness retained at 0.1m axial / 128 circumferential geometry resolution; not the untouched original scan.',
         geometry_period_m=period,bounds=mesh.bounds.tolist(),minimum_route_center_clearance_m=float(clearance.min()),
         route_length_m=distance,route_speed_mps=.5,full_body_swept_check=False,
         textures='Original diffuse image crops, random clustered placement, rotation, reflection, scale, hue and intensity. Natural concrete texture retained. No layout metadata sent to controller.',
         lighting='Uniform ambient, no dynamic illumination; original photo shading remains baked into texture.'))
    return entry


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--length',type=float,default=100);p.add_argument('--width',type=float,default=5);p.add_argument('--seeds',type=int,nargs='+',default=[24101,24102,24103]);a=p.parse_args()
    a.output=a.output.resolve();a.output.mkdir(exist_ok=False,parents=True)
    prepare(a.source,a.output/'source',a.width)
    entries={}
    for seed in a.seeds:
        print('Generating layout',seed,flush=True);entries[f'layout{seed}']=generate(a.output/f'layout{seed}',a.output/'source',seed,a.length)
    save(a.output/'catalog.json',entries)
    save(a.output/'hashes.json',{str(p.relative_to(a.output)):sha(p) for p in a.output.rglob('*') if p.is_file()})
    print(a.output/'catalog.json')


if __name__=='__main__':main()
