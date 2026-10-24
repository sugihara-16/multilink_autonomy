#!/usr/bin/env python3
"""Procedural concrete tunnel with irregular, overlapping weathering clusters.

No photographs or scan geometry; no estimator performance enters generation.
The density field and layout metadata are for inspection only, not control.
"""
import argparse, hashlib, json, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import build_switching_tunnel as mesh_writer

PPM=128
SURFACES=['left','roof','right','floor']

def save(path,data):
    path.write_text(json.dumps(data,indent=2)+'\n')

def profile():
    a=np.linspace(0,np.pi,49)
    return [np.array([[2.5,0],[2.5,2.]]),
            np.column_stack([2.5*np.cos(a),2+2.5*np.sin(a)]),
            np.array([[-2.5,2.],[-2.5,0]]),np.array([[-2.5,0],[2.5,0]])]

def smooth_noise(rng,w,h,scale):
    z=Image.fromarray(rng.integers(0,256,(max(2,h//scale),max(2,w//scale)),dtype=np.uint8))
    return np.asarray(z.resize((w,h),Image.BICUBIC),dtype=np.float32)/255-.5

def density(x,s,clusters):
    result=np.zeros(np.broadcast_shapes(np.shape(x),np.shape(s)),dtype=np.float32)
    for c in clusters:
        # Uneven edges and branching shape; no rectangular region boundaries.
        centre=c['arc_m']+.3*np.sin((x-c['x_m'])/3+c['phase'])
        q=np.exp(-.5*((x-c['x_m'])/c['axial_sigma_m'])**2-.5*((s-centre)/c['arc_sigma_m'])**2)
        result=np.maximum(result,q*c['strength'])
    return .012+.988*result

def make_texture(root,seed,length,arcs):
    rng=np.random.default_rng(seed);w=round(length*PPM)
    boundaries=np.r_[0,np.cumsum(arcs)];pixelbounds=np.rint(boundaries*PPM).astype(int);h=int(pixelbounds[-1])
    # Stratified RANDOM centres ensure spatial change without left-roof-right schedule.
    n=max(4,round(length/20));xs=np.linspace(9,length-9,n)+rng.uniform(-3,3,n)
    sectors=np.resize(rng.permutation(3),n)
    for k in range(3,n,3):sectors[k:k+3]=rng.permutation(3)[:len(sectors[k:k+3])]
    ranges=[(.55,2.4),(4.5,7.4),(9.9,11.6)]
    clusters=[]
    for x,sector in zip(xs,sectors):
        low,high=ranges[int(sector)]
        clusters.append(dict(x_m=float(x),arc_m=float(rng.uniform(low,high)),axial_sigma_m=float(rng.uniform(5,8.5)),arc_sigma_m=float(rng.uniform(.65,1.05)),strength=float(rng.uniform(.8,1)),phase=float(rng.uniform(0,2*np.pi))))
    # Broad concrete variations remain everywhere, without ubiquitous sharp corners.
    gray=154+14*smooth_noise(rng,w,h,300)+6*smooth_noise(rng,w,h,75)+2*smooth_noise(rng,w,h,8)
    rgb=np.clip(gray[:,:,None]+np.array([3,1,-3]),0,255).astype(np.uint8)
    im=Image.fromarray(rgb);del gray,rgb
    overlay=Image.new('RGBA',(w,h));d=ImageDraw.Draw(overlay)
    records=[]
    # Macro damp patches accumulate around the same regions, with soft irregular edges.
    for c in clusters:
        for _ in range(24):
            x=rng.normal(c['x_m'],c['axial_sigma_m']*.7);s=rng.normal(c['arc_m'],c['arc_sigma_m']*.6)
            rx=rng.uniform(.25,1.3);ry=rng.uniform(.15,.55)
            theta=np.linspace(0,2*np.pi,21,endpoint=False);rr=rng.uniform(.7,1.2,len(theta))
            pts=[((x+rx*r*np.cos(t))*PPM,(s+ry*r*np.sin(t))*PPM) for t,r in zip(theta,rr)]
            d.polygon(pts,fill=(68,66,55,int(rng.integers(12,38))))
    im=Image.alpha_composite(im.convert('RGBA'),overlay.filter(ImageFilter.GaussianBlur(9))).convert('RGB')
    del overlay
    marks=Image.new('RGBA',(w,h));d=ImageDraw.Draw(marks)
    for _ in range(round(length*230)):
        x=float(rng.uniform(0,length));s=float(rng.uniform(0,boundaries[3]))
        if rng.random()>float(density(x,s,clusters)):continue
        kind=str(rng.choice(['flake','crack','mineral'],p=[.62,.23,.15]))
        if kind=='crack':
            dx=float(rng.uniform(.03,.25));dy=float(rng.uniform(.08,.55));sign=float(rng.choice([-1,1]))
            pts=[((x+dx*t+rng.normal(0,.015))*PPM,(s+sign*dy*t)*PPM) for t in np.linspace(0,1,6)]
            d.line(pts,fill=(53,51,46,int(rng.integers(80,155))),width=int(rng.integers(1,3)))
        else:
            dx=float(rng.uniform(.018,.15));dy=float(rng.uniform(.018,.12))
            a=rng.uniform(0,2*np.pi);theta=np.linspace(0,2*np.pi,9,endpoint=False)+a;rr=rng.uniform(.55,1.2,9)
            pts=[((x+dx*r*np.cos(t))*PPM,(s+dy*r*np.sin(t))*PPM) for t,r in zip(theta,rr)]
            color=(222,216,197) if kind=='mineral' else (65,62,53)
            d.polygon(pts,fill=(*color,int(rng.integers(95,190))))
        records.append(dict(x_m=x,arc_m=s,kind=kind))
    im=Image.alpha_composite(im.convert('RGBA'),marks.filter(ImageFilter.GaussianBlur(.45))).convert('RGB');del marks
    im.resize((2000,round(h*2000/w))).save(root/'texture_overview.jpg',quality=92)
    for j,name in enumerate(SURFACES):
        for k,start in enumerate(range(0,round(length),10)):
            stop=min(start+10,length)
            # UV V increases along the boundary, whereas image rows go downward.
            tile=im.crop((round(start*PPM),int(pixelbounds[j]),round(stop*PPM),int(pixelbounds[j+1]))).transpose(Image.FLIP_TOP_BOTTOM)
            tile.save(root/'textures'/f'{name}_{k:02}.png')
    save(root/'distribution.json',dict(seed=seed,clusters=clusters,marks=records,arc_boundaries_m=boundaries.tolist(),surfaces=SURFACES,background_density=.012,texture_pixels_per_m=PPM,note='Density-weighted random marks; continuous overlap, no rectangular contrast masks. Floor has only concrete background.'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xx=np.linspace(0,length,600);ss=np.linspace(0,boundaries[3],200)
    field=density(xx[None,:],ss[:,None],clusters)
    fig,axes=plt.subplots(2,1,figsize=(14,6),sharex=True)
    axes[0].imshow(np.asarray(im.resize((1800,round(h*1800/w)))),extent=(0,length,boundaries[-1],0),aspect='auto')
    axes[0].set(title='Procedural concrete, weathering and mineral marks',ylabel='Unwrapped surface [m]')
    axes[1].imshow(field,extent=(0,length,boundaries[3],0),aspect='auto',vmin=0,vmax=1,cmap='magma')
    for ax in axes:
        for y in boundaries[1:4]:ax.axhline(y,color='cyan',alpha=.5,lw=.6)
    axes[1].set(xlabel='Tunnel distance [m]',ylabel='Unwrapped surface [m]',title='Smooth density field: left wall / arched ceiling / right wall')
    fig.tight_layout();fig.savefig(root/'distribution.png',dpi=140);plt.close(fig)
    return boundaries

def build(output,seed,length=100):
    output.mkdir(parents=True,exist_ok=False);(output/'meshes').mkdir();(output/'textures').mkdir()
    yz=profile();arcs=[np.linalg.norm(np.diff(v,axis=0),axis=1).sum() for v in yz]
    make_texture(output,seed,length,arcs)
    base='''<sdf version="1.9"><world name="zab"><physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.8</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><scene><ambient>1 1 1 1</ambient><background>0.1 0.1 0.1 1</background><shadows>false</shadows><grid>false</grid></scene></world></sdf>'''
    tree=ET.fromstring(base);world=tree.find('world');model=ET.SubElement(world,'model',name='weathered_tunnel');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='shell')
    normals=[]
    for k,start in enumerate(range(0,round(length),10)):
        mesh_writer.XMIN=float(start);mesh_writer.XMAX=float(min(start+10,length))
        for name,points in zip(SURFACES,yz):
            mesh=output/'meshes'/f'{name}_{k:02}.dae';v,f=mesh_writer.write_mesh(mesh,points)
            triangles=v[f];n=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normals.extend(n/np.linalg.norm(n,axis=1)[:,None])
            for kind in ['visual','collision']:
                obj=ET.SubElement(link,kind,name=f'{name}_{k}');geom=ET.SubElement(obj,'geometry');me=ET.SubElement(geom,'mesh');ET.SubElement(me,'uri').text=str(mesh)
                if kind=='visual':
                    mat=ET.SubElement(obj,'material');ET.SubElement(mat,'ambient').text='1 1 1 1';ET.SubElement(mat,'diffuse').text='1 1 1 1';pbr=ET.SubElement(mat,'pbr');metal=ET.SubElement(pbr,'metal');ET.SubElement(metal,'albedo_map').text=str(output/'textures'/f'{name}_{k:02}.png');ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='1'
    ET.indent(tree);ET.ElementTree(tree).write(output/'world.sdf',encoding='unicode')
    n=np.array(normals);assert np.max(np.abs(n[:,0]))==0
    save(output/'environment.json',dict(type='fully procedural synthetic concrete',seed=seed,length_m=length,width_m=5,height_m=4.5,constant_cross_section=True,texture_only_weathering=True,end_caps=False,axial_normal_component_max=float(np.abs(n[:,0]).max()),controller_receives_distribution=False,external_images_used=False))
    route=output/'route';route.mkdir();start=[5,0,2.];end=[length-5,0,2.];distance=length-10;duration=distance/.5+2
    save(route/'route.json',dict(frame='world',reference='FC IMU',length_m=distance,waypoints=[start,end]))
    save(route/'segments.json',dict(segments=[dict(kind='polyline',t0=0.,duration=duration,start=start,end=end,yaw0=0.,yaw1=0.,speed_mps=.5,ramp_s=2.,points=[start,end],arc=[0.,distance])]))
    save(output/'catalog.json',{f'weathered{seed}':dict(route=str(route),world=str(output/'world.sdf'),spawn=[5,0,.4],route_end_s=duration)})
    save(output/'hashes.json',{str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.rglob('*')) if p.is_file()})
    print(output,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seed',type=int,default=24201);p.add_argument('--length',type=int,default=100);a=p.parse_args()
    if a.length<40 or a.length>500:p.error('length must be between 40 and 500 m')
    build(a.output.resolve(),a.seed,a.length)
