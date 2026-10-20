#!/usr/bin/env python3
"""Export a mesh-checked polyline as a smooth, stop-at-corner reference trajectory."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from dragon_cameras import link_transforms, transform


def envelope(urdf):
    root = ET.parse(urdf).getroot()
    angles = {}
    for joint in root.findall('./ros2_control/joint'):
        initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if initial is not None:
            angles[joint.get('name')] = float(initial.text)
    frames = link_transforms(root, angles)
    inverse = np.linalg.inv(frames['fc'])
    maximum = 0.; sources = {str(urdf): hashlib.sha256(urdf.read_bytes()).hexdigest()}
    from ament_index_python.packages import get_package_share_directory
    for link in root.findall('link'):
        for visual in list(link.findall('visual')) + list(link.findall('collision')):
            g = visual.find('geometry'); mesh = g.find('mesh')
            if mesh is not None:
                name = mesh.get('filename')
                if name.startswith('package://'):
                    pkg, rel = name[10:].split('/', 1)
                    name = str(Path(get_package_share_directory(pkg))/rel)
                points = trimesh.load(name, force='mesh', process=False).vertices
                points = points * np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
                sources[name] = hashlib.sha256(Path(name).read_bytes()).hexdigest()
            else:
                if g.find('box') is not None:
                    half = np.array(list(map(float, g.find('box').get('size').split())))/2
                elif g.find('cylinder') is not None:
                    c = g.find('cylinder'); half = [float(c.get('radius'))]*2 + [float(c.get('length'))/2]
                elif g.find('sphere') is not None:
                    half = [float(g.find('sphere').get('radius'))]*3
                else:
                    raise ValueError(ET.tostring(g))
                points = np.array(list(itertools.product(*[[-v,v] for v in half])))
            o = visual.find('origin')
            local = transform(list(map(float, o.get('xyz','0 0 0').split())),
                              list(map(float, o.get('rpy','0 0 0').split()))) if o is not None else np.eye(4)
            t = inverse @ frames[link.get('name')] @ local
            maximum = max(maximum, float(np.linalg.norm(points @ t[:3,:3].T + t[:3,3],axis=1).max()))
    return dict(maximum_radius_m=maximum,reference='fc',joints=angles,sha256=sources,
                method='All visual/collision mesh vertices and primitive bounding boxes; nominal square and zero gimbals')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--urdf',type=Path,required=True)
    args=p.parse_args(); out=args.output;out.mkdir(parents=True,exist_ok=True)
    plan=json.loads((args.input/'route.json').read_text());route=np.array(plan['waypoints'])
    body=envelope(args.urdf)
    assert body['maximum_radius_m'] < plan['enclosing_radius_m'], body
    (out/'robot_envelope.json').write_text(json.dumps(body,indent=2)+'\n')
    # Quintic ease-in/out is monotone: every position stays on its certified straight segment.
    # vmax = 1.875*distance/T; amax = (10/sqrt(3))*distance/T**2.
    vmax=.8;amax=.4;wmax=.3;alphamax=.3;dt=.5
    rows=[];segments=[];now=0.;yaw=0.
    def append(a,b,ya,yb,duration,kind):
        nonlocal now
        delta=np.asarray(b)-a;dy=yb-ya
        for t in np.arange(0,duration,dt):
            u=t/duration;s=10*u**3-15*u**4+6*u**5
            ds=(30*u**2-60*u**3+30*u**4)/duration
            dds=(60*u-180*u**2+120*u**3)/duration**2
            rows.append([now+t,*(a+s*delta),ya+s*dy,*(ds*delta),ds*dy,*(dds*delta)])
        segments.append(dict(t0=now,duration=duration,start=list(a),end=list(b),yaw0=ya,yaw1=yb,kind=kind))
        now+=duration
    for a,b in zip(route,route[1:]):
        delta=b-a;distance=float(np.linalg.norm(delta))
        target=math.atan2(delta[1],delta[0]) if np.linalg.norm(delta[:2])>1e-9 else yaw
        change=math.atan2(math.sin(target-yaw),math.cos(target-yaw))
        if abs(change)>1e-8:
            T=max(1.875*abs(change)/wmax,math.sqrt((10/math.sqrt(3))*abs(change)/alphamax))
            append(a,a,yaw,yaw+change,T,'rotate');yaw+=change
        T=max(1.875*distance/vmax,math.sqrt((10/math.sqrt(3))*distance/amax))
        append(a,b,yaw,yaw,T,'translate')
    rows.append([now,*route[-1],yaw,0,0,0,0,0,0,0]);rows=np.array(rows)
    assert np.all(np.diff(rows[:,0])>0)
    assert np.linalg.norm(rows[:,5:8],axis=1).max()<=vmax+1e-8
    assert np.linalg.norm(rows[:,9:12],axis=1).max()<=amax+1e-8
    assert np.abs(rows[:,8]).max()<=wmax+1e-8
    plan.update(duration_s=now,max_speed_m_s=vmax,max_acceleration_m_s2=amax,max_yaw_rate_rad_s=wmax,
                sample_period_s=dt,interpolation='Use segments.json quintic polynomial; do not apply unconstrained cubic splines',
                robot_envelope_m=body['maximum_radius_m'],tracking_allowance_with_1m_envelope_m=plan['conservative_center_surface_distance_m']-1.)
    (out/'route.json').write_text(json.dumps(plan,indent=2)+'\n')
    (out/'segments.json').write_text(json.dumps(dict(polynomial='s(u)=10u^3-15u^4+6u^5, u=(t-t0)/duration',segments=segments),indent=2)+'\n')
    np.savetxt(out/'trajectory.csv',rows,delimiter=',',fmt='%.7f',header='time_s,x_m,y_m,z_m,yaw_rad,vx_m_s,vy_m_s,vz_m_s,yaw_rate_rad_s,ax_m_s2,ay_m_s2,az_m_s2',comments='')
    np.savetxt(out/'waypoints.csv',route,delimiter=',',fmt='%.4f',header='x_m,y_m,z_m',comments='')
    (out/'mesh_sources.json').write_text((args.input/'mesh_sources.json').read_text())
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    grid=np.load(args.input/'overview.npz');lo=grid['lower'];h=float(grid['pitch']);free=grid['free_xy']
    fig,(ax,az)=plt.subplots(1,2,figsize=(14,7),gridspec_kw={'width_ratios':[1.3,1]})
    ax.imshow(free.T,origin='lower',extent=[lo[0]-.5*h,lo[0]+(free.shape[0]-.5)*h,lo[1]-.5*h,lo[1]+(free.shape[1]-.5)*h],cmap='Greys',vmin=0,vmax=2)
    ax.plot(route[:,0],route[:,1],color='tab:orange',lw=2)
    ax.scatter(*route[0,:2],color='tab:green',s=60,zorder=5)
    for stage in plan['stages']:
        if stage['name'] not in ('home','junction_return'):
            x,y,z=stage['position'];ax.annotate(stage['name'],(x,y),xytext=(5,7),textcoords='offset points',fontsize=8)
    ax.set(xlabel='World X [m]',ylabel='World Y [m]',xlim=(20,195),ylim=(-125,60),title='Cave corridor projection and route (return overlaps outbound)');ax.set_aspect('equal')
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    az.plot(arc,route[:,2]);az.set(xlabel='Distance along route [m]',ylabel='World Z [m]',title=f'{plan["length_m"]:.1f} m / {now/60:.1f} min / max {vmax} m/s');az.grid(True)
    fig.tight_layout();fig.savefig(out/'overview.png',dpi=150);plt.close(fig)
    print(json.dumps(dict(length_m=plan['length_m'],duration_min=now/60,envelope_m=body['maximum_radius_m'],clearance_m=plan['conservative_center_surface_distance_m'],samples=len(rows),z_range=[route[:,2].min(),route[:,2].max()]),indent=2))

if __name__=='__main__':main()
