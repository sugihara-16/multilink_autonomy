"""Conservative visual-frustum audit of the fixed-square DRAGON gimbal.

Every URDF visual is enclosed by its mesh/primitive box, with 15 mm padding.
Rotor modules additionally use a swept sphere bound. A 2-degree angle grid
uses enlarged image frusta; this checks the picture, not just the optical ray.
"""
import argparse,itertools,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.optimize import linprog
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_cameras import transform,link_transforms,OPTICAL
from dragon_articulated import NAMES
from dragon_gimbal import GIMBAL_NAMES,MOUNT_XYZ,MOUNT_RPY


def corners(lo,hi):return np.array(list(itertools.product(*zip(lo,hi))))


def visual_bounds(root):
    angles=dict(zip(NAMES,[0,1.56]*3));frames=link_transforms(root,angles)
    inv=np.linalg.inv(frames['link4']);boxes=[];labels=[]
    for link in root.findall('link'):
        if link.get('name') in ('active_camera_yaw','active_camera_pitch','camera_optical'):continue
        for visual in link.findall('visual'):
            g=visual.find('geometry');o=visual.find('origin')
            T=inv@frames[link.get('name')]
            if o is not None:T=T@transform(np.fromstring(o.get('xyz','0 0 0'),sep=' '),np.fromstring(o.get('rpy','0 0 0'),sep=' '))
            if g.find('mesh') is not None:
                e=g.find('mesh');filename=e.get('filename')
                if filename.startswith('package://'):
                    package,tail=filename[10:].split('/',1);filename=str(Path(get_package_share_directory(package))/tail)
                mesh=trimesh.load(filename,force='mesh');scale=np.fromstring(e.get('scale','1 1 1'),sep=' ')
                bounds=np.array(mesh.bounds)*scale
            elif g.find('box') is not None:
                size=np.fromstring(g.find('box').get('size'),sep=' ');bounds=np.array([-size/2,size/2])
            elif g.find('cylinder') is not None:
                e=g.find('cylinder');r=float(e.get('radius'));h=float(e.get('length'))/2;bounds=np.array([[-r,-r,-h],[r,r,h]])
            elif g.find('sphere') is not None:
                r=float(g.find('sphere').get('radius'));bounds=np.array([[-r]*3,[r]*3])
            else:raise ValueError('Unsupported visual geometry')
            p=corners(bounds[0]-.015,bounds[1]+.015)@T[:3,:3].T+T[:3,3]
            boxes.append(p);labels.append(link.get('name')+'/'+visual.get('name','visual'))
    # Bounds allow rotor gimbals to move during flight, unlike a static mesh test.
    for i in range(1,5):
        c=(inv@frames[f'thrust{i}'])[:3,3]
        boxes.append(corners(c-.25,c+.25));labels.append(f'rotor{i}_swept_bound')
    return np.array(boxes),labels


def audit(root,mount=MOUNT_XYZ):
    boxes,labels=visual_bounds(root)
    camera=root.find(".//sensor[@type='camera']/camera")
    h=float(camera.findtext('horizontal_fov'))/2
    v=np.arctan(np.tan(h)*float(camera.findtext('image/height'))/float(camera.findtext('image/width')))
    # Two-axis 2-degree grid: angular deviation within a cell <=2 degrees.
    # Widen by 3 degrees to cover between-grid poses plus a margin.
    th,tv=np.tan(h+np.deg2rad(3)),np.tan(v+np.deg2rad(3))
    bad=[];count=0
    for yaw in np.arange(-50,51,2):
        for pitch in np.arange(-90,91,2):
            R=transform(rpy=MOUNT_RPY)[:3,:3]@transform(rpy=(0,0,np.deg2rad(yaw)))[:3,:3]@transform(rpy=(0,np.deg2rad(pitch),0))[:3,:3]@OPTICAL
            p=(boxes-mount)@R;x,y,z=p[...,0],p[...,1],p[...,2]
            outside=(np.all(z<=0,axis=1)|np.all(x>th*z,axis=1)|np.all(x<-th*z,axis=1)|np.all(y>tv*z,axis=1)|np.all(y<-tv*z,axis=1))
            # Different corners can lie outside different image edges. Resolve
            # those cases by convex box/frustum intersection, not by declaring
            # their projected axis-aligned rectangle to be an obstruction.
            A=np.array([[1,0,-th],[-1,0,-th],[0,1,-tv],[0,-1,-tv],[0,0,-1]])
            hits=[]
            for i in np.flatnonzero(~outside):
                origin=p[i,0];B=np.column_stack([p[i,k]-origin for k in (4,2,1)])
                if linprog(np.zeros(3),A_ub=A@B,b_ub=-A@origin-np.array([0,0,0,0,1e-5]),bounds=[(0,1)]*3,method='highs').success:hits.append(labels[i])
            count+=1
            if hits:bad.append(dict(yaw=int(yaw),pitch=int(pitch),bounds=hits))
    return dict(passed=not bad,checked_views=count,possible_occlusion_views=len(bad),examples=bad[:15],mount_xyz=list(mount),
                bounds=len(boxes),padding_m=.015,rotor_swept_bound_radius_m=.25,angular_grid_deg=2,frustum_margin_deg=3,
                scope='Full image frustum vs padded visual bounds, square body; excludes ideal camera housing behind lens; no hidden robot visuals or image masks')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('urdf',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    result=audit(ET.parse(a.urdf).getroot());print(json.dumps(result,indent=2))
    if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
    if not result['passed']:raise SystemExit(1)
