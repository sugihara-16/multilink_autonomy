"""Simulation calibration and three-camera UniPilot model from one source of truth.
Nominal mount geometry, NOT a calibration for a physical UniPilot unit.
Native IMX296 resolution 1440x1080 is simulated with 2x reduction (720x540).
"""
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from prepare_slam_model import prepare_model

CAMERAS = [('front',[.18,0,.18],0,118,False),
           ('left',[.02,.08,.18],math.pi/2,185,True),
           ('right',[.02,-.08,.18],-math.pi/2,185,True)]


def prepare(source, output):
    output = Path(output)
    prepare_model(source, output/'unipilot_vio.sdf')
    root = ET.parse(output/'unipilot_vio.sdf')
    body = root.getroot().find("model/link[@name='base_link']")
    template = (Path('/root/unified_autonomy_stack/workspaces/robot_bringup/config/ros1/rovio.info')).read_text()
    template = re.sub(r'Camera\d+\s*\{[^}]*\}', '', template)
    template = template.replace('doFrameVisualisation true','doFrameVisualisation false')
    cameras = []
    for i,(name,xyz,yaw,fov_deg,fisheye) in enumerate(CAMERAS):
        width,height=720,540
        hfov=math.radians(fov_deg)
        fx=width/hfov if fisheye else width/(2*math.tan(hfov/2))
        distortion='equidistant' if fisheye else 'plumb_bob'
        camera_cfg=dict(image_width=width,image_height=height,camera_name='cam_'+name,
            camera_matrix=dict(rows=3,cols=3,data=[fx,0,width/2,0,fx,height/2,0,0,1]),
            distortion_model=distortion,distortion_coefficients=dict(rows=1,cols=4 if fisheye else 5,data=[0.]*(4 if fisheye else 5)))
        (output/f'raw_cam{i}.yaml').write_text(yaml.safe_dump(camera_cfg))
        rect_width,rect_height=480,360
        rect_fov=118 if i==0 else 120
        rect_fx=rect_width/(2*math.tan(math.radians(rect_fov)/2))
        rect_cfg=dict(image_width=rect_width,image_height=rect_height,camera_name='cam_'+name,
            camera_matrix=dict(rows=3,cols=3,data=[rect_fx,0,rect_width/2,0,rect_fx,rect_height/2,0,0,1]),
            distortion_model='plumb_bob',distortion_coefficients=dict(rows=1,cols=5,data=[0.]*5))
        (output/f'cam{i}.yaml').write_text(yaml.safe_dump(rect_cfg))
        # Gazebo camera: x forward, y left, z up; optical: x right, y down, z forward.
        R_B_C=Rotation.from_euler('z',yaw).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
        q=Rotation.from_matrix(R_B_C.T).as_quat()
        template += '\nCamera%d\n{\n  CalibrationFile\n'%i
        template += ''.join('  qCM_%s %.15g;\n'%(k,v) for k,v in zip('xyzw',q))
        template += ''.join('  MrMC_%s %.15g;\n'%(k,v) for k,v in zip('xyz',xyz))+'}\n'
        sensor=ET.SubElement(body,'sensor',name='cam_'+name,type='wideanglecamera' if fisheye else 'camera')
        ET.SubElement(sensor,'pose').text=' '.join(map(str,xyz+[0,0,yaw]))
        ET.SubElement(sensor,'always_on').text='true'
        ET.SubElement(sensor,'update_rate').text='20'
        ET.SubElement(sensor,'topic').text='/rmf_unipilot/cam_'+name+'/image'
        ET.SubElement(sensor,'gz_frame_id').text='rmf_unipilot/cam_'+name+'_optical'
        c=ET.SubElement(sensor,'camera');ET.SubElement(c,'horizontal_fov').text=str(hfov)
        im=ET.SubElement(c,'image')
        for key,value in [('width',width),('height',height),('format','R8G8B8')]:ET.SubElement(im,key).text=str(value)
        clip=ET.SubElement(c,'clip');ET.SubElement(clip,'near').text='.05';ET.SubElement(clip,'far').text='60'
        if fisheye:
            lens=ET.SubElement(c,'lens')
            for key,value in [('type','equidistant'),('scale_to_hfov','true'),('cutoff_angle',str(math.pi-1e-6)),('env_texture_size','512')]:ET.SubElement(lens,key).text=value
        cameras.append(dict(id=i,name=name,xyz=xyz,yaw=yaw,hfov_degrees=fov_deg,model=distortion,qCM=q.tolist(),resolution=[width,height],rovio_resolution=[rect_width,rect_height],rovio_hfov=rect_fov))
    (output/'rovio.info').write_text(template)
    (output/'camera_manifest.json').write_text(json.dumps(dict(cameras=cameras,calibration='Simulation-only nominal mounts; not physical-unit calibration',native_resolution=[1440,1080],simulation_scale=.5),indent=2)+'\n')
    root.write(output/'unipilot_vio.sdf',encoding='unicode',xml_declaration=True)
