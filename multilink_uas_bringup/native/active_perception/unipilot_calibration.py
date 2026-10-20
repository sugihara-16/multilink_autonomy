"""Use the generated UniPilot sensor geometry; no trajectory/truth calibration."""
import json,math,re,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from core import camera_rotation
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from rovio_initialization import match_accelerometer_bias_prior

def fusion_white_noise(config,model,profile='legacy'):
    """Measurement density only; leave bias evolution and fusion factors intact."""
    if profile not in ('legacy','sensor','sensor_gyro'):raise ValueError('Unknown fusion IMU noise profile')
    sensor=ET.parse(model).getroot().find(".//sensor[@name='imu']")
    rate=float(sensor.findtext('update_rate'))
    if not math.isfinite(rate) or rate<=0:raise ValueError('Invalid IMU rate')
    pre=config['imu']['preintegration'];report={'profile':profile,'rate_hz':rate,'axes':{},'previous':{},'applied':{}}
    for group,key in [('linear_acceleration','acc_noise_density'),('angular_velocity','gyro_noise_density')]:
        density=[]
        for axis in 'xyz':
            noise=sensor.find(f'imu/{group}/{axis}/noise')
            sigma=float(noise.findtext('stddev'));precision=float(noise.findtext('precision','0'))
            if not math.isfinite(sigma+precision) or sigma<0 or precision<0:raise ValueError('Invalid sensor measurement noise')
            density.append(math.sqrt((sigma*sigma+precision*precision/12)/rate))
        report['axes'][key]=density;report['previous'][key]=pre[key]
        # MIMOSA accepts one scalar per sensor kind; max covers all three axes.
        if profile=='sensor' or (profile=='sensor_gyro' and key=='gyro_noise_density'):
            if max(density)<=0:raise ValueError('Zero-noise sensor needs an explicit integration-error model')
            pre[key]=max(density)
        report['applied'][key]=pre[key]
    return report

def imu_runtime_parameters(model, world, mode='sensor'):
    """Physical sensor parameters only. Zero tau preserves historical random walk."""
    if mode not in ('sensor','legacy'):
        raise ValueError('Unknown IMU model')
    values=dict(acc_bias_tau=[0.]*3,gyro_bias_tau=[0.]*3,gravity_magnitude=9.81)
    if mode=='legacy':return values
    root=ET.parse(model).getroot()
    for group,key in [('linear_acceleration','acc_bias_tau'),('angular_velocity','gyro_bias_tau')]:
        for i,axis in enumerate('xyz'):
            noise=root.find(f".//sensor[@name='imu']/imu/{group}/{axis}/noise")
            if float(noise.findtext('dynamic_bias_stddev','0'))>0:
                tau=float(noise.findtext('dynamic_bias_correlation_time','0'))
                if not math.isfinite(tau) or tau<=0:raise ValueError('Invalid dynamic bias correlation time')
                values[key][i]=tau
    gravity=np.fromstring(ET.parse(world).getroot().findtext('.//world/gravity','0 0 -9.8'),sep=' ')
    if gravity.size!=3 or not np.isfinite(gravity).all() or np.linalg.norm(gravity)<=0:
        raise ValueError('Invalid world gravity')
    values['gravity_magnitude']=float(np.linalg.norm(gravity))
    return values

def prepare(out, noise_profile="sensor"):
    out=Path(out);here=Path(__file__).resolve().parent
    root=ET.parse(out/'unipilot_gimbal.sdf').getroot()
    mount=json.loads((out/'camera_mount.json').read_text())
    camera=root.find(".//sensor[@name='rgb_cam']/camera")
    width=int(camera.findtext('image/width'));height=int(camera.findtext('image/height'));hfov=float(camera.findtext('horizontal_fov'))
    fx=width/(2*math.tan(hfov/2))
    cfg=dict(camera_name='cam_gimbal',image_width=width,image_height=height,distortion_model='plumb_bob',
        camera_matrix=dict(rows=3,cols=3,data=[fx,0,width/2,0,fx,height/2,0,0,1]),
        distortion_coefficients=dict(rows=1,cols=5,data=[0.]*5))
    (out/'raw_cam0.yaml').write_text(yaml.safe_dump(cfg))
    if mount.get('processed_image_size'):
        width,height=mount['processed_image_size']
        fx=width/(2*math.tan(hfov/2))
        cfg.update(image_width=width,image_height=height,
            camera_matrix=dict(rows=3,cols=3,data=[fx,0,width/2,0,fx,height/2,0,0,1]))
    (out/'cam0.yaml').write_text(yaml.safe_dump(cfg))
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',mount['mount_rpy']).as_matrix();T[:3,3]=mount['mount_xyz']
    q=Rotation.from_matrix(camera_rotation(T[:3,:3],0,0)).inv().as_quat()
    text='\nCamera0\n{\n  CalibrationFile\n'
    for name,value in zip(('qCM_x','qCM_y','qCM_z','qCM_w','MrMC_x','MrMC_y','MrMC_z'),[*q,*T[:3,3]]):text+=f'  {name} {value:.16g};\n'
    text+='}\n'
    config,variance=match_accelerometer_bias_prior((here/'rovio_template.info').read_text()+text,out/'unipilot_gimbal.sdf',sensor_name='imu')
    # DRAGON's almost noiseless gyro prior is invalid for UniPilot's specified
    # random gyro bias. Use only the SDF noise model, never truth/error fitting.
    gyro_variance=[]
    imu=root.find(".//sensor[@name='imu']");rate=float(imu.findtext('update_rate'))
    for kind,state,pred in [('angular_velocity','gyb','att'),('linear_acceleration','acb','vel')]:
        for i,axis in enumerate('xyz'):
            noise=imu.find(f'imu/{kind}/{axis}/noise')
            bias=float(noise.findtext('bias_mean'))**2+float(noise.findtext('bias_stddev'))**2
            if state=='gyb':
                gyro_variance.append(bias)
                a=config.index('    Covariance\n');b=config.index('ImgUpdate\n',a)
                block=re.sub(rf'(gyb_{i}\s+)[^;\s]+;',lambda m:m[1]+format(bias,'.12g')+';',config[a:b],count=1)
                config=config[:a]+block+config[b:]
            # ROVIO's PredictionNoise is continuous-time covariance density.
            # Discrete rate-sample variance = density / sampling interval.
            white=float(noise.findtext('stddev'))**2+float(noise.findtext('precision','0'))**2/12
            density=white/rate
            a=config.index('Prediction\n');b=config.index('PoseUpdate\n',a)
            block=config[a:b]
            block=re.sub(rf'({pred}_{i}\s+)[^;\s]+;',lambda m:m[1]+format(density,'.12g')+';',block,count=1)
            driving=float(noise.findtext('dynamic_bias_stddev','0'))**2
            block=re.sub(rf'({state}_{i}\s+)[^;\s]+;',lambda m:m[1]+format(driving,'.12g')+';',block,count=1)
            config=config[:a]+block+config[b:]
    if noise_profile not in ('sensor','dragon'):
        raise ValueError('Unknown ROVIO noise profile')
    if noise_profile=='dragon':
        reference=json.loads((here.parent/'sensor_profiles/dragon_rovio_noise.json').read_text())
        gyro_variance=reference['initial_gyro_covariance']
        a=config.index('    Covariance\n');b=config.index('ImgUpdate\n',a)
        block=config[a:b]
        for i,v in enumerate(gyro_variance):
            block=re.sub(rf'(gyb_{i}\s+)[^;\s]+;',lambda m:m[1]+format(v,'.12g')+';',block,count=1)
        config=config[:a]+block+config[b:]
        a=config.index('Prediction\n');b=config.index('PoseUpdate\n',a);block=config[a:b]
        for key,values in reference['prediction'].items():
            for i,v in enumerate(values):
                block=re.sub(rf'({key}_{i}\s+)[^;\s]+;',lambda m:m[1]+format(v,'.12g')+';',block,count=1)
        config=config[:a]+block+config[b:]
        (out/'rovio_noise_profile.json').write_text(json.dumps(reference,indent=2)+'\n')
    (out/'rovio.info').write_text(config)
    (out/'camera_manifest.json').write_text(json.dumps(dict(robot='unipilot',expected_joints={},cameras=[dict(T_FC_camera=T.tolist(),topic='/rmf_unipilot/cam_gimbal/image')]),indent=2)+'\n')
    fusion=yaml.safe_load((here/'mimosa.yaml').read_text())
    lidar=np.array(list(map(float,root.findtext(".//sensor[@name='lidar']/pose").split())))
    fusion['lidar']['T_B_S']=[*map(float,lidar[:3]),*map(float,Rotation.from_euler('xyz',lidar[3:]).as_quat())]
    planner=yaml.safe_load((here/'config.yaml').read_text())
    limits=np.deg2rad([mount['gimbal_limits_deg'][a] for a in ('yaw','pitch')])
    planner.update(limits_min=(-limits).tolist(),limits_max=limits.tolist(),bearing_sigma=1/fx,
        tan_half_hfov=width/(2*fx),tan_half_vfov=height/(2*fx),range_max=float(camera.findtext('clip/far')))
    (out/'planner.yaml').write_text(yaml.safe_dump(planner))
    return fusion,dict(accelerometer=variance,gyroscope=gyro_variance,noise_profile=noise_profile,source=('DRAGON reference prediction noise and gyro prior; sensor-derived accelerometer prior' if noise_profile=='dragon' else 'UniPilot SDF bias second moments and sample noise density'))
