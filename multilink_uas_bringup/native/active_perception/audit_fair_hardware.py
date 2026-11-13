"""Audit generated mass/sensors/initial camera pose and actual preparation."""
import argparse,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from compact_record_io import record_path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from dragon_articulated import Kinematics,sensor_spec

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root
    protocol=json.loads((root/'protocol.json').read_text());reference=None;rows=[]
    for task in protocol['tasks']:
        out=root/task['name'];row=dict(name=task['name'],checks={})
        try:
            model=ET.parse(out/'dragon_slam.urdf').getroot();kin=Kinematics(out/'dragon_slam.urdf')
            home=np.r_[kin.home,0.,0.] if len(kin.names)>6 else kin.home
            inertias={l.get('name'):ET.tostring(l.find('inertial'),encoding='unicode') for l in model.findall('link') if l.find('inertial') is not None}
            sensors={name:sensor_spec(model.find(f".//sensor[@name='{name}']")) for name in ('end_camera','end_lidar','spinal_imu')}
            current=dict(inertias=inertias,sensors=sensors,camera=kin.camera(home),lidar=model.findtext(".//sensor[@name='end_lidar']/pose"))
            if reference is None:reference=current
            row['checks'].update(inertias=current['inertias']==reference['inertias'],sensor_physics=current['sensors']==reference['sensors'],
                initial_camera_pose=bool(np.allclose(current['camera'],reference['camera'],atol=1e-12,rtol=0)),lidar_mount=current['lidar']==reference['lidar'])
            row['total_mass_kg']=sum(float(l.find('inertial/mass').get('value')) for l in model.findall('link') if l.find('inertial/mass') is not None)
            events={e['stage']:e['time'] for e in json.loads((out/'flight_events.json').read_text())}
            row['approach_s']=events['preparation']-events['approach'];row['excitation_s']=events['active']-events['preparation']
            row['checks']['same_preparation_durations']=abs(row['approach_s']-12)<.15 and abs(row['excitation_s']-52)<.15
            q=np.loadtxt(record_path(out/'joint_measurements.csv'),delimiter=',',skiprows=1,ndmin=2)
            hold=q[(q[:,0]>=events['preparation']+28)&(q[:,0]<events['active'])][::10]
            if len(hold)<2:raise ValueError('No second-cycle joint measurements')
            cams=Rotation.from_matrix([kin.camera(x[1:])[:3,:3] for x in hold])
            row['second_cycle_camera_hold_excursion_deg']=float(np.rad2deg((cams[0].inv()*cams).magnitude()).max())
            row['checks']['second_cycle_view_held']=row['second_cycle_camera_hold_excursion_deg']<1.
            if 'bootstrap' in events:
                row['bootstrap_s']=events['approach']-events['bootstrap']
                row['checks']['common_bootstrap_duration']=abs(row['bootstrap_s']-26)<.15
                initial=q[(q[:,0]>=events['bootstrap'])&(q[:,0]<events['approach'])][::10]
                row['checks']['bootstrap_fixed_home']=len(initial)>2 and bool(np.max(abs(initial[:,1:]-home))<np.deg2rad(.5))
            first=q[(q[:,0]>=events['approach'])&(q[:,0]<events['preparation']+26)][::10]
            cams=Rotation.from_matrix([kin.camera(x[1:])[:3,:3] for x in first])
            row['first_cycle_camera_excursion_deg']=float(np.rad2deg((cams[0].inv()*cams).magnitude()).max())
        except (OSError,ValueError,KeyError,IndexError) as e:row['error']=str(e);row['checks']['complete']=False
        row['passed']=all(row['checks'].values());rows.append(row)
    result=dict(passed=all(r['passed'] for r in rows),trials=rows,
        scope='Identical generated link inertias, camera/IMU/LiDAR specifications, initial optical pose and LiDAR mounting. Actual shared preparation timing and held second-cycle view; the selected view can differ by mechanism/policy.')
    (root/'hardware_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],failures={r['name']:r for r in rows if not r['passed']}),indent=2))

if __name__=='__main__':main()
