"""Small acceptance report; truth is used only after the flight has finished."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

def read_csv(path):
    a=np.loadtxt(path,delimiter=',',skiprows=1,ndmin=2)
    if len(a)<2:raise ValueError('Insufficient data: '+str(path))
    return a

def report(out):
    out=Path(out);events=json.loads((out/'flight_events.json').read_text())
    phases={v['stage']:v['time'] for v in events};start=phases['active'];end=phases['landing']
    protocol=json.loads((out/'protocol.json').read_text())
    end=min(end,start+protocol['distance_m']/protocol.get('speed_mps',.3)+2.) if protocol['distance_m'] else end
    truth=read_csv(out/'truth.csv');truth[:,0]*=1e-9
    truth=truth[np.r_[True,np.diff(truth[:,0])>0]]
    results={}
    for name in ('rovio','mimosa'):
        a=read_csv(out/(name+'.csv'));a[:,0]*=1e-9
        a=a[(a[:,0]>=max(start,truth[0,0]))&(a[:,0]<=min(end,truth[-1,0]))]
        pos=np.column_stack([np.interp(a[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)])
        rot=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))(a[:,0])
        R=rot[0]*Rotation.from_quat(a[0,4:8]).inv()
        aligned=R.apply(a[:,1:4]-a[0,1:4])+pos[0]
        m=(a[:,0]>=start)&(a[:,0]<=end);err=np.linalg.norm(aligned-pos,axis=1)
        angle=(rot.inv()*R*Rotation.from_quat(a[:,4:8])).magnitude()*180/np.pi
        results[name]=dict(count=len(a),evaluated_count=int(m.sum()),max_gap_s=float(np.max(np.diff(a[m,0]))),
            position_rmse_m=float(np.sqrt(np.mean(err[m]**2))),position_max_m=float(err[m].max()),
            rotation_rmse_deg=float(np.sqrt(np.mean(angle[m]**2))))
    flight=read_csv(out/'flight.csv');active=flight[(flight[:,0]>=phases['active'])&(flight[:,0]<end)]
    decisions=[json.loads(l) for l in (out/'joint_decisions.jsonl').read_text().splitlines()]
    encoder=json.loads((out/'encoder_audit.json').read_text());inputs=json.loads((out/'input_audit.json').read_text())
    model=json.loads((out/'articulated_manifest.json').read_text())
    from dragon_articulated import Kinematics
    kin=Kinematics(out/'dragon_slam.urdf');gimbal=len(kin.names)>6
    T0=kin.camera(np.r_[kin.home,0.,0.] if gimbal else kin.home)
    q=flight[(flight[:,0]>=start)&(flight[:,0]<end),4:]
    camera=[kin.camera(v) for v in q[::4]]
    translation_span=np.ptp([T[:3,3] for T in camera],axis=0)
    rotation=max(Rotation.from_matrix(T0[:3,:3].T@T[:3,:3]).magnitude() for T in camera)*180/np.pi
    optical_angle=max(np.arccos(np.clip(T0[:3,2]@T[:3,2],-1.,1.)) for T in camera)*180/np.pi
    commands=sum(d['reason'] in ('select_global_view','follow_joint_path') for d in decisions)
    mode=json.loads((out/'protocol.json').read_text())['mode'] if (out/'protocol.json').exists() else 'active'
    requested=json.loads((out/'flight_result.json').read_text())['requested_distance_m']
    active_fc=truth[(truth[:,0]>=start)&(truth[:,0]<=end)]
    travel=float(np.linalg.norm(active_fc[-1,1:4]-active_fc[0,1:4]))
    speeds=np.abs(np.diff(active[:,4:],axis=0)/np.diff(active[:,0])[:,None])
    max_speed=float(np.max(speeds))*180/np.pi
    rotation_gt=Rotation.from_quat(truth[:,4:8]);ii=(truth[:,0]>=start)&(truth[:,0]<end)
    attitude_excursion=float((rotation_gt[np.flatnonzero(ii)[0]].inv()*rotation_gt[ii]).magnitude().max()*180/np.pi)
    checks=dict(flight=json.loads((out/'flight_result.json').read_text())['passed'],
                sensor_parity=all(model['parity'].values()) and model['flight_controller_magnetometer']['parity'],
                imu_complete=inputs['imu']['missing_periods']==0,lidar_complete=inputs['lidar']['missing_periods']==0,
                images_continuous=encoder['max_gap_ns']<=50000000,
                estimators_running=all(r['evaluated_count']>200 and r['max_gap_s']<.5 for r in results.values()),
                moving_camera_observed=(rotation>5 and (gimbal or max(translation_span)>.02)) if mode=='active' else True,
                autonomous_view_selection=commands>0 if mode=='active' else True,
                controlled_flight=abs(travel-requested)<.5 and np.max(abs(active_fc[:,3]-2.))<.3,
                joint_slew=max_speed<(21. if gimbal else 12.))
    if gimbal:
        checks['body_joints_fixed']=bool(np.max(np.ptp(flight[:,4:10],axis=0))<np.deg2rad(1.))
    checks={k:bool(v) for k,v in checks.items()}
    result=dict(passed=all(checks.values()),checks=checks,estimators=results,
                evaluation_interval_s=[start,end],alignment='First tracking pose SE3 alignment; no scale fitting; takeoff and preparation excluded',
                active_travel_m=travel,max_measured_joint_speed_deg_s=max_speed,lidar_carrier_attitude_excursion_deg=attitude_excursion,
                active_altitude_range_m=[float(active_fc[:,3].min()),float(active_fc[:,3].max())],
                joint_span_deg=(np.ptp(q,axis=0)*180/np.pi).tolist(),camera_rotation_excursion_deg=rotation,camera_optical_axis_excursion_deg=float(optical_angle),
                camera_translation_span_m=translation_span.tolist(),all_flight_camera_translation_span_m=encoder['translation_span_m'],decisions=len(decisions),movement_decisions=commands,
                sensor_counts={'imu':inputs['imu']['count'],'lidar':inputs['lidar']['count'],'images':encoder['paired']},
                scope='Fixed-square DRAGON with camera gimbal' if gimbal else 'Observed-map multi-joint view search; geometric bounds, full dynamics verified on this flight only')
    (out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
    print(json.dumps(report(sys.argv[1]),indent=2))
