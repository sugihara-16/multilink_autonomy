#!/usr/bin/env python3
"""Small, transparent paired-flight report; no raw images required."""
import argparse,json,html,csv,gzip
import copy
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from report_estimator_validation import evaluate
from report_routes import load,csv_path
from simulation_shutdown import gazebo_exit_status, launch_exit_status
from sensor_bias_parameters import parameters as sensor_bias_parameters

def summarize(campaign,expected_drpm=False):
    results={}
    for base in sorted(campaign.glob('*_active')):
        if not (base/'runtime.json').exists():continue
        runtime=json.loads((base/'runtime.json').read_text())
        if runtime['status']!='route_complete':
            results[base.name]={'valid':False,'runtime':runtime};continue
        proto=json.loads((base/'protocol.json').read_text())
        fixed=base/'fixed'
        checks={
            'stopped':runtime['stopped'],
            'processes_exit_zero':all(v==0 for v in runtime['processes'].values()),
            'gazebo_child_exit_zero':gazebo_exit_status((base/'simulation.log').read_text(errors='replace'))['all_clean'],
            'launch_children_exit_zero':launch_exit_status((base/'simulation.log').read_text(errors='replace'))['all_clean'],
            'same_rovio_parameters':(base/'rovio.info').read_bytes()==(fixed/'rovio.info').read_bytes(),
            'same_camera_intrinsics':(base/'cam0.yaml').read_bytes()==(fixed/'cam0.yaml').read_bytes(),
            'no_truth_estimator_input':not proto['truth_input_to_estimators'],
            ('drpm_enabled' if expected_drpm else 'drpm_disabled'):proto['drpm_fusion']==expected_drpm,
        }
        fusion=[]
        for path in (base,fixed):
            c=yaml.safe_load((path/'mimosa.yaml').read_text());c.pop('logs_directory');fusion.append(c)
        checks['same_fusion_parameters']=fusion[0]==fusion[1]
        if proto.get('joint_rovio_information'):
            manifest=json.loads((base/'implementation/joint_rovio_build.json').read_text())['sha256']
            expected={Path(k).name:v for k,v in manifest.items()}
            checks['joint_rovio_declared_binary']=proto['binary_sha256']['rovio_gimbal']==expected['rovio_gimbal']
        if proto.get('batched_fusion'):
            manifest=json.loads((base/'implementation/batched_fusion_build.json').read_text())['sha256']
            expected={Path(k).name:v for k,v in manifest.items()}
            checks['batched_fusion_declared_binaries']=bool(proto['velocity_fusion'] and proto['sensor_bias_model'] and
                proto['binary_sha256']['mimosa_node']==expected['mimosa_batched_node'] and
                proto['binary_sha256']['libmimosa_lib.so']==expected['libmimosa_lib.so'])
        if proto.get('fusion_initial_velocity_sigma') is not None:
            expected=proto['fusion_initial_velocity_sigma']
            checks['declared_initial_velocity_uncertainty']=bool(np.isfinite(expected) and expected>0 and
                all(c['graph']['manager']['smoother']['initial_velocity_sigma']==expected for c in fusion))
        if proto.get('lidar_four_dof'):
            scan=fusion[0]['lidar']['geometric']['scan_to_map']
            checks['four_dof_configuration']=bool(scan['reg_4_dof'] and not scan['project_on_degneneracy'] and not scan['drpm_enabled'] and proto['fixed_gravity_imu'] and proto['tilt_fusion'])
        if proto.get('fusion_imu_noise','legacy')!='legacy':
            from unipilot_calibration import fusion_white_noise
            calibrated=copy.deepcopy(fusion[0])
            expected=fusion_white_noise(calibrated,base/'unipilot_gimbal.sdf',proto['fusion_imu_noise'])['applied']
            checks['imu_white_noise_matches_sensor']=all(abs(fusion[0]['imu']['preintegration'][k]-v)<=max(1e-14,abs(v)*1e-12) for k,v in expected.items())
        if expected_drpm:
            checks['adopted_drpm_configuration']=all(
                c['lidar']['geometric']['scan_to_map']['drpm_enabled']
                and c['lidar']['geometric']['scan_to_map']['drpm_registration_mode']==1
                and c['lidar']['geometric']['scan_to_map']['drpm_noise_model']==2
                for c in fusion)
        route=load(base/'route_tracking.csv');tracking=route['route_t']>0
        begin=float(route['sim_t'][tracking][0]);end=float(route['sim_t'][tracking][-1])
        decisions=[json.loads(line) for line in (base/'decisions.jsonl').read_text().splitlines()]
        times=np.array([d['stamp_ns']*1e-9 for d in decisions])
        inside=times[(times>=begin)&(times<=end)]
        controller=dict(decisions=len(inside),max_gap_s=float(np.diff(np.r_[begin,inside,end]).max()),
                        exception_in_log=any(s in (base/'planner.log').read_text() for s in ['Traceback','Exception in thread']))
        planner=yaml.safe_load((base/'planner.yaml').read_text());limit=float(planner['max_dt'])
        checks['planner_continuous']=controller['max_gap_s']<=limit+.001 and not controller['exception_in_log']
        enc=load(base/'encoders.csv');angles=np.c_[enc['yaw'],enc['pitch']];dt=np.diff(enc['t']);positive=dt>0
        actuation=dict(min_angle_deg=np.rad2deg(angles.min(axis=0)).tolist(),max_angle_deg=np.rad2deg(angles.max(axis=0)).tolist(),
            max_speed_deg_s=float(np.rad2deg((np.linalg.norm(np.diff(angles,axis=0),axis=1)[positive]/dt[positive]).max())))
        if proto.get('fixed_angles_deg') is not None:
            # This is a static mounting-angle control, not a slewing trial.
            # The simulator's spawn transient precedes both estimators. Keep
            # its peak visible, and require the requested angle to be settled
            # from the beginning of IMU initialization through the whole run.
            start=min(load(p/'rovio.csv')['stamp_ns'][0]*1e-9 for p in (base,fixed))-proto['stationary_initialization_duration']
            established=enc['t']>=start
            checks['fixed_view_settled_before_initialization']=bool(established.any() and np.all(np.abs(angles[established]-np.deg2rad(proto['fixed_angles_deg']))<1e-4))
            mask=positive&(enc['t'][:-1]>=start)
            actuation['full_simulation_max_speed_deg_s']=actuation['max_speed_deg_s']
            actuation['speed_check_start_s']=float(start)
            actuation['max_speed_deg_s']=float(np.rad2deg((np.linalg.norm(np.diff(angles,axis=0),axis=1)[mask]/dt[mask]).max()))
        checks['gimbal_physical_limits']=bool(np.all(angles>=np.asarray(planner['limits_min'])-1e-6) and np.all(angles<=np.asarray(planner['limits_max'])+1e-6))
        checks['gimbal_speed_limit']=actuation['max_speed_deg_s']<=float(np.rad2deg(planner['max_speed']))+.01
        if planner.get('match_actuator_reachability',False):
            from actuator_reachability import parameters as actuator_parameters
            # Old saved flights used one step; do not reinterpret them using
            # the operating clearance selected by a later implementation.
            expected=actuator_parameters(base/'unipilot_gimbal.sdf',base/'world_serial.sdf',planner.get('joint_stop_steps',1))
            checks['planner_matches_actual_actuator']=all(np.array_equal(planner.get(k,1 if k=='joint_stop_steps' else None),v) for k,v in expected.items())
        if planner.get('fixation_predictive_geometry',False):
            checks['predictive_geometry_export_selected']=bool(proto.get('geometry_rovio_information') and proto.get('joint_rovio_information'))
        if planner.get('fixation_lidar_conditioned_velocity',False):
            checks['planner_matches_fusion_calibration']=bool(
                planner['fusion_velocity_covariance_scale']==fusion[0]['odometry']['manager']['velocity_covariance_scale']
                and planner['fusion_velocity_period_s']==fusion[0]['odometry']['manager']['velocity_period_s'])
        retention={}
        if planner.get('direct_track_input',False) and proto.get('fixed_angles_deg') is None:
            selected=[d for d in decisions if begin<=d['stamp_ns']*1e-9<=end]
            reports=[d.get('motion_alignment',{}).get('tracks') for d in selected]
            checks['current_tracks_aligned']=bool(selected) and all(v is not None and -2e-9<=v['age_s']<=planner['max_sensor_age'] and v['count']==d['image_tracks'] for d,v in zip(selected,reports))
            if checks['current_tracks_aligned']:
                retention=dict(age_p50_p95_max_s=np.percentile([v['age_s'] for v in reports],[50,95,100]).tolist(),
                    current_count_p10_p50_p90=np.percentile([v['count'] for v in reports],[10,50,90]).tolist(),
                    truly_empty_fraction=float(np.mean([v['count']==0 for v in reports])))
            if planner.get('track_lidar_depth',False):
                depths=[v.get('lidar_depth') if v is not None else None for v in reports]
                checks['track_depth_counts_valid']=bool(depths) and all(
                    d is not None and d['total']==v['count'] and 0<=d['supported']<=d['total']
                    for d,v in zip(depths,reports))
                matched=[(d,v) for d,v in zip(depths,reports) if d is not None and d['supported']>0]
                checks['track_depth_acquisition_alignment']=all(
                    abs(d['acquisition_encoder_stamp']-v['stamp'])<=2e-9
                    and d['scan_age_s'] is not None and 0<=d['scan_age_s']<.5
                    and .3<d['range_min_median_max_m'][0]<=d['range_min_median_max_m'][-1]<planner['range_max']
                    for d,v in matched)
                retention['lidar_depth']=dict(decisions=len(depths),supported_decisions=len(matched),
                    supported_points=sum(d['supported'] for d in depths if d is not None),
                    acquisition_time_checked=True)
        audit=json.loads((base/'input_audit.json').read_text())
        checks['raw_no_missing']=all(v['missing_periods']==0 and v['duplicates']==0 and v.get('reordered',0)==0 for v in audit.values())
        shared=None
        if proto.get('shared_initialization'):
            shared=json.loads((base/'shared_initialization.json').read_text())
            checks['shared_relay_continuous']=shared['missing_periods']==0
            if all(csv_path(p/'vio_diagnostic.csv').exists() for p in (base,fixed)):
                streams=[np.genfromtxt(csv_path(p/'vio_diagnostic.csv'),delimiter=',',names=True) for p in (base,fixed)]
                until=shared['switch_stamp_ns'] or shared['last_stamp_ns']+1
                start=max(v['stamp_ns'][0] for v in streams)
                streams=[v[(v['stamp_ns']>=start)&(v['stamp_ns']<until)] for v in streams]
                shared['common_vio_samples']=len(streams[0])
                shared['common_vio_byte_equal']=streams[0].tobytes()==streams[1].tobytes()
                checks['common_vio_initial_state_identical']=bool(len(streams[0]) and shared['common_vio_byte_equal'])
            if proto.get('ordered_fusion'):
                streams=[np.genfromtxt(csv_path(p/'mimosa.csv'),delimiter=',',names=True) for p in (base,fixed)]
                until=shared['switch_stamp_ns'] or shared['last_stamp_ns']+1
                times,i,j=np.intersect1d(streams[0]['stamp_ns'],streams[1]['stamp_ns'],return_indices=True)
                i=i[times<until];j=j[times<until]
                shared['common_fusion_samples']=len(i)
                shared['common_fusion_byte_equal']=streams[0][i].tobytes()==streams[1][j].tobytes()
                checks['common_fusion_initial_state_identical']=bool(len(i) and shared['common_fusion_byte_equal'])
        stats={};gaps={}
        for mode,path in [('fixed',fixed),('active',base)]:
            stats[mode]={}
            for estimator in ('rovio','mimosa'):
                stats[mode][estimator],_,_,_=evaluate(path,estimator)
                checks[mode+'_'+estimator+'_continuous']=stats[mode][estimator]['max_output_gap_s']<(.05001 if estimator=='rovio' else .10001)
            if csv_path(path/'vio_diagnostic.csv').exists():
                data=load(path/'vio_diagnostic.csv')
                gaps[mode]=dict(samples=len(data['stamp_ns']),max_gap_s=float(np.diff(data['stamp_ns']).max()*1e-9))
        ordered={}
        if proto.get('ordered_fusion'):
            for mode,path in [('fixed',fixed),('active',base)]:
                ordered_path=csv_path(path/'mimosa/ordered_inputs.csv')
                with (gzip.open(ordered_path,'rt') if ordered_path.suffix=='.gz' else ordered_path.open()) as f:rows=list(csv.DictReader(f))
                stamps=np.array([int(r['stamp_ns']) for r in rows],dtype=np.int64)
                report={'event_count':len(rows),'chronological':bool(np.all(np.diff(stamps)>=0)),'sources':{}}
                for source,period in [('lidar',100000000),('vio',50000000)]:
                    t=np.array([int(r['stamp_ns']) for r in rows if r['source']==source],dtype=np.int64)
                    report['sources'][source]={'count':len(t),'max_gap_ns':int(np.diff(t).max()) if len(t)>1 else None,
                        'continuous':bool(len(t)>1 and np.all(np.abs(np.diff(t)-period)<=2))}
                if rows and 'imu_upto_ns' in rows[0]:
                    endpoints=np.array([int(r['imu_upto_ns']) for r in rows],dtype=np.int64)
                    valid=endpoints>0
                    report['max_imu_interpolation_boundary_ns']=int((endpoints[valid]-stamps[valid]).max())
                    report['imu_samples_processed']=sum(int(r['imu_count']) for r in rows)
                    report['imu_prefix_monotonic']=bool(np.all(np.diff(endpoints)>=0))
                    report['imu_brackets_events']=bool(np.all((endpoints[valid]-stamps[valid]>=0)&(endpoints[valid]-stamps[valid]<=1000000)))
                    checks[mode+'_ordered_imu_prefix']=report['imu_prefix_monotonic'] and report['imu_brackets_events']
                checks[mode+'_ordered_factors_continuous']=report['chronological'] and all(v['continuous'] for v in report['sources'].values())
                ordered[mode]=report
        if proto.get('fixed_gravity_imu'):
            for mode,path in [('fixed',fixed),('active',base)]:
                data=load(path/'gravity_state.csv');ts=data['stamp_ns']*1e-9
                g=np.column_stack([data['gravity_'+axis] for axis in 'xyz']);expected=[0.,0.,-float(np.float32(fusion[0]['imu']['preintegration']['gravity_magnitude']))]
                checks[mode+'_fixed_gravity_honored']=bool(len(ts)>2 and np.max(np.abs(g-expected))<1e-8)
                checks[mode+'_gravity_audit_coverage']=bool(len(ts)>2 and np.diff(ts).max()<1.051 and ts[0]<begin and ts[-1]>end-1.051)
        if proto.get('sensor_bias_model'):
            model=sensor_bias_parameters(base/'unipilot_gimbal.sdf')
            checks['bias_parameters_match_sensor']=all(fusion[0]['imu']['preintegration'].get(k)==v for k,v in model.items())
            tau=np.array(model['bias_tau_s']);driving=np.array(model['bias_driving_sigma'])
            for mode,path in [('fixed',fixed),('active',base)]:
                data=load(path/'mimosa/imu_bias_model.csv');ts=data['stamp_ns']*1e-9;dt=data['dt'][:,None]
                phi=np.column_stack([data['phi'+str(i)] for i in range(6)])
                sigma=np.column_stack([data['sigma'+str(i)] for i in range(6)])
                checks[mode+'_bias_factor_audit_coverage']=bool(len(ts)>2 and np.diff(ts).max()<1.051 and ts[0]<begin and ts[-1]>end-1.051)
                checks[mode+'_bias_factor_matches_sensor']=bool(np.all(dt>0) and np.isfinite(phi).all() and np.isfinite(sigma).all()
                    and np.allclose(phi,np.exp(-dt/tau),atol=1e-14,rtol=1e-12)
                    and np.allclose(sigma,driving*np.sqrt(-tau/2*np.expm1(-2*dt/tau)),atol=1e-14,rtol=1e-12))
        if proto.get('tilt_fusion'):
            for mode,path in [('fixed',fixed),('active',base)]:
                data=load(path/'mimosa/tilt_factors.csv');ts=data['stamp_ns']*1e-9
                manager=fusion[0]['odometry']['manager']
                checks[mode+'_tilt_factors_continuous']=bool(len(ts)>2 and np.all(np.abs(np.diff(ts)-manager['tilt_period_s'])<.050001)
                    and ts[0]<begin and ts[-1]>end-1.05)
                up=np.column_stack([data['up_'+axis] for axis in 'xyz'])
                checks[mode+'_tilt_unit_direction']=bool(np.all(np.isfinite(up)) and np.allclose(np.linalg.norm(up,axis=1),1.))
                checks[mode+'_tilt_noise_matches']=bool(np.allclose(data['sigma_rad'],manager['tilt_sigma_rad']))
        if proto.get('velocity_fusion'):
            for mode,path in [('fixed',fixed),('active',base)]:
                data=load(path/'mimosa/velocity_factors.csv');ts=data['stamp_ns']*1e-9
                manager=fusion[0]['odometry']['manager']
                C=np.array([data['c'+str(i)+str(j)] for i in range(3) for j in range(3)]).T.reshape(-1,3,3)
                checks[mode+'_velocity_factors_continuous']=bool(len(ts)>2 and np.all(np.abs(np.diff(ts)-manager['velocity_period_s'])<.050001) and ts[0]<begin and ts[-1]>end-1.05)
                checks[mode+'_velocity_covariance_valid']=bool(np.isfinite(C).all() and np.linalg.eigvalsh(C).min()>0 and np.allclose(C,C.transpose(0,2,1)))
                checks[mode+'_velocity_calibration_matches']=bool(np.allclose(data['covariance_scale'],manager['velocity_covariance_scale']))
        ratio={k:stats['active'][k]['position_rmse_m']/stats['fixed'][k]['position_rmse_m'] for k in ('rovio','mimosa')}
        results[base.name]=dict(valid=all(checks.values()),checks=checks,seed=proto['seed'],
            comparison_kind='fixed_orientation' if proto.get('fixed_angles_deg') is not None else 'dynamic_gimbal',
            test_angles_deg=proto.get('fixed_angles_deg'),metrics=stats,active_fixed_position_ratio=ratio,input_audit=audit,shared_initialization=shared,diagnostic_output=gaps,ordered_inputs=ordered,controller_timing=controller,gimbal_actuation=actuation,retention_input=retention,wall_s=runtime['wall_s'])
    return results

def main():
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('--plots',action='store_true')
    p.add_argument('--drpm-fusion',action='store_true',help='Require adopted DRPM settings in both estimators')
    a=p.parse_args()
    results=summarize(a.campaign,a.drpm_fusion)
    (a.campaign/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    for label,r in results.items():
        print(label,'valid=',r['valid'])
        for mode,stats in r.get('metrics',{}).items():print(mode,{k:tuple(round(v[n],3) for n in ('position_rmse_m','rotation_rmse_deg')) for k,v in stats.items()})
        if not r['valid']:print(r.get('checks',r.get('runtime')))
    if not a.plots:return
    plotdir=a.campaign/'plots';plotdir.mkdir(exist_ok=True);images=[]
    with PdfPages(plotdir/'comparison.pdf') as pdf:
        for label,r in results.items():
            if 'metrics' not in r:continue
            base=a.campaign/label
            for estimator in ('rovio','mimosa'):
                fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
                for mode,path,color in [('fixed',base/'fixed','#2474b5'),('active',base,'#df6619')]:
                    _,grid,err,_=evaluate(path,estimator)
                    for i,ax in enumerate([*axs[:,0],*axs[:,1]]):
                        legend=mode
                        if r.get('comparison_kind')=='fixed_orientation':
                            legend='fixed yaw/pitch '+str(r['test_angles_deg'] if mode=='active' else [0,0])+' deg'
                        ax.plot(grid,err[:,i],label=legend,color=color,lw=.8)
                        ax.set_ylabel(['x [m]','y [m]','z [m]','roll [deg]','pitch [deg]','yaw [deg]'][i]);ax.grid(alpha=.3)
                axs[0,0].legend();axs[-1,0].set_xlabel('Trajectory time [s]');axs[-1,1].set_xlabel('Trajectory time [s]')
                fig.suptitle(label+' / '+estimator+' / seed '+str(r['seed'])+'\nSame physical flight; first-pose alignment, no scale fit')
                fig.tight_layout();name=label+'_'+estimator+'.png';fig.savefig(plotdir/name,dpi=120);pdf.savefig(fig);plt.close(fig);images.append(name)
    rows=[]
    for label,r in results.items():
        for mode,s in r.get('metrics',{}).items():
            rows.append('<tr>'+''.join('<td>'+str(v)+'</td>' for v in [html.escape(label),mode,r['valid'],*[f'{s[k][n]:.3f}' for k in ('rovio','mimosa') for n in ('position_rmse_m','rotation_rmse_deg')]])+'</tr>')
    body='<!doctype html><meta charset="utf-8"><title>Gimbal paired comparison</title><style>body{font-family:sans-serif;margin:30px auto;max-width:1150px}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}img{width:100%}</style><h1>固定・可動の同時比較</h1><p>同じ物理飛行、同じIMU、同じ推定器設定。DRPM OFF。軌道追従区間のみ、開始時の位置・姿勢で整合し、尺度補正なし。未完了や監査不合格は除外せずJSONに残します。</p><table><tr><th>run</th><th>mode</th><th>監査</th><th>ROVIO位置 m</th><th>ROVIO回転 deg</th><th>融合位置 m</th><th>融合回転 deg</th></tr>'+''.join(rows)+'</table><p><a href="plots/comparison.pdf">六軸比較PDF</a> / <a href="results.json">数値・検証記録</a></p>'+''.join('<img src="plots/'+im+'">' for im in images)
    if a.drpm_fusion:body=body.replace('DRPM OFF。','DRPM ON（採用済みの更新方法＋測距方向の誤差モデル、両側共通）。')
    if any(r.get('comparison_kind')=='fixed_orientation' for r in results.values()):
        body=body.replace('<h1>固定・可動の同時比較</h1>',
            '<h1>固定角度の比較（動的偏向の実験ではありません）</h1><p>表のactiveは保存形式上の呼称です。試験側は指定角度に固定し、reference側はyaw/pitchとも0度です。初期化時から視野が異なるため、初期化への影響も含みます。</p>')
    (a.campaign/'index.html').write_text(body)
if __name__=='__main__':main()
