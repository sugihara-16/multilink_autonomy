#!/usr/bin/env python3
"""Audit conditional fusion experiments and diagnose directional complementarity.

Visual information below is a local feature-geometry proxy, not the covariance
of the fused estimator. Truth is used only for evaluation, never sensor fusion.
"""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import numpy as np
import yaml
from scipy.linalg import cho_factor, cho_solve
from scipy.spatial.transform import Rotation
from report_estimator_validation import evaluate
from report_fusion_sidecar import tum, ordered
from report_gimbal_campaign import summarize
from report_routes import load, poses, interp, csv_path
from diagnose_joint_fusion_consistency import describe
from core import Fisher, lidar_model
from joint_geometry import decode

HERE = Path(__file__).resolve().parent


def reading(path):
    p = csv_path(path)
    return gzip.open(p, 'rt') if p.suffix == '.gz' else p.open()


def rows(path):
    with reading(path) as f:
        return list(csv.DictReader(f))


def evaluate_tum(root, flight, path):
    with tempfile.TemporaryDirectory(dir=root, prefix='evaluation_') as d:
        p = Path(d)
        for name in ['protocol.json', 'truth.csv', 'route_tracking.csv']:
            source = csv_path(flight/name)
            (p/source.name).symlink_to(source)
        a = tum(path).copy()
        a[:, 0] = np.rint(a[:, 0]*1e9)
        np.savetxt(p/'mimosa.csv', a, delimiter=',',
                   header='stamp_ns,x,y,z,qx,qy,qz,qw', comments='',
                   fmt=['%.0f']+['%.17g']*7)
        return evaluate(p, 'mimosa')


def audit(root):
    f = root/'campaign/run2a_prefix_active'
    o = root/'observer'
    manifest = json.loads((o/'manifest.json').read_text())
    # The legacy paired audit still expects plain JSONL. Restore only while
    # that audit reads it; persistent diagnostic archives remain compressed.
    decision_file = f/'decisions.jsonl'
    restored = not decision_file.exists()
    if restored:
        with reading(decision_file) as stream:
            decision_file.write_text(stream.read())
    try:
        paired = summarize(root/'campaign')[f.name]
    finally:
        if restored:
            decision_file.unlink()
    (root/'paired_reference_audit.json').write_text(json.dumps(paired, indent=2)+'\n')
    checks = dict(reference=paired['valid'],
        observers_stopped=manifest.get('stopped', False),
        observers_completed=manifest['status']=='reference_ended',
        no_observer_errors=not manifest['errors'],
        all_observer_exits_zero=all(x == 0 for x in manifest['processes'].values()),
        paired_odometry_exact_stamps=all(x == 0 for x in manifest['unmatched_odometry'].values()))
    metrics, errors = {}, {}
    for side, p in [('fixed', f/'fixed'), ('active', f)]:
        for name in ['mimosa', 'rovio']:
            key = side+'_'+name
            metrics[key], grid, errors[key], stamps = evaluate(p, name)
    reference, _ = ordered(f/'mimosa/ordered_inputs.csv', 0, 2**63-1)
    signature = lambda rr: [tuple(r[k] for k in ['stamp_ns','source','imu_count','imu_upto_ns']) for r in rr]
    def scans(rr):
        total = 0
        out = []
        for r in rr:
            total += int(r['imu_count'])
            if r['source'] == 'lidar':
                out.append((int(r['stamp_ns']), int(r['imu_upto_ns']), total))
        return out
    refscan = scans(reference)
    refpose = tum(f/'mimosa/graph_manager_odometry.tum')
    audits = {}
    refconfig = yaml.safe_load((f/'mimosa.yaml').read_text())
    refconfig.pop('logs_directory')
    for name in manifest['variants']:
        p = o/name
        rr, aa = ordered(p/'mimosa/ordered_inputs.csv', 0, 2**63-1)
        audits[name] = aa
        checks[name+'_chronology'] = all(aa[k] for k in ['chronological','imu_monotonic','imu_brackets_events'])
        checks[name+'_continuous_lidar'] = aa['sources']['lidar']['continuous']
        sc = scans(rr)
        checks[name+'_complete_reference_lidar_imu_prefix'] = sc[:len(refscan)] == refscan
        checks[name+'_extra_scans_after_evaluation'] = all(x[0] > stamps[-1] for x in sc[len(refscan):])
        aa['extra_scan_stamps_ns'] = [x[0] for x in sc[len(refscan):]]
        if name != 'pure_lio':
            checks[name+'_identical_input_event_signature'] = signature(rr) == signature(reference)
            checks[name+'_continuous_vio'] = aa['sources']['vio']['continuous']
        else:
            checks[name+'_no_vio'] = aa['sources']['vio']['count'] == 0
            for fn in ['velocity_factors.csv', 'tilt_factors.csv']:
                pp = csv_path(p/'mimosa'/fn)
                checks[name+'_no_'+fn] = not pp.exists() or not rows(pp)
        config = yaml.safe_load((p/'mimosa.yaml').read_text())
        config.pop('logs_directory')
        if name == 'pure_lio':
            config['odometry']['manager']['enabled'] = refconfig['odometry']['manager']['enabled']
        checks[name+'_same_configuration'] = config == refconfig
        metrics[name], grid, errors[name], stamps = evaluate_tum(root, f, p/'mimosa/graph_manager_odometry.tum')
        checks[name+'_full_continuous_output'] = metrics[name]['coverage'] == 1 and metrics[name]['max_output_gap_s'] <= .10001
        if name == 'duplicate_active':
            duplicate = tum(p/'mimosa/graph_manager_odometry.tum')
            checks['duplicate_entire_trajectory_exact'] = np.array_equal(refpose, duplicate)
        if name not in ['duplicate_active', 'pure_lio']:
            vsource = 'fixed' if name.startswith('fixed_velocity') else 'active'
            tsource = 'fixed' if name == 'active_velocity_fixed_tilt' else 'active'
            csource = ('fixed' if name == 'active_velocity_fixed_cov' else
                       'active' if name == 'fixed_velocity_active_cov' else vsource)
            paths = {'fixed': f/'fixed/mimosa', 'active': f/'mimosa'}
            targetv = rows(p/'mimosa/velocity_factors.csv')
            sourcev = rows(paths[vsource]/'velocity_factors.csv')
            sourcec = rows(paths[csource]/'velocity_factors.csv')
            targett = rows(p/'mimosa/tilt_factors.csv')
            sourcet = rows(paths[tsource]/'tilt_factors.csv')
            for label, expected, keys in [
                ('velocity',sourcev,['stamp_ns','vx','vy','vz']),
                ('covariance',sourcec,['stamp_ns','covariance_scale']+[f'c{i}{j}' for i in range(3) for j in range(3)])]:
                checks[name+'_exact_'+label] = (
                    [[r[k] for k in keys] for r in targetv] == [[r[k] for k in keys] for r in expected])
            checks[name+'_exact_tilt'] = targett == sourcet
    protocol = json.loads((f/'protocol.json').read_text())
    checks['declared_binary_and_library_identical'] = all(
        v['binary_sha256'] == protocol['binary_sha256']['mimosa_node'] and
        v['library_sha256'] == protocol['binary_sha256']['libmimosa_lib.so']
        for v in manifest['variants'].values())
    loaded_path = o/'loaded_libraries.json'
    if loaded_path.exists():
        checks['loaded_libraries_identical'] = all(
            set(v.values()) == {protocol['binary_sha256']['libmimosa_lib.so']}
            for v in json.loads(loaded_path.read_text()).values())
    selected = json.loads((HERE/'selected_gimbal.json').read_text())
    checks['selected_81_files_unchanged'] = all(
        hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p,h in selected['files'].items())
    r = load(f/'route_tracking.csv')
    times = r['sim_t'][r['route_t'] > 0]
    measurement = {side: describe(p, times) for side,p in [('fixed',f/'fixed'),('active',f)]}
    for key in errors:
        np.savetxt(root/(key+'_errors.csv'), np.column_stack([grid,stamps,errors[key]]),
            delimiter=',',header='route_t,stamp_ns,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg',
            comments='',fmt=['%.6f','%.0f']+['%.12g']*6)
    return dict(valid=all(checks.values()),checks=checks,metrics=metrics,
        measurement=measurement,input_audits=audits,
        loaded_libraries_observed=loaded_path.exists(),
        scope='120 s tracking, initial rigid alignment, no scale fit. Conditional estimators share flight and observations; only main active estimate controls gimbal.',
        initial_sim_s=float(stamps[0]*1e-9-grid[0])), errors, grid, stamps


def information_diagnosis(root, stamps):
    f = root/'campaign/run2a_prefix_active'
    data = {}
    with gzip.open(root/'observer/geometry.jsonl.gz', 'rt') as stream:
        for line in stream:
            item = json.loads(line)
            data.setdefault(item['kind'], {})[item['stamp_ns']] = item['data']
    cfg = yaml.safe_load((f/'planner.yaml').read_text())
    sigma = cfg['bearing_sigma']
    prior = np.diag(1/np.asarray(cfg['prior_std'])**2)
    ext = yaml.safe_load((f/'mimosa.yaml').read_text())['lidar']['T_B_S']
    TBL = np.eye(4)
    TBL[:3,:3] = Rotation.from_quat(ext[3:]).as_matrix()
    TBL[:3,3] = ext[:3]
    fisher = Fisher('/root/multilink_autonomy_ws/build/active_perception/libfif_active.so')
    matrix_cache = {}
    def matrices(side, t):
        if (side,t) in matrix_cache:
            return matrix_cache[(side,t)]
        _, ids, points, velocity, C = decode(data['geometry_'+side][t])
        ext = data['extrinsic_'+side][t]
        R = Rotation.from_quat(ext[3:]).as_matrix()
        position = np.array(ext[:3])
        n = len(points)
        covs = np.array([C[3+3*i:6+3*i,3+3*i:6+3*i] for i in range(n)]).reshape(n,3,3)
        marginal,J = fisher.information(points,covs,R,position,sigma)
        geometric,_ = fisher.information(points,np.zeros_like(covs),R,position,sigma)
        D = np.zeros((3*n,3*n))
        pc = (points-position)@R
        for i,p in enumerate(pc):
            norm = np.linalg.norm(p)
            D[3*i:3*i+3,3*i:3*i+3] = (np.eye(3)/norm-np.outer(p,p)/norm**3)@R.T
        S = D@C[3:,3:]@D.T+sigma**2*np.eye(3*n)
        j = J.reshape(3*n,6)
        correlated = j.T@cho_solve(cho_factor(S),j) if n else np.zeros((6,6))
        result = dict(marginal=marginal,geometric=geometric,correlated=correlated), dict(
            tracks=n,median_depth=float(np.median(np.linalg.norm(pc,axis=1))) if n else None,
            mean_track_age=float(np.mean(data['tracks_'+side][t]['ages_s'])) if n else 0.)
        matrix_cache[(side,t)] = result
        return result
    keys = set(data['geometry_fixed']) & set(data['geometry_active']) & set(data['extrinsic_fixed']) & set(data['extrinsic_active'])
    keys = sorted(t for t in keys if stamps[0] <= t <= stamps[-1])
    output = []
    weak_axes = {}
    for source in ['pure','active']:
        if 'lidar_'+source not in data:continue
        lidars = data['lidar_'+source]
        lt = np.array(sorted(lidars))
        for t in keys:
            nearest = lt[np.argmin(np.abs(lt-t))]
            age = abs(int(nearest)-t)*1e-9
            if age > .10000001:continue
            m = np.array(lidars[int(nearest)])
            H,U,weak = lidar_model(m[18:54].reshape(6,6),m[54:90].reshape(6,6),m[90:96],TBL)
            H0 = (H+H.T)*.5+prior
            P0 = np.linalg.inv(H0)
            ev,E = np.linalg.eigh(P0[3:,3:])
            direction = E[:,-1]
            if direction[0] < 0:direction = -direction
            weak_axes[(source,t)] = direction
            baseline = np.einsum('ij,ij->j',U,P0@U)
            row = dict(stamp_ns=t,lidar_source=source,lidar_age_s=age,
                weak_body_translation=direction.tolist(),weakness=weak.tolist(),
                weakest_translation_variance=float(ev[-1]),
                translation_variance_ratio=float(ev[-1]/ev[-2]))
            for side in ['fixed','active']:
                infos,stats = matrices(side,t)
                row[side] = stats
                for mode,Hv in infos.items():
                    post = np.linalg.inv(H0+Hv)
                    variance = np.einsum('ij,ij->j',U,post@U)
                    row[side][mode] = dict(
                        weak_log_gain=float(np.sum(weak*np.maximum(0,np.log(baseline/variance)))),
                        weak_translation_log_gain=float(np.log(ev[-1]/(direction@post[3:,3:]@direction))),
                        direct_weak_translation_information=float(direction@Hv[3:,3:]@direction))
            output.append(row)
    # Velocity error and covariance along exactly the same LiDAR weak direction.
    truth = poses(f/'truth.csv')
    directional_velocity = []
    measured_data = {}
    for side,p in [('fixed',f/'fixed'),('active',f)]:
        d = load(p/'mimosa/velocity_factors.csv')
        t = d['stamp_ns']
        _,q = interp(truth,t)
        before,_ = interp(truth,t-1e7)
        after,_ = interp(truth,t+1e7)
        measured_data[side] = d, q.inv().apply((after-before)/.02)
    for source in ['pure','active']:
        times = np.array(sorted(t for s,t in weak_axes if s == source))
        if not len(times):continue
        for side in ['fixed','active']:
            d,true_velocities = measured_data[side]
            for index,t in enumerate(d['stamp_ns']):
                if not stamps[0] <= t <= stamps[-1]:continue
                close = times[np.argmin(abs(times-t))]
                if abs(close-t)>300000001:continue
                w = weak_axes[(source,int(close))]
                vtrue = true_velocities[index]
                measured = np.array([d['v'+c][index] for c in 'xyz'])
                C = np.array([d[f'c{i}{j}'][index] for i in range(3) for j in range(3)]).reshape(3,3)
                directional_velocity.append(dict(side=side,lidar_source=source,stamp_ns=int(t),
                    axis_age_s=float(abs(close-t)*1e-9),error=float(w@(measured-vtrue)),
                    variance=float(w@C@w),conditional_information=float(w@np.linalg.solve(C,w))))
    summary = {}
    for source in ['pure','active']:
        rr = [r for r in output if r['lidar_source']==source]
        if not rr:continue
        ss = dict(samples=len(rr),max_lidar_join_age_s=max(r['lidar_age_s'] for r in rr),
            median_abs_weak_body_x=float(np.median([abs(r['weak_body_translation'][0]) for r in rr])),
            median_translation_variance_ratio=float(np.median([r['translation_variance_ratio'] for r in rr])),
            weak_probability_sum_range=[float(min(sum(r['weakness']) for r in rr)),float(max(sum(r['weakness']) for r in rr))])
        for mode in ['geometric','marginal','correlated']:
            ss[mode] = {}
            for metric in ['weak_log_gain','weak_translation_log_gain','direct_weak_translation_information']:
                fixed = np.array([r['fixed'][mode][metric] for r in rr])
                active = np.array([r['active'][mode][metric] for r in rr])
                ss[mode][metric] = dict(fixed_mean=float(fixed.mean()),active_mean=float(active.mean()),
                    fixed_better_fraction=float(np.mean(fixed>active)))
        for side in ['fixed','active']:
            vv = [r for r in directional_velocity if r['lidar_source']==source and r['side']==side]
            ss[side] = dict(mean_tracks=float(np.mean([r[side]['tracks'] for r in rr])),
                mean_track_age_s=float(np.mean([r[side]['mean_track_age'] for r in rr])),
                median_feature_range_m=float(np.median([r[side]['median_depth'] for r in rr if r[side]['median_depth'] is not None])),
                weak_velocity_rmse=float(np.sqrt(np.mean([r['error']**2 for r in vv]))),
                weak_velocity_bias=float(np.mean([r['error'] for r in vv])),
                median_weak_velocity_sigma=float(np.median([np.sqrt(r['variance']) for r in vv])),
                max_velocity_axis_join_age_s=float(max(r['axis_age_s'] for r in vv)))
        summary[source] = ss
    with gzip.open(root/'geometry_scores.json.gz','wt',compresslevel=6) as f:
        json.dump(dict(rows=output,velocity=directional_velocity),f)
    return dict(scope='Same-time fixed and active tracked landmarks, common LiDAR matrix and pose prior; not independent evidence and not actual fused posterior. ROVIO-estimated depths may be biased. geometric assumes exact points; marginal uses per-point depth covariance; correlated includes inter-landmark covariance. Pure LIO map also contains estimation errors. No truth depth was available.',
        summary=summary)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    a = p.parse_args()
    result,errors,grid,stamps = audit(a.root)
    result['directional'] = information_diagnosis(a.root,stamps)
    truth = poses(a.root/'campaign/run2a_prefix_active/truth.csv')
    _, attitude = interp(truth,stamps)
    forward = attitude.apply(np.tile([1.,0,0],(len(stamps),1)))
    result['signed_forward_error'] = {
        key: dict(final_m=float(np.sum(err[-1,:3]*forward[-1])),
                  mean_m=float(np.mean(np.sum(err[:,:3]*forward,axis=1))))
        for key,err in errors.items()}
    decisions = []
    with reading(a.root/'campaign/run2a_prefix_active/decisions.jsonl') as stream:
        for line in stream:
            row = json.loads(line)
            if not stamps[0] <= row['stamp_ns'] <= stamps[-1] or not row.get('selected'):
                continue
            candidates = row.get('candidates', [])
            if not candidates:
                continue
            home = min(candidates,key=lambda c:np.linalg.norm(c['angles']))
            if np.linalg.norm(home['angles'])>1e-8:
                raise ValueError('No fixed home-angle candidate')
            decisions.append(dict(stamp_ns=row['stamp_ns'],selected=row['selected'],
                home=home,actual_angles=row['encoder'],scene=row['motion_alignment']['scene']))
    result['planner_candidates'] = dict(
        samples=len(decisions),
        selected_gain_above_home_fraction=float(np.mean([
            r['selected']['gain']>r['home']['gain']+1e-10 for r in decisions])),
        selected_gain_mean=float(np.mean([r['selected']['gain'] for r in decisions])),
        home_gain_mean=float(np.mean([r['home']['gain'] for r in decisions])),
        median_actual_pitch_deg=float(np.median([np.rad2deg(r['actual_angles'][1]) for r in decisions])),
        mean_lidar_supported_points=float(np.mean([r['scene']['lidar_depth'] for r in decisions])),
        mean_triangulated_points=float(np.mean([r['scene']['triangulated_depth'] for r in decisions])),
        scope='Home candidate evaluated in the active camera observed map, not a counterfactual complete fixed-camera map. Selected endpoint differs from instantaneous reached angles.')
    with gzip.open(a.root/'candidate_scores.json.gz','wt') as stream:
        json.dump(decisions,stream)
    (a.root/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(valid=result['valid'],failed=[k for k,v in result['checks'].items() if not v],
        metrics=result['metrics'],directional=result['directional']),indent=2))


if __name__ == '__main__':
    main()
