#!/usr/bin/env python3
"""Audit declared initialization or numerical-solver/IMU diagnostics without tuning."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import numpy as np
import yaml
from report_fusion_sidecar import ordered, tum
from report_routes import csv_path
from report_estimator_validation import evaluate
from unipilot_calibration import fusion_white_noise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.output/'manifest.json').read_text())
    if manifest['comparison_kind'] not in ('initial_velocity','numerical_imu','full_solver_noise','temporal_velocity'):
        raise ValueError('Wrong diagnostic')
    flight = Path(manifest['flight'])
    runtime = json.loads((flight/'runtime.json').read_text())
    reference = tum(flight/'mimosa/graph_manager_odometry.tum')
    lower, upper = (int(round(t*1e9)) for t in reference[[0,-1], 0])
    refrows, _ = ordered(flight/'mimosa/ordered_inputs.csv', lower, upper)
    def sensor_rows(rows):
        return [tuple(r[k] for k in ['stamp_ns','source','imu_count','imu_upto_ns']) for r in rows]
    library = json.loads((flight/'protocol.json').read_text())['binary_sha256']['libmimosa_lib.so']
    libraries = json.loads((args.output/'loaded_libraries.json').read_text())
    checks = dict(reference_complete=runtime['status']=='route_complete' and runtime['stopped'],
                  reference_processes_zero=all(c==0 for c in runtime['processes'].values()),
                  diagnostic_complete=manifest['status']=='reference_ended' and manifest['stopped'],
                  diagnostic_processes_zero=all(c==0 for c in manifest['processes'].values()),
                  libraries_unchanged=all(set(v.values())=={library} for v in libraries.values()))
    metrics = {'reference_active': evaluate(flight,'mimosa')[0]}
    if manifest['comparison_kind'] in ('full_solver_noise','temporal_velocity'):
        metrics['reference_fixed']=evaluate(flight/'fixed','mimosa')[0]
    for name in manifest['variants']:
        source = args.output/name
        variant=manifest['variants'][name]
        mode_reference=flight/('fixed' if variant['mode']=='fixed' else '')
        refrows, _ = ordered(mode_reference/'mimosa/ordered_inputs.csv', lower, upper)
        data = tum(source/'mimosa/graph_manager_odometry.tum')
        rows, audit = ordered(source/'mimosa/ordered_inputs.csv', lower, upper)
        checks[name+'_same_all_reference_input'] = sensor_rows(rows)==sensor_rows(refrows)
        checks[name+'_continuous_input'] = audit['chronological'] and audit['imu_monotonic'] and audit['imu_brackets_events'] and all(v['continuous'] for v in audit['sources'].values())
        original = yaml.safe_load((mode_reference/'mimosa.yaml').read_text())
        changed = yaml.safe_load((source/'mimosa.yaml').read_text())
        original.pop('logs_directory'); changed.pop('logs_directory')
        if variant['kind'].startswith('loose_initial'):
            original['graph']['manager']['smoother']['initial_velocity_sigma']=1.
        if variant['kind'].startswith('full_relinearization'):
            from numerical_relinearization import apply
            apply(original)
        if variant['kind'].endswith('_sensor'):
            fusion_white_noise(original,flight/'unipilot_gimbal.sdf','sensor')
        if manifest['comparison_kind']=='temporal_velocity':
            temporal=json.loads((args.output/'temporal_calibration.json').read_text())
            if variant['kind']!='temporal_disabled':
                original['graph']['manager']['smoother']['lag']=2*temporal['period_s']
            if variant['kind']=='temporal_velocity':
                manager=original['odometry']['manager']
                manager['temporal_velocity_enabled']=True
                manager['temporal_velocity_rho']=temporal['rho']
                manager['temporal_velocity_innovation_variance']=temporal['innovation_variance']
        checks[name+'_only_declared_changes'] = original==changed
        for kind in ['tilt_factors.csv','velocity_factors.csv']:
            a=np.loadtxt(csv_path(mode_reference/'mimosa'/kind),delimiter=',',skiprows=1,ndmin=2)
            b=np.loadtxt(csv_path(source/'mimosa'/kind),delimiter=',',skiprows=1,ndmin=2)
            checks[name+'_'+kind+'_unchanged']=np.array_equal(a[(a[:,0]>=lower)&(a[:,0]<=upper)],b[(b[:,0]>=lower)&(b[:,0]<=upper)])
        if manifest['comparison_kind']=='temporal_velocity' and variant['kind']=='temporal_velocity':
            audit=np.loadtxt(source/'mimosa/temporal_velocity_factors.csv',delimiter=',',skiprows=1,ndmin=2)
            velocity=np.loadtxt(source/'mimosa/velocity_factors.csv',delimiter=',',skiprows=1,ndmin=2)
            checks[name+'_temporal_factor_every_observation']=np.array_equal(audit[:,0],velocity[:,0])
            checks[name+'_one_initial_factor']=bool(audit[0,-1]==1 and np.all(audit[1:,-1]==0))
            checks[name+'_temporal_parameters_exact']=bool(np.all(audit[:,2]==temporal['rho']) and np.all(audit[:,3]==temporal['innovation_variance']))
            checks[name+'_temporal_period_exact']=bool(np.all(np.abs(audit[1:,0]*1e-9-audit[1:,1]-temporal['period_s'])<1e-7))
        if name=='duplicate_active':
            common,ri,di=np.intersect1d(np.rint(reference[:,0]*1e9),np.rint(data[:,0]*1e9),return_indices=True)
            checks['duplicate_covers_reference']=len(common)==len(reference)
            checks['duplicate_exact_pose']=np.array_equal(reference[ri,1:],data[di,1:])
        with tempfile.TemporaryDirectory(prefix='evaluation_',dir=args.output) as tmp:
            proxy=Path(tmp)
            (proxy/'protocol.json').symlink_to(flight/'protocol.json')
            for filename in ['truth.csv','route_tracking.csv']:
                p=csv_path(flight/filename);(proxy/p.name).symlink_to(p)
            data[:,0]=np.rint(data[:,0]*1e9)
            np.savetxt(proxy/'mimosa.csv',data,delimiter=',',header='stamp_ns,x,y,z,qx,qy,qz,qw',comments='',fmt=['%.0f']+['%.17g']*7)
            metrics[name]=evaluate(proxy,'mimosa')[0]
            checks[name+'_full_declared_tracking']=metrics[name]['coverage']==1.
            checks[name+'_output_continuous']=metrics[name]['max_output_gap_s']<=.10001
    common=['reference_complete','reference_processes_zero','diagnostic_complete',
            'diagnostic_processes_zero','libraries_unchanged','duplicate_covers_reference','duplicate_exact_pose']
    variant_valid={name:all(checks[k] for k in common) and all(value for key,value in checks.items()
                    if key.startswith(name+'_') or key.startswith('duplicate_active_'))
                   for name in manifest['variants']}
    result=dict(scope=__doc__,valid=all(checks.values()),variant_valid=variant_valid,checks=checks,metrics=metrics,
                acceptance_use=False,report_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (args.output/'analysis_implementation').mkdir(exist_ok=True)
    (args.output/'analysis_implementation'/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
