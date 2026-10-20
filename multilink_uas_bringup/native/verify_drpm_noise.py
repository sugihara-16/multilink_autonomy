#!/usr/bin/env python3
"""Audit input identity, unchanged fusion settings, complete source timestamps."""
import hashlib,json,re
from pathlib import Path
import numpy as np
import yaml
from plot_livo_comparison import read,LABELS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_noise'
BASE=ROOT/'logs/zab-offline/livo_comparison';CAL=ROOT/'logs/zab-offline/livo_calibrated'
MODELS={'legacy_isotropic':0,'propagated_isotropic':1,'radial':2}
result={}
for label in LABELS:
    olddir=CAL/'mimosa_drpm_calibrated'/label;original=yaml.safe_load((olddir/'config.yaml').read_text())
    a0=read(olddir/'odometry.csv');result[label]={};binary_hashes=set();input_hashes=set()
    assert json.loads((BASE/'inputs'/label/'bag_summary.json').read_text())['zero_point_time']
    for mode,code in MODELS.items():
        d=OUT/mode/label;cfg=yaml.safe_load((d/'config.yaml').read_text());assert cfg['lidar']['geometric']['scan_to_map'].pop('drpm_noise_model')==code
        cfg['logs_directory']=original['logs_directory'];assert cfg==original
        s=json.loads((d/'summary.json').read_text());assert s['returncode']==0;binary_hashes.add(s['binary_sha256']);input_hashes.add(s['input_bag_sha256'])
        a=read(d/'odometry.csv');assert np.array_equal(a[0],a0[0])
        w=s['window'];tracking=a[0][(a[0]>=w['start_ns'])&(a[0]<=w['end_ns'])]
        expected=np.arange(((w['start_ns']+49999999)//50000000)*50000000,w['end_ns']+1,50000000,dtype=np.int64)
        assert np.array_equal(tracking,expected)
        log=(d/'run.log').read_text();assert 'Using calibrated full relative-pose covariance' in log
        assert 'is higher than threshold' not in log
        probabilities=[]
        for line in log.splitlines():
            if line.startswith('DRPM probabilities '):
                try:r=list(map(float,line.removeprefix('DRPM probabilities ').split()))
                except ValueError:continue
                if len(r)==6:probabilities.append(r)
        p=np.array(probabilities);assert len(p)>0 and np.isfinite(p).all() and p.min()>=0 and p.max()<=1
        m=dict(config_only_noise_model_changed=True,all_original_output_stamps_exact=True,samples=len(a[0]),tracking_samples=len(tracking),tracking_max_gap_s=.05,full_max_gap_s=float(np.diff(a[0]).max()*1e-9),
          zero_point_time_verified=True,calibrated_covariance_loaded=True,vio_gate_rejections=0,
          sampled_probability_rows=len(p),sampled_probability_mean_by_sorted_eigenvalue=p.mean(axis=0).tolist(),
          sampled_mean_suppressed_directions_at_p_below_half=float((p<.5).sum(axis=1).mean()),
          probability_sampling_note='Every 100 factor linearizations, all phases. Trajectories/correspondences differ; not matched-frame causality.')
        if mode=='legacy_isotropic':
            assert np.array_equal(a[1],a0[1]) and np.array_equal(a[2].as_quat(),a0[2].as_quat())
            m['legacy_positions_and_quaternions_exact']=True
            m['legacy_max_position_difference_m']=float(np.linalg.norm(a[1]-a0[1],axis=1).max())
            m['legacy_max_rotation_difference_deg']=float(np.rad2deg((a0[2].inv()*a[2]).magnitude()).max())
        result[label][mode]=m
    assert len(binary_hashes)==len(input_hashes)==1
result['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'comparison/drpm.hpp',ROOT/'comparison/mimosa-drpm-noise.patch',ROOT/'run_drpm_noise.py']}
(OUT/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
