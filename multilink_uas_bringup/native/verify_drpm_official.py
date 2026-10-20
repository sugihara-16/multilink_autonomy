#!/usr/bin/env python3
import csv,hashlib,json,subprocess
from collections import Counter
from pathlib import Path
import numpy as np,yaml
from plot_livo_comparison import read,LABELS
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/drpm_official';CAL=ROOT/'logs/zab-offline/livo_calibrated';BASE=ROOT/'logs/zab-offline/livo_comparison'
MODES={'official_both':(1,0),'official_update':(1,2),'official_noise':(2,0),'neither':(2,2),'official_noise_legacy_graph':(0,0)}
MODES['official_both_30']=(1,0)
ref=Path('/root/multilink_autonomy_ws/comparison_ws/drpm')
original=subprocess.check_output(['git','-C',str(ref),'show','b39a540c946f4059f5e3dfde625216c437eaf158:src/degeneracy.h'])
assert original==(ROOT/'comparison/official_drpm/degeneracy.h').read_bytes()
assert original==Path('/root/multilink_autonomy_ws/comparison_ws/src/mimosa/include/mimosa/lidar/official_drpm/degeneracy.h').read_bytes()
result=dict(official_header_byte_identical=True,official_revision='b39a540c946f4059f5e3dfde625216c437eaf158',flights={})
all_artifacts=[]
for label in LABELS:
    baseline=read(CAL/'mimosa_drpm_calibrated'/label/'odometry.csv')
    original_cfg=yaml.safe_load((CAL/'mimosa_drpm_calibrated'/label/'config.yaml').read_text());records={};inputs=set()
    for method,(update,noise) in MODES.items():
        limit=30 if method=='official_both_30' else 10
        dest=OUT/'iteration_check_30/official_both'/label if method=='official_both_30' else OUT/method/label
        s=json.loads((dest/'summary.json').read_text());assert s['returncode']==0
        all_artifacts.append(s['artifacts']);inputs.add(s['input_sha256'])
        cfg=yaml.safe_load((dest/'config.yaml').read_text());cfg['logs_directory']=original_cfg['logs_directory'];reg=cfg['lidar']['geometric']['scan_to_map']
        assert reg.pop('drpm_registration_mode')==update and reg.pop('drpm_noise_model')==noise and reg.pop('drpm_icp_max_iterations')==limit
        assert cfg==original_cfg
        a=read(dest/'odometry.csv');assert np.array_equal(a[0],baseline[0]);w=s['window'];t=a[0][(a[0]>=w['start_ns'])&(a[0]<=w['end_ns'])]
        expected=np.arange(((w['start_ns']+49999999)//50000000)*50000000,w['end_ns']+1,50000000,dtype=np.int64);assert np.array_equal(t,expected)
        log=(dest/'run.log').read_text();assert 'Using calibrated full relative-pose covariance' in log;assert 'is higher than threshold' not in log
        m=dict(exact_original_output_stamps=True,tracking_samples=len(t),tracking_max_gap_s=.05,only_controlled_config_changes=True,wall_s=s['wall_s'],iteration_limit=limit)
        if update:
            rows=list(csv.DictReader((dest/'drpm_registration.csv').open()))
            assert all(int(r['update_mode'])==update and int(r['noise_mode'])==noise for r in rows)
            for group,items in [('all',rows),('tracking',[r for r in rows if w['start_ns']<=int(r['stamp_ns'])<=w['end_ns']])]:
                counts=Counter(r['status'] for r in items);ok=[r for r in items if r['status']=='ok'];assert len(items)>0
                for r in ok:
                    p=np.array([float(r[f'p{k}']) for k in range(6)]);assert np.isfinite(p).all() and (p>=0).all() and (p<=1).all()
                    assert int(r['iterations'])<=limit
                m[group]=dict(scans=len(items),status_counts=dict(counts),valid_fraction=len(ok)/len(items),
                    converged_scans=sum(int(r['converged']) for r in ok),
                    median_correspondences=float(np.median([int(r['correspondences']) for r in ok])) if ok else None,
                    mean_iterations=float(np.mean([int(r['iterations']) for r in ok])) if ok else None,
                    median_last_translation_m=float(np.median([float(r['last_translation_m']) for r in ok])) if ok else None,
                    median_last_rotation_rad=float(np.median([float(r['last_rotation_rad']) for r in ok])) if ok else None)
            assert len([r for r in rows if w['start_ns']<=int(r['stamp_ns'])<=w['end_ns']]) in [495,2002]
        else:
            m['previous_graph_positions_exact']=np.array_equal(a[1],baseline[1]);m['previous_graph_quaternions_exact']=np.array_equal(a[2].as_quat(),baseline[2].as_quat())
            assert m['previous_graph_positions_exact'] and m['previous_graph_quaternions_exact']
        records[method]=m
    assert len(inputs)==1;result['flights'][label]=records
assert all(x==all_artifacts[0] for x in all_artifacts)
for path,sha in all_artifacts[0].items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha
result['artifacts']=all_artifacts[0]
(OUT/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
