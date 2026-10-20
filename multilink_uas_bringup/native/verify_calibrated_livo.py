#!/usr/bin/env python3
"""Check held-out provenance, isolated changes, exact output stamps and ablations."""
import hashlib,json
from pathlib import Path
import numpy as np,yaml
from plot_livo_comparison import read,LABELS
from run_calibrated_livo import METHODS

ROOT=Path(__file__).resolve().parent;BASE=ROOT/'logs/zab-offline/livo_comparison';OUT=ROOT/'logs/zab-offline/livo_calibrated'

def main():
    cal=json.loads((OUT/'calibration.json').read_text());results={}
    for label in LABELS:
        fold=cal['folds'][label];train=fold['training_flight'];assert train!=label and train in LABELS
        path=BASE/'rovio'/train/'result_vio.csv'
        assert hashlib.sha256(path.read_bytes()).hexdigest()==cal['characteristics'][train]['input_sha256']
        C=np.array(fold['covariance']);assert np.linalg.eigvalsh(C).min()>0
        w=json.loads((BASE/'inputs'/label/'manifest.json').read_text())['window']
        per={};configs={}
        for method in METHODS:
            d=OUT/method/label;s=json.loads((d/'summary.json').read_text());assert s['returncode']==0
            configs[method]=yaml.safe_load((d/'config.yaml').read_text())
            t,p,r=read(d/'odometry.csv');assert np.isfinite(p).all() and np.isfinite(r.as_quat()).all()
            assert t[0]<w['start_ns'] and t[-1]>=w['end_ns']
            t=t[(t>=w['start_ns'])&(t<=w['end_ns'])]
            step=100000000 if method in ['mimosa_lio','fast_lio'] else 50000000
            expected=np.arange(((w['start_ns']+step-1)//step)*step,w['end_ns']+1,step,dtype=np.int64)
            assert np.array_equal(t,expected),(label,method,len(t),len(expected))
            per[method]=dict(samples=len(t),max_gap_s=float(np.diff(t).max()*1e-9),exact_source_grid=True)
        # Only the DRPM flag changes between the two calibrated MIMOSA cases.
        a=configs['mimosa_calibrated'];b=configs['mimosa_drpm_calibrated']
        for c in [a,b]:
            c.pop('logs_directory');assert np.array_equal(np.array(c['odometry']['manager']['relative_pose_covariance']).reshape(6,6),C)
        assert a['lidar']['geometric']['scan_to_map']['drpm_enabled'] is False
        assert b['lidar']['geometric']['scan_to_map']['drpm_enabled'] is True
        b['lidar']['geometric']['scan_to_map']['drpm_enabled']=False;assert a==b
        assert configs['mimosa_lio']['odometry']['manager']['enabled'] is False
        a=configs['fast_full'];b=configs['fast_no_visual'];c=configs['fast_lio']
        assert a['disable_visual_update'] is False and b['disable_visual_update'] is True
        b['disable_visual_update']=False;assert a==b
        assert c['parameters']['common']['img_en']==0
        c['parameters']['common']['img_en']=1;assert c==a
        d=configs['fast_tracking_no_visual']
        assert d.pop('disable_visual_after_s')==w['start_ns']*1e-9
        assert d==a
        results[label]=dict(calibration_flight=train,methods=per,isolated_parameter_changes=True)
    (OUT/'verification.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))

if __name__=='__main__':main()
