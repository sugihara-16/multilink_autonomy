#!/usr/bin/env python3
"""Freeze a declared successful development set before any independent trial."""
import argparse,hashlib,json,zipfile
from datetime import datetime,timezone
from pathlib import Path
from report_gimbal_improvement import assess,frozen_evidence

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--candidate',required=True);p.add_argument('--seeds',type=int,nargs=2,required=True)
    p.add_argument('--description',required=True);a=p.parse_args();a.root=a.root.resolve()
    output=a.root/f'frozen_candidate_{a.candidate}.json'
    archive=a.root/f'frozen_{a.candidate}_sources.zip'
    if output.exists() or archive.exists():raise RuntimeError('Existing freeze must never be overwritten')
    declaration=json.loads((a.root/f'development_{a.candidate}.json').read_text())
    routes=['run1_tail_a1','run1_tail_b1','run1_prefix'];rows=[];protocols=[]
    for campaign in declaration['campaigns']:
        results=json.loads((a.root/campaign/'results.json').read_text())
        for label,row in results.items():
            base=a.root/campaign/label
            checks=frozen_evidence(base,declaration)
            if not row['valid'] or not all(checks.values()):raise RuntimeError('Invalid or changed development evidence: '+str(base))
            rows.append(dict(row,campaign=campaign,route=label.removesuffix('_active')))
            protocols.append(json.loads((base/'protocol.json').read_text()))
    verdict=assess(rows,declaration['criterion'],routes,declaration['seeds'])
    if not verdict['passed']:raise RuntimeError('Development criteria failed: '+json.dumps(verdict))
    for path in a.root.glob('*/run*_active/protocol.json'):
        if json.loads(path.read_text())['seed'] in a.seeds:
            raise RuntimeError('Independent seed already examined: '+str(path))
    variable={'seed','route','route_end','binary_sha256','upstream'}
    fields={k:v for k,v in protocols[0].items() if k not in variable}
    for proto in protocols:
        if {k:v for k,v in proto.items() if k not in variable}!=fields:
            raise RuntimeError('Development protocols differ')
        if proto['binary_sha256']!=protocols[0]['binary_sha256'] or proto['upstream']!=protocols[0]['upstream']:
            raise RuntimeError('Development binaries differ')
    contents={f:Path(f).read_bytes() for f in declaration['files']}
    for f,b in contents.items():
        if hashlib.sha256(b).hexdigest()!=declaration['files'][f]:raise RuntimeError('Current source changed: '+f)
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f,b in contents.items():z.writestr(f.lstrip('/'),b)
    with zipfile.ZipFile(archive) as z:
        for f,h in declaration['files'].items():
            if hashlib.sha256(z.read(f.lstrip('/'))).hexdigest()!=h:raise RuntimeError('Archive hash mismatch')
    frozen=dict(candidate=a.description,frozen_utc=datetime.now(timezone.utc).isoformat(),
        validation_seeds=a.seeds,routes=routes,files=declaration['files'],
        fusion_calibration=declaration.get('fusion_calibration',str(a.root/'native_fusion_calibration.json')),
        criterion=declaration['criterion'],drpm_fusion=fields['drpm_fusion'],
        no_truth_to_estimators_or_planner=True,binary_sha256=protocols[0]['binary_sha256'],
        upstream=protocols[0]['upstream'],protocol_fields=fields,
        validation_campaigns=[f'validation-{a.candidate}-seed{s-20261000}{suffix}' for s in a.seeds for suffix in ('','-prefix')],
        development_outcomes=rows,development_verdict=verdict,
        development_bootstrap_equivalence=declaration.get('bootstrap_equivalence'),
        policy='The six declared development pairs are complete. No changes during independent validation. Preserve every outcome; no reuse of examined independent seeds after tuning.',
        scope=dict(prefix_route_end_s=1070.,common_preparation_s=26.,same_physical_flight=True,
            first_pose_alignment=True,scale_fit=False,camera_far_m=fields['camera_far']),
        archive=dict(path=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest()))
    output.write_text(json.dumps(frozen,indent=2)+'\n')
    print('FROZEN',a.candidate,'independent seeds',a.seeds)
if __name__=='__main__':main()
