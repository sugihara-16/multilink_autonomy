#!/usr/bin/env python3
"""Audit a conditional ROVIO-only LiDAR distance-initialization diagnostic."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from report_routes import csv_path
from diagnose_unipilot_drift import evaluate


def load(path):
    return np.atleast_1d(np.genfromtxt(csv_path(path),delimiter=',',names=True))


def main():
    p=argparse.ArgumentParser();p.add_argument('diagnostic',type=Path);p.add_argument('--flight',type=Path,required=True);a=p.parse_args();out=a.diagnostic
    runtime=json.loads((out/'runtime.json').read_text());main_runtime=json.loads((a.flight/'runtime.json').read_text())
    protocol=json.loads((a.flight/'protocol.json').read_text())
    checks=dict(stopped=runtime['stopped'],processes_exit_zero=all(v==0 for v in runtime['processes'].values()),
                reference_completed=main_runtime['status']=='route_complete' and main_runtime['stopped'])
    main_active=load(a.flight/'vio_diagnostic.csv');main_fixed=load(a.flight/'fixed/vio_diagnostic.csv')
    duplicate=load(out/'duplicate_active/vio.csv')
    checks['disabled_duplicate_all_values_exact']=np.array_equal(duplicate,main_active)
    manifest=json.loads((out/'build_manifest.json').read_text())
    checks['build_unchanged']=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in manifest['sha256'].items())
    modes={}
    for name,reference in [('depth_active',main_active),('depth_fixed',main_fixed)]:
        folder=out/name;values=load(folder/'vio.csv');depth=load(folder/'initial_depth.csv')
        checks[name+'_timestamps_exact']=np.array_equal(values['stamp_ns'],reference['stamp_ns'])
        supported=depth[depth['supported']>0]
        metrics=evaluate(folder/'vio.csv',a.flight,protocol['route_end'],folder)
        modes[name]=dict(metrics=metrics,initializations_with_scan=len(depth),supported=len(supported),
                         all_values_equal_reference=np.array_equal(values,reference))
        if not len(supported):checks[name+'_zero_support_identity']=np.array_equal(values,reference)
        if len(supported):modes[name]['initial_to_original_depth_ratio_p10_p50_p90']=np.quantile(supported['initial_depth_m']/supported['original_depth_m'],[.1,.5,.9]).tolist()
    result=dict(scope=__doc__,valid=all(checks.values()),checks=checks,results=modes,acceptance_use=False,
                limitations=['Camera decisions and MIMOSA remain driven by the original ROVIO.',
                             'Only new-feature distance means change; original relative covariance retained.',
                             'Not pure VIO and not evidence of an online fused improvement.'])
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
