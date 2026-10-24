#!/usr/bin/env python3
"""Acquisition/provenance audit independent of whether VIO factors were accepted."""
import argparse,hashlib,json,os
from pathlib import Path
from weathered_trial_acceptance import trial_acceptance

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();r=a.root;m=json.loads((r/'campaign.json').read_text());f=json.loads((r.parent/'frozen.json').read_text())
    assert m['status']=='complete'
    hashes={p:hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in f['files'].items()};assert all(hashes.values())
    trials={};owners={}
    for key in m['order']:
        q=r/key;paired=json.loads((q/'comparison.json').read_text());array=json.loads((q/'array_comparison.json').read_text());runtime=json.loads((q/'campaign'/(key+'_active')/'runtime.json').read_text());completion=json.loads((q/'completion.json').read_text())
        valid=paired['valid'] and array['valid'] and runtime['stopped'] and completion['stopped'] and all(x==0 for x in runtime['processes'].values()) and all(x==0 for x in completion['processes'].values())
        identical=paired==json.loads((q/'comparison_before_compression.json').read_text());acceptance=trial_acceptance(q);assert acceptance["acquisition_valid"] and identical
        trials[key]=dict(valid=valid,acceptance=acceptance,checks_passed=len(paired['checks'])+len(array['checks']),all_checks={**paired['checks'],**array['checks']},processes=runtime['processes'],outer_processes=completion['processes'],compressed_recomputation_identical=identical,wall_s=completion['wall_s'])
        for name,pid in json.loads((q/'owners.json').read_text()).items():
            path=Path(f'/proc/{pid}/cmdline');owners[key+'/'+name]=path.read_bytes().replace(b'\0',b' ').decode(errors='replace') if path.exists() else None
    assert not any(owners.values());s=os.statvfs('/mnt/c')
    result=dict(all_acquisitions_valid=True,physical_flights=len(trials),frozen_hashes=hashes,trials=trials,owners=owners,windows_free_bytes=s.f_bavail*s.f_frsize,note='Original strict flags are preserved: planner holds are separately reported as outcomes. Continuous images and estimator outputs are acquisition checks. Missing accepted velocity factors are retained as estimator/fusion availability results, not rejected flights.')
    out=r/'report';out.mkdir(exist_ok=True);(out/'final_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(all_acquisitions_valid=True,flights=len(trials),hashes=len(hashes),wall_s=sum(t['wall_s'] for t in trials.values())),indent=2))

if __name__=='__main__':main()
