"""Separate complete acquisition from the observed planner-hold outcome.

Keep all original audit flags. A valid acquisition is not a continuous planner.
"""
import json
from pathlib import Path

def allowed_checks(checks,allowed):
    failed={k for k,v in checks.items() if not v}
    if not failed<=set(allowed):raise RuntimeError('Acquisition checks failed: '+str(sorted(failed-set(allowed))))
    return sorted(failed)

def trial_acceptance(path):
    path=Path(path)
    paired=json.loads((path/'paired_reference_audit.json').read_text())
    lio=json.loads((path/'comparison.json').read_text())
    array=json.loads((path/'array_comparison.json').read_text())
    first=allowed_checks(paired['checks'],{'planner_continuous'})
    allowed_checks(lo:=lio['checks'],{'reference_valid'} if first else set())
    allowed_checks(array['checks'],{'reference_paired_audit'} if first else set())
    return dict(acquisition_valid=True,original_strict_valid=paired['valid'] and lio['valid'] and array['valid'],method_outcomes=first,controller_timing=paired['controller_timing'],note='Planner holds are reported outcomes, not permission to drop data. All raw/estimator continuity, pairing, parameters and clean-exit checks remain required.')
