#!/usr/bin/env python3
"""Same physical comparison with committed recovery from lost tracks."""
import difflib
import sys
from pathlib import Path
import yaml
from run_prepared_view_array import prepared_sources
from run_view_array import HERE, VIEWS, replace_once
from run_lio_failure_comparisons import verify_selected


def guarded_sources():
    source, pacer, _ = prepared_sources()
    source = replace_once(source, "HERE/'prepared_view_planner_node.py'", "HERE/'guarded_view_planner_node.py'")
    source = replace_once(source, "    snapshot=out/'implementation';snapshot.mkdir()",
        "    snapshot=out/'implementation';snapshot.mkdir()\n    for name in ['committed_tracking_view.py','guarded_view_planner_node.py','run_guarded_view_array.py']:\n        shutil.copy2(HERE/name,snapshot/name)")
    source = replace_once(source, 'common_preparation_cycles=2,hold_active_view_in_second_cycle=True,estimator_changes=False,truth_in_planner=False',
        'common_preparation_cycles=2,hold_active_view_in_second_cycle=True,committed_track_recovery=True,estimator_changes=False,truth_in_planner=False')
    patch = ''.join(difflib.unified_diff((HERE / 'unipilot_run.py').read_text().splitlines(True), source.splitlines(True),
                                        fromfile='selected/unipilot_run.py', tofile='guarded_view_owner.py'))
    return source, pacer, patch


if __name__ == '__main__':
    cfg = yaml.safe_load(Path(sys.argv[sys.argv.index('--planner-config') + 1]).read_text())
    if not (cfg.get('committed_track_recovery') and cfg.get('fixation_retinal_candidates') and
            cfg.get('common_preparation_cycles') == 2 and cfg.get('hold_active_view_in_second_cycle')):
        raise ValueError('Guarded owner requires its explicit recovery and preparation config')
    verify_selected()
    source, pacer, patch = guarded_sources()
    for name in VIEWS:
        sys.argv.extend(['--pacing-extra-lidar', '/array_' + name + '_mimosa/lidar/manager/debug'])
    exec(compile(source, str(HERE / 'unipilot_run.py'), 'exec'),
         dict(__name__='__main__', __file__=str(HERE / 'unipilot_run.py'), ARRAY_SOURCE=source, ARRAY_PACER=pacer, ARRAY_DIFF=patch))
