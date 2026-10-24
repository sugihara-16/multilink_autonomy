#!/usr/bin/env python3
"""Paired fixed-five comparison with an explicit common route preview."""
import difflib,sys
import yaml
from run_prepared_view_array import prepared_sources
from run_view_array import HERE,VIEWS,replace_once
from run_lio_failure_comparisons import verify_selected
from preview_motion import MAX_DISTANCE_M


def preview_sources():
    source,pacer,_=prepared_sources()
    source=replace_once(source,"HERE/'prepared_view_planner_node.py'","HERE/'preview_view_planner_node.py'")
    source=replace_once(source,"HERE/'prepared_view_route.py'","HERE/'preview_view_route.py'")
    source=replace_once(source,"    snapshot=out/'implementation';snapshot.mkdir()", "    snapshot=out/'implementation';snapshot.mkdir()\n    for name in ['preview_motion.py','preview_preparation.py','preview_view_route.py','preview_view_planner_node.py','run_preview_view_array.py']:\n        shutil.copy2(HERE/name,snapshot/name)")
    source=replace_once(source,'common_preparation_cycles=2,hold_active_view_in_second_cycle=True,estimator_changes=False,truth_in_planner=False',
        'common_preparation_cycles=2,hold_active_view_in_second_cycle=True,preview_first_polyline_max_m=15.,return_holds_active_view=True,preview_speed_max_mps=.5,estimator_changes=False,truth_in_planner=False')
    patch=''.join(difflib.unified_diff((HERE/'unipilot_run.py').read_text().splitlines(True),source.splitlines(True),fromfile='selected/unipilot_run.py',tofile='preview_view_owner.py'))
    return source,pacer,patch


if __name__=='__main__':
    from pathlib import Path
    cfg=yaml.safe_load(Path(sys.argv[sys.argv.index('--planner-config')+1]).read_text())
    if not (cfg.get('preview_preparation') and cfg.get('preview_max_distance_m')==MAX_DISTANCE_M and cfg.get('common_preparation_cycles')==2 and cfg.get('hold_active_view_in_second_cycle')):
        raise ValueError('Preview owner requires its explicit preparation config')
    verify_selected();source,pacer,patch=preview_sources()
    for name in VIEWS:sys.argv.extend(['--pacing-extra-lidar','/array_'+name+'_mimosa/lidar/manager/debug'])
    exec(compile(source,str(HERE/'unipilot_run.py'),'exec'),dict(__name__='__main__',__file__=str(HERE/'unipilot_run.py'),ARRAY_SOURCE=source,ARRAY_PACER=pacer,ARRAY_DIFF=patch))
