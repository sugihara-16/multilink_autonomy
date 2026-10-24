#!/usr/bin/env python3
"""Candidate view policy, unchanged frozen sensor and estimator owner."""
import difflib,sys
from run_view_array import effective_sources,replace_once,HERE,VIEWS
from run_lio_failure_comparisons import verify_selected
if __name__=='__main__':
    verify_selected()
    source,pacer,patch=effective_sources()
    source=replace_once(source,"HERE/'planner_node.py'","HERE/'motion_view_planner_node.py'")
    source=replace_once(source,"    snapshot=out/'implementation';snapshot.mkdir()",
        "    snapshot=out/'implementation';snapshot.mkdir()\n"
        "    for name in ['motion_view_planner.py','motion_view_planner_node.py','run_motion_view_array.py']:\n"
        "        shutil.copy2(HERE/name,snapshot/name)")
    patch=''.join(difflib.unified_diff((HERE/'unipilot_run.py').read_text().splitlines(True),source.splitlines(True),fromfile='selected/unipilot_run.py',tofile='candidate_array_owner.py'))
    for name in VIEWS:sys.argv.extend(['--pacing-extra-lidar','/array_'+name+'_mimosa/lidar/manager/debug'])
    exec(compile(source,str(HERE/'unipilot_run.py'),'exec'),dict(__name__='__main__',__file__=str(HERE/'unipilot_run.py'),ARRAY_SOURCE=source,ARRAY_PACER=pacer,ARRAY_DIFF=patch))
