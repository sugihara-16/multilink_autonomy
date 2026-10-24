#!/usr/bin/env python3
"""Same-flight fixed five and active with explicit extra common preparation."""
import difflib,sys
from run_view_array import effective_sources,replace_once,HERE,VIEWS
from run_lio_failure_comparisons import verify_selected
def prepared_sources():
 source,pacer,patch=effective_sources()
 source=replace_once(source,"HERE/'planner_node.py'","HERE/'prepared_view_planner_node.py'")
 source=replace_once(source,"HERE/'unipilot_route.py'","HERE/'prepared_view_route.py'")
 source=replace_once(source,"    snapshot=out/'implementation';snapshot.mkdir()", "    snapshot=out/'implementation';snapshot.mkdir()\n    for name in ['motion_view_planner.py','view_preparation.py','prepared_view_route.py','prepared_view_planner_node.py','run_prepared_view_array.py']:\n        shutil.copy2(HERE/name,snapshot/name)\n    (out/'view_preparation_protocol.json').write_text(json.dumps(dict(common_preparation_cycles=2,hold_active_view_in_second_cycle=True,estimator_changes=False,truth_in_planner=False),indent=2))")
 patch=''.join(difflib.unified_diff((HERE/'unipilot_run.py').read_text().splitlines(True),source.splitlines(True),fromfile='selected/unipilot_run.py',tofile='prepared_view_owner.py'))
 return source,pacer,patch

if __name__=='__main__':
 verify_selected();source,pacer,patch=prepared_sources()
 for name in VIEWS:sys.argv.extend(['--pacing-extra-lidar','/array_'+name+'_mimosa/lidar/manager/debug'])
 exec(compile(source,str(HERE/'unipilot_run.py'),'exec'),dict(__name__='__main__',__file__=str(HERE/'unipilot_run.py'),ARRAY_SOURCE=source,ARRAY_PACER=pacer,ARRAY_DIFF=patch))
