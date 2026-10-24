#!/usr/bin/env python3
from pathlib import Path
from view_preparation import replace_once
path=Path(__file__).with_name('run_motion_view_trials.py')
source=path.read_text().replace("'run_motion_view_array.py'","'run_preview_view_array.py'")
source=replace_once(source,"files=[HERE/n for n in [", "files=[HERE/n for n in ['preview_motion.py','preview_preparation.py','preview_view_route.py','preview_view_planner_node.py','run_preview_view_trials.py','run_prepared_view_array.py','view_preparation.py',")
exec(compile(source,str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
