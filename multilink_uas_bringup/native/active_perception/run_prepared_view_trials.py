#!/usr/bin/env python3
from pathlib import Path
from view_preparation import replace_once
path=Path(__file__).with_name('run_motion_view_trials.py')
source=path.read_text().replace("'run_motion_view_array.py'","'run_prepared_view_array.py'")
source=replace_once(source,"files=[HERE/n for n in [", "files=[HERE/n for n in ['view_preparation.py','prepared_view_route.py','prepared_view_planner_node.py','run_prepared_view_trials.py',")
exec(compile(source,str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
