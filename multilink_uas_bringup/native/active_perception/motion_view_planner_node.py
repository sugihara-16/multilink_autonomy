#!/usr/bin/env python3
"""Run the frozen ROS node with an explicit candidate policy and start gate."""
from pathlib import Path
import core
from motion_view_planner import Planner
core.Planner=Planner
path=Path(__file__).with_name('planner_node.py')
source=path.read_text()
old="['stage'] not in ('tracking','finish_hold','landing','complete')"
new="['stage'] not in (('approach','excitation','tracking','finish_hold','landing','complete') if self.cfg.get('preflight_view_selection',False) else ('tracking','finish_hold','landing','complete'))"
assert source.count(old)==1
source=source.replace(old,new)
exec(compile(source,str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
