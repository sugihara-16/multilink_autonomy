#!/usr/bin/env python3
from pathlib import Path
import core
from motion_view_planner import Planner
from preview_preparation import planner_source
core.Planner=Planner
path=Path(__file__).with_name('planner_node.py')
exec(compile(planner_source(path.read_text()),str(path),'exec'),dict(__name__='__main__',__file__=str(path)))
