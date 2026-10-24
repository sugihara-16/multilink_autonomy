#!/usr/bin/env python3
from pathlib import Path
import core
from motion_view_planner import Planner
from view_preparation import planner_source, replace_once

core.Planner = Planner
path = Path(__file__).with_name('planner_node.py')
source = planner_source(path.read_text())
source = replace_once(source,
    'from retinal_tracking_view import RetinalTrackingView as Fixation',
    'from committed_tracking_view import CommittedTrackingView as Fixation')
exec(compile(source, str(path), 'exec'), dict(__name__='__main__', __file__=str(path)))
