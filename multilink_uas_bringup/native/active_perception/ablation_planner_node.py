#!/usr/bin/env python3
"""Identical weathered planner lifecycle with a scoring-only ablation."""
from pathlib import Path
from view_preparation import replace_once
path = Path(__file__).with_name('weathered_planner_node.py')
source = replace_once(path.read_text(),
    'from committed_tracking_view import CommittedTrackingView as Fixation',
    'from view_information_ablation import InformationAblationView as Fixation')
exec(compile(source, str(path), 'exec'), dict(__name__='__main__', __file__=str(path)))
