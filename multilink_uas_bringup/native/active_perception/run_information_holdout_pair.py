#!/usr/bin/env python3
"""Scoring ablation with mandatory front reference, without four extra estimators.

Gazebo still renders the unchanged six-camera model. Only unused auxiliary
estimator processes and their completion waits are removed in both arms.
"""
import difflib
from fixed_view_array import VIEWS
from run_information_ablation_array import ablation_sources, HERE, verify_selected


def pair_sources():
    # The imported dictionaries are shared by the owner-generation modules.
    # The Gazebo model is generated in a separate process using original VIEWS.
    original=dict(VIEWS)
    try:
        VIEWS.clear()
        source,pacer,_=ablation_sources()
    finally:
        VIEWS.update(original)
    # Disable the additional estimators in the executed owner as well.
    source=source.replace('from fixed_view_array import add_bridge_topics, start_views, VIEWS',
                          'from fixed_view_array import add_bridge_topics, start_views, VIEWS\nVIEWS.clear()')
    patch=''.join(difflib.unified_diff((HERE/'unipilot_run.py').read_text().splitlines(True),source.splitlines(True),fromfile='selected/unipilot_run.py',tofile='holdout_pair_owner.py'))
    return source,pacer,patch

if __name__=='__main__':
    verify_selected();source,pacer,patch=pair_sources()
    exec(compile(source,str(HERE/'unipilot_run.py'),'exec'),dict(__name__='__main__',__file__=str(HERE/'unipilot_run.py'),ARRAY_SOURCE=source,ARRAY_PACER=pacer,ARRAY_DIFF=patch))
