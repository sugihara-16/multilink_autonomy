#!/usr/bin/env python3
"""Keep the frozen weathered owner, replacing only its view scoring wrapper."""
import sys
from run_weathered_view_array import paced_guarded_sources
from run_view_array import HERE, VIEWS, replace_once
from run_lio_failure_comparisons import verify_selected


def ablation_sources():
    source, pacer, patch = paced_guarded_sources()
    source = replace_once(source, "HERE/'weathered_planner_node.py'", "HERE/'ablation_planner_node.py'")
    return source, pacer, patch


if __name__ == '__main__':
    verify_selected()
    source, pacer, patch = ablation_sources()
    for name in VIEWS:
        sys.argv.extend(['--pacing-extra-lidar', '/array_' + name + '_mimosa/lidar/manager/debug'])
    exec(compile(source, str(HERE/'unipilot_run.py'), 'exec'),
         dict(__name__='__main__', __file__=str(HERE/'unipilot_run.py'),
              ARRAY_SOURCE=source, ARRAY_PACER=pacer, ARRAY_DIFF=patch))
