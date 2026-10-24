#!/usr/bin/env python3
"""Extend the frozen experiment owner without editing the selected sources."""
import difflib
from pathlib import Path
import sys
from fixed_view_array import VIEWS
from run_lio_failure_comparisons import verify_selected

HERE = Path(__file__).resolve().parent


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('Selected owner no longer matches extension anchor: '+old[:80])
    return text.replace(old, new, 1)


def effective_sources():
    original = (HERE/'unipilot_run.py').read_text()
    source = original
    source = replace_once(source, "def run(args):",
        "from fixed_view_array import add_bridge_topics, start_views, VIEWS\n\ndef run(args):")
    source = replace_once(source, "    snapshot=out/'implementation';snapshot.mkdir()",
        "    snapshot=out/'implementation';snapshot.mkdir()\n"
        "    (snapshot/'effective_array_owner.py').write_text(ARRAY_SOURCE)\n"
        "    (snapshot/'array_owner.patch').write_text(ARRAY_DIFF)\n"
        "    (out/'array_pacer.py').write_text(ARRAY_PACER)\n")
    source = replace_once(source, "        bridge=dict(active_lidar_diagnostics=",
        "        add_bridge_topics(types)\n        bridge=dict(active_lidar_diagnostics=")
    source = replace_once(source, "NATIVE/'unipilot_gimbal_sim.launch.py'",
        "NATIVE/'unipilot_view_array.launch.py'")
    source = replace_once(source, "        if args.fixed_gravity_imu:start('gravity_state'",
        "        start_views(out,start,encoder,monitor,rovio_binary,imu_remap,launch_path,BUILD,WS,fast_extra,args)\n"
        "        start('array_input_audit',['python3',HERE/'audit_view_array_inputs.py','--output',out],True)\n"
        "        if args.fixed_gravity_imu:start('gravity_state'")
    source = replace_once(source, "HERE/'pace_simulation.py'", "out/'array_pacer.py'")
    source = replace_once(source, "                        while True:\n                            try:\n                                state=json.loads((out/'pacing.json').read_text())",
        "                        for i in range(len(args.pacing_extra_lidar)):\n"
        "                            required['extra_lidar_'+str(i)]=int((target+1e-7)/.1)*.1\n"
        "                        for name in VIEWS:\n"
        "                            required['array_vio_'+name]=int((target+1e-7)/.05)*.05\n"
        "                        while True:\n                            try:\n                                state=json.loads((out/'pacing.json').read_text())")
    pacer = (HERE/'pace_simulation.py').read_text()
    pacer = replace_once(pacer, "from pacing_limits import planner_lag_limit",
        "import sys\nsys.path.insert(0, "+repr(str(HERE))+")\nfrom pacing_limits import planner_lag_limit")
    pacer = replace_once(pacer, "node=Node();last_save=0.",
        "for view in "+repr(list(VIEWS))+":\n"
        "    key='array_vio_'+view;extra_names.append(key)\n"
        "    subs.append(rospy.Subscriber('/view_array/'+view+'/rovio/odometry',Odometry,lambda m,key=key:record(key,m),queue_size=100,tcp_nodelay=True))\n"
        "node=Node();last_save=0.")
    patch = ''.join(difflib.unified_diff(original.splitlines(True), source.splitlines(True),
                                       fromfile='selected/unipilot_run.py', tofile='effective_array_owner.py'))
    return source, pacer, patch


if __name__ == '__main__':
    verify_selected()
    source, pacer, patch = effective_sources()
    for name in VIEWS:
        sys.argv.extend(['--pacing-extra-lidar', '/array_'+name+'_mimosa/lidar/manager/debug'])
    namespace = dict(__name__='__main__', __file__=str(HERE/'unipilot_run.py'),
                     ARRAY_SOURCE=source, ARRAY_PACER=pacer, ARRAY_DIFF=patch)
    exec(compile(source, str(HERE/'unipilot_run.py'), 'exec'), namespace)
