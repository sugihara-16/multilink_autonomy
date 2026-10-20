#!/usr/bin/env python3
"""Build isolated transport executable; retain installed upstream conversions."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parents[3]
OUT = WS / 'build/reliable-bridge'
BRIDGE = Path('/root/unified_autonomy_stack/workspaces/ws_ros1_bridge/install/ros1_bridge')
shell = '\n'.join([
    'source /opt/ros/humble/setup.bash',
    f'source {BRIDGE.parent}/local_setup.bash',
    'export PKG_CONFIG_PATH=/root/ros1_noetic/install_isolated/lib/pkgconfig:${PKG_CONFIG_PATH:-}',
    shlex.join(['cmake', '-S', str(HERE/'performance/bridge'), '-B', str(OUT),
                '-DCMAKE_BUILD_TYPE=Release', '-DPython3_EXECUTABLE=/usr/bin/python3',
                '-DPYTHON_EXECUTABLE=/usr/bin/python3']),
    shlex.join(['cmake', '--build', str(OUT), '-j2']),
])
env = {k: os.environ[k] for k in ('HOME', 'PATH', 'LANG') if k in os.environ}
subprocess.run(['bash', '--noprofile', '--norc', '-ec', shell], env=env, check=True)
paths = [HERE/'performance/reliable_bridge.cpp', HERE/'performance/bridge/CMakeLists.txt',
         OUT/'reliable_bridge', BRIDGE/'lib/libros1_bridge.so']
(OUT/'manifest.json').write_text(json.dumps(
    {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, indent=2)+'\n')
