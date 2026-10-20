#!/usr/bin/env python3
"""Compile a separate timestamp-ordered LIVO node; preserve the normal node."""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3];out=WS/'build/mimosa-compact';out.mkdir(exist_ok=True)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir';src=HERE/'performance/ordered_mimosa_node.cpp'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l};obj=out/'ordered_node.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-O1','-c',str(src),'-o',str(obj)],check=True)
binary=out/'mimosa_ordered_node';temporary=out/'mimosa_ordered_node.building'
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(temporary);cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
temporary.replace(binary)
(out/'ordered_manifest.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),change='Timestamp-ordered LiDAR/VIO with exact IMU prefix per event, unchanged samples/callbacks; zero-offset simulated sensors, deterministic interpolation boundary',binary_sha256=hashlib.sha256((out/'mimosa_ordered_node').read_bytes()).hexdigest()),indent=2)+'\n')
