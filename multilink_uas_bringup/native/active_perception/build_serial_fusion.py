#!/usr/bin/env python3
"""Serialize LiDAR/odometry callbacks; retain the dedicated high-rate IMU queue."""
from pathlib import Path
import shlex,subprocess,hashlib,json
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3];out=WS/'build/mimosa-speed';out.mkdir(exist_ok=True)
base=WS/'comparison_ws';src=base/'src/mimosa/src/mimosa_node.cpp';s=src.read_text();assert s.count('ros::AsyncSpinner spinner(2);')==1
s=s.replace('ros::AsyncSpinner spinner(2);','ros::AsyncSpinner spinner(1);').replace('  imu_thread.join();','  spinner.stop();\n  imu_thread.join();');dest=out/'mimosa_serial_node.cpp';dest.write_text(s)
d=base/'build/mimosa/CMakeFiles/mimosa_node.dir';f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l};obj=out/'mimosa_serial_node.o'
subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-O1','-c',str(dest),'-o',str(obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'mimosa_serial_node');cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in cmd];subprocess.run(cmd,cwd=d.parents[1],check=True)
(out/'serial_manifest.json').write_text(json.dumps(dict(original_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),modified_sha256=hashlib.sha256(s.encode()).hexdigest(),change='Two global callback workers to one; separate IMU worker unchanged; stop global spinner before worker join and manager destruction'),indent=2)+'\n')
