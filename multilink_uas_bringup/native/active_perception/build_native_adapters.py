#!/usr/bin/env python3
"""Build isolated ROS2 image and ROS1 sensor adapters with unchanged messages."""
import subprocess,shlex,json,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3];out=WS/'build/native-adapters';out.mkdir(exist_ok=True)
shell='source /opt/ros/humble/setup.bash\n'+shlex.join(['cmake','-S',str(HERE/'performance'),'-B',str(out),'-DCMAKE_BUILD_TYPE=Release','-DPython3_EXECUTABLE=/usr/bin/python3','-DPYTHON_EXECUTABLE=/usr/bin/python3'])+'\n'+shlex.join(['cmake','--build',str(out),'-j2'])
subprocess.run(['bash','--noprofile','--norc','-ec',shell],check=True)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
lib='/root/ros1_noetic/install_isolated/lib'
for name in ['native_slam_inputs','native_monitor']:
 subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],'-std=c++17','-O3',str(HERE/('performance/'+name+'.cpp')),'-L'+lib,'-Wl,-rpath,'+lib,'-lroscpp','-lrosconsole','-lrostime','-lroscpp_serialization','-lxmlrpcpp','-lboost_system','-pthread','-o',str(out/name)],check=True)
(out/'manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'performance/native_image_encoder.cpp',HERE/'performance/native_slam_inputs.cpp',HERE/'performance/native_monitor.cpp',HERE/'performance/native_clock_bridge.cpp',out/'native_clock_bridge',out/'native_image_encoder',out/'native_slam_inputs',out/'native_monitor']},indent=2)+'\n')
