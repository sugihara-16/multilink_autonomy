#!/usr/bin/env python3
"""Build optional AR(1) external-odometry node without replacing baseline binaries."""
import hashlib,json,shlex,subprocess,os,argparse
from pathlib import Path
HERE=Path(__file__).resolve().parent
WS=HERE.parent.parents[3]
parser=argparse.ArgumentParser();parser.add_argument('--translation-only',action='store_true');args=parser.parse_args()
variant='translation' if args.translation_only else 'temporal'
out=WS/('build/mimosa-'+variant);out.mkdir(exist_ok=True)
d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
for name,source,extra in [('test_'+variant+'_odometry_factor',HERE/('performance/test_'+variant+'_odometry_factor.cpp'),[]),
                           ('mimosa_'+variant+'_node',HERE/'performance/ordered_mimosa_node.cpp',['-DMIMOSA_'+variant.upper()+'_VIO'])]:
    obj=out/(name+'.o')
    subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],*extra,'-O1','-c',str(source),'-o',str(obj)],check=True)
    cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/name)
    cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else x for x in cmd]
    subprocess.run(cmd,cwd=d.parents[1],check=True)
    if name.startswith('test_'):
        env=dict(os.environ);env['LD_LIBRARY_PATH']=str(WS/'slam_ros1_ws/devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
        subprocess.run([str(out/name)],check=True,env=env)
sources=[HERE/'performance'/n for n in ['ordered_mimosa_node.cpp',variant+'_odometry_factor.hpp',variant+'_odometry_manager.hpp','test_'+variant+'_odometry_factor.cpp']]
(out/'manifest.json').write_text(json.dumps(dict(source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
    binary_sha256=hashlib.sha256((out/('mimosa_'+variant+'_node')).read_bytes()).hexdigest(),
    scope='Experimental '+variant+' external-odometry factors. Existing baseline binaries and managers unchanged.'),indent=2)+'\n')
