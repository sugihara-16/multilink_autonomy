#!/usr/bin/env python3
"""Build a separate diagnostic binary from the current ROS1 ROVIO build flags."""
import argparse,json,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'logs/zab-offline/livo_comparison'
def build(count):
 p=ROOT.parents[3]/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';OUT.mkdir(parents=True,exist_ok=True)
 flags={line.split(' = ')[0]:shlex.split(line.split(' = ')[1]) for line in (p/'flags.make').read_text().splitlines() if ' = ' in line}
 defines=[f'-DROVIO_NMAXFEATURE={count}' if s.startswith('-DROVIO_NMAXFEATURE=') else '-DROVIO_NCAM=1' if s.startswith('-DROVIO_NCAM=') else s for s in flags['CXX_DEFINES']]
 obj=OUT/f'audit{count}.o';compile=['/usr/bin/c++',*defines,*flags['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(ROOT/'rovio_monocular.cpp'),'-o',str(obj)]
 link=shlex.split((p/'link.txt').read_text());link=[str(obj) if s=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else s for s in link];link[link.index('-o')+1]=str(OUT/f'audit{count}')
 (OUT/f'build{count}.json').write_text(json.dumps([compile,link],indent=2))
 with (OUT/f'build{count}.log').open('w') as f:
  for cmd in [compile,link]:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('counts',type=int,nargs='+');a=p.parse_args()
 for count in a.counts:build(count)
