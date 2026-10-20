#!/usr/bin/env python3
"""Check search, snapshot and mutation equivalence for the selected index."""
import argparse,json,os,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--neighbors',action='store_true');p.add_argument('--pruned',action='store_true');p.add_argument('--prefetch',action='store_true');a=p.parse_args()
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/('build/mimosa-compact-prefetch' if a.prefetch else 'build/mimosa-compact-neighbors' if a.neighbors else 'build/mimosa-compact-pruned' if a.pruned else 'build/mimosa-compact');out.mkdir(exist_ok=True)
a.neighbors=a.neighbors or a.prefetch
flags=(['-DUAS_IVOX_STATIC5=1','-DUAS_IVOX_AABB=1'] if a.neighbors or a.pruned else [])+(['-DUAS_IVOX_NEIGHBOR_CACHE=1'] if a.neighbors else [])+(['-DUAS_IVOX_PREFETCH=1'] if a.prefetch else [])
env=os.environ.copy();env['LD_LIBRARY_PATH']=f'{WS}/slam_ros1_ws/devel/lib:{WS}/comparison_ws/deps/lib:/root/ros1_noetic/install_isolated/lib'
result={}
for name in ['test_compact_ivox','test_compact_search']:
 exe=out/name
 subprocess.run(['c++','-O3','-std=c++17',*flags,'-I'+str(HERE),'-I/usr/include/eigen3','-I'+str(WS/'slam_ros1_ws/devel/include'),str(HERE/(name+'.cpp')),'-L'+str(WS/'slam_ros1_ws/devel/lib'),'-lgtsam_points','-o',str(exe)],env=env,check=True)
 r=subprocess.run([str(exe)],env=env,capture_output=True,text=True,check=True);result[name]=r.stdout
(out/'search_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
