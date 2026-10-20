#!/usr/bin/env python3
"""Skip an unchanged voxel-index rebuild; leave stored points/covariances intact."""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]
out=WS/'build/gtsam-points-speed';out.mkdir(exist_ok=True)
source=WS/'slam_ros1_ws/src/gtsam_points'
relative=Path('gtsam_points/ann/impl/incremental_voxelmap_impl.hpp')
original=(source/'include'/relative).read_text()
begin=original.index('    flat_voxels.erase(remove_counter, flat_voxels.end());')
end=original.index('\n  }\n\n  // Finalize',begin)
modified=original[:begin]+'    if (remove_counter != flat_voxels.end()) {\n'+original[begin:end]+'\n    }'+original[end:]
header=out/'include'/relative;header.parent.mkdir(parents=True,exist_ok=True);header.write_text(modified)
base=WS/'slam_ros1_ws/build/gtsam_points';directory=base/'CMakeFiles/gtsam_points.dir'
f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (directory/'flags.make').read_text().splitlines() if ' = ' in l}
obj=out/'ivox.o'
subprocess.run(['c++',*f['CXX_DEFINES'],f'-I{out}/include',*f['CXX_INCLUDES'],*f['CXX_FLAGS'],
    '-c',str(source/'src/gtsam_points/ann/ivox.cpp'),'-o',str(obj)],check=True)
cmd=shlex.split((directory/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libgtsam_points.so')
cmd=[str(obj) if x.endswith('/ann/ivox.cpp.o') else x for x in cmd]
subprocess.run(cmd,cwd=base,check=True)
(out/'manifest.json').write_text(json.dumps(dict(original_header_sha256=hashlib.sha256(original.encode()).hexdigest(),
    optimized_header_sha256=hashlib.sha256(modified.encode()).hexdigest(),change='Rebuild voxel index only when at least one voxel is erased'),indent=2)+'\n')
