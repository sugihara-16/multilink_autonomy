#!/usr/bin/env python3
"""Isolate an odometry transport-queue fix; preserve all estimator arithmetic."""
import difflib,hashlib,json,shlex,subprocess
from build import HERE,WS,BUILD,flags


def main():
    original=WS/'build/rovio-joint-geometry/rovio_gimbal.cpp'
    out=WS/'build/rovio-lossless-output';out.mkdir(exist_ok=False)
    old=original.read_text()
    anchor='  rovio::RovioNode<Filter> node(nh,np,filter);'
    assert old.count(anchor)==1
    new=old.replace(anchor,anchor+'\n  // Keep every estimate during callback catch-up; no filter-state changes.\n  node.pubOdometry_.shutdown();\n  node.pubOdometry_=nh.advertise<nav_msgs::Odometry>("rovio/odometry",2000);')
    dest=out/'rovio_gimbal.cpp';dest.write_text(new)
    (out/'transport_only.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(original),tofile=str(dest))))
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make')
    defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
    obj=out/'rovio_gimbal.o'
    command=['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
    subprocess.run(command,check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'rovio_gimbal');subprocess.run(link,check=True)
    paths=[original,dest,HERE/'build_rovio_lossless_output.py',HERE/'rovio_joint_information.hpp',HERE/'rovio_joint_geometry.hpp',HERE/'gimbal_rovio.hpp',HERE/'imu_bias_model.hpp',out/'rovio_gimbal']
    (out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,compiler=command,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
    print('Built',out,flush=True)


if __name__=='__main__':main()
