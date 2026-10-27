"""Isolated SE(3) camera adapter; never replace frozen UniPilot binaries."""
import argparse,hashlib,json,shlex,subprocess
from build import HERE,WS,BUILD,flags

def main():
    out=WS/'build/rovio-articulated';out.mkdir(exist_ok=True)
    include=out/'include/rovio';include.mkdir(parents=True,exist_ok=True)
    # Keep the tested IMU prediction and read-only geometry publisher.
    (include/'ImgUpdate.hpp').write_bytes((BUILD/'include/rovio/ImgUpdate.hpp').read_bytes())
    (out/'include/gimbal_rovio.hpp').write_bytes((HERE/'articulated_rovio.hpp').read_bytes())
    source=WS/'build/rovio-joint-geometry/rovio_gimbal.cpp'
    dest=out/'rovio_articulated.cpp';dest.write_bytes(source.read_bytes())
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make')
    defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
    obj=out/'rovio_articulated.o'
    command=['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{out}/include',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
    subprocess.run(command,check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'rovio_articulated');subprocess.run(link,check=True)
    paths=[dest,HERE/'articulated_rovio.hpp',include/'ImgUpdate.hpp',BUILD/'include/rovio/ImuPrediction.hpp',out/'rovio_articulated']
    (out/'manifest.json').write_text(json.dumps({'command':command,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}},indent=2))

def test():
    out=WS/'build/rovio-articulated';out.mkdir(exist_ok=True)
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(directory/'flags.make');obj=out/'test.o'
    subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],f'-I{HERE}','-O1','-c',str(HERE/'test_articulated_rovio.cpp'),'-o',str(obj)],check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
    link[link.index('-o')+1]=str(out/'test_se3');subprocess.run(link,check=True);subprocess.run([out/'test_se3'],check=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--test-only',action='store_true');a=p.parse_args()
    if not a.test_only:main()
    test()
