#!/usr/bin/env python3
"""Build an isolated fixed-camera reference with upstream ImgUpdate, no gimbal hook.
Same input wrapper and physically specified IMU prediction as production.
"""
import argparse,hashlib,json,shlex
from pathlib import Path
from build import WS,HERE,BUILD,flags,run
from patch_imu_prediction import patch

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--measurement-audit',action='store_true');args=parser.parse_args()
 out=BUILD/('measurement_audit' if args.measurement_audit else 'fixed_reference');inc=out/'include/rovio';inc.mkdir(parents=True,exist_ok=True)
 source=WS/'slam_ros1_ws/src/rovio/include/rovio'
 (inc/'ImgUpdate.hpp').write_bytes(((BUILD/'include/rovio/ImgUpdate.hpp') if args.measurement_audit else (source/'ImgUpdate.hpp')).read_bytes())
 (inc/'ImuPrediction.hpp').write_text(patch((source/'ImuPrediction.hpp').read_text()))
 cpp=HERE/'rovio_gimbal.cpp'
 if args.measurement_audit:
  text=cpp.read_text().replace('total_count\\n','total_count,mx,my,mz\\n')
  text=text.replace('const auto& n=s.CfP(i).get_nor().getVec();','const auto& n=s.CfP(i).get_nor().getVec(); const auto nm=s.aux().feaCoorMeas_[i].get_nor().getVec();')
  text=text.replace("<<stats.totCount_<<'\\n';","<<stats.totCount_<<','<<nm.x()<<','<<nm.y()<<','<<nm.z()<<'\\n';")
  cpp=out/'measurement_audit.cpp';cpp.write_text(text)
 d=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir';f=flags(d/'flags.make');defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))];obj=out/'reference.o'
 run(['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{out}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',cpp,'-o',obj])
 link=shlex.split((d/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link];link[link.index('-o')+1]=str(out/'rovio_fixed_reference');run(link)
 (out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,image_update_sha256=hashlib.sha256((inc/'ImgUpdate.hpp').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((out/'rovio_fixed_reference').read_bytes()).hexdigest()),indent=2)+'\n')
if __name__=='__main__':main()
