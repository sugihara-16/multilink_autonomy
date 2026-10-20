#!/usr/bin/env python3
"""Build an isolated ROVIO feature-capacity diagnostic; never replace defaults."""
import argparse,hashlib,json,shlex,subprocess
from pathlib import Path
from build import HERE,WS,BUILD,flags


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--features',type=int,default=75);args=parser.parse_args()
    if not 25 <= args.features <= 100:raise ValueError('Feature budget must be 25..100')
    out=WS/f'build/rovio-capacity-{args.features}';out.mkdir(parents=True,exist_ok=False)
    original=(HERE/'rovio_gimbal.cpp').read_text()
    assert original.count('FilterState<25,')==1 and original.count('i<25;')==2
    source=original.replace('FilterState<25,',f'FilterState<{args.features},').replace('i<25;',f'i<{args.features};')
    dest=out/'rovio_gimbal.cpp';dest.write_text(source)
    directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir'
    f=flags(directory/'flags.make');defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
    obj=out/'rovio_gimbal.o'
    command=['c++',*defines,'-DROVIO_NCAM=1',f'-DROVIO_NMAXFEATURE={args.features}',f'-I{BUILD}/include',f'-I{HERE}',*f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',str(dest),'-o',str(obj)]
    subprocess.run(command,check=True)
    link=shlex.split((directory/'link.txt').read_text());link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link];link[link.index('-o')+1]=str(out/'rovio_gimbal');subprocess.run(link,check=True)
    manifest=dict(scope=__doc__,features=args.features,compiler=command,linker=link,
                  original_source_sha256=hashlib.sha256(original.encode()).hexdigest(),
                  inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dest,HERE/'gimbal_rovio.hpp',HERE/'imu_bias_model.hpp',BUILD/'include/rovio/ImuPrediction.hpp',BUILD/'include/rovio/ImgUpdate.hpp']},
                  binary_sha256=hashlib.sha256((out/'rovio_gimbal').read_bytes()).hexdigest())
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT',out,flush=True)


if __name__=='__main__':main()
