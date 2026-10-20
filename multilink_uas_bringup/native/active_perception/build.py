#!/usr/bin/env python3
"""Build isolated adapters; never overwrite the adopted estimator binaries."""
import argparse
import hashlib
import json
import shlex
import subprocess
from pathlib import Path

HERE=Path(__file__).resolve().parent
NATIVE=HERE.parent
WS=NATIVE.parents[3]
BUILD=WS/'build/active_perception'
SOURCES={
 'rpg_information_field':('https://github.com/uzh-rpg/rpg_information_field.git','76d7b834128bd3488d1aae3888a79f3d5216a4ab'),
 'minkindr_fif':('https://github.com/ethz-asl/minkindr.git','564f12639a8447d4d3e5e7707851424302941056')}

def run(cmd,**kw):
    print(shlex.join(map(str,cmd)),flush=True);subprocess.run(list(map(str,cmd)),check=True,**kw)

def flags(path):
    return {l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in path.read_text().splitlines() if ' = ' in l}

def main(parts):
    BUILD.mkdir(parents=True,exist_ok=True)
    manifest={}
    if 'bridge' in parts:
        run(['python3',HERE/'build_reliable_bridge.py'])
        run(['python3',HERE/'build_zmq_retry.py'])
    for name,(url,rev) in SOURCES.items():
        d=WS/'assets'/name
        if not d.exists():
            run(['git','clone','--no-checkout',url,d])
            run(['git','-C',d,'checkout','--detach',rev])
        actual=subprocess.check_output(['git','-C',str(d),'rev-parse','HEAD'],text=True).strip()
        if actual!=rev: raise RuntimeError(f'{name} revision mismatch: {actual}; expected {rev}')
        if subprocess.check_output(['git','-C',str(d),'diff','--name-only']):raise RuntimeError(f'{name} modified')
        manifest[name]=dict(url=url,revision=rev)
    fif=WS/'assets/rpg_information_field'
    if 'fif' in parts:
        includes=[fif/'third_party/rpg_common/include',fif/'third_party/rpg_vi_utils/include',
                  WS/'assets/minkindr_fif/minkindr/include','/usr/include/eigen3']
        run(['c++','-std=c++17','-O2','-fPIC','-shared',*[f'-I{x}' for x in includes],
             HERE/'fif_bridge.cpp','-lglog','-o',BUILD/'libfif_active.so'])
    if 'rovio' in parts:
        from patch_imu_prediction import patch
        prediction=WS/'slam_ros1_ws/src/rovio/include/rovio/ImuPrediction.hpp'
        patched=BUILD/'include/rovio/ImuPrediction.hpp';patched.parent.mkdir(parents=True,exist_ok=True)
        patched.write_text(patch(prediction.read_text()))
        manifest['rovio_imu_prediction_upstream_sha256']=hashlib.sha256(prediction.read_bytes()).hexdigest()
        manifest['rovio_imu_prediction_patch_sha256']=hashlib.sha256(patched.read_bytes()).hexdigest()
        manifest['rovio_imu_bias_model_sha256']=hashlib.sha256((HERE/'imu_bias_model.hpp').read_bytes()).hexdigest()
        source=WS/'slam_ros1_ws/src/rovio/include/rovio/ImgUpdate.hpp'
        s=source.read_text();needle='    if(isFinished){ // gets called if this is the first call\n'
        if s.count(needle)!=1:raise RuntimeError('ROVIO image hook location changed')
        dest=BUILD/'include/rovio/ImgUpdate.hpp';dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text('#include "gimbal_rovio.hpp"\n'+s.replace(needle,needle+'      active_gimbal::before_image(filterState);\n'))
        directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir'
        f=flags(directory/'flags.make');defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
        obj=BUILD/'rovio_gimbal.o'
        run(['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{BUILD}/include',f'-I{HERE}',
             *f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',HERE/'rovio_gimbal.cpp','-o',obj])
        link=shlex.split((directory/'link.txt').read_text())
        link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
        link[link.index('-o')+1]=str(BUILD/'rovio_gimbal');run(link)
        manifest['rovio_img_update_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    if 'mimosa' in parts:
        source=WS/'comparison_ws/src/mimosa/src/lidar/drpm_registration.cpp'
        s=source.read_text();needle=' if(status!="ok")return nullptr;'
        if s.count(needle)!=1:raise RuntimeError('DRPM telemetry hook changed')
        s=s.replace('#include <iomanip>','#include <iomanip>\n#include "drpm_telemetry.hpp"')
        # Include telemetry after the kernel header has defined its result type.
        s=s.replace(needle,needle+'\n publishActiveDrpm(stamp,T_W_L,result);')
        s += (HERE/'diagnostic_registration.cpp.inc').read_text()
        dest=BUILD/'drpm_registration.cpp';dest.write_text(s)
        directory=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_lib.dir'
        f=flags(directory/'flags.make');obj=BUILD/'drpm_registration.o'
        run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],f'-I{HERE}',*f['CXX_FLAGS'],
             '-c',dest,'-o',obj])
        # Independently compile a read-only hook in the ordinary (DRPM OFF) branch.
        geo=WS/'comparison_ws/src/mimosa/src/lidar/geometric.cpp'
        g=geo.read_text()
        marker='  // Scan to map\n  factor_ ='
        if g.count(marker)!=1:raise RuntimeError('Normal LiDAR hook changed')
        g='#include "diagnostic_registration.hpp"\n'+g.replace(marker,
          '  if(activeDiagnosticsEnabled()) publishActiveDiagnostics(values.at<gtsam::Pose3>(key),config.T_B_L,sm_Be_cloud_ds_,ivox_map_,config.scan_to_map,ts_);\n'+marker)
        needle='if (config.scan_to_map.drpm_enabled && config.scan_to_map.drpm_noise_model == 2)'
        if g.count(needle)!=1:raise RuntimeError('Map covariance hook changed')
        g=g.replace(needle,'if ((config.scan_to_map.drpm_enabled || activeDiagnosticsEnabled()) && config.scan_to_map.drpm_noise_model == 2)')
        gd=BUILD/'geometric.cpp';gd.write_text(g);go=BUILD/'geometric.o'
        run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],f'-I{HERE}',*f['CXX_FLAGS'],'-c',gd,'-o',go])
        link=shlex.split((directory/'link.txt').read_text())
        replacements={'CMakeFiles/mimosa_lib.dir/src/lidar/drpm_registration.cpp.o':str(obj),
                      'CMakeFiles/mimosa_lib.dir/src/lidar/geometric.cpp.o':str(go)}
        link=[replacements.get(x,x) for x in link]
        link[link.index('-o')+1]=str(BUILD/'libmimosa_lib.so')
        run(link,cwd=directory.parents[1])
        manifest['mimosa_source_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    if 'test' in parts:
        f=flags(WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir/flags.make')
        run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],f'-I{HERE}','-O1',HERE/'test_gimbal.cpp',
             '-o',BUILD/'test_gimbal',f'-L{WS}/slam_ros1_ws/devel/lib','-lrovio','-lopencv_core','-lopencv_imgproc'])
    if 'replay' in parts:
        source=(NATIVE/'rovio_monocular.cpp').read_text()
        needle='for(int c=0;c<1;c++)node.imgCallback(batch[c],c);'
        if source.count(needle)!=1:raise RuntimeError('Replay hook location changed')
        source='#include "gimbal_rovio.hpp"\n'+source.replace(needle,
            'active_gimbal::samples()[stamp]={filter->init_.state_.qCM(0),filter->init_.state_.MrMC(0)};'+needle)
        dest=BUILD/'rovio_static_replay.cpp';dest.write_text(source)
        directory=WS/'slam_ros1_ws/build/rovio/CMakeFiles/rovio_node.dir'
        f=flags(directory/'flags.make');defines=[x for x in f['CXX_DEFINES'] if not x.startswith(('-DROVIO_NCAM=','-DROVIO_NMAXFEATURE='))]
        obj=BUILD/'rovio_static_replay.o'
        run(['c++',*defines,'-DROVIO_NCAM=1','-DROVIO_NMAXFEATURE=25',f'-I{BUILD}/include',f'-I{HERE}',
             *f['CXX_INCLUDES'],'-O2','-DNDEBUG','-c',dest,'-o',obj])
        link=shlex.split((directory/'link.txt').read_text())
        link=[str(obj) if x=='CMakeFiles/rovio_node.dir/src/rovio_node.cpp.o' else x for x in link]
        link[link.index('-o')+1]=str(BUILD/'rovio_static_replay');run(link)
    if (BUILD/'source_manifest.json').exists():
        old=json.loads((BUILD/'source_manifest.json').read_text());old.update(manifest);manifest=old
    (BUILD/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('parts',nargs='*',default=['fif','rovio','mimosa','bridge']);a=p.parse_args();main(a.parts)
