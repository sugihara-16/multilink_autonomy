#!/usr/bin/env python3
"""Parallel correspondence lookup, retaining the original serial reduction order."""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3];out=WS/'build/mimosa-speed';out.mkdir(exist_ok=True)
base=WS/'comparison_ws';source=base/'src/mimosa';rel=Path('mimosa/lidar/geometric_factor.hpp')
s=(source/'include'/rel).read_text();original=s
old='#if USE_OPENMP\n#pragma omp parallel for num_threads(n_threads)\n#endif'
assert s.count(old)==1
s=s.replace(old,'#pragma omp parallel for num_threads(4) if(!is_binary_ && !config_.drpm_enabled)')
# Existing build uses USE_OPENMP=TRUE, where TRUE is not defined, hence its
# accumulation was strictly serial. Keep that exact order for ordinary LIO.
begin=s.index('      // Accumulate the Hessian\n');end=s.index('\n      if (is_binary_) {',begin)
s=s[:begin]+'      if (is_binary_ || config_.drpm_enabled) {\n'+s[begin:end]+'\n      }'+s[end:]
marker='    // Combine the thread results\n'
reduction='''    if (!is_binary_ && !config_.drpm_enabled) {
      for (size_t i=0; i<cloud_source_.points.size(); ++i) {
        if (statuses_[i]!=RejectStatus::Valid) continue;
        const auto& J_source=J_source_whitened_weighted_arr[i];
        const double e=e_whitened_weigehted_arr[i];
        J_source_T_J_source_thread[0] += J_source.transpose()*J_source;
        J_source_T_b_thread[0] += J_source.transpose()*e;
        f_thread[0] += e*e;
      }
    }
'''
assert s.count(marker)==1;s=s.replace(marker,reduction+marker)
header=out/'include'/rel;header.parent.mkdir(parents=True,exist_ok=True);header.write_text(s)
d=base/'build/mimosa/CMakeFiles/mimosa_lib.dir';flags={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
assert '-DUSE_OPENMP=TRUE' in flags['CXX_DEFINES']
obj=out/'geometric.o';active=WS/'build/active_perception'
subprocess.run(['c++',*flags['CXX_DEFINES'],f'-I{out}/include',*flags['CXX_INCLUDES'],f'-I{HERE}',*flags['CXX_FLAGS'],'-c',str(active/'geometric.cpp'),'-o',str(obj)],check=True)
imu_source=source/'src/imu/manager.cpp'
imu=imu_source.read_text();needle_imu='size_t Manager::getNumMeasurementsBetween(const double t1, const double t2)\n{'
assert imu.count(needle_imu)==1
imu=imu.replace(needle_imu,needle_imu+'\n  std::lock_guard<std::mutex> lock(buffer_mutex_);')
imu_path=out/'imu_manager.cpp';imu_path.write_text(imu);imu_obj=out/'imu_manager.o'
subprocess.run(['c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-c',str(imu_path),'-o',str(imu_obj)],check=True)
cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'libmimosa_lib.so')
cmd=[str(imu_obj) if x.endswith('/imu/manager.cpp.o') else str(obj) if x.endswith('/lidar/geometric.cpp.o') else str(active/'drpm_registration.o') if x.endswith('/lidar/drpm_registration.cpp.o') else x for x in cmd]
subprocess.run(cmd,cwd=d.parents[1],check=True)
(out/'manifest.json').write_text(json.dumps(dict(original_sha256=hashlib.sha256(original.encode()).hexdigest(),modified_sha256=hashlib.sha256(s.encode()).hexdigest(),imu_original_sha256=hashlib.sha256(imu_source.read_bytes()).hexdigest(),imu_modified_sha256=hashlib.sha256(imu.encode()).hexdigest(),change='Parallel independent point matching, original serial summation; unary DRPM-OFF only; lock IMU deque count traversal with existing buffer mutex'),indent=2)+'\n')

subprocess.run(['c++','-shared','-fPIC','-O2',str(HERE/'performance/tbb_limit.cpp'),'-ltbb','-o',str(out/'libtbb_limit.so')],check=True)
