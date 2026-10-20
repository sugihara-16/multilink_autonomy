#!/usr/bin/env python3
"""Isolated temporal velocity node, using the verified batched fusion library.

The disabled branch retains the existing independent-factor behavior. Existing
binaries and source headers are never replaced. This is an experimental error
model, not a correction of a proven implementation bug.
"""
import hashlib,json,shlex,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parent.parents[3]


def main():
    out=WS/'build/mimosa-temporal-velocity';out.mkdir(exist_ok=True)
    if (out/'manifest.json').exists():raise ValueError('Preserve completed build')
    original=HERE/'performance/velocity_odometry_manager.hpp';s=original.read_text()
    def replace(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace('#include "body_velocity_factor.hpp"','#include "temporal_velocity_factor.hpp"')
    replace('  double period_=1.,scale_=1.;','''  double period_=1.,scale_=1.;
  bool temporal_enabled_=false,have_previous_velocity_=false;
  double rho_=0.,innovation_variance_=1.;
  gtsam::Key previous_velocity_key_=0;
  gtsam::Vector3 previous_measured_;
  gtsam::Matrix3 previous_covariance_;
  gtsam::Rot3 previous_rotation_;
  gtsam::SharedNoiseModel innovation_noise_;
  std::ofstream temporal_audit_;''')
    replace('    subscribeIfEnabled(pnh);','''    pnh.param("odometry/manager/temporal_velocity_enabled",temporal_enabled_,false);
    if(temporal_enabled_){
      double lag=0.;pnh.getParam("graph/manager/smoother/lag",lag);
      pnh.getParam("odometry/manager/temporal_velocity_rho",rho_);
      pnh.getParam("odometry/manager/temporal_velocity_innovation_variance",innovation_variance_);
      if(!std::isfinite(rho_)||rho_<0.||rho_>=1.||!std::isfinite(innovation_variance_)||innovation_variance_<1.-rho_*rho_-1e-12||lag<2.*period_)
        throw std::invalid_argument("Invalid temporal velocity calibration or insufficient smoother history");
      innovation_noise_=gtsam::noiseModel::Isotropic::Variance(3,innovation_variance_);
      temporal_audit_.open(directory+"temporal_velocity_factors.csv");
      temporal_audit_<<std::setprecision(17)<<"stamp_ns,previous_stamp_s,rho,innovation_variance,initial\\n";
    }
    subscribeIfEnabled(pnh);''')
    replace('      factors.add(BodyVelocityFactor(X(0),V(0),measured,gtsam::noiseModel::Gaussian::Covariance(C)));','''      if(temporal_enabled_ && have_previous_velocity_){
        if(std::abs(corrected_ts_-last_update_-period_)>1e-7)throw std::runtime_error("Temporal velocity input gap");
        factors.add(TemporalVelocityFactor(X(previous_velocity_key_),V(previous_velocity_key_),X(0),V(0),
            previous_measured_,measured,previous_covariance_,C,body_pose.rotation().inverse()*previous_rotation_,rho_,innovation_noise_));
      }else factors.add(BodyVelocityFactor(X(0),V(0),measured,gtsam::noiseModel::Gaussian::Covariance(C)));''')
    replace('      last_update_=corrected_ts_;const auto u=up.unitVector();','''      if(temporal_enabled_){
        temporal_audit_<<msg->header.stamp.toNSec()<<','<<(have_previous_velocity_?last_update_:corrected_ts_)<<','<<rho_<<','<<innovation_variance_<<','<<(!have_previous_velocity_)<<'\\n';
        previous_velocity_key_=new_key_;previous_measured_=measured;previous_covariance_=C;
        previous_rotation_=body_pose.rotation();have_previous_velocity_=true;
      }
      last_update_=corrected_ts_;const auto u=up.unitVector();''')
    header=out/'temporal_velocity_manager.hpp';header.write_text(s)
    original_node=WS/'build/mimosa-batched-vision/ordered_mimosa_node.cpp'
    node=out/'ordered_mimosa_node.cpp';source=original_node.read_text()
    assert source.count('#include "velocity_odometry_manager.hpp"')==1
    node.write_text(source.replace('#include "velocity_odometry_manager.hpp"','#include "temporal_velocity_manager.hpp"'))
    d=WS/'comparison_ws/build/mimosa/CMakeFiles/mimosa_node.dir'
    f={l.split(' = ')[0]:shlex.split(l.split(' = ')[1]) for l in (d/'flags.make').read_text().splitlines() if ' = ' in l}
    obj=out/'node.o'
    subprocess.run(['c++',*f['CXX_DEFINES'],*f['CXX_INCLUDES'],*f['CXX_FLAGS'],'-I'+str(HERE/'performance'),'-DMIMOSA_BODY_VELOCITY','-O1','-c',str(node),'-o',str(obj)],check=True)
    cmd=shlex.split((d/'link.txt').read_text());cmd[cmd.index('-o')+1]=str(out/'mimosa_temporal_velocity_node')
    lib=WS/'build/mimosa-batched-vision/libmimosa_lib.so'
    cmd=[str(obj) if x.endswith('/src/mimosa_node.cpp.o') else str(lib) if x.endswith('/libmimosa_lib.so') else x for x in cmd]
    subprocess.run(cmd,cwd=d.parents[1],check=True)
    paths=[Path(__file__).resolve(),original,header,original_node,node,HERE/'performance/temporal_velocity_factor.hpp',lib,out/'mimosa_temporal_velocity_node']
    (out/'manifest.json').write_text(json.dumps(dict(scope=__doc__,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}),indent=2)+'\n')
    print('BUILT',out,flush=True)


if __name__=='__main__':main()
