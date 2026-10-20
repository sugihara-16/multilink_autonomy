#include "gimbal_rovio.hpp"
#include "rovio_lidar_bootstrap.hpp"
#include <rovio/FilterStates.hpp>
#include <memory>
#include <iostream>
void require(bool test,const char* why){if(!test)throw std::runtime_error(why);}
int main(){
  using FS=rovio::FilterState<25,4,6,1,0>;
  auto fs=std::make_unique<FS>();auto& s=fs->state_;auto& f=fs->fsm_.features_[0];
  auto& K=active_depth_bootstrap::intrinsics();K.setIdentity();K(0,0)=K(1,1)=240.;K(0,2)=240.;K(1,2)=135.;
  int count=0;
  for(int type=0;type<4;++type)for(int n=0;n<40;++n){
    fs->t_=1.+.1*count++;const long long t=std::llround(fs->t_*1e9);
    s.MrMC(0)=V3D::Random();s.qCM(0)=s.qCM(0).exponentialMap(V3D::Random()*2.);
    LWF::NormalVectorElement ray;ray.setFromVector(V3D(0.,0.,1.));s.CfP(0).set_nor(ray);
    s.dep(0).setType(type);s.dep(0).setParameter(10.);fs->cov_.setRandom();
    const auto original_cov=fs->cov_;const auto original_position=s.WrWM(),original_velocity=s.MvM();
    auto& scan=active_depth_bootstrap::scans()[t];
    for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y)scan.push_back(s.qCM(0).inverseRotate(V3D(.04*x,.04*y,5.))+s.MrMC(0));
    active_depth_bootstrap::enabled()=false;active_depth_bootstrap::initialize(*fs,f,0);
    require(std::abs(s.dep(0).getDistance()-10.)<1e-12,"Disabled branch changed a feature");
    active_depth_bootstrap::enabled()=true;active_depth_bootstrap::initialize(*fs,f,0);
    require(std::abs(s.dep(0).getDistance()-5.)<1e-10,"Known extrinsic/parameterization incorrect");
    require((fs->cov_-original_cov).norm()==0.,"Bootstrap unexpectedly changed posterior covariance");
    require((s.WrWM()-original_position).norm()==0.&&(s.MvM()-original_velocity).norm()==0.,"Bootstrap changed body motion");
    // Same upstream relative prior strength, after transforming the new mean
    // through each of ROVIO's four depth parameterizations.
    double parameter_var=.5*std::pow(s.dep(0).getParameterDerivative()*s.dep(0).getDistance(),2);
    require(std::abs(parameter_var*std::pow(s.dep(0).getDistanceDerivative(),2)-12.5)<1e-10,"Relative prior changed");
    fs->t_+=.001;s.dep(0).setParameter(10.);active_depth_bootstrap::initialize(*fs,f,0);
    require(std::abs(s.dep(0).getDistance()-10.)<1e-12,"Stale scan used");
    active_depth_bootstrap::scans().clear();
  }
  require(active_depth_bootstrap::needs_scan(1000000002LL)&&!active_depth_bootstrap::needs_scan(1050000000LL),"10 Hz acquisition joining incorrect");
  std::cout<<count<<" frame/parameterization cases; disabled/stale identity and unchanged body/covariance: PASS\n";
}
