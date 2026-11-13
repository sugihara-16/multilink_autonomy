// Independent world-point oracle for simultaneous vehicle and camera motion.
// This exercises the production IMU predictor and image-time SE3 reframe.
#include "articulated_rovio.hpp"
#include <rovio/ImuPrediction.hpp>
#include <memory>
#include <iostream>
#include <cassert>
using FS=rovio::FilterState<25,4,6,1,0>;
int main(){
  double worst=0;
  for(int type=0;type<4;++type){
    auto f=std::make_unique<FS>();auto& s=f->state_;
    s.setIdentity();s.aux().doVECalibration_=false;
    f->cov_.setIdentity();
    for(int i=0;i<25;++i){f->fsm_.isValid_[i]=i==0;s.CfP(i).camID_=-1;}
    s.qCM(0).setIdentity();s.MrMC(0).setZero();
    const V3D world(.5,1.,6.),velocity(.5,.1,0.);
    s.MvM()=-velocity;s.CfP(0).camID_=0;
    s.CfP(0).set_nor(LWF::NormalVectorElement(world.normalized()));
    s.CfP(0).set_warp_nor(Eigen::Matrix2d::Identity());
    s.dep(0).setType(type);s.dep(0).setParameter(world.norm());
    rovio::ImuPrediction<FS> prediction;
    prediction.g_=V3D(0,0,-9.8);
    prediction.meas_.template get<FS::mtPredictionMeas::_gyr>().setZero();
    prediction.meas_.template get<FS::mtPredictionMeas::_acc>()=V3D(0,0,9.8);
    FS::mtPredictionNoise noise;noise.setIdentity();
    for(int step=1;step<=4000;++step){
      FS::mtState next=s;prediction.evalPrediction(next,s,noise,.001);s=next;
      const double t=step*.001;
      if(step%50==0){
        const QPD q(Eigen::Quaterniond(Eigen::AngleAxisd(.3*t,V3D(.3,.7,.2).normalized())));
        const V3D centre(.4*std::sin(t),.3*std::cos(t),.1*std::sin(2*t));
        active_gimbal::reframe(*f,{q,centre});
        const V3D reconstructed=s.WrWM()+s.qWM().rotate(V3D(s.MrMC(0)+s.qCM(0).inverseRotate(V3D(s.CfP(0).get_nor().getVec()*s.dep(0).getDistance()))));
        worst=std::max(worst,(reconstructed-world).norm());
        assert((s.WrWM()-velocity*t).norm()<1e-10);
        assert((s.MvM()+velocity).norm()<1e-10);
        assert((reconstructed-world).norm()<.002);
      }
    }
  }
  std::cout<<"Simultaneous motion, 1 kHz IMU, 20 Hz camera, four depth types: max world landmark error "<<worst<<" m; no vehicle velocity or pose corruption.\n";
}
