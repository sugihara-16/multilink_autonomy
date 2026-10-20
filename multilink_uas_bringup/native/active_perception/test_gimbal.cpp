#include "gimbal_rovio.hpp"
#include <rovio/FilterStates.hpp>
#include <iostream>
#include <memory>
int main(){
  using FS=rovio::FilterState<25,4,6,1,0>;
  auto storage=std::make_unique<FS>();auto& fs=*storage;fs.state_.aux().doVECalibration_=false;
  auto& s=fs.state_;s.qCM(0).setIdentity();s.MrMC(0)=V3D(.2,-.1,.3);
  fs.fsm_.isValid_[0]=true;s.CfP(0).camID_=0;s.CfP(0).trackWarping_=true;
  LWF::NormalVectorElement b;b.setFromVector(V3D(1,2,5));s.CfP(0).set_nor(b);s.CfP(0).set_warp_nor(Eigen::Matrix2d::Identity());
  s.dep(0).setType(1);s.dep(0).setParameter(5.);
  const V3D point=s.MrMC(0)+s.qCM(0).inverseRotate(V3D(s.CfP(0).get_nor().getVec()*s.dep(0).getDistance()));
  fs.cov_.setIdentity();auto original=fs.cov_;
  active_gimbal::Extrinsic ex;ex.position=s.MrMC(0);ex.qCM=ex.qCM.exponentialMap(V3D(.2,-.4,.8));
  active_gimbal::reframe(fs,ex);
  const V3D after=s.MrMC(0)+s.qCM(0).inverseRotate(V3D(s.CfP(0).get_nor().getVec()*s.dep(0).getDistance()));
  if((point-after).norm()>1e-10 || (fs.cov_-original).norm()>1e-10)return 1;
  QPD identity;identity.setIdentity();active_gimbal::reframe(fs,{identity,ex.position});
  if((s.CfP(0).get_nor().getVec()-b.getVec()).norm()>1e-10)return 2;
  // Non-isotropic covariance with pose/feature cross terms: a pure encoder
  // rotation must not create distance or body-motion information.
  MXD C=MXD::Random(fs.cov_.rows(),fs.cov_.cols());
  fs.cov_=(C*C.transpose()).eval();original=fs.cov_;
  const auto body_position=s.WrWM();const auto body_velocity=s.MvM();
  const auto body_rotation=s.qWM();const double distance=s.dep(0).getDistance();
  for(const V3D angles : {V3D(.87,0,0),V3D(0,1.57,0),V3D(-.87,-1.2,.4)}){
    QPD turn;turn=turn.exponentialMap(angles);
    const auto old=s.CfP(0).get_nor(),next=old.rotated(turn);
    const auto J=(next.getM().transpose()*MPD(turn).matrix()*old.getM()).eval();
    for(int axis=0;axis<2;++axis){
      Eigen::Vector2d delta=Eigen::Vector2d::Zero(),plus,minus;// Avoid acos cancellation at the upstream 1e-6 rad branch boundary.
      const double eps=1e-4;delta[axis]=eps;
      LWF::NormalVectorElement fp,fm;old.boxPlus(delta,fp);old.boxPlus(-delta,fm);
      fp.rotated(turn).boxMinus(next,plus);fm.rotated(turn).boxMinus(next,minus);
      if(((plus-minus)/(2*eps)-J.col(axis)).norm()>1e-6){std::cerr<<"Jacobian numeric="<<((plus-minus)/(2*eps)).transpose()<<" expected="<<J.col(axis).transpose()<<" error="<<((plus-minus)/(2*eps)-J.col(axis)).norm()<<"\n";return 4;}
    }
    active_gimbal::reframe(fs,{turn,ex.position});
    if(std::abs(s.dep(0).getDistance()-distance)>1e-12 || (s.WrWM()-body_position).norm()>1e-12 || (s.MvM()-body_velocity).norm()>1e-12 || (MPD(s.qWM()).matrix()-MPD(body_rotation).matrix()).norm()>1e-12)return 5;
    active_gimbal::reframe(fs,{identity,ex.position});
    if((fs.cov_-original).norm()>1e-8 || (s.CfP(0).get_nor().getVec()-b.getVec()).norm()>1e-10)return 6;
  }
  try{active_gimbal::reframe(fs,{identity,ex.position+V3D(.01,0,0)});return 3;}catch(const std::runtime_error&){}
  std::cout<<"Gimbal point, distance, body state and correlated covariance invariance; finite-difference Jacobian; round trip; translation rejection passed\n";
}
