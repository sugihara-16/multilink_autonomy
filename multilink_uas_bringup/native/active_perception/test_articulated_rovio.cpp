#include "articulated_rovio.hpp"
#include <rovio/FilterStates.hpp>
#include <iostream>
#include <cassert>
#include <memory>
using FS=rovio::FilterState<25,4,6,1,0>;
Eigen::MatrixXd worldJac(const FS& f){
  using S=FS::mtState;
  Eigen::MatrixXd J=Eigen::MatrixXd::Zero(3,f.cov_.cols());
  const auto& s=f.state_;const auto& n=s.CfP(0).get_nor();
  const M3D R=MPD(s.qCM(0).inverted()).matrix();
  const int k=S::getId<S::_fea>(0);
  J.block<3,2>(0,k)=R*n.getM()*s.dep(0).getDistance();
  J.block<3,1>(0,k+2)=R*n.getVec()*s.dep(0).getDistanceDerivative();return J;
}
V3D point(const FS& f){const auto&s=f.state_;const V3D v=s.CfP(0).get_nor().getVec()*s.dep(0).getDistance();return s.MrMC(0)+s.qCM(0).inverseRotate(v);}
int main(){
  for(int type=0;type<4;++type){
    auto storage=std::make_unique<FS>();auto& f=*storage;f.state_.aux().doVECalibration_=false;
    for(int i=0;i<25;++i)f.fsm_.isValid_[i]=i==0;
    auto&s=f.state_;s.qCM(0).setIdentity();s.MrMC(0)=V3D(.1,.2,.3);
    s.CfP(0).set_nor(LWF::NormalVectorElement(V3D(.2,.3,1).normalized()));s.CfP(0).camID_=0;s.CfP(0).set_warp_nor(Eigen::Matrix2d::Identity());
    s.dep(0).setType(type);s.dep(0).setParameter(3.);
    auto M=Eigen::MatrixXd::Random(f.cov_.rows(),f.cov_.cols()).eval();f.cov_=M*M.transpose()+Eigen::MatrixXd::Identity(M.rows(),M.cols());
    const V3D initial=point(f);auto J=worldJac(f);const auto C=(J*f.cov_*J.transpose()).eval();
    const auto cross=(J*f.cov_).eval().leftCols<6>().eval();
    active_gimbal::Extrinsic ex{QPD(Eigen::Quaterniond(Eigen::AngleAxisd(.35,V3D(.4,.7,.1).normalized()))),V3D(.7,-.3,.5)};
    active_gimbal::reframe(f,ex);
    J=worldJac(f);
    assert((point(f)-initial).norm()<1e-10);
    assert((J*f.cov_*J.transpose()-C).norm()<1e-7);
    assert((J*f.cov_.leftCols<6>()-cross).norm()<1e-8);
    assert((f.cov_-f.cov_.transpose()).norm()<1e-8);
    active_gimbal::reframe(f,{QPD(),V3D(.1,.2,.3)});
    assert((point(f)-initial).norm()<1e-10);
  }
  std::cout<<"All four depth parametrizations: landmark, covariance and cross-covariance preserved under SE3 camera motion.\n";
}
