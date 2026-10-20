#include <rovio/FilterStates.hpp>
#include <rovio/CoordinateTransform/LandmarkOutput.hpp>
#include "rovio_joint_geometry.hpp"
#include <iostream>
#include <memory>
int main(){
  using FS=rovio::FilterState<25,4,6,1,0>;using S=FS::mtState;
  auto storage=std::make_unique<FS>();auto& fs=*storage;auto& s=fs.state_;
  s.aux().doVECalibration_=false;fs.t_=12.345;
  s.qCM(0)=QPD().exponentialMap(V3D(.4,-.3,.8));s.MrMC(0)=V3D(.18,0.,-.025);s.MvM()=V3D(-.4,.2,-.1);
  const std::vector<int> slots{0,5,19};
  for(int i=0;i<S::nMax_;++i)fs.fsm_.isValid_[i]=false;
  for(const int i:slots){
    fs.fsm_.isValid_[i]=true;fs.fsm_.features_[i].idx_=100+i;
    fs.fsm_.features_[i].mpStatistics_->status_[0]=rovio::TRACKED;
    s.CfP(i).camID_=0;LWF::NormalVectorElement b;b.setFromVector(V3D(1.+i*.01,.4,2.));s.CfP(i).set_nor(b);
    s.dep(i).setType(1);s.dep(i).setParameter(3.+i*.1);
  }
  MXD A=MXD::Random(fs.cov_.rows(),fs.cov_.cols());fs.cov_=A*A.transpose();const MXD before=fs.cov_;
  const auto data=active_joint_geometry::encode(fs);
  if(data.size()!=6+4*slots.size()+144 || data[0]!=2 || data[1]!=12345000000. || data[2]!=3)return 1;
  if(data[3]!=.4||data[4]!=-.2||data[5]!=.1)return 2;
  rovio::MultiCamera<1> camera;rovio::LandmarkOutputImuCT<S> official(&camera);rovio::LandmarkOutput output;
  MXD J=MXD::Zero(12,fs.cov_.cols());J.block<3,3>(0,S::getId<S::_vel>())=-M3D::Identity();
  for(size_t k=0;k<slots.size();++k){
    official.setFeatureID(slots[k]);MXD local=MXD::Zero(3,fs.cov_.cols());official.jacTransform(local,s);J.block(3+3*k,0,3,fs.cov_.cols())=local;
    official.transformState(s,output);
    if(data[6+4*k]!=100+slots[k])return 3;
    for(int j=0;j<3;++j)if(std::abs(data[7+4*k+j]-output.lmk()[j])>1e-12)return 4;
  }
  const MXD expected=J*fs.cov_*J.transpose();MXD actual(12,12);
  for(int i=0;i<12;++i)for(int j=0;j<12;++j)actual(i,j)=data[18+i*12+j];
  if((expected-actual).norm()>1e-10*expected.norm())return 5;
  if((fs.cov_-before).norm()!=0)return 6;
  s.aux().doVECalibration_=true;
  try{active_joint_geometry::encode(fs);return 7;}catch(const std::runtime_error&){}
  std::cout<<"Export matches official landmark transform and all covariance blocks; original state covariance unchanged\n";
}
