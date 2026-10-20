#include "body_velocity_factor.hpp"
#include <random>
#include <iostream>
#include <stdexcept>
void require(bool yes,const char* why){if(!yes)throw std::runtime_error(why);}
int main(){
  std::mt19937 rng(20261018);std::normal_distribution<double> n;
  auto vec=[&](){return gtsam::Vector3(n(rng),n(rng),n(rng));};
  auto noise=gtsam::noiseModel::Isotropic::Sigma(3,.1);
  for(int k=0;k<40;k++){
    const auto pose=gtsam::Pose3(gtsam::Rot3::Expmap(vec()),vec());
    const auto velocity=vec(),omega=vec();
    const auto ext=gtsam::Pose3(gtsam::Rot3::Expmap(vec()),vec());
    gtsam::Vector6 twist;
    twist.head<3>()=ext.rotation().unrotate(pose.rotation().unrotate(velocity)+omega.cross(ext.translation()));
    twist.tail<3>()=ext.rotation().unrotate(omega);
    const auto A=bodyVelocityTransform(ext);
    const gtsam::Vector3 measured=A*twist;
    require((measured-pose.rotation().unrotate(velocity)).norm()<1e-12,"Lever-arm velocity correction wrong");
    BodyVelocityFactor f(1,2,measured,noise);gtsam::Matrix H1,H2;
    require(f.evaluateError(pose,velocity,&H1,&H2).norm()<1e-12,"True velocity residual nonzero");
    const auto gauge=gtsam::Pose3(gtsam::Rot3::Expmap(vec()),vec());
    require(f.evaluateError(gauge*pose,gauge.rotation().rotate(velocity)).norm()<1e-12,"World coordinate gauge changed local measurement");
    for(int j=0;j<6;j++){
      gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=1e-6;
      gtsam::Vector3 numerical=(f.evaluateError(pose.retract(dx),velocity)-f.evaluateError(pose.retract(-dx),velocity))/(2e-6);
      require((numerical-H1.col(j)).norm()<1e-7,"Pose Jacobian wrong");
      gtsam::Vector6 ds=gtsam::Vector6::Zero();ds[j]=1e-6;
      const gtsam::Vector3 d=(A*(twist+ds)-A*(twist-ds))/(2e-6);
      require((d-A.col(j)).norm()<1e-7,"Twist covariance transform wrong");
    }
    for(int j=0;j<3;j++){
      gtsam::Vector3 dx=gtsam::Vector3::Zero();dx[j]=1e-6;
      gtsam::Vector3 numerical=(f.evaluateError(pose,velocity+dx)-f.evaluateError(pose,velocity-dx))/(2e-6);
      require((numerical-H2.col(j)).norm()<1e-7,"Velocity Jacobian wrong");
    }
    require(H1.rightCols(3).norm()==0.,"Absolute position constrained");
    require(f.rekey(gtsam::KeyVector{8,9})->keys()==gtsam::KeyVector({8,9}),"Graph rekey failed");
  }
  std::cout<<"40 frame/lever-arm/gauge cases; 360 state and 240 covariance-transform Jacobian columns: PASS\n";
}
