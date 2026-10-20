#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <random>
#include <iostream>
#include <stdexcept>
void require(bool x,const char* why){if(!x)throw std::runtime_error(why);}
int main(){
  std::mt19937 rng(20261017);std::normal_distribution<double> n;
  auto noise=gtsam::noiseModel::Isotropic::Sigma(2,.01);
  for(int k=0;k<40;k++){
    auto truth=gtsam::Pose3(gtsam::Rot3::RzRyRx(.2*n(rng),.2*n(rng),n(rng)),gtsam::Point3(1,2,3));
    auto body_sensor=gtsam::Pose3(gtsam::Rot3::RzRyRx(.3,.2,-.4),gtsam::Point3(.18,0,-.025));
    const auto observed=truth*body_sensor;
    auto measured=(observed*body_sensor.inverse()).rotation().unrotate(gtsam::Point3(0,0,1));
    gtsam::Pose3AttitudeFactor f(1,gtsam::Unit3(0,0,1),noise,gtsam::Unit3(measured));
    require(f.evaluateError(truth).norm()<1e-12,"Correct tilt must have zero error");
    auto yaw=gtsam::Pose3(gtsam::Rot3::Rz(n(rng)),gtsam::Point3(3,4,5));
    require(f.evaluateError(yaw*truth).norm()<1e-12,"Yaw or translation was constrained");
    const auto perturbed=truth.retract((gtsam::Vector6()<<.02,-.03,.01,.4,.5,.6).finished());
    gtsam::Matrix J;const auto e=f.evaluateError(perturbed,&J);
    require(std::abs(f.evaluateError(yaw*perturbed).squaredNorm()-e.squaredNorm())<1e-12,"Cost must be invariant to global yaw/translation");
    for(int j=0;j<6;j++){
      gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=1e-6;
      const auto a=f.evaluateError(perturbed.retract(dx));const auto b=f.evaluateError(perturbed.retract(-dx));
      require(((a-b)/(2e-6)-J.col(j)).norm()<1e-7,"Tilt Jacobian mismatch");
    }
    require(J.rightCols(3).norm()==0.,"Position entered tilt factor");
    require(f.rekey(gtsam::KeyVector{8})->keys()[0]==8,"Graph placeholder rekey failed");
  }
  std::cout<<"40 frame/extrinsic/yaw/position invariance cases and 240 Jacobian columns: PASS\n";
}
