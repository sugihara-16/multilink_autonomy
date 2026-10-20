#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <random>
#include <iostream>
#include <stdexcept>
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){
  std::mt19937 rng(20261017);std::normal_distribution<double> n;
  auto params=gtsam::PreintegrationParams::MakeSharedU(9.8);
  params->setAccelerometerCovariance(gtsam::Matrix3::Identity()*.04);
  params->setGyroscopeCovariance(gtsam::Matrix3::Identity()*.0001);
  params->setIntegrationCovariance(gtsam::Matrix3::Identity()*1e-8);
  const gtsam::Unit3 down(0,0,-1),tilted(.15,-.1,-1);
  for(int k=0;k<30;++k){
    gtsam::imuBias::ConstantBias bias(gtsam::Vector3(.01*n(rng),.01*n(rng),.01*n(rng)),gtsam::Vector3(.001*n(rng),.001*n(rng),.001*n(rng)));
    gtsam::PreintegratedImuMeasurements pim(params,bias);
    for(int i=0;i<20;++i)pim.integrateMeasurement(gtsam::Vector3(n(rng),n(rng),9.8+n(rng)),gtsam::Vector3(.1*n(rng),.1*n(rng),.1*n(rng)),.01);
    gtsam::NavState start(gtsam::Pose3(gtsam::Rot3::RzRyRx(.2*n(rng),.2*n(rng),n(rng)),gtsam::Point3(n(rng),n(rng),n(rng))),gtsam::Vector3(n(rng),n(rng),n(rng)));
    auto fixed=pim.predict(start,bias),free=pim.predict(start,bias,down),other=pim.predict(start,bias,tilted);
    require(fixed.equals(free,1e-12),"Fixed/vector-gravity predictions disagree at reference gravity");
    require(!fixed.equals(other,1e-6),"Estimated gravity must affect the estimated-gravity branch");
    auto end_pose=fixed.pose().retract((gtsam::Vector6()<<.01,-.02,.03,.04,-.05,.06).finished());
    auto end_vel=fixed.velocity()+gtsam::Vector3(.02,.03,-.01);
    gtsam::ImuFactor f(1,2,3,4,5,pim);
    gtsam::ImuFactorWithGravity g(1,2,3,4,5,6,pim);
    gtsam::Matrix F[5],G[6];
    auto e=f.evaluateError(start.pose(),start.velocity(),end_pose,end_vel,bias,&F[0],&F[1],&F[2],&F[3],&F[4]);
    auto eg=g.evaluateError(start.pose(),start.velocity(),end_pose,end_vel,bias,down,&G[0],&G[1],&G[2],&G[3],&G[4],&G[5]);
    require((e-eg).norm()<1e-12,"Fixed/vector-gravity factor residual mismatch");
    for(int j=0;j<5;j++)require((F[j]-G[j]).norm()<1e-10,"Fixed/vector-gravity Jacobian mismatch");
    require(f.keys().size()==5 && g.keys().size()==6,"Fixed factor must not constrain an estimated gravity key");
    auto e_other=g.evaluateError(start.pose(),start.velocity(),end_pose,end_vel,bias,tilted,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr);
    require((eg-e_other).norm()>1e-4,"Estimated-gravity factor ignored its gravity state");
  }
  std::cout<<"30 prediction/residual/Jacobian equivalence cases; fixed 5-key versus estimated 6-key gravity: PASS\n";
}
