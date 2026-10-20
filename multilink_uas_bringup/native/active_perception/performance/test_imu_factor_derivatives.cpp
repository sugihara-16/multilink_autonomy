#include <gtsam/navigation/ImuFactor.h>
#include <random>
#include <iostream>
#include <stdexcept>
int main(){
  std::mt19937 rng(20261018);std::normal_distribution<double> normal;
  auto random3=[&](double s){return gtsam::Vector3(s*normal(rng),s*normal(rng),s*normal(rng));};
  auto params=gtsam::PreintegrationParams::MakeSharedU(9.8);
  params->setAccelerometerCovariance(gtsam::Matrix3::Identity()*2.2154e-6);
  params->setGyroscopeCovariance(gtsam::Matrix3::Identity()*4.0503e-7);
  params->setIntegrationCovariance(gtsam::Matrix3::Zero());
  double maximum=0.;
  for(int k=0;k<40;++k){
    gtsam::imuBias::ConstantBias bias(random3(.05),random3(.005));
    gtsam::PreintegratedImuMeasurements pim(params,bias);
    for(int i=0;i<50;++i)pim.integrateMeasurement(gtsam::Vector3(0,0,9.8)+random3(1.),random3(.2),.001);
    gtsam::Pose3 pi(gtsam::Rot3::Expmap(random3(.5)),random3(1.));
    gtsam::Vector3 vi=random3(.5);
    auto predicted=pim.predict(gtsam::NavState(pi,vi),bias);
    auto pj=predicted.pose().retract((gtsam::Vector6()<<random3(.01),random3(.01)).finished());
    gtsam::Vector3 vj=predicted.velocity()+random3(.01);
    gtsam::ImuFactor factor(1,2,3,4,5,pim);gtsam::Matrix H[5];
    factor.evaluateError(pi,vi,pj,vj,bias,&H[0],&H[1],&H[2],&H[3],&H[4]);
    for(int argument=0;argument<5;++argument){
      int dimension=(argument==1||argument==3)?3:6;
      for(int j=0;j<dimension;++j){
        gtsam::Vector9 error[2];
        for(int sign=0;sign<2;++sign){
          auto a=pi,c=pj;gtsam::Vector3 b=vi,d=vj;auto e=bias;
          gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=(sign?1.:-1.)*1e-6;
          if(argument==0)a=a.retract(dx);
          if(argument==1)b+=dx.head<3>();
          if(argument==2)c=c.retract(dx);
          if(argument==3)d+=dx.head<3>();
          if(argument==4)e=gtsam::imuBias::ConstantBias(bias.vector()+dx);
          error[sign]=factor.evaluateError(a,b,c,d,e);
        }
        maximum=std::max(maximum,((error[1]-error[0])/(2e-6)-H[argument].col(j)).cwiseAbs().maxCoeff());
      }
    }
  }
  if(maximum>1e-6)throw std::runtime_error("Fixed-gravity IMU derivative mismatch");
  std::cout<<"PASS 960 finite-difference columns; maximum error "<<maximum<<"\n";
}
