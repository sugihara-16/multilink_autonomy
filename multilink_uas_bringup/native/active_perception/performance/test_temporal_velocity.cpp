#include "temporal_velocity_factor.hpp"
#include <random>
#include <iostream>
void require(bool x,const char* why){if(!x)throw std::runtime_error(why);}
int main(){
  std::mt19937 rng(54391);std::normal_distribution<double> n;
  auto vec=[&](){return gtsam::Vector3(n(rng),n(rng),n(rng));};
  auto cov=[&](){gtsam::Matrix3 A;for(int i=0;i<9;i++)A.data()[i]=n(rng);return gtsam::Matrix3(A*A.transpose()+.01*gtsam::Matrix3::Identity());};
  auto noise=gtsam::noiseModel::Isotropic::Sigma(3,.2);
  for(int k=0;k<40;k++){
    gtsam::Pose3 p[2]={{gtsam::Rot3::Expmap(vec()),vec()},{gtsam::Rot3::Expmap(vec()),vec()}};
    gtsam::Vector3 v[2]={vec(),vec()},z[2]={vec(),vec()};auto C0=cov(),C1=cov();const auto R=gtsam::Rot3::Expmap(vec());
    TemporalVelocityFactor f(1,2,3,4,z[0],z[1],C0,C1,R,.92,noise);gtsam::Matrix H[4];
    const auto e=f.evaluateError(p[0],v[0],p[1],v[1],&H[0],&H[1],&H[2],&H[3]);
    auto eval=[&](){return f.evaluateError(p[0],v[0],p[1],v[1]);};
    for(int i=0;i<4;i++)for(int j=0;j<(i%2?3:6);j++){
      auto pp=p[i/2];auto vv=v[i/2];gtsam::Vector plus,minus;
      if(i%2){v[i/2][j]+=1e-6;plus=eval();v[i/2][j]-=2e-6;minus=eval();v[i/2]=vv;}
      else{gtsam::Vector6 d=gtsam::Vector6::Zero();d[j]=1e-6;p[i/2]=pp.retract(d);plus=eval();p[i/2]=pp.retract(-d);minus=eval();p[i/2]=pp;}
      require(((plus-minus)/(2e-6)-H[i].col(j)).norm()<1e-7,"Temporal velocity Jacobian");
    }
    const gtsam::Pose3 gauge(gtsam::Rot3::Expmap(vec()),vec());
    require((e-f.evaluateError(gauge*p[0],gauge.rotation().rotate(v[0]),gauge*p[1],gauge.rotation().rotate(v[1]))).norm()<1e-10,"World gauge");
    TemporalVelocityFactor independent(1,2,3,4,z[0],z[1],C0,C1,R,0.,gtsam::noiseModel::Unit::Create(3));
    const auto raw=p[1].rotation().unrotate(v[1])-z[1];
    require(std::abs(independent.evaluateError(p[0],v[0],p[1],v[1]).squaredNorm()-raw.dot(C1.inverse()*raw))<1e-9,"Zero-rho independent likelihood");
    require((velocityInverseRoot(C0)*C0*velocityInverseRoot(C0)-gtsam::Matrix3::Identity()).norm()<1e-10,"Symmetric whitening");
    require(f.rekey(gtsam::KeyVector{11,12,13,14})->keys()==gtsam::KeyVector({11,12,13,14}),"Rekey");
  }
  // Dense two-time covariance gives the same Gaussian quadratic form.
  const double rho=.92,q=1-rho*rho;gtsam::Matrix66 covariance=gtsam::Matrix66::Identity();
  covariance.topRightCorner<3,3>()=rho*gtsam::Matrix3::Identity();covariance.bottomLeftCorner<3,3>()=rho*gtsam::Matrix3::Identity();
  for(int k=0;k<100;k++){gtsam::Vector6 e;e<<vec(),vec();double ar=e.head<3>().squaredNorm()+(e.tail<3>()-rho*e.head<3>()).squaredNorm()/q;require(std::abs(ar-e.dot(covariance.inverse()*e))<1e-9,"Dense Gaussian likelihood");}
  std::cout<<"40 frame/gauge cases, 720 Jacobian columns, 100 dense Gaussian cases: PASS\n";
}
