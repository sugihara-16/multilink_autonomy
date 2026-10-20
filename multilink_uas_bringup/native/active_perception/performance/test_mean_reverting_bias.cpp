#include "mean_reverting_bias_factor.hpp"
#include <random>
#include <iostream>
void require(bool yes,const char* why){if(!yes)throw std::runtime_error(why);}
int main(){
  using Bias=gtsam::imuBias::ConstantBias;
  std::mt19937 rng(20261018);std::normal_distribution<double> n;
  gtsam::Vector6 tau,driving;tau<<175,175,175,400,400,400;driving<<.000375,.000375,.000375,.00002,.00002,.00002;
  for(int k=0;k<40;k++){
    const double dt=.001+double(k)*.023,half=dt/2;
    const auto phi=biasDecay(tau,dt),p1=biasDecay(tau,half);
    const auto sigma=biasInnovationSigma(tau,driving,dt),s1=biasInnovationSigma(tau,driving,half);
    require((phi-p1.cwiseProduct(p1)).norm()<1e-14,"Bias transition composition wrong");
    require((sigma.cwiseAbs2()-(p1.cwiseAbs2()+gtsam::Vector6::Ones()).cwiseProduct(s1.cwiseAbs2())).norm()<1e-16,"Bias variance composition wrong");
    for(int j=0;j<6;j++){
      const double direct=driving[j]*driving[j]*tau[j]/2*(1-std::exp(-2*dt/tau[j]));
      require(std::abs(sigma[j]*sigma[j]-direct)<1e-17,"Gazebo variance formula mismatch");
    }
    gtsam::Vector6 b;for(int j=0;j<6;j++)b[j]=.1*n(rng);
    const gtsam::Vector6 next=phi.cwiseProduct(b);Bias a(b.head<3>(),b.tail<3>()),c(next.head<3>(),next.tail<3>());
    MeanRevertingBiasFactor f(1,2,tau,driving,dt);gtsam::Matrix H1,H2;
    require(f.evaluateError(a,c,&H1,&H2).norm()<1e-14,"Mean evolution must have zero residual");
    for(int j=0;j<6;j++){
      gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=1e-6;
      const gtsam::Vector6 plus=b+dx,minus=b-dx,nplus=next+dx,nminus=next-dx;
      const gtsam::Vector6 J1=(f.evaluateError(Bias(plus.head<3>(),plus.tail<3>()),c)-f.evaluateError(Bias(minus.head<3>(),minus.tail<3>()),c))/(2e-6);
      const gtsam::Vector6 J2=(f.evaluateError(a,Bias(nplus.head<3>(),nplus.tail<3>()))-f.evaluateError(a,Bias(nminus.head<3>(),nminus.tail<3>())))/(2e-6);
      require((J1-H1.col(j)).norm()<1e-9 && (J2-H2.col(j)).norm()<1e-9,"Bias Jacobian mismatch");
    }
    require(f.dim()==6 && f.keys().size()==2,"Out-of-order factor replacement requires same shape as original bias factor");
    require(f.rekey(gtsam::KeyVector{8,9})->keys()==gtsam::KeyVector({8,9}),"Bias factor rekey failed");
  }
  const auto infinite=gtsam::Vector6::Constant(1e12);
  require((biasInnovationSigma(infinite,driving,.05)-std::sqrt(.05)*driving).norm()<1e-15,"Random-walk limiting variance wrong");
  std::cout<<"40 sensor-model/semigroup cases, 480 Jacobian columns, rekey and random-walk limit: PASS\n";
}
