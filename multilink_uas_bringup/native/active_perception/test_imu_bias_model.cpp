#include "imu_bias_model.hpp"
#include <cassert>
#include <iostream>
int main(){
  const std::array<double,3> tau{175,400,0};
  const Eigen::Vector3d initial(.05,.008,.02),q(1.40625e-7,4e-10,1e-8),P0(.00255625,2.55625e-5,.001);
  int checks=0;
  for(double dt:{.001,.005,.02,.1}){
    Eigen::Vector3d b=initial,P=P0;const int steps=std::lround(100/dt);
    const auto phi=active_imu::decay(tau,dt);const auto G=active_imu::noise_scale(tau,dt);
    for(int j=0;j<steps;++j){b=phi.cwiseProduct(b);P=phi.cwiseProduct(phi).cwiseProduct(P)+G.cwiseProduct(G).cwiseProduct(q);}
    for(int axis=0;axis<3;++axis){
      const double expected_b=tau[axis]>0?initial[axis]*std::exp(-100/tau[axis]):initial[axis];
      const double expected_P=tau[axis]>0?P0[axis]*std::exp(-200/tau[axis])+q[axis]*tau[axis]/2*(1-std::exp(-200/tau[axis])):P0[axis]+100*q[axis];
      assert(std::abs(b[axis]-expected_b)<1e-12);assert(std::abs(P[axis]-expected_P)<1e-12);checks+=2;
    }
  }
  std::cout<<checks<<" mean/covariance checks passed; random walk and sensor correlation times\n";
}
