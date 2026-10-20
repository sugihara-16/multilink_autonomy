#pragma once
#include <array>
#include <cmath>
#include <Eigen/Core>

// The Gazebo GaussianNoiseModel evolves the WHOLE initial bias as
// b(t+dt)=exp(-dt/tau)*b(t)+N(0,q*tau/2*(1-exp(-2dt/tau))).
// tau=0 retains upstream ROVIO's random walk exactly. Parameters come from
// the sensor specification, never from trajectory error or ground truth.
namespace active_imu {
inline std::array<double,3>& acc_tau(){static std::array<double,3> a{};return a;}
inline std::array<double,3>& gyro_tau(){static std::array<double,3> a{};return a;}
inline Eigen::Vector3d decay(const std::array<double,3>& tau,double dt){
  Eigen::Vector3d p;for(int i=0;i<3;++i)p[i]=tau[i]>0?std::exp(-dt/tau[i]):1.;return p;
}
inline Eigen::Vector3d noise_scale(const std::array<double,3>& tau,double dt){
  Eigen::Vector3d p;for(int i=0;i<3;++i)p[i]=tau[i]>0?std::sqrt(-tau[i]/2*std::expm1(-2*dt/tau[i])):std::sqrt(dt);return p;
}
}
