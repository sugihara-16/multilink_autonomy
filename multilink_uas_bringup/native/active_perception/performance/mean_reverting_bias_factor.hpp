#pragma once
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <cmath>
#include <stdexcept>

inline gtsam::Vector6 biasDecay(const gtsam::Vector6& tau,double dt) {
  if(!std::isfinite(dt)||dt<=0||!tau.allFinite()||tau.minCoeff()<=0)
    throw std::invalid_argument("Positive dt and six positive bias time constants required");
  return (-dt*tau.cwiseInverse().array()).exp().matrix();
}
inline gtsam::Vector6 biasInnovationSigma(const gtsam::Vector6& tau,const gtsam::Vector6& driving,double dt) {
  biasDecay(tau,dt);
  if(!driving.allFinite()||driving.minCoeff()<=0)throw std::invalid_argument("Positive bias driving noise required");
  gtsam::Vector6 out;
  for(int i=0;i<6;i++)out[i]=driving[i]*std::sqrt(-tau[i]/2*std::expm1(-2*dt/tau[i]));
  return out;
}
class MeanRevertingBiasFactor:public gtsam::NoiseModelFactorN<gtsam::imuBias::ConstantBias,gtsam::imuBias::ConstantBias> {
  gtsam::Vector6 phi_;
public:
  using Bias=gtsam::imuBias::ConstantBias;
  using Base=gtsam::NoiseModelFactorN<Bias,Bias>;
  MeanRevertingBiasFactor(gtsam::Key first,gtsam::Key second,const gtsam::Vector6& tau,const gtsam::Vector6& driving,double dt)
    :Base(gtsam::noiseModel::Diagonal::Sigmas(biasInnovationSigma(tau,driving,dt)),first,second),phi_(biasDecay(tau,dt)){}
  gtsam::NonlinearFactor::shared_ptr clone() const override {return std::make_shared<MeanRevertingBiasFactor>(*this);}
  gtsam::Vector evaluateError(const Bias& first,const Bias& second,gtsam::OptionalMatrixType H1=nullptr,gtsam::OptionalMatrixType H2=nullptr) const override {
    if(H1)*H1=-phi_.asDiagonal().toDenseMatrix();
    if(H2)*H2=gtsam::Matrix66::Identity();
    return second.vector()-phi_.cwiseProduct(first.vector());
  }
};
