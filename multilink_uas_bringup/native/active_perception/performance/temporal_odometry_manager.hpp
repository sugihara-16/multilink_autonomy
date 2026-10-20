#pragma once
// Optional experimental manager. The standard MIMOSA manager remains intact.
// Callback lifecycle follows NTNU MIMOSA (BSD-licensed odometry/manager.cpp).
#include "mimosa/odometry/manager.hpp"
#include "temporal_odometry_factor.hpp"

class TemporalOdometryManager : public mimosa::SensorManagerBase<mimosa::odometry::ManagerConfig,nav_msgs::Odometry> {
  gtsam::Key previous_key_=0, before_previous_key_=0;
  gtsam::Pose3 previous_pose_, previous_increment_;
  gtsam::Vector6 rho_;
  gtsam::SharedNoiseModel initial_, innovation_;
  bool have_increment_=false;
  double previous_time_=0.,period_=.05;
  gtsam::SharedNoiseModel noise(ros::NodeHandle& pnh,const std::string& name) {
    std::vector<double> values;
    if(!pnh.getParam(name,values)||values.size()!=36)throw std::invalid_argument(name+" must have 36 entries");
    gtsam::Matrix66 C;
    for(int i=0;i<6;i++)for(int j=0;j<6;j++)C(i,j)=values[6*i+j];
    Eigen::LLT<gtsam::Matrix66> llt(C);
    if(!C.allFinite()||!C.isApprox(C.transpose(),1e-10)||llt.info()!=Eigen::Success)
      throw std::invalid_argument(name+" must be symmetric positive definite");
    return gtsam::noiseModel::Gaussian::Covariance(C);
  }
public:
  TemporalOdometryManager(ros::NodeHandle& pnh,mimosa::imu::Manager::Ptr imu,
      mimosa::graph::Manager::Ptr graph)
    :SensorManagerBase(config::checkValid(config::fromRos<mimosa::odometry::ManagerConfig>(pnh)),imu,graph,"odometry") {
    initial_=noise(pnh,"odometry/manager/relative_pose_covariance");
    innovation_=noise(pnh,"odometry/manager/temporal_innovation_covariance");
    std::vector<double> rho;
    if(!pnh.getParam("odometry/manager/temporal_rho",rho)||rho.size()!=6)
      throw std::invalid_argument("temporal_rho must have 6 entries");
    for(int i=0;i<6;i++){
      if(!std::isfinite(rho[i])||std::abs(rho[i])>=1.)throw std::invalid_argument("AR coefficients must have magnitude < 1");
      rho_[i]=rho[i];
    }
    pnh.param("odometry/manager/temporal_period_s",period_,.05);
    if(!(period_>0.))throw std::invalid_argument("Invalid AR interval");
    subscribeIfEnabled(pnh);
  }
  void callback(const nav_msgs::Odometry::ConstPtr& msg) override {
    using namespace mimosa;
    if(!passesCommonValidations(msg))return;
    corrected_ts_=msg->header.stamp.toSec()+config_.base.ts_offset;
    MXD covariance;odometry::convert(msg->pose.covariance,covariance);
    if(odometry::calcDoptimality(covariance)>config_.d_opt_thresh)return;
    gtsam::NonlinearFactorGraph factors;
    const auto pose=toGtsam(msg->pose.pose);
    gtsam::Pose3 increment;
    if(initialized_){
      if(std::abs(corrected_ts_-previous_time_-period_)>1e-6)
        throw std::runtime_error("Temporal odometry requires continuous calibrated-period measurements");
      increment=config_.base.T_B_S*previous_pose_.inverse()*pose*config_.base.T_B_S.inverse();
      if(have_increment_)
        factors.add(TemporalOdometryFactor(X(before_previous_key_),X(previous_key_),X(0),
                    previous_increment_,increment,rho_,innovation_));
      else factors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(previous_key_),X(0),increment,initial_));
    }
    const auto result=graph_manager_->declare(corrected_ts_,new_key_,config_.base.use_to_init,factors);
    if(!handleDeclarationResult(result))return;
    if(initialized_){previous_increment_=increment;before_previous_key_=previous_key_;have_increment_=true;}
    previous_pose_=pose;previous_key_=new_key_;previous_time_=corrected_ts_;initialized_=true;
  }
};
