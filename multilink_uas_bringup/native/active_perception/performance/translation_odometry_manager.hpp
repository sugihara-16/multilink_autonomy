#pragma once
// Separate experimental manager following the NTNU MIMOSA BSD callback lifecycle.
#include "mimosa/odometry/manager.hpp"
#include "translation_odometry_factor.hpp"

class TranslationOdometryManager : public mimosa::SensorManagerBase<mimosa::odometry::ManagerConfig,nav_msgs::Odometry> {
  gtsam::Key previous_key_=0;
  gtsam::Pose3 previous_pose_;
  gtsam::SharedNoiseModel noise_;
  double previous_time_=0.;
public:
  TranslationOdometryManager(ros::NodeHandle& pnh,mimosa::imu::Manager::Ptr imu,
      mimosa::graph::Manager::Ptr graph)
    :SensorManagerBase(config::checkValid(config::fromRos<mimosa::odometry::ManagerConfig>(pnh)),imu,graph,"odometry") {
    std::vector<double> values;
    if(!pnh.getParam("odometry/manager/translation_covariance",values)||values.size()!=9)
      throw std::invalid_argument("translation_covariance must have 9 entries");
    gtsam::Matrix3 C;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)C(i,j)=values[3*i+j];
    Eigen::LLT<gtsam::Matrix3> llt(C);
    if(!C.allFinite()||!C.isApprox(C.transpose(),1e-10)||llt.info()!=Eigen::Success)
      throw std::invalid_argument("translation covariance must be symmetric positive definite");
    noise_=gtsam::noiseModel::Gaussian::Covariance(C);
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
    if(initialized_){
      if(std::abs(corrected_ts_-previous_time_-.05)>1e-6)
        throw std::runtime_error("Translation odometry requires continuous 50ms measurements");
      const auto increment=config_.base.T_B_S*previous_pose_.inverse()*pose*config_.base.T_B_S.inverse();
      factors.add(TranslationOdometryFactor(X(previous_key_),X(0),increment.translation(),noise_));
    }
    const auto result=graph_manager_->declare(corrected_ts_,new_key_,config_.base.use_to_init,factors);
    if(!handleDeclarationResult(result))return;
    previous_pose_=pose;previous_key_=new_key_;previous_time_=corrected_ts_;initialized_=true;
  }
};
