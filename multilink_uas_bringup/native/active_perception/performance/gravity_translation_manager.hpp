#pragma once
// Experimental extension of the NTNU MIMOSA BSD odometry-manager lifecycle.
// Visual displacement and gravity tilt are constrained separately. No relative
// visual rotation observation is included. The displacement still uses the
// preceding VIO orientation to define its local coordinates.
#include "mimosa/odometry/manager.hpp"
#include <gtsam/navigation/AttitudeFactor.h>
#include "translation_odometry_factor.hpp"
#include <limits>
#include <fstream>
#include <iomanip>

class GravityTranslationOdometryManager : public mimosa::SensorManagerBase<mimosa::odometry::ManagerConfig,nav_msgs::Odometry> {
  gtsam::Key previous_key_=0;
  gtsam::Pose3 previous_pose_;
  gtsam::SharedNoiseModel relative_noise_,tilt_noise_;
  double last_tilt_=-std::numeric_limits<double>::infinity();
  double period_=1.;
  std::ofstream audit_;
public:
  GravityTranslationOdometryManager(ros::NodeHandle& pnh,mimosa::imu::Manager::Ptr imu,
      mimosa::graph::Manager::Ptr graph)
    :SensorManagerBase(config::checkValid(config::fromRos<mimosa::odometry::ManagerConfig>(pnh)),imu,graph,"odometry") {
    bool aligned=false,vio_aligned=false;
    pnh.param("imu/preintegration/gravity_aligned_initialization",aligned,false);
    pnh.param("odometry/manager/vio_gravity_aligned",vio_aligned,false);
    if(!aligned||!vio_aligned)throw std::invalid_argument("Both odometry frames must use gravity-aligned Z");
    std::vector<double> values;
    if(!pnh.getParam("odometry/manager/translation_covariance",values)||values.size()!=9)
      throw std::invalid_argument("translation_covariance must have 9 entries");
    gtsam::Matrix3 C;
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)C(i,j)=values[3*i+j];
    Eigen::LLT<gtsam::Matrix3> llt(C);
    if(!C.allFinite()||!C.isApprox(C.transpose(),1e-10)||llt.info()!=Eigen::Success)
      throw std::invalid_argument("relative covariance must be symmetric positive definite");
    relative_noise_=gtsam::noiseModel::Gaussian::Covariance(C);
    double sigma=0.;
    pnh.getParam("odometry/manager/tilt_sigma_rad",sigma);
    pnh.param("odometry/manager/tilt_period_s",period_,1.);
    if(!std::isfinite(sigma)||sigma<=0.||!std::isfinite(period_)||period_<1.)
      throw std::invalid_argument("Positive tilt noise and period >= 1 s required");
    tilt_noise_=gtsam::noiseModel::Isotropic::Sigma(2,sigma);
    std::string directory;pnh.param<std::string>("logs_directory",directory,"");
    audit_.open(directory+"tilt_factors.csv");
    audit_<<std::setprecision(17)<<"stamp_ns,up_x,up_y,up_z,sigma_rad\n";
    subscribeIfEnabled(pnh);
  }
  void callback(const nav_msgs::Odometry::ConstPtr& msg) override {
    using namespace mimosa;
    if(!passesCommonValidations(msg))return;
    corrected_ts_=msg->header.stamp.toSec()+config_.base.ts_offset;
    MXD covariance;odometry::convert(msg->pose.covariance,covariance);
    if(odometry::calcDoptimality(covariance)>config_.d_opt_thresh)return;
    const auto pose=toGtsam(msg->pose.pose);
    gtsam::NonlinearFactorGraph factors;
    const bool add_tilt=initialized_ && corrected_ts_-last_tilt_>=period_-1e-8;
    const auto body_pose=pose*config_.base.T_B_S.inverse();
    const gtsam::Unit3 up(body_pose.rotation().unrotate(gtsam::Point3(0,0,1)));
    if(initialized_){
      const auto increment=config_.base.T_B_S*previous_pose_.inverse()*pose*config_.base.T_B_S.inverse();
      factors.add(TranslationOdometryFactor(X(previous_key_),X(0),increment.translation(),relative_noise_));
      if(add_tilt)factors.add(gtsam::Pose3AttitudeFactor(X(0),gtsam::Unit3(0,0,1),tilt_noise_,up));
    }
    const auto result=graph_manager_->declare(corrected_ts_,new_key_,config_.base.use_to_init,factors);
    if(!handleDeclarationResult(result))return;
    if(add_tilt){
      last_tilt_=corrected_ts_;const auto u=up.unitVector();
      audit_<<msg->header.stamp.toNSec()<<','<<u.x()<<','<<u.y()<<','<<u.z()<<','<<tilt_noise_->sigmas()[0]<<'\n';
    }
    previous_pose_=pose;previous_key_=new_key_;initialized_=true;
  }
};
