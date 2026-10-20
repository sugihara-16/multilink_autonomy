#pragma once
// Sparse direct body velocity and gravity direction from external VIO.
// No VIO global position, heading, or pose differencing enters a factor.
#include "mimosa/odometry/manager.hpp"
#include <gtsam/navigation/AttitudeFactor.h>
#include "body_velocity_factor.hpp"
#include <limits>
#include <fstream>
#include <iomanip>

class VelocityOdometryManager : public mimosa::SensorManagerBase<mimosa::odometry::ManagerConfig,nav_msgs::Odometry> {
  double last_update_=-std::numeric_limits<double>::infinity();
  double period_=1.,scale_=1.;
  gtsam::SharedNoiseModel tilt_noise_;
  std::ofstream tilt_audit_,velocity_audit_;
public:
  VelocityOdometryManager(ros::NodeHandle& pnh,mimosa::imu::Manager::Ptr imu,mimosa::graph::Manager::Ptr graph)
    :SensorManagerBase(config::checkValid(config::fromRos<mimosa::odometry::ManagerConfig>(pnh)),imu,graph,"odometry") {
    bool aligned=false,vio_aligned=false,estimated=true;
    pnh.param("imu/preintegration/gravity_aligned_initialization",aligned,false);
    pnh.param("imu/preintegration/use_estimated_gravity",estimated,true);
    pnh.param("odometry/manager/vio_gravity_aligned",vio_aligned,false);
    if(!aligned||!vio_aligned||estimated)throw std::invalid_argument("Fixed gravity-aligned frames required");
    double sigma=0.,tilt_period=0.;
    pnh.getParam("odometry/manager/tilt_sigma_rad",sigma);
    pnh.getParam("odometry/manager/tilt_period_s",tilt_period);
    pnh.getParam("odometry/manager/velocity_period_s",period_);
    pnh.getParam("odometry/manager/velocity_covariance_scale",scale_);
    if(!std::isfinite(sigma)||sigma<=0.||!std::isfinite(period_)||period_<1.||std::abs(period_-tilt_period)>1e-10||!std::isfinite(scale_)||scale_<1.)
      throw std::invalid_argument("Calibrated covariance and matching sparse velocity/tilt periods required");
    tilt_noise_=gtsam::noiseModel::Isotropic::Sigma(2,sigma);
    std::string directory;pnh.param<std::string>("logs_directory",directory,"");
    tilt_audit_.open(directory+"tilt_factors.csv");velocity_audit_.open(directory+"velocity_factors.csv");
    tilt_audit_<<std::setprecision(17)<<"stamp_ns,up_x,up_y,up_z,sigma_rad\n";
    velocity_audit_<<std::setprecision(17)<<"stamp_ns,vx,vy,vz,covariance_scale,c00,c01,c02,c10,c11,c12,c20,c21,c22\n";
    subscribeIfEnabled(pnh);
  }
  void callback(const nav_msgs::Odometry::ConstPtr& msg) override {
    using namespace mimosa;
    if(!passesCommonValidations(msg))return;
    corrected_ts_=msg->header.stamp.toSec()+config_.base.ts_offset;
    MXD pose_cov;odometry::convert(msg->pose.covariance,pose_cov);
    if(odometry::calcDoptimality(pose_cov)>config_.d_opt_thresh)return;
    gtsam::NonlinearFactorGraph factors;
    const bool update=initialized_ && corrected_ts_-last_update_>=period_-1e-8;
    const auto body_pose=toGtsam(msg->pose.pose)*config_.base.T_B_S.inverse();
    const gtsam::Unit3 up(body_pose.rotation().unrotate(gtsam::Point3(0,0,1)));
    gtsam::Vector3 measured=gtsam::Vector3::Zero();gtsam::Matrix3 C=gtsam::Matrix3::Zero();
    if(update){
      const auto& linear=msg->twist.twist.linear;const auto& angular=msg->twist.twist.angular;
      gtsam::Vector6 twist;twist<<linear.x,linear.y,linear.z,angular.x,angular.y,angular.z;
      gtsam::Matrix66 full;
      for(int i=0;i<6;i++)for(int j=0;j<6;j++)full(i,j)=msg->twist.covariance[6*i+j];
      const auto A=bodyVelocityTransform(config_.base.T_B_S);measured=A*twist;
      C=scale_*A*full*A.transpose();Eigen::LLT<gtsam::Matrix3> llt(C);
      if(!measured.allFinite()||!C.allFinite()||!C.isApprox(C.transpose(),1e-10)||llt.info()!=Eigen::Success)
        throw std::runtime_error("Invalid reported VIO body velocity/covariance");
      factors.add(BodyVelocityFactor(X(0),V(0),measured,gtsam::noiseModel::Gaussian::Covariance(C)));
      factors.add(gtsam::Pose3AttitudeFactor(X(0),gtsam::Unit3(0,0,1),tilt_noise_,up));
    }
    // Retain original timestamped state creation even between sparse updates.
    const auto result=graph_manager_->declare(corrected_ts_,new_key_,config_.base.use_to_init,factors);
    if(!handleDeclarationResult(result))return;
    if(update){
      last_update_=corrected_ts_;const auto u=up.unitVector();
      tilt_audit_<<msg->header.stamp.toNSec()<<','<<u.x()<<','<<u.y()<<','<<u.z()<<','<<tilt_noise_->sigmas()[0]<<'\n';
      velocity_audit_<<msg->header.stamp.toNSec()<<','<<measured.x()<<','<<measured.y()<<','<<measured.z()<<','<<scale_;
      for(int i=0;i<3;i++)for(int j=0;j<3;j++)velocity_audit_<<','<<C(i,j);
      velocity_audit_<<'\n';
    }
    initialized_=true;
  }
};
