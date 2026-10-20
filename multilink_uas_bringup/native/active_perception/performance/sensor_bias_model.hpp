#pragma once
#include "mean_reverting_bias_factor.hpp"
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <limits>

namespace active_fusion {
// The isolated experiment runs one IMU manager in each process. Keep the
// existing Manager ABI; do not add members that old linked objects can't see.
struct SensorBiasModel {
  bool loaded=false,enabled=false;
  gtsam::Vector6 tau=gtsam::Vector6::Ones(),driving=gtsam::Vector6::Ones();
  std::ofstream audit;
  double last_record=-std::numeric_limits<double>::infinity();
  void load(ros::NodeHandle& pnh) {
    if(loaded)throw std::runtime_error("Experimental sensor-bias library supports one IMU manager per process");
    loaded=true;pnh.param("imu/preintegration/sensor_bias_model",enabled,false);
    if(!enabled)return;
    std::vector<double> t,q;
    if(!pnh.getParam("imu/preintegration/bias_tau_s",t)||t.size()!=6||
       !pnh.getParam("imu/preintegration/bias_driving_sigma",q)||q.size()!=6)
      throw std::invalid_argument("Sensor bias model requires six time constants and driving noise sigmas");
    for(int i=0;i<6;i++){tau[i]=t[i];driving[i]=q[i];}
    biasInnovationSigma(tau,driving,.001);
    std::string directory;
    if(!pnh.getParam("logs_directory",directory)||directory.empty())throw std::invalid_argument("Bias model requires an audit directory");
    audit.open(directory+"imu_bias_model.csv");
    if(!audit)throw std::runtime_error("Cannot open bias model audit");
    audit<<std::setprecision(17)<<"stamp_ns,dt,phi0,phi1,phi2,phi3,phi4,phi5,sigma0,sigma1,sigma2,sigma3,sigma4,sigma5\n";
  }
  void record(double stamp,double dt) {
    if(stamp-last_record<1.-1e-9)return;
    last_record=stamp;const auto phi=biasDecay(tau,dt),sigma=biasInnovationSigma(tau,driving,dt);
    audit<<std::llround(stamp*1e9)<<','<<dt;
    for(int i=0;i<6;i++)audit<<','<<phi[i];
    for(int i=0;i<6;i++)audit<<','<<sigma[i];
    audit<<'\n';audit.flush();
  }
};
inline SensorBiasModel& biasModel(){static SensorBiasModel model;return model;}
}
