// Serialize the existing managers in sensor timestamp order. No observation,
// covariance, factor, iteration count, timestamp or IMU sample is changed.
// This entry point requires synchronous zero-offset simulated sensors.
#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"
#include "mimosa/lidar/manager.hpp"
#include "mimosa/odometry/manager.hpp"
#include "mimosa/radar/manager.hpp"
#include <ros/callback_queue.h>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include "ordered_imu_prefix.hpp"
#ifdef MIMOSA_GRAVITY_TRANSLATION
#include "gravity_translation_manager.hpp"
using VioManager=GravityTranslationOdometryManager;
#elif defined(MIMOSA_TILT_VIO)
#include "tilt_odometry_manager.hpp"
using VioManager=TiltOdometryManager;
#elif defined(MIMOSA_TRANSLATION_VIO)
#include "translation_odometry_manager.hpp"
using VioManager=TranslationOdometryManager;
#elif defined(MIMOSA_TEMPORAL_VIO)
#include "temporal_odometry_manager.hpp"
using VioManager=TemporalOdometryManager;
#else
using VioManager=mimosa::odometry::Manager;
#endif

template<class Base> class DetachedManager:public Base {
public:
  using Base::Base;
  void disconnect(){this->sub_.shutdown();}
};

int main(int argc,char** argv) {
  config::Settings().print_missing=true;
  ros::init(argc,argv,"mimosa_node");ros::NodeHandle pnh("~");
  // Keep the upstream IMU manager unchanged. Its original subscriber has a
  // private, unspun callback queue; the ordered dispatcher calls it explicitly.
  ros::CallbackQueue unused_imu_queue;
  ros::NodeHandle pnh_imu("~",ros::M_string{{"imu/manager/imu_in","imu/manager/ordered_unused"}});
  pnh_imu.setCallbackQueue(&unused_imu_queue);
  auto imu=std::make_shared<mimosa::imu::Manager>(pnh_imu);
  auto graph=std::make_shared<mimosa::graph::Manager>(pnh,imu);
  DetachedManager<mimosa::lidar::Manager> lidar(pnh,imu,graph);lidar.disconnect();
  DetachedManager<VioManager> vio(pnh,imu,graph);vio.disconnect();
  // This entry point is explicitly limited to LiDAR + external visual odometry.
  bool radar=false;pnh.param("radar/manager/enabled",radar,false);
  if(radar)throw std::runtime_error("Ordered LIVO node does not schedule radar");
  bool vio_enabled=false;
  pnh.param("odometry/manager/enabled",vio_enabled,false);
  double lidar_offset=0,vio_offset=0,imu_offset=0;
  pnh.param("imu/manager/ts_offset",imu_offset,0.);
  pnh.param("lidar/manager/ts_offset",lidar_offset,0.);
  pnh.param("odometry/manager/ts_offset",vio_offset,0.);
  if(lidar_offset!=0 || vio_offset!=0 || imu_offset!=0)throw std::runtime_error("Ordered node requires zero configured timestamp offsets");
  std::mutex mutex;std::condition_variable cv;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> clouds;
  std::deque<nav_msgs::Odometry::ConstPtr> poses;
  std::deque<sensor_msgs::Imu::ConstPtr> inertial;
  uint64_t latest_cloud=0,latest_pose=0,latest_imu=0,processed_imu=0;
  bool seen_pose=false;
  auto sub_imu=pnh.subscribe<sensor_msgs::Imu>("imu/manager/imu_in",10000,
    [&](const sensor_msgs::Imu::ConstPtr& m){
      std::lock_guard<std::mutex> lock(mutex);
      if(m->header.stamp.toNSec()<=latest_imu)throw std::runtime_error("Nonmonotonic ordered IMU input");
      inertial.push_back(m);latest_imu=m->header.stamp.toNSec();cv.notify_one();
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto sub_lidar=pnh.subscribe<sensor_msgs::PointCloud2>("lidar/manager/lidar_in",1000,
    [&](const sensor_msgs::PointCloud2::ConstPtr& m){
      std::lock_guard<std::mutex> lock(mutex);clouds.push_back(m);
      latest_cloud=std::max(latest_cloud,m->header.stamp.toNSec());cv.notify_one();
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto sub_vio=pnh.subscribe<nav_msgs::Odometry>("odometry/manager/odometry_in",1000,
    [&](const nav_msgs::Odometry::ConstPtr& m){
      std::lock_guard<std::mutex> lock(mutex);poses.push_back(m);
      latest_pose=std::max(latest_pose,m->header.stamp.toNSec());seen_pose=true;cv.notify_one();
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  ros::AsyncSpinner inputs(1);
  inputs.start();
  std::string directory;pnh.param<std::string>("logs_directory",directory,"");
  std::ofstream audit(directory+"ordered_inputs.csv");audit<<"stamp_ns,source,wall_s,cloud_queue,vio_queue,imu_count,imu_upto_ns\n";
  try {
    while(ros::ok()) {
      sensor_msgs::PointCloud2::ConstPtr cloud;nav_msgs::Odometry::ConstPtr pose;
      size_t nc=0,np=0;
      std::vector<sensor_msgs::Imu::ConstPtr> batch;
      {
        std::unique_lock<std::mutex> lock(mutex);
        auto ready=[&](){
          // Buffer the startup clouds until the first visual pose is known.
          // Otherwise arrival latency decides whether a newer LiDAR scan is
          // processed before the first VIO factor, even with identical data.
          // Pure LIO has no visual stream to wait for.
          if(vio_enabled && !seen_pose)return false;
          uint64_t watermark=std::min(latest_imu,seen_pose?std::min(latest_cloud,latest_pose):latest_cloud);
          return (!clouds.empty() && clouds.front()->header.stamp.toNSec()<=watermark) ||
                 (!poses.empty() && poses.front()->header.stamp.toNSec()<=watermark);
        };
        if(!cv.wait_for(lock,std::chrono::milliseconds(20),ready))continue;
        const auto tc=clouds.empty()?UINT64_MAX:clouds.front()->header.stamp.toNSec();
        const auto tp=poses.empty()?UINT64_MAX:poses.front()->header.stamp.toNSec();
        if(tc<=tp){cloud=clouds.front();clouds.pop_front();}
        else{pose=poses.front();poses.pop_front();}
        // Provide exactly the IMU samples up to this event, not all samples
        // that happen to have arrived. estimateAttitude() averages its whole
        // buffer, so feeding future samples changes the initial state.
        const uint64_t event_stamp=std::min(tc,tp);
        const auto count=orderedImuPrefix(inertial,processed_imu,event_stamp,[](const auto& m){return m->header.stamp.toNSec();});
        for(size_t i=0;i<count;++i){
          batch.push_back(inertial.front());inertial.pop_front();
        }
        // A first cloud can precede the gate's first IMU. Leave the manager
        // uninitialized in that case; its existing isImuReady() check skips it.
        // Otherwise provide the deterministic interpolation bracket (also
        // handles a one-nanosecond ROS floating-point timestamp roundoff).
        if(!batch.empty())processed_imu=batch.back()->header.stamp.toNSec();
        nc=clouds.size();np=poses.size();
      }
      auto start=std::chrono::steady_clock::now();
      for(const auto& m:batch)imu->callback(m);
      if(cloud)lidar.callback(cloud);else vio.callback(pose);
      const double dt=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      audit<<(cloud?cloud->header.stamp.toNSec():pose->header.stamp.toNSec())<<','<<(cloud?"lidar":"vio")<<','
           <<std::setprecision(10)<<dt<<','<<nc<<','<<np<<','<<batch.size()<<','<<processed_imu<<'\n';
    }
  } catch(...) {inputs.stop();throw;}
  inputs.stop();
  return 0;
}
