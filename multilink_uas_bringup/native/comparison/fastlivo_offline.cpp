// Ordered adapter around upstream callbacks; no queue drops or timestamp scaling.
#include "LIVMapper.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <iomanip>
int main(int argc,char**argv) {
  ros::init(argc,argv,"laserMapping");ros::NodeHandle nh;
  if(argc<3)return 2;
  image_transport::ImageTransport it(nh);LIVMapper mapper(nh);
  mapper.initializeSubscribersAndPublishers(nh,it);
  bool disable_visual_update=false;
  nh.param<bool>("comparison_disable_visual_update",disable_visual_update,false);
  double disable_visual_after_s=-1;
  nh.param<double>("comparison_disable_visual_after_s",disable_visual_after_s,-1.0);
  rosbag::Bag bag;bag.open(argv[1],rosbag::bagmode::Read);
  std::ofstream out(argv[2]);out<<std::setprecision(17)<<"stamp_ns,x,y,z,qx,qy,qz,qw\n";
  std::vector<std::string> topics={mapper.imu_topic,mapper.lid_topic};
  if(mapper.img_en){if(mapper.camera_names.empty())topics.push_back(mapper.img_topic);
    else topics.insert(topics.end(),mapper.camera_topics.begin(),mapper.camera_topics.end());}
  auto subscribed=topics;
  std::vector<uint64_t> counts(topics.size(),0);
  uint64_t visual_calls=0,visual_skipped=0,lio_calls=0;double last=-1;
  for(const auto&m:rosbag::View(bag,rosbag::TopicQuery(subscribed))) {
    if(!ros::ok())return 3;
    if(m.getTopic()==topics[0]){mapper.imu_cbk(m.instantiate<sensor_msgs::Imu>());++counts[0];}
    if(m.getTopic()==topics[1]){mapper.standard_pcl_cbk(m.instantiate<sensor_msgs::PointCloud2>());++counts[1];}
    for(size_t i=2;i<topics.size();++i)if(m.getTopic()==topics[i]){
      if(mapper.camera_names.empty())mapper.img_cbk(m.instantiate<sensor_msgs::Image>());
      else mapper.camera_cbk(m.instantiate<sensor_msgs::Image>(),i-2);
      ++counts[i];
    }
    while(mapper.sync_packages(mapper.LidarMeasures)) {
      mapper.handleFirstFrame();mapper.processImu();
      const bool visual=mapper.LidarMeasures.lio_vio_flg==VIO;
      const bool stop_visual=disable_visual_update || (disable_visual_after_s>=0 &&
          visual && mapper.LidarMeasures.measures.back().vio_time>=disable_visual_after_s);
      if(visual && stop_visual)++visual_skipped;
      else {mapper.stateEstimationAndMapping();if(visual)++visual_calls;else ++lio_calls;}
      if(mapper.img_en && !visual)continue;
      double t=mapper.img_en ? mapper.LidarMeasures.measures.back().vio_time
                            : mapper.LidarMeasures.measures.back().lio_time;
      if(t<=last)continue;last=t;
      const auto&p=mapper._state.pos_end;Eigen::Quaterniond q(mapper._state.rot_end);
      out<<llround(t*1e9)<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
    }
  }
  std::cerr<<"INPUT_COUNTS";for(auto n:counts)std::cerr<<" "<<n;std::cerr<<std::endl;
  for(size_t i=0;i<mapper.camera_names.size();++i)std::cerr<<"CAMERA_COUNTS "<<mapper.camera_names[i]<<" "<<mapper.camera_calls[i]<<" "<<mapper.camera_constraints[i]<<std::endl;
  if(mapper.camera_sync)std::cerr<<"CAMERA_SYNC pending="<<mapper.camera_sync->pending()<<" dropped="<<mapper.camera_sync->dropped<<" duplicates="<<mapper.camera_sync->duplicates<<" late="<<mapper.camera_sync->late<<" ready_dropped="<<mapper.camera_ready_dropped<<std::endl;
  std::cerr<<"UPDATE_COUNTS lio="<<lio_calls<<" visual="<<visual_calls<<" skipped_visual="<<visual_skipped<<std::endl;
  for(size_t i=0;i<mapper.camera_names.size();++i)if(counts[i+2]==0 || mapper.camera_calls[i]==0){
    std::cerr<<"No usable frames from camera "<<mapper.camera_names[i]<<std::endl;return 4;
  }
  return 0;
}
