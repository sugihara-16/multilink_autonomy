// Evaluation-only recorder. Never publishes truth to estimation or planning.
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <array>
#include <cmath>

struct Monitor {
 std::filesystem::path out;
 std::array<std::ofstream,3> files;
 std::ofstream encoders;
 std::array<size_t,3> counts{};
 std::array<ros::Subscriber,3> subs;
 ros::Subscriber enc_sub;ros::Timer timer;
 bool initial=false,checked=false;double first_time=0,max_displacement=0;
 std::array<double,3> origin{};
 Monitor(ros::NodeHandle& n,std::string directory,std::string truth):out(directory) {
  const std::array<std::string,3> names{"rovio","mimosa","truth"},topics{"/rovio/odometry","/slam/odometry",truth};
  for(size_t i=0;i<3;++i) {
   files[i].open(out/(names[i]+".csv"));files[i]<<std::setprecision(17)<<"stamp_ns,x,y,z,qx,qy,qz,qw\n";
   subs[i]=n.subscribe<nav_msgs::Odometry>(topics[i],100,[this,i](const nav_msgs::Odometry::ConstPtr& m){
    const auto&p=m->pose.pose.position;const auto&q=m->pose.pose.orientation;
    files[i]<<m->header.stamp.toNSec()<<','<<p.x<<','<<p.y<<','<<p.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<'\n';++counts[i];
    if(i==0 && !checked) {
     if(!initial) {initial=true;origin={p.x,p.y,p.z};first_time=m->header.stamp.toSec();}
     const double dx=p.x-origin[0],dy=p.y-origin[1],dz=p.z-origin[2];
     max_displacement=std::max(max_displacement,std::sqrt(dx*dx+dy*dy+dz*dz));
     if(m->header.stamp.toSec()-first_time>=3.)checked=true;
    }
   });
  }
  encoders.open(out/"encoders.csv");encoders<<std::setprecision(17)<<"t,yaw,pitch\n";
  enc_sub=n.subscribe<std_msgs::Float64MultiArray>("/active/encoder",2000,[this](const std_msgs::Float64MultiArray::ConstPtr& m){
   for(size_t i=0;i<m->data.size();++i)encoders<<(i?",":"")<<m->data[i];encoders<<'\n';
  });
  timer=n.createTimer(ros::Duration(1),[this](const ros::TimerEvent&){report();});
 }
 void countsJson(std::ostream& f) const {f<<"{\"rovio\":"<<counts[0]<<",\"mimosa\":"<<counts[1]<<",\"truth\":"<<counts[2]<<'}';}
 void report() {
  for(auto& f:files)f.flush();encoders.flush();
  auto tmp=out/"evaluation_status.tmp";std::ofstream f(tmp);f<<std::setprecision(17)<<"{\"counts\":";countsJson(f);
  f<<",\"startup\":{\"checked\":"<<(checked?"true":"false")<<",\"max_displacement_m\":"<<max_displacement;
  if(initial)f<<",\"valid\":"<<(max_displacement<1.?"true":"false");f<<"}}\n";f.close();
  std::filesystem::rename(tmp,out/"evaluation_status.json");
 }
 ~Monitor(){report();std::ofstream f(out/"monitor_counts.json");countsJson(f);f<<'\n';}
};
int main(int argc,char**argv) {
 ros::init(argc,argv,"active_validation_monitor");std::string output,truth="/dragon/ground_truth";
 for(int i=1;i+1<argc;++i) {if(std::string(argv[i])=="--output")output=argv[++i];else if(std::string(argv[i])=="--truth-topic")truth=argv[++i];}
 if(output.empty())return 2;ros::NodeHandle n;Monitor m(n,output,truth);ros::spin();return 0;
}
