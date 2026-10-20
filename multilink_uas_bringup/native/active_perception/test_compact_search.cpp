#include "performance/compact_ivox.hpp"
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>

struct Result {size_t n=0;size_t ids[5]={};double d[5]={};};
int main() {
  std::mt19937 gen(239);std::uniform_real_distribution<double> u(-1,1);
  size_t total=0;double old_time=0,new_time=0;
  for(double leaf:{.3,.5,1.}) {
    std::vector<Eigen::Vector3d> points,queries;
    for(int i=0;i<300000;++i) {
      points.emplace_back(80*u(gen),2*u(gen),2*u(gen));
      points.back()[i%2+1]=(i%4<2?-2.:2.)+.01*u(gen);
    }
    for(double v:{-100.,-1.,-.5,0.,.5,1.,100.})for(int axis=0;axis<3;++axis) {
      Eigen::Vector3d p=Eigen::Vector3d::Zero();p[axis]=v;
      points.push_back(p);queries.push_back(p);
      p[axis]=std::nextafter(v,-INFINITY);points.push_back(p);queries.push_back(p);
      p[axis]=std::nextafter(v,INFINITY);points.push_back(p);queries.push_back(p);
    }
    for(size_t i=0;i<20000;++i) queries.push_back(points[(i*13)%points.size()]+Eigen::Vector3d(.03*u(gen),.03*u(gen),.03*u(gen)));
    gtsam_points::iVox original(leaf);uas_performance::CompactIVox compact(leaf);
    original.set_neighbor_voxel_mode(19);compact.set_neighbor_voxel_mode(19);
    gtsam_points::PointCloudCPU cloud(points);original.insert(cloud);compact.insert(cloud);
    std::vector<Result> expected(queries.size());
    auto start=std::chrono::steady_clock::now();
    for(size_t i=0;i<queries.size();++i)expected[i].n=original.knn_search(queries[i].data(),5,expected[i].ids,expected[i].d);
    old_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::vector<Result> actual(queries.size());start=std::chrono::steady_clock::now();
    for(size_t i=0;i<queries.size();++i)actual[i].n=compact.knn_search(queries[i].data(),5,actual[i].ids,actual[i].d);
    new_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    for(size_t i=0;i<queries.size();++i) {
      if(expected[i].n!=actual[i].n || std::memcmp(expected[i].ids,actual[i].ids,sizeof(Result::ids)) ||
          std::memcmp(expected[i].d,actual[i].d,sizeof(Result::d)))throw std::runtime_error("Exact query result mismatch");
    }
    total+=queries.size();
  }
  std::cout<<"{\"queries\":"<<total<<",\"byte_identical\":true,\"original_s\":"<<old_time<<",\"candidate_s\":"<<new_time<<"}\n";
}
