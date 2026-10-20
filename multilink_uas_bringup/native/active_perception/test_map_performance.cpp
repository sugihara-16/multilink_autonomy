#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  std::ofstream out(argv[1],std::ios::binary);
  const bool bench=std::string(argv[2])=="bench";
  auto start=std::chrono::steady_clock::now();
  for(int horizon: {3,10000}) {
    gtsam_points::IncrementalVoxelMap<gtsam_points::FlatContainer> map(.5);
    map.set_lru_horizon(horizon);map.set_lru_clear_cycle(10);map.set_neighbor_voxel_mode(19);
    for(int batch=0;batch<(bench?600:60);++batch) {
      std::vector<Eigen::Vector3d> points;
      std::vector<Eigen::Matrix3d> covs;
      for(int i=0;i<(bench?4000:100);++i) {
        points.emplace_back((batch%83)*.03+(i%71)*.17,(i%37)*.23+batch*.05,(i%17)*.19);
        covs.push_back(Eigen::Matrix3d::Identity()*(i+1)*1e-6);
      }
      gtsam_points::PointCloudCPU cloud(points);cloud.add_covs(covs);
      map.insert(cloud);
      if(bench)continue;
      auto data=map.voxel_data();size_t n=data->size();out.write((char*)&n,sizeof n);
      for(size_t i=0;i<n;++i) {
        out.write((char*)data->points[i].data(),sizeof(double)*4);
        out.write((char*)data->covs[i].data(),sizeof(double)*16);
      }
      for(const auto& p:points) {
        size_t ids[5]={};double d[5]={};size_t k=map.knn_search(p.data(),5,ids,d);
        out.write((char*)&k,sizeof k);out.write((char*)ids,sizeof ids);out.write((char*)d,sizeof d);
      }
    }
  }
  std::cout<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"\n";
}
