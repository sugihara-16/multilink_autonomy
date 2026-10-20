#include "performance/compact_ivox.hpp"
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>

using Map=gtsam_points::iVox;
using Fast=uas_performance::CompactIVox;
void require(bool b,const char* what){if(!b)throw std::runtime_error(what);}
void compare(const Map& a,const Map& b,const std::vector<Eigen::Vector3d>& queries) {
  auto x=a.voxel_data(),y=b.voxel_data();
  require(a.num_voxels()==b.num_voxels() && x->size()==y->size(),"size mismatch");
  for(size_t i=0;i<x->size();++i) {
    require(std::memcmp(x->points[i].data(),y->points[i].data(),4*sizeof(double))==0,"point mismatch");
    require(std::memcmp(x->covs[i].data(),y->covs[i].data(),16*sizeof(double))==0,"covariance mismatch");
    require(x->intensities[i]==y->intensities[i],"intensity mismatch");
  }
  for(auto& p:queries)for(size_t k:{1,5,13})for(double limit:{.01,1.,10000.}) {
    size_t ia[13]={},ib[13]={};double da[13]={},db[13]={};
    auto na=a.knn_search(p.data(),k,ia,da,limit),nb=b.knn_search(p.data(),k,ib,db,limit);
    require(na==nb,"neighbor count mismatch");
    require(std::memcmp(ia,ib,na*sizeof(size_t))==0,"neighbor index mismatch");
    require(std::memcmp(da,db,na*sizeof(double))==0,"distance mismatch");
  }
}
int main(int argc,char**) {
  const bool bench=argc>1;
  std::mt19937 gen(531);std::uniform_real_distribution<double> u(-20,20);
  size_t checked=0;double original_time=0,compact_time=0;
  for(int mode:bench?std::vector<int>{7}:std::vector<int>{1,7,19,27})
  for(int horizon:bench?std::vector<int>{10000}:std::vector<int>{3,10000}) {
    auto original=std::make_shared<Map>(.5);auto compact=std::make_shared<Fast>(.5);
    for(Map* m:{original.get(),static_cast<Map*>(compact.get())}) {
      m->set_lru_horizon(horizon);m->set_lru_clear_cycle(10);m->set_neighbor_voxel_mode(mode);
    }
    std::vector<std::shared_ptr<Map>> old;std::vector<std::shared_ptr<Fast>> oldfast;
    for(int batch=0;batch<(bench?160:45);++batch) {
      std::vector<Eigen::Vector3d> points;std::vector<Eigen::Matrix3d> covs;std::vector<double> intensities;
      for(int i=0;i<(bench?24000:150);++i) {
        points.emplace_back(u(gen)+batch*.5,u(gen),u(gen)*.2);
        covs.push_back(Eigen::Matrix3d::Identity()*(i+1)*1e-6);intensities.push_back(i*.25);
      }
      gtsam_points::PointCloudCPU cloud(points);cloud.add_covs(covs);cloud.add_intensities(intensities);
      old.push_back(original);oldfast.push_back(compact);
      auto t=std::chrono::steady_clock::now();
      original=std::make_shared<Map>(*original);original->insert(cloud);
      original_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();
      t=std::chrono::steady_clock::now();
      compact=std::make_shared<Fast>(*compact);compact->insert(cloud);
      compact_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();
      if(!bench) {
        compare(*original,*compact,points);++checked;
        for(size_t j=0;j<old.size();++j){compare(*old[j],*oldfast[j],points);++checked;}
      }
      if(old.size()>3){old.erase(old.begin());oldfast.erase(oldfast.begin());}
      if(!bench && batch==23) {original->clear();compact->clear();compare(*original,*compact,points);++checked;}
    }
  }
  std::cout<<"{\"snapshot_comparisons\":"<<checked<<",\"original_copy_insert_s\":"<<original_time
           <<",\"compact_copy_insert_s\":"<<compact_time<<"}\n";
}
