#include <mimosa/lidar/geometric_factor.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
using namespace mimosa;using namespace mimosa::lidar;
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ofstream out(argv[1],std::ios::binary);
#if USE_OPENMP
 std::cout<<"USE_OPENMP resolves true\n";
#else
 std::cout<<"USE_OPENMP resolves false\n";
#endif
 pcl::PointCloud<Point> target,source;
 for(int axis=0;axis<3;++axis)for(int x=0;x<55;++x)for(int y=0;y<55;++y){
  Eigen::Vector3f v;v[axis]=3.;v[(axis+1)%3]=(x-27)*.12;v[(axis+2)%3]=(y-27)*.12;
  Point p{};p.x=v.x();p.y=v.y();p.z=v.z();target.push_back(p);
  if(x%2==0&&y%2==0){p.x+=.015;p.z-=.01;source.push_back(p);}
 }
 auto map=std::make_shared<IncrementalVoxelMapPCL>(.5);map->underlying()->set_neighbor_voxel_mode(19);map->insert(target);
 auto start=std::chrono::steady_clock::now();
 for(int projected: {0,1})for(int binary: {0,1}){
  RegistrationConfig cfg;cfg.project_on_degneneracy=projected;cfg.drpm_enabled=false;
  std::shared_ptr<ICPFactor> f=binary?std::make_shared<ICPFactor>(X(0),X(1),map,source,cfg):std::make_shared<ICPFactor>(X(0),map,source,cfg);
  for(int k=0;k<120;++k){
   const int j=k/3;
   gtsam::Values values;values.insert(X(0),gtsam::Pose3(gtsam::Rot3::Rz(.001*(j%5)),gtsam::Point3(.03*(j%3),.001*(j%7),.005)));
   if(binary)values.insert(X(1),gtsam::Pose3());values.insert(G(0),gtsam::Unit3(Eigen::Vector3d(0,(k%3==2)?.001:0,-1)));
   // Shared map growth must not invalidate the existing correspondence policy.
   if (k==44) map->insert(target);
   if (k==88) f=std::dynamic_pointer_cast<ICPFactor>(f->clone());
   auto g=std::dynamic_pointer_cast<gtsam::HessianFactor>(f->linearize(values));auto h=g->augmentedInformation();
   out.write((char*)h.data(),sizeof(double)*h.size());
   for(auto x:f->getStatuses())out.write((char*)&x,sizeof x);
   for(const auto& x:f->getCorresNormalsTarget())out.write((char*)x.data(),sizeof(double)*3);
  }
 }
 std::cout<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"\n";
}
