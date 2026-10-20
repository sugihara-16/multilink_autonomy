#include "lidar_depth_bootstrap.hpp"
#include <iostream>
#include <stdexcept>
using Eigen::Vector3d;using Eigen::Matrix3d;
void require(bool test,const char* why){if(!test)throw std::runtime_error(why);}
int main(){
  Matrix3d K=Matrix3d::Identity();K(0,0)=K(1,1)=240.;K(0,2)=240.;K(1,2)=135.;
  int supported=0;
  for(double slope:{-.4,0.,.4})for(double distance:{1.,3.,8.,20.}){
    std::vector<Vector3d> cloud;
    for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y){
      Vector3d ray(.008*x,.008*y,1.);cloud.push_back(ray*(distance/(1.-slope*ray.x())));
    }
    lidar_depth_bootstrap::View view(cloud,K);
    for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y){
      Vector3d ray(.008*x,.008*y,1.);auto s=view.at(ray);
      require(s.valid(),"Supported plane rejected");
      require(std::abs(s.distance-distance*ray.norm()/(1.-slope*ray.x()))<1e-9,"Incorrect ray-plane distance");
      require(s.sigma>=.08,"Uncertainty floor lost");++supported;
    }
    require(!view.at(Vector3d(1,1,1)).valid(),"Unobserved direction acquired depth");
    require(!view.at(Vector3d(0,0,-1)).valid(),"Backward ray acquired depth");
    for(auto& p:cloud)p=-p;
    require(!lidar_depth_bootstrap::View(cloud,K).at(Vector3d(0,0,1)).valid(),"Rear hemisphere leaked into forward camera");
  }
  std::vector<Vector3d> line;
  for(int i=-10;i<=10;++i)line.emplace_back(.01*i,0.,5.);
  require(!lidar_depth_bootstrap::View(line,K).at(Vector3d(0,0,1)).valid(),"Line accepted as plane");
  require(!lidar_depth_bootstrap::View({},K).at(Vector3d(0,0,1)).valid(),"Empty scan acquired depth");
  std::cout<<supported<<" exact planar intersections; absent/behind/line-support rejection: PASS\n";
}
