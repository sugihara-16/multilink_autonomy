#include "drpm.hpp"
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <cassert>
int main() {
 using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
 std::vector<V> p{V(.2,.2,.2),V(.4,.2,.2),V(.6,.2,.2)};
 std::vector<M> c;for(const auto& x:p)c.push_back(mimosa::lidar::drpmRadialCovariance(x-V(-1,0,0),.03));
 gtsam_points::PointCloudCPU frame(p);frame.add_covs(c);
 gtsam_points::iVox map(.5);map.insert(frame);assert(map.has_covs());
 auto copy=map;
 for(size_t j=0;j<p.size();++j){size_t i;double d;assert(copy.knn_search(p[j].data(),1,&i,&d)==1);assert(d<1e-20);assert((copy.cov(i).topLeftCorner<3,3>()-c[j]).norm()<1e-15);}
 // Rejected near-duplicate points must not replace covariance of old points.
 std::vector<M> fake(3,M::Identity());frame.add_covs(fake);copy.insert(frame);
 for(size_t j=0;j<p.size();++j){size_t i;double d;assert(copy.knn_search(p[j].data(),1,&i,&d)==1);assert((copy.cov(i).topLeftCorner<3,3>()-c[j]).norm()<1e-15);}
 std::cout<<"Map covariance survives insertion, copying, nearest-neighbor lookup and duplicate rejection"<<std::endl;
}
