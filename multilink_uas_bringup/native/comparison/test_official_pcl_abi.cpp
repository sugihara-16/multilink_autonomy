#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <iostream>
int main(){
  using Cloud=pcl::PointCloud<pcl::PointXYZINormal>;
  Cloud::Ptr input(new Cloud);Cloud output;
  for(int i=0;i<20000;++i){pcl::PointXYZINormal p{};p.x=i*.001f;p.y=(i%17)*.01f;p.z=1;input->push_back(p);}
  pcl::VoxelGrid<pcl::PointXYZINormal> filter;filter.setInputCloud(input);filter.setLeafSize(.1,.1,.1);filter.filter(output);
  std::cout<<"EIGEN_MAX_ALIGN_BYTES="<<EIGEN_MAX_ALIGN_BYTES<<" DEFAULT="<<EIGEN_DEFAULT_ALIGN_BYTES<<" MALLOC="<<EIGEN_MALLOC_ALREADY_ALIGNED<<" output="<<output.size()<<std::endl;
}
