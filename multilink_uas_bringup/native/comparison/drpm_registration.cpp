#include "mimosa/lidar/drpm_registration.hpp"
#include "mimosa/lidar/drpm_kernel.hpp"
#include <iomanip>
#include <omp.h>

namespace mimosa { namespace lidar {
namespace {
using V3=Eigen::Vector3d;using M3=Eigen::Matrix3d;
struct Linearization {
 DrpmM6 H=DrpmM6::Zero();DrpmV6 rhs=DrpmV6::Zero();
 std::vector<V3> points,normals;std::vector<M3> normal_covariances,point_covariances;
 std::vector<double> weights;
};
struct Match { bool valid=false;V3 p,n;M3 cn,cp;double weight=0,residual=0; };
Linearization linearize(const gtsam::Pose3& T_W_L,const std::vector<V3>& source,
 const IncrementalVoxelMapPCL::Ptr& target,const RegistrationConfig& cfg) {
 const gtsam::Pose3 T_L_W=T_W_L.inverse();const M3 R_L_W=T_L_W.rotation().matrix();
 std::vector<Match> matches(source.size());
 if(cfg.drpm_noise_model==2 && target->underlying()->num_voxels()>0 && !target->underlying()->has_covs())
   throw std::runtime_error("DRPM missing map acquisition covariance");
 #pragma omp parallel for num_threads(4)
 for(size_t i=0;i<source.size();++i) {
   const V3 p=source[i],pw=T_W_L*p;
   std::vector<size_t> ids(5);std::vector<double> distances(5);
   if(!target->knn_search(pw,5,ids,distances) || distances.back()>cfg.max_corres_distance*cfg.max_corres_distance)continue;
   Eigen::Matrix<double,3,5> q;
   for(int j=0;j<5;++j)q.col(j)=T_L_W*V3(target->underlying()->point(ids[j]).head<3>());
   const V3 mean=q.rowwise().mean();
   const Eigen::Matrix<double,Eigen::Dynamic,3> centered=q.transpose().rowwise()-mean.transpose();
   Eigen::SelfAdjointEigenSolver<M3> es(centered.transpose()*centered/5.0);
   if(es.info()!=Eigen::Success || es.eigenvalues()[2]>3*es.eigenvalues()[1])continue;
   V3 n;M3 cn;double normal_variance,distance;
   // The upstream reference header is vendored byte-for-byte, including its
   // robust eigenvalue adjustment. All inputs are in current LiDAR coordinates.
   std::tie(n,normal_variance,distance,cn)=degeneracy::EstimateNormal<5,double>(q,cfg.lidar_point_noise_std_dev,true);
   (void)distance;
   if(cfg.drpm_noise_model==2) {
     std::vector<M3> cs(5);
     for(int j=0;j<5;++j)cs[j]=R_L_W*target->underlying()->cov(ids[j]).topLeftCorner<3,3>()*R_L_W.transpose();
     cn=drpmNormalCovariance(centered,es.eigenvalues(),es.eigenvectors(),cs);
     // Same 0.1 rad normal-uncertainty rejection threshold (paper IV-A),
     // evaluated under the selected noise model in all conditions.
     Eigen::SelfAdjointEigenSolver<M3> ce(cn);
     if(ce.info()!=Eigen::Success)continue;
     normal_variance=ce.eigenvalues().maxCoeff();
   }
   if(!cn.allFinite() || normal_variance>0.01)continue;
   if((centered*n).array().abs().maxCoeff()>cfg.plane_validity_distance)continue;
   if(n.dot(-mean)<0)n=-n;
   double residual=n.dot(p-mean);
   if(1-.9*std::abs(residual)/std::sqrt(p.norm())<.9)continue;
   double weight=1.0/(cfg.lidar_point_noise_std_dev*cfg.lidar_point_noise_std_dev);
   if(cfg.use_huber && std::abs(residual)/cfg.lidar_point_noise_std_dev>cfg.huber_threshold)
     weight*=cfg.huber_threshold*cfg.lidar_point_noise_std_dev/std::abs(residual);
   auto& m=matches[i];m.valid=true;m.p=p;m.n=n;m.cn=cn;m.weight=weight;m.residual=residual;
   if(cfg.drpm_noise_model==2)m.cp=drpmRadialCovariance(p,cfg.lidar_point_noise_std_dev);
 }
 Linearization l;
 for(const auto& m:matches)if(m.valid){
   l.points.push_back(m.p);l.normals.push_back(m.n);l.normal_covariances.push_back(m.cn);l.weights.push_back(m.weight);
   if(cfg.drpm_noise_model==2)l.point_covariances.push_back(m.cp);
   DrpmV6 j;j<<m.p.cross(m.n),m.n;l.H+=m.weight*j*j.transpose();l.rhs-=m.weight*j*m.residual;
 }
 return l;
}
}
std::shared_ptr<DrpmPoseFactor> drpmRegistration(
 gtsam::Key key,const gtsam::Pose3& initial_body,const gtsam::Pose3& body_to_lidar,
 const pcl::PointCloud<Point>& source,const IncrementalVoxelMapPCL::Ptr& target,
 const RegistrationConfig& cfg,double stamp,std::ofstream& audit) {
 if(cfg.num_corres_points!=5 || (cfg.drpm_noise_model!=0 && cfg.drpm_noise_model!=2))
   throw std::runtime_error("DRPM comparison requires 5 neighbors and noise mode 0 or 2");
 std::vector<V3> points;points.reserve(source.size());
 const gtsam::Pose3 T_L_B=body_to_lidar.inverse();
 for(const auto& p:source.points) {
   if(cfg.drpm_noise_model==2 && p.t!=0)throw std::runtime_error("Radial mode requires instantaneous scans");
   points.push_back(T_L_B*V3(p.getVector3fMap().cast<double>()));
 }
 gtsam::Pose3 T_W_L=initial_body*body_to_lidar;int iterations=0;bool converged=false;
 std::string status="ok";DrpmKernelResult result;size_t correspondences=0;
 double last_translation=0,last_rotation=0;
 try {
   for(int i=0;i<cfg.drpm_icp_max_iterations;++i) {
     auto l=linearize(T_W_L,points,target,cfg);correspondences=l.points.size();
     if(correspondences<6)throw std::runtime_error("insufficient_correspondences");
     result=drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,l.normal_covariances,l.point_covariances,
       cfg.lidar_point_noise_std_dev,cfg.drpm_snr_factor,cfg.drpm_registration_mode);
     last_rotation=result.update.head<3>().norm();last_translation=result.update.tail<3>().norm();
     T_W_L=T_W_L*gtsam::Pose3::Expmap(result.update);iterations=i+1;
     if(last_rotation<1e-4 && last_translation<1e-3){converged=true;break;}
   }
   // Information evaluated at the final registration pose. No zero-update
   // regularizer is carried into the graph as fake sensor information.
   auto l=linearize(T_W_L,points,target,cfg);correspondences=l.points.size();
   if(correspondences<6)throw std::runtime_error("insufficient_final_correspondences");
   result=drpmKernel(l.H,l.rhs,l.points,l.normals,l.weights,l.normal_covariances,l.point_covariances,
     cfg.lidar_point_noise_std_dev,cfg.drpm_snr_factor,cfg.drpm_registration_mode);
 } catch(const std::runtime_error& e) { status=e.what(); }
 audit<<std::fixed<<std::setprecision(0)<<stamp*1e9<<','<<cfg.drpm_registration_mode<<','<<cfg.drpm_noise_model
      <<','<<status<<','<<iterations<<','<<converged<<','<<correspondences<<std::setprecision(12)
      <<','<<last_translation<<','<<last_rotation;
 if(status=="ok") {
   const DrpmV6 total=gtsam::Pose3::Logmap((initial_body*body_to_lidar).between(T_W_L));
   for(int k=0;k<6;++k)audit<<','<<result.probabilities[k];
   for(int k=0;k<6;++k)audit<<','<<result.eigenvalues[k];
   for(int k=0;k<6;++k)audit<<','<<total[k];
 }
 audit<<'\n';audit.flush();
 if(status!="ok")return nullptr;
 return std::make_shared<DrpmPoseFactor>(key,T_W_L,body_to_lidar,result.sqrt_information);
}
}}
