#include "temporal_odometry_factor.hpp"
#include <iostream>
#include <array>
#include <random>
#include <stdexcept>

void require(bool good,const char* what){if(!good)throw std::runtime_error(what);}
int main(){
  std::mt19937 engine(20261017);std::normal_distribution<double> normal;
  auto random=[&](double scale){gtsam::Vector6 v;for(int i=0;i<6;i++)v[i]=scale*normal(engine);return v;};
  auto noise=gtsam::noiseModel::Isotropic::Sigma(6,.1);
  gtsam::Vector6 rho;rho<<.04,.03,.4,.9,.62,.8;
  for(int trial=0;trial<30;trial++){
    std::array<gtsam::Pose3,3> p{gtsam::Pose3::Expmap(random(.3)),gtsam::Pose3::Expmap(random(.3)),gtsam::Pose3::Expmap(random(.3))};
    const auto a=p[0].between(p[1])*gtsam::Pose3::Expmap(random(.03));
    const auto b=p[1].between(p[2])*gtsam::Pose3::Expmap(random(.03));
    TemporalOdometryFactor f(1,2,3,a,b,rho,noise);
    gtsam::Matrix H[3];const auto e=f.evaluateError(p[0],p[1],p[2],&H[0],&H[1],&H[2]);
    require(e.allFinite(),"nonfinite residual");
    for(int i=0;i<3;i++)for(int j=0;j<6;j++){
      auto plus=p,minus=p;gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=1e-6;
      plus[i]=p[i].retract(dx);minus[i]=p[i].retract(-dx);
      gtsam::Vector diff=(f.evaluateError(plus[0],plus[1],plus[2])-f.evaluateError(minus[0],minus[1],minus[2]))/(2e-6);
      require((diff-H[i].col(j)).norm()<1e-7,"Jacobian mismatch");
    }
    TemporalOdometryFactor independent(1,2,3,a,b,gtsam::Vector6::Zero(),noise);
    require((independent.evaluateError(p[0],p[1],p[2])-gtsam::Pose3::Logmap(b.inverse()*p[1].between(p[2]))).norm()<1e-12,"rho=0 is not independent increment");
    auto eprev=random(.01),innovation=random(.001);
    gtsam::Vector6 current=rho.asDiagonal()*b.inverse().AdjointMap()*eprev+innovation;
    auto p1=a*gtsam::Pose3::Expmap(eprev),p2=p1*b*gtsam::Pose3::Expmap(current);
    require((f.evaluateError(gtsam::Pose3(),p1,p2)-innovation).norm()<1e-12,"transported AR innovation mismatch");
    auto clone=f.clone();require(clone->keys()==f.keys(),"clone keys differ");
    gtsam::KeyVector keys{7,8,9};auto rekeyed=f.rekey(keys);require(rekeyed->keys()==keys,"rekey failed");
  }
  std::cout<<"30 nonzero residual Jacobians (540 columns), rho=0, transported innovations, clone/rekey: PASS\n";
}
