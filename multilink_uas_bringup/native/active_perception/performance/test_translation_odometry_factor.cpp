#include "translation_odometry_factor.hpp"
#include <array>
#include <iostream>
#include <random>
#include <stdexcept>
void require(bool good,const char* what){if(!good)throw std::runtime_error(what);}
int main(){
  std::mt19937 engine(20261017);std::normal_distribution<double> normal;
  auto random=[&](double scale){gtsam::Vector6 v;for(int i=0;i<6;i++)v[i]=scale*normal(engine);return v;};
  auto noise=gtsam::noiseModel::Isotropic::Sigma(3,.1);
  for(int trial=0;trial<30;trial++){
    std::array<gtsam::Pose3,2> p{gtsam::Pose3::Expmap(random(.3)),gtsam::Pose3::Expmap(random(.3))};
    gtsam::Point3 measured=p[0].between(p[1]).translation()+gtsam::Point3(.01,-.02,.03);
    TranslationOdometryFactor f(1,2,measured,noise);
    gtsam::Matrix H[2];const auto e=f.evaluateError(p[0],p[1],&H[0],&H[1]);
    for(int i=0;i<2;i++)for(int j=0;j<6;j++){
      auto plus=p,minus=p;gtsam::Vector6 dx=gtsam::Vector6::Zero();dx[j]=1e-6;
      plus[i]=p[i].retract(dx);minus[i]=p[i].retract(-dx);
      gtsam::Vector diff=(f.evaluateError(plus[0],plus[1])-f.evaluateError(minus[0],minus[1]))/(2e-6);
      require((diff-H[i].col(j)).norm()<1e-7,"Jacobian mismatch");
    }
    // Turning only the later body about its own origin changes no displacement.
    auto later=gtsam::Pose3(gtsam::Rot3::Expmap(gtsam::Vector3(.4,-.3,.2)),p[1].translation());
    require((f.evaluateError(p[0],later)-e).norm()<1e-12,"Visual end rotation leaked into translation constraint");
    auto world=gtsam::Pose3::Expmap(random(1.));
    require((f.evaluateError(world*p[0],world*p[1])-e).norm()<1e-12,"Global-frame invariance failed");
    auto clone=f.clone();require(clone->keys()==f.keys(),"clone keys differ");
    gtsam::KeyVector keys{7,8};require(f.rekey(keys)->keys()==keys,"rekey failed");
  }
  std::cout<<"30 cases / 360 Jacobian columns, no visual end-rotation constraint, global-frame invariance, clone/rekey: PASS\n";
}
