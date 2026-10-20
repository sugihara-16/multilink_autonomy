#include "drpm.hpp"
#include <random>
#include <cassert>
using namespace mimosa::lidar;
using V=Eigen::Vector3d;using M=Eigen::Matrix3d;
using Cloud=Eigen::Matrix<double,Eigen::Dynamic,3>;
M normalCov(const Cloud& p,const std::vector<M>& c) {
 Cloud y=p.rowwise()-p.colwise().mean();
 Eigen::SelfAdjointEigenSolver<M> es(y.transpose()*y/p.rows());
 return drpmNormalCovariance(y,es.eigenvalues(),es.eigenvectors(),c);
}
V normal(const Cloud& p) {Cloud y=p.rowwise()-p.colwise().mean();Eigen::SelfAdjointEigenSolver<M> es(y.transpose()*y/p.rows());return es.eigenvectors().col(0);}
int main(){
 std::mt19937 gen(739);std::normal_distribution<double> dist;
 const double sigma=.03;const int count=100000;
 for(bool radial:{false,true}) for(double tilt:{0.0,0.9}) {
  Cloud p(5,3);p<<-.6,-.6,3,.6,-.6,3,.6,.6,3,-.6,.6,3,0,0,3;
  M R=Eigen::AngleAxisd(tilt,V::UnitY()).toRotationMatrix();p=(p.rowwise()-V(0,0,3).transpose())*R.transpose();p.rowwise()+=V(0,0,3).transpose();
  std::vector<M> cs(5),roots(5);
  for(int j=0;j<5;++j){V u=p.row(j).normalized();cs[j]=radial?drpmRadialCovariance(u,sigma):M(M::Identity()*sigma*sigma);roots[j]=radial?M(sigma*u*u.transpose()):M(sigma*M::Identity());}
  M eta=normalCov(p,cs);V n=normal(p);M nx=drpmSkew(n);M predicted=nx*eta*nx.transpose();
  // Central finite difference of the *actual* PCA operation.
  M fd=M::Zero();
  for(int j=0;j<5;++j){M D;for(int k=0;k<3;++k){Cloud a=p,b=p;a(j,k)+=1e-5;b(j,k)-=1e-5;V na=normal(a),nb=normal(b);if(na.dot(n)<0)na=-na;if(nb.dot(n)<0)nb=-nb;D.col(k)=(na-nb)/2e-5;}fd+=D*cs[j]*D.transpose();}
  assert((fd-predicted).norm()/predicted.norm()<1e-7);
  V sum=V::Zero();M squares=M::Zero();
  for(int i=0;i<count;++i){Cloud q=p;for(int j=0;j<5;++j)q.row(j)+=(roots[j]*V(dist(gen),dist(gen),dist(gen))).transpose();V ni=normal(q);if(ni.dot(n)<0)ni=-ni;V e=ni-n;sum+=e;squares+=e*e.transpose();}
  M empirical=squares/count-(sum/count)*(sum/count).transpose();
  double rel=(empirical-predicted).norm()/predicted.norm();
  std::cout<<"normal radial="<<radial<<" tilt="<<tilt<<" MonteCarlo_relative_error="<<rel<<std::endl;
  assert(rel<.035);
  M transformed=normalCov(p*R.transpose(),[&](){std::vector<M> c;for(auto& x:cs)c.push_back(R*x*R.transpose());return c;}());
  assert((transformed-R*eta*R.transpose()).norm()<1e-10);
 }
 V origin(.4,-.2,.1),p(3,4,5);M C=drpmRadialCovariance(p-origin,sigma);assert(std::abs(C.trace()-sigma*sigma)<1e-15);
 M R=Eigen::AngleAxisd(.7,V(1,2,3).normalized()).toRotationMatrix();V t(9,-4,2);
 assert((drpmRadialCovariance((R*p+t)-(R*origin+t),sigma)-R*C*R.transpose()).norm()<1e-15);
 // Existing default is exactly the explicit isotropic covariance path.
 using M6=Eigen::Matrix<double,6,6>;using V6=Eigen::Matrix<double,6,1>;
 std::vector<V> ps,ns;std::vector<M> nc,pc;std::vector<double> ws;M6 H=M6::Zero();
 for(int i=0;i<1200;++i){V p(4*dist(gen),4*dist(gen),dist(gen)),n=V(.06*dist(gen),.06*dist(gen),1).normalized();ps.push_back(p);ns.push_back(n);nc.push_back(.003*M::Identity());pc.push_back(sigma*sigma*M::Identity());ws.push_back(2);V6 v;v<<p.cross(n),n;H+=2*v*v.transpose();}
 M6 H1=H,H2=H;V6 b1=V6::Ones(),b2=b1;auto q1=drpmReweight(H1,b1,ps,ns,ws,nc,sigma,10);auto q2=drpmReweight(H2,b2,ps,ns,ws,nc,sigma,10,pc);
 assert((q1-q2).norm()==0 && (H1-H2).norm()==0 && (b1-b2).norm()==0);
 // Monte Carlo checks the mean/variance of projected J^T J noise used by DRPM.
 V n=V(.2,.3,1).normalized();M nc0=.0002*(M::Identity()-n*n.transpose());
 M6 B=M6::Zero(),N=M6::Zero();B.topLeftCorner<3,3>()=-drpmSkew(n);B.topRightCorner<3,3>()=drpmSkew(p)*drpmSkew(n);B.bottomRightCorner<3,3>()=drpmSkew(n);N.topLeftCorner<3,3>()=C;N.bottomRightCorner<3,3>()=nc0;
 M6 Cv=B*N*B.transpose();V6 v;v<<p.cross(n),n;V6 u;u<<.3,.4,-.2,.6,-.1,.5;u.normalize();double a=u.dot(Cv*u),b=u.dot(v),mu=0,m2=0;
 Eigen::SelfAdjointEigenSolver<M6> es(Cv);M6 root=es.eigenvectors()*es.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal();
 for(int i=0;i<count;++i){V6 z;for(int k=0;k<6;++k)z[k]=dist(gen);double e=u.dot(root*z),q=2*b*e+e*e;mu+=q;m2+=q*q;}
 mu/=count;m2=m2/count-mu*mu;double expected_var=2*a*a+4*a*b*b;
 assert(std::abs(mu-a)<5*std::sqrt(expected_var/count));assert(std::abs(m2/expected_var-1)<.025);
 std::cout<<"All radial covariance, PCA propagation, frame transformation, legacy and Hessian-noise tests passed"<<std::endl;
}
