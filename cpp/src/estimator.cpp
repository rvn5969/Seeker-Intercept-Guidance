#include "intercept/estimator.hpp"
#include "intercept/guidance.hpp"
#include <algorithm>
#include <cmath>

namespace intercept {
namespace {
using M4=std::array<std::array<double,4>,4>;
M4 eye(){ M4 a{}; for(int i=0;i<4;++i) a[i][i]=1.0; return a; }
M4 mul(const M4&a,const M4&b){ M4 c{}; for(int i=0;i<4;++i)for(int k=0;k<4;++k)for(int j=0;j<4;++j)c[i][j]+=a[i][k]*b[k][j]; return c; }
M4 trans(const M4&a){ M4 t{}; for(int i=0;i<4;++i)for(int j=0;j<4;++j)t[i][j]=a[j][i]; return t; }
M4 add(const M4&a,const M4&b){ M4 c{}; for(int i=0;i<4;++i)for(int j=0;j<4;++j)c[i][j]=a[i][j]+b[i][j]; return c; }
}

RelativeEkf::RelativeEkf(EstimatorConfig estimator,SeekerConfig seeker):estimator_(estimator),seeker_(seeker){ P_=eye(); }

void RelativeEkf::reset(const SeekerMeasurement& z){
  const auto p=fromPolar(z.range,z.bearing); x_={p.x,p.y,0.0,0.0}; P_={};
  P_[0][0]=P_[1][1]=estimator_.initial_position_variance;
  P_[2][2]=P_[3][3]=estimator_.initial_velocity_variance; initialized_=true;
}

void RelativeEkf::predict(double dt,Vec2 a){
  if(!initialized_||dt<=0) return;
  M4 F=eye(); F[0][2]=dt; F[1][3]=dt;
  x_[0]+=dt*x_[2]+0.5*dt*dt*a.x; x_[1]+=dt*x_[3]+0.5*dt*dt*a.y;
  x_[2]+=dt*a.x; x_[3]+=dt*a.y;
  const double q=estimator_.process_accel_sigma*estimator_.process_accel_sigma;
  const double d2=dt*dt,d3=d2*dt,d4=d2*d2; M4 Q{};
  Q[0][0]=Q[1][1]=0.25*d4*q; Q[0][2]=Q[2][0]=Q[1][3]=Q[3][1]=0.5*d3*q;
  Q[2][2]=Q[3][3]=d2*q; P_=add(mul(mul(F,P_),trans(F)),Q);
}

void RelativeEkf::update(const SeekerMeasurement& z){
  if(!z.valid) return;
  if(!initialized_){ reset(z); return; }
  const double rx=x_[0],ry=x_[1],r2=std::max(rx*rx+ry*ry,1e-12),r=std::sqrt(r2);
  const double H[2][4]={{rx/r,ry/r,0,0},{-ry/r2,rx/r2,0,0}};
  const double residual[2]={z.range-r,wrapAngle(z.bearing-std::atan2(ry,rx))};
  const double R[2]={seeker_.range_sigma*seeker_.range_sigma,seeker_.bearing_sigma*seeker_.bearing_sigma};
  double S[2][2]{};
  for(int a=0;a<2;++a)for(int b=0;b<2;++b){
    for(int i=0;i<4;++i)for(int j=0;j<4;++j) S[a][b]+=H[a][i]*P_[i][j]*H[b][j];
    if(a==b) S[a][b]+=R[a];
  }
  const double det=S[0][0]*S[1][1]-S[0][1]*S[1][0]; if(std::abs(det)<1e-18) return;
  const double Si[2][2]={{S[1][1]/det,-S[0][1]/det},{-S[1][0]/det,S[0][0]/det}};
  double PHt[4][2]{},K[4][2]{};
  for(int i=0;i<4;++i)for(int a=0;a<2;++a)for(int j=0;j<4;++j) PHt[i][a]+=P_[i][j]*H[a][j];
  for(int i=0;i<4;++i)for(int a=0;a<2;++a)for(int b=0;b<2;++b) K[i][a]+=PHt[i][b]*Si[b][a];
  for(int i=0;i<4;++i) x_[i]+=K[i][0]*residual[0]+K[i][1]*residual[1];

  // Joseph update keeps covariance symmetric and nonnegative.
  M4 A=eye();
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int a=0;a<2;++a) A[i][j]-=K[i][a]*H[a][j];
  M4 KRKt{};
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)for(int a=0;a<2;++a) KRKt[i][j]+=K[i][a]*R[a]*K[j][a];
  P_=add(mul(mul(A,P_),trans(A)),KRKt);
}

RelativeState RelativeEkf::state() const { return {{x_[0],x_[1]},{x_[2],x_[3]}}; }
std::array<double,4> RelativeEkf::variances() const { return {P_[0][0],P_[1][1],P_[2][2],P_[3][3]}; }

RelativeState propagateRelative(RelativeState s,double dt,Vec2 a){
  if(dt>0){ s.r=s.r+s.v*dt+a*(0.5*dt*dt); s.v=s.v+a*dt; }
  return s;
}

} // namespace intercept
