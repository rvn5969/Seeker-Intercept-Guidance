#include "intercept/config.hpp"
#include "intercept/guidance.hpp"
#include "intercept/guidance_stack.hpp"
#include <cmath>
#include <iostream>
#define CHECK(x) do{if(!(x)){std::cerr<<"CHECK failed: " #x " at line "<<__LINE__<<'\n';return 1;}}while(0)
using namespace intercept;

double miss(const ExperimentConfig& cfg,double target_accel){
  GuidanceStack stack(cfg,"predictive"); const double dt=cfg.simulation.guidance_dt;
  Vec2 pI{0,0},pT{3000,500}; double psiI=0,psiT=3.141592653589793,a=0,cmd=0,best=1e9;
  for(int k=0;k<900;++k){
    const double t=k*dt; Vec2 vI{300*std::cos(psiI),300*std::sin(psiI)},vT{220*std::cos(psiT),220*std::sin(psiT)};
    RelativeState rel{pT-pI,vT-vI}; best=std::min(best,norm(rel.r)); if(best<5) break;
    SeekerMeasurement z{t,norm(rel.r),std::atan2(rel.r.y,rel.r.x),300,psiI,a,true}; auto c=stack.update(z,t); if(c.valid) cmd=c.lateral_accel;
    a+=(1-std::exp(-dt/.08))*(cmd-a); psiI+=a/300*dt; psiT+=target_accel/220*dt;
    pI=pI+Vec2{300*std::cos(psiI),300*std::sin(psiI)}*dt; pT=pT+Vec2{220*std::cos(psiT),220*std::sin(psiT)}*dt;
  }
  return best;
}

int main(int argc,char** argv){ CHECK(argc==2); auto cfg=loadConfig(argv[1]); CHECK(miss(cfg,20)<5); CHECK(miss(cfg,-20)<5); return 0; }
