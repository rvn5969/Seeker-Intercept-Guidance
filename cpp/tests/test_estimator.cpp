#include "intercept/estimator.hpp"
#include <cmath>
#include <iostream>
#define CHECK(x) do{if(!(x)){std::cerr<<"CHECK failed: " #x " at line "<<__LINE__<<'\n';return 1;}}while(0)
using namespace intercept;

int main(){
  EstimatorConfig e{25,400,2.0,.25,40}; SeekerConfig q{20,.5,1e-4,0,{0,0}}; RelativeEkf f(e,q);
  for(int k=0;k<120;++k){
    const double t=.05*k; const Vec2 r{1000-40*t,200+2*t}; SeekerMeasurement z{t,norm(r),std::atan2(r.y,r.x),300,0,0,true};
    if(k) f.predict(.05); f.update(z);
  }
  const auto s=f.state(); CHECK(std::abs(s.r.x-(1000-40*5.95))<5); CHECK(std::abs(s.r.y-(200+2*5.95))<5);
  CHECK(std::abs(s.v.x+40)<5); CHECK(std::abs(s.v.y-2)<5);
  RelativeState p{{100,50},{-10,2}}; p=propagateRelative(p,.5,{2,-4});
  CHECK(std::abs(p.r.x-95.25)<1e-12); CHECK(std::abs(p.r.y-50.5)<1e-12); CHECK(std::abs(p.v.x+9)<1e-12);
  return 0;
}
