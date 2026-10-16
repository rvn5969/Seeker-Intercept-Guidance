#include "intercept/guidance.hpp"
#include <cmath>
#include <iostream>
#define CHECK(x) do{if(!(x)){std::cerr<<"CHECK failed: " #x " at line "<<__LINE__<<'\n';return 1;}}while(0)
using namespace intercept;

int main(){
  PnConfig pn{4}; InterceptorConfig v{300,0,0,80,250,.08,{0,0}};
  PredictiveConfig p{.05,1,1,.002,.01,.25,.5,.5,1.5,60,20,5,5,true};
  CHECK(std::abs(proportionalNavigation({{1000,0},{-100,0}},pn,80))<1e-12);
  CHECK(std::abs(proportionalNavigation({{1000,100},{-100,-20}},pn,80))>.1);
  PredictiveGuidance g(pn,p,v); const RelativeState rel{{3000,500},{-520,-10}}; const OwnshipState own{{0,0},300,0,0};
  const double u=g.command(rel,own,0,{0,10}); CHECK(std::isfinite(u)); CHECK(std::abs(u)<=80+1e-12); CHECK(g.lastSequence().size()==20);
  return 0;
}
