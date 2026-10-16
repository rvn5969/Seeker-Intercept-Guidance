#include "intercept/config.hpp"
#include "intercept/guidance.hpp"
#include <cmath>
#include <iostream>
#define CHECK(x) do{if(!(x)){std::cerr<<"CHECK failed: " #x " at line "<<__LINE__<<'\n';return 1;}}while(0)
using namespace intercept;

int main(int argc,char** argv){
  CHECK(argc==2); const auto cfg=loadConfig(argv[1]); CHECK(cfg.monte_carlo.runs>0); CHECK(!cfg.monte_carlo.seed.has_value());
  RelativeState s{{3,4},{-1,0}}; const auto g=geometry(s);
  CHECK(std::abs(g.range-5.0)<1e-12); CHECK(std::abs(g.range_rate+0.6)<1e-12); CHECK(g.closing_speed>0); CHECK(g.los_rate>0);
  RelativeState collision{{1000,50},{-100,-5}}; CHECK(zeroEffortMiss(collision,60.0)<1e-9);
  CHECK(std::abs(clampAccel(100,80)-80)<1e-12); CHECK(std::abs(clampSlew(10,0,250,.01)-2.5)<1e-12);
  return 0;
}
