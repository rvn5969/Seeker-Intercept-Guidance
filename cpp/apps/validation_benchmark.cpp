#include "intercept/config.hpp"
#include "intercept/guidance.hpp"
#include "intercept/guidance_stack.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

using namespace intercept;

namespace {
struct Scenario { double downrange{},lateral{},heading{},target_accel{},latency{}; std::uint64_t noise_seed{}; };
struct Pending { double deliver{}; SeekerMeasurement measurement{}; };
struct Result {
  double miss{},effort{},position_rmse{},velocity_rmse{},duration{};
  bool hit{};
};

std::uint64_t freshSeed(){
  std::random_device rd;
  const auto clock=static_cast<std::uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  return (static_cast<std::uint64_t>(rd())<<32)^static_cast<std::uint64_t>(rd())^clock;
}

double sample(std::mt19937_64& rng,Range r){ return std::uniform_real_distribution<double>(r.lo,r.hi)(rng); }

double percentile(std::vector<double> v,double p){
  std::sort(v.begin(),v.end()); if(v.empty()) return 0.0;
  const double x=p*(v.size()-1); const auto lo=static_cast<std::size_t>(std::floor(x));
  const auto hi=std::min(v.size()-1,lo+1); const double f=x-lo;
  return v[lo]*(1.0-f)+v[hi]*f;
}

Result run(const ExperimentConfig& cfg,const Scenario& s,const std::string& mode){
  GuidanceStack stack(cfg,mode); std::mt19937_64 rng(s.noise_seed);
  std::normal_distribution<double> range_noise(0.0,cfg.seeker.range_sigma),bearing_noise(0.0,cfg.seeker.bearing_sigma);
  std::uniform_real_distribution<double> uniform(0.0,1.0);

  const double dt=cfg.simulation.truth_dt,seeker_dt=1.0/cfg.seeker.update_rate;
  Vec2 pI=cfg.interceptor.initial_position,pT{s.downrange,s.lateral};
  double psiI=cfg.interceptor.initial_heading,psiT=s.heading,aI=cfg.interceptor.initial_accel,cmd=aI;
  double miss=1e100,effort=0.0,pos_sq=0.0,vel_sq=0.0; std::size_t estimate_count=0;
  bool hit=false,was_closing=false; double next_seeker=0.0,next_guidance=0.0,last_time=0.0;
  std::deque<Pending> pending; SeekerMeasurement latest{};
  const auto steps=static_cast<std::size_t>(std::ceil(cfg.simulation.max_time/dt));

  for(std::size_t k=0;k<=steps;++k){
    const double t=std::min(k*dt,cfg.simulation.max_time); last_time=t;
    const Vec2 vI{cfg.interceptor.speed*std::cos(psiI),cfg.interceptor.speed*std::sin(psiI)};
    const Vec2 vT{cfg.target.speed*std::cos(psiT),cfg.target.speed*std::sin(psiT)};
    const RelativeState truth{pT-pI,vT-vI}; const auto geom=geometry(truth); const double R=geom.range; miss=std::min(miss,R);
    hit=hit || R<cfg.simulation.intercept_radius; was_closing=was_closing || geom.range_rate<0.0;
    if(was_closing && geom.range_rate>0.0) return {miss,effort,std::sqrt(pos_sq/std::max<std::size_t>(1,estimate_count)),std::sqrt(vel_sq/std::max<std::size_t>(1,estimate_count)),t,hit};
    if(t>=cfg.simulation.max_time-1e-12) break;

    if(t+1e-12>=next_seeker){
      next_seeker+=seeker_dt;
      if(uniform(rng)>=cfg.seeker.dropout){
        SeekerMeasurement z{t,R+range_noise(rng),wrapAngle(std::atan2(truth.r.y,truth.r.x)+bearing_noise(rng)),0,0,0,true};
        pending.push_back({t+s.latency,z});
      }
    }

    if(t+1e-12>=next_guidance){
      next_guidance+=cfg.simulation.guidance_dt; bool got=false;
      while(!pending.empty()&&pending.front().deliver<=t+1e-12){ latest=pending.front().measurement; pending.pop_front(); got=true; }
      SeekerMeasurement z=latest; z.valid=got; z.own_speed=cfg.interceptor.speed; z.own_heading=psiI; z.own_lateral_accel=aI;
      const auto c=stack.update(z,t); if(c.valid) cmd=c.lateral_accel;
      if(stack.hasEstimate()){
        const auto e=stack.estimate(); const Vec2 ep=e.r-truth.r,ev=e.v-truth.v;
        pos_sq+=dot(ep,ep); vel_sq+=dot(ev,ev); ++estimate_count;
      }
    }

    effort+=cmd*cmd*dt;
    const double actuator_alpha=1.0-std::exp(-dt/cfg.interceptor.actuator_tau);
    aI+=actuator_alpha*(cmd-aI); aI=clampAccel(aI,cfg.interceptor.max_accel);
    psiI+=aI/cfg.interceptor.speed*dt;
    psiT+=s.target_accel/cfg.target.speed*dt;
    pI=pI+Vec2{cfg.interceptor.speed*std::cos(psiI),cfg.interceptor.speed*std::sin(psiI)}*dt;
    pT=pT+Vec2{cfg.target.speed*std::cos(psiT),cfg.target.speed*std::sin(psiT)}*dt;
  }
  return {miss,effort,std::sqrt(pos_sq/std::max<std::size_t>(1,estimate_count)),std::sqrt(vel_sq/std::max<std::size_t>(1,estimate_count)),last_time,hit};
}

void timingCheck(const ExperimentConfig& cfg){
  if(!cfg.timing.enabled) return;
  const auto seed=cfg.timing.seed.value_or(freshSeed()); std::mt19937_64 rng(seed);
  PredictiveGuidance g(cfg.guidance.pn,cfg.guidance.predictive,cfg.interceptor);
  std::vector<double> times; times.reserve(cfg.timing.samples); double previous=0.0;
  for(int i=0;i<cfg.timing.samples;++i){
    RelativeState s{{sample(rng,cfg.timing.relative_x),sample(rng,cfg.timing.relative_y)},
                    {sample(rng,cfg.timing.relative_vx),sample(rng,cfg.timing.relative_vy)}};
    OwnshipState own{{0,0},cfg.interceptor.speed,cfg.interceptor.initial_heading,0};
    const auto a=std::chrono::steady_clock::now(); previous=g.command(s,own,previous); const auto b=std::chrono::steady_clock::now();
    times.push_back(std::chrono::duration<double,std::micro>(b-a).count());
  }
  std::cout<<"Timing seed: "<<seed<<"\nTiming: median="<<percentile(times,.5)<<" us, p99="<<percentile(times,.99)
           <<" us, max="<<*std::max_element(times.begin(),times.end())<<" us\n";
}
}

int main(int argc,char** argv){
  try{
    const std::string config_path=argc>1?argv[1]:"config/experiment.json"; const auto cfg=loadConfig(config_path);
    const auto benchmark_seed=cfg.monte_carlo.seed.value_or(freshSeed()); std::mt19937_64 rng(benchmark_seed);
    std::vector<Scenario> scenarios(cfg.monte_carlo.runs); std::vector<Result> pn(cfg.monte_carlo.runs),predictive(cfg.monte_carlo.runs);
    for(auto& s:scenarios){
      s.downrange=sample(rng,cfg.target.downrange); s.lateral=sample(rng,cfg.target.lateral);
      s.heading=cfg.target.heading_center+sample(rng,cfg.target.heading_offset); s.target_accel=sample(rng,cfg.target.accel);
      s.latency=sample(rng,cfg.seeker.latency); s.noise_seed=rng();
    }

    std::atomic<int> next{0}; const unsigned hardware=std::max(1u,std::thread::hardware_concurrency());
    const unsigned workers=std::min<unsigned>(cfg.monte_carlo.max_workers,hardware); std::vector<std::thread> pool;
    for(unsigned w=0;w<workers;++w) pool.emplace_back([&]{
      for(int i;(i=next.fetch_add(1))<cfg.monte_carlo.runs;){ pn[i]=run(cfg,scenarios[i],"pn"); predictive[i]=run(cfg,scenarios[i],"predictive"); }
    });
    for(auto& t:pool) t.join();

    std::filesystem::path out(cfg.output.benchmark_csv); if(out.has_parent_path()) std::filesystem::create_directories(out.parent_path());
    std::ofstream f(out); if(!f) throw std::runtime_error("Cannot write "+out.string());
    f<<"run,benchmark_seed,scenario_seed,initial_downrange_m,initial_lateral_m,target_heading_rad,target_accel_mps2,latency_s,"
       "pn_miss_m,predictive_miss_m,pn_hit,predictive_hit,pn_effort,predictive_effort,pn_position_rmse_m,predictive_position_rmse_m,"
       "pn_velocity_rmse_mps,predictive_velocity_rmse_mps,pn_duration_s,predictive_duration_s\n";
    std::vector<double> mp,mq,ep,eq,rpp,rpq; int hp=0,hq=0;
    for(int i=0;i<cfg.monte_carlo.runs;++i){
      const auto& s=scenarios[i]; mp.push_back(pn[i].miss); mq.push_back(predictive[i].miss); ep.push_back(pn[i].effort); eq.push_back(predictive[i].effort);
      rpp.push_back(pn[i].position_rmse); rpq.push_back(predictive[i].position_rmse); hp+=pn[i].hit; hq+=predictive[i].hit;
      f<<i<<','<<benchmark_seed<<','<<s.noise_seed<<','<<s.downrange<<','<<s.lateral<<','<<s.heading<<','<<s.target_accel<<','<<s.latency<<','
       <<pn[i].miss<<','<<predictive[i].miss<<','<<pn[i].hit<<','<<predictive[i].hit<<','<<pn[i].effort<<','<<predictive[i].effort<<','
       <<pn[i].position_rmse<<','<<predictive[i].position_rmse<<','<<pn[i].velocity_rmse<<','<<predictive[i].velocity_rmse<<','
       <<pn[i].duration<<','<<predictive[i].duration<<'\n';
    }
    const double med_p=percentile(mp,.5),med_q=percentile(mq,.5);
    std::cout<<std::fixed<<std::setprecision(3)<<"Monte Carlo seed: "<<benchmark_seed<<"\nRuns: "<<cfg.monte_carlo.runs<<"\n"
      <<"PN:         hit "<<hp<<'/'<<cfg.monte_carlo.runs<<", median miss "<<med_p<<" m, p95 "<<percentile(mp,.95)<<" m\n"
      <<"Predictive: hit "<<hq<<'/'<<cfg.monte_carlo.runs<<", median miss "<<med_q<<" m, p95 "<<percentile(mq,.95)<<" m\n"
      <<"Median miss change: "<<(med_p>0?100.0*(med_p-med_q)/med_p:0.0)<<"%\n"
      <<"Median effort: PN "<<percentile(ep,.5)<<", predictive "<<percentile(eq,.5)<<"\n"
      <<"Median position RMSE: PN "<<percentile(rpp,.5)<<" m, predictive "<<percentile(rpq,.5)<<" m\n";
    timingCheck(cfg); return 0;
  } catch(const std::exception& e){ std::cerr<<"error: "<<e.what()<<'\n'; return 1; }
}
