#include "intercept/guidance.hpp"
#include <algorithm>
#include <cmath>


namespace intercept {

constexpr double pi = 3.14159265358979323846;
double wrapAngle(double a){ return std::remainder(a,2.0*pi); }

Geometry geometry(const RelativeState& s){
  const double R=norm(s.r); if(R<1e-12) return {};
  const double rr=dot(s.r,s.v)/R;
  return {R,std::atan2(s.r.y,s.r.x),rr,-rr,cross(s.r,s.v)/(R*R)};
}

double timeToGo(const RelativeState& s,double max_time){
  const double vv=dot(s.v,s.v); if(vv<1e-12) return max_time;
  return std::clamp(-dot(s.r,s.v)/vv,0.0,max_time);
}

double zeroEffortMiss(const RelativeState& s,double max_time){ return norm(s.r+s.v*timeToGo(s,max_time)); }
double clampAccel(double u,double m){ return std::clamp(u,-m,m); }
double clampSlew(double u,double p,double rate,double dt){ return std::clamp(u,p-rate*dt,p+rate*dt); }

double proportionalNavigation(const RelativeState& s,const PnConfig& pn,double max_accel){
  const auto g=geometry(s); return clampAccel(pn.navigation_constant*g.closing_speed*g.los_rate,max_accel);
}

PredictiveGuidance::PredictiveGuidance(PnConfig pn,PredictiveConfig cfg,InterceptorConfig vehicle)
  :pn_(pn),cfg_(cfg),vehicle_(vehicle){}

double PredictiveGuidance::cost(const RelativeState& rel,const OwnshipState& own,const std::vector<double>& u,
                                double previous,Vec2 target_accel,std::array<double,4> var,
                                double measurement_age,double process_accel_sigma) const {
  Vec2 own_p=own.p,target_p=own.p+rel.r;
  double psi=own.heading,actual_a=own.lateral_accel,prev=previous,J=0.0;
  Vec2 own_v{own.speed*std::cos(psi),own.speed*std::sin(psi)},target_v=own_v+rel.v;
  const double alpha=1.0-std::exp(-cfg_.prediction_dt/vehicle_.actuator_tau);

  for(double raw:u){
    const double cmd=clampSlew(clampAccel(raw,vehicle_.max_accel),prev,vehicle_.max_slew,cfg_.prediction_dt);
    actual_a+=alpha*(cmd-actual_a); psi+=(actual_a/std::max(own.speed,1e-9))*cfg_.prediction_dt;
    own_v={own.speed*std::cos(psi),own.speed*std::sin(psi)}; own_p=own_p+own_v*cfg_.prediction_dt;
    target_p=target_p+target_v*cfg_.prediction_dt+target_accel*(0.5*cfg_.prediction_dt*cfg_.prediction_dt);
    target_v=target_v+target_accel*cfg_.prediction_dt;
    J+=cfg_.weight_accel*cmd*cmd+cfg_.weight_slew*(cmd-prev)*(cmd-prev); prev=cmd;
  }
  const RelativeState terminal{target_p-own_p,target_v-own_v};
  const double tgo=timeToGo(terminal,cfg_.max_time_to_go);
  const double zem=norm(terminal.r+terminal.v*tgo);

  // Approximate expected squared miss. Older measurements carry more propagated
  // state uncertainty, so latency enters the optimization through this risk term.
  const double q=process_accel_sigma*process_accel_sigma;
  auto propagate_variance=[q](std::array<double,4> x,double dt){
    const double d2=dt*dt,d4=d2*d2;
    x[0]+=d2*x[2]+0.25*d4*q; x[1]+=d2*x[3]+0.25*d4*q;
    x[2]+=d2*q; x[3]+=d2*q; return x;
  };
  var=propagate_variance(var,std::max(0.0,measurement_age));
  var=propagate_variance(var,cfg_.horizon_steps*cfg_.prediction_dt);
  const double miss_variance=var[0]+var[1]+tgo*tgo*(var[2]+var[3]);

  return cfg_.weight_zem*zem*zem+cfg_.weight_uncertainty*miss_variance+J;
}

double PredictiveGuidance::command(const RelativeState& rel,const OwnshipState& own,double previous,Vec2 target_accel,
                                   std::array<double,4> variances,double measurement_age,double process_accel_sigma){
  const double pn=proportionalNavigation(rel,pn_,vehicle_.max_accel);
  const double horizon_time=cfg_.horizon_steps*cfg_.prediction_dt;
  if(timeToGo(rel,cfg_.max_time_to_go)<=cfg_.pn_handoff_tgo_multiplier*horizon_time) return pn;

  if(static_cast<int>(sequence_.size())!=cfg_.horizon_steps) sequence_.assign(cfg_.horizon_steps,pn);
  else if(sequence_.size()>1){ std::rotate(sequence_.begin(),sequence_.begin()+1,sequence_.end()); sequence_.back()=sequence_[sequence_.size()-2]; }

  const int blocks=std::min(cfg_.control_blocks,cfg_.horizon_steps);
  const int block_size=(cfg_.horizon_steps+blocks-1)/blocks;
  double step=std::max(cfg_.optimizer_min_step,cfg_.optimizer_step_fraction*vehicle_.max_accel);
  for(int it=0;it<cfg_.optimizer_iterations;++it){
    for(int b=0;b<blocks;++b){
      const int first=b*block_size,last=std::min(cfg_.horizon_steps,first+block_size); if(first>=last) break;
      const auto base=sequence_; auto best=sequence_; double best_cost=cost(rel,own,best,previous,target_accel,variances,measurement_age,process_accel_sigma);
      for(double sign:{-1.0,1.0}){
        auto candidate=base; for(int k=first;k<last;++k) candidate[k]=clampAccel(candidate[k]+sign*step,vehicle_.max_accel);
        const double c=cost(rel,own,candidate,previous,target_accel,variances,measurement_age,process_accel_sigma); if(c<best_cost){ best=std::move(candidate); best_cost=c; }
      }
      sequence_=std::move(best);
    }
    step=std::max(cfg_.optimizer_min_step,step*cfg_.step_decay);
  }

  if(cfg_.fallback_to_pn){
    const std::vector<double> baseline(cfg_.horizon_steps,pn);
    if(cost(rel,own,baseline,previous,target_accel,variances,measurement_age,process_accel_sigma)<
       cost(rel,own,sequence_,previous,target_accel,variances,measurement_age,process_accel_sigma)) sequence_=baseline;
  }
  return sequence_.front();
}

} // namespace intercept
