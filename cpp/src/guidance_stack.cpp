#include "intercept/guidance_stack.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace intercept {

GuidanceStack::GuidanceStack(const ExperimentConfig& cfg,std::string mode_override)
  :sim_(cfg.simulation),vehicle_(cfg.interceptor),estimator_cfg_(cfg.estimator),guidance_(cfg.guidance),
   ekf_(cfg.estimator,cfg.seeker),predictive_(cfg.guidance.pn,cfg.guidance.predictive,cfg.interceptor),
   mode_(mode_override.empty()?cfg.guidance.mode:std::move(mode_override)),last_command_(cfg.interceptor.initial_accel){
  if(mode_!="pn"&&mode_!="predictive") throw std::runtime_error("Guidance mode must be pn or predictive");
}

GuidanceCommand GuidanceStack::update(const SeekerMeasurement& z,double now){
  const Vec2 own_a{-z.own_lateral_accel*std::sin(z.own_heading),z.own_lateral_accel*std::cos(z.own_heading)};
  const Vec2 known_rel_a=-own_a;

  if(z.valid){
    if(!have_filter_time_){ ekf_.update(z); filter_time_=z.timestamp; have_filter_time_=true; }
    else if(z.timestamp>filter_time_+1e-12){
      ekf_.predict(std::max(0.0,z.timestamp-filter_time_),known_rel_a); filter_time_=z.timestamp; ekf_.update(z);
    }
  }
  if(!ekf_.initialized()) return {now,0.0,false};

  current_estimate_=propagateRelative(ekf_.state(),std::max(0.0,now-filter_time_),known_rel_a); have_estimate_=true;
  const OwnshipState own{{0,0},z.own_speed,z.own_heading,z.own_lateral_accel};

  if(have_rel_v_&&now>last_est_time_+1e-9){
    const double dt=now-last_est_time_;
    Vec2 a_meas{(current_estimate_.v.x-last_rel_v_.x)/dt+own_a.x,(current_estimate_.v.y-last_rel_v_.y)/dt+own_a.y};
    const double mag=norm(a_meas); if(mag>estimator_cfg_.accel_max) a_meas=a_meas*(estimator_cfg_.accel_max/mag);
    const double a=estimator_cfg_.accel_smoothing_alpha; target_accel_est_=target_accel_est_*(1.0-a)+a_meas*a;
  }
  last_rel_v_=current_estimate_.v; last_est_time_=now; have_rel_v_=true;

  const double measurement_age=std::max(0.0,now-filter_time_);
  const auto variances=ekf_.variances();
  double u=mode_=="predictive"
    ? predictive_.command(current_estimate_,own,last_command_,target_accel_est_,variances,measurement_age,estimator_cfg_.process_accel_sigma)
    : proportionalNavigation(current_estimate_,guidance_.pn,vehicle_.max_accel);
  u=clampSlew(clampAccel(u,vehicle_.max_accel),last_command_,vehicle_.max_slew,sim_.guidance_dt);
  last_command_=u; return {now,u,true};
}

} // namespace intercept
