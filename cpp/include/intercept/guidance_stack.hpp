#pragma once
#include "estimator.hpp"
#include "guidance.hpp"

namespace intercept {

class GuidanceStack {
public:
  explicit GuidanceStack(const ExperimentConfig& cfg,std::string mode_override={});
  GuidanceCommand update(const SeekerMeasurement& z,double now);
  bool hasEstimate() const { return have_estimate_; }
  RelativeState estimate() const { return current_estimate_; }
  std::array<double,4> estimateVariances() const { return ekf_.variances(); }
private:
  SimulationConfig sim_{}; InterceptorConfig vehicle_{}; EstimatorConfig estimator_cfg_{};
  GuidanceConfig guidance_{}; RelativeEkf ekf_; PredictiveGuidance predictive_;
  std::string mode_;
  double filter_time_{}, last_command_{}, last_est_time_{};
  Vec2 target_accel_est_{}, last_rel_v_{}; RelativeState current_estimate_{};
  bool have_filter_time_{false}, have_rel_v_{false}, have_estimate_{false};
};

} // namespace intercept
