#pragma once
#include "types.hpp"
#include <cstdint>
#include <optional>
#include <string>

namespace intercept {

struct SimulationConfig { double truth_dt{}, guidance_dt{}, max_time{}, intercept_radius{}; };
struct InterceptorConfig {
  double speed{}, initial_heading{}, initial_accel{}, max_accel{}, max_slew{}, actuator_tau{};
  Vec2 initial_position{};
};
struct TargetConfig { double speed{}, heading_center{}; Range downrange{}, lateral{}, heading_offset{}, accel{}; };
struct SeekerConfig { double update_rate{}, range_sigma{}, bearing_sigma{}, dropout{}; Range latency{}; };
struct EstimatorConfig {
  double initial_position_variance{}, initial_velocity_variance{}, process_accel_sigma{};
  double accel_smoothing_alpha{}, accel_max{};
};
struct PnConfig { double navigation_constant{}; };
struct PredictiveConfig {
  double prediction_dt{}, weight_zem{}, weight_uncertainty{}, weight_accel{}, weight_slew{};
  double optimizer_step_fraction{}, optimizer_min_step{}, step_decay{};
  double pn_handoff_tgo_multiplier{}, max_time_to_go{};
  int horizon_steps{}, optimizer_iterations{}, control_blocks{};
  bool fallback_to_pn{};
};
struct GuidanceConfig { std::string mode; PnConfig pn{}; PredictiveConfig predictive{}; };
struct MonteCarloConfig { int runs{}, max_workers{}; std::optional<std::uint64_t> seed; };
struct TimingConfig {
  bool enabled{}; int samples{}; std::optional<std::uint64_t> seed;
  Range relative_x{}, relative_y{}, relative_vx{}, relative_vy{};
};
struct HilConfig { int udp_port{}; };
struct OutputConfig { std::string benchmark_csv, summary_csv, figure_dir; };

struct ExperimentConfig {
  SimulationConfig simulation{};
  InterceptorConfig interceptor{};
  TargetConfig target{};
  SeekerConfig seeker{};
  EstimatorConfig estimator{};
  GuidanceConfig guidance{};
  MonteCarloConfig monte_carlo{};
  TimingConfig timing{};
  HilConfig hil{};
  OutputConfig output{};
};

ExperimentConfig loadConfig(const std::string& path);
void validateConfig(const ExperimentConfig& cfg);

} // namespace intercept
