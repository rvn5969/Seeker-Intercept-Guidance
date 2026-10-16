#pragma once
#include "config.hpp"
#include <vector>

namespace intercept {

struct Geometry { double range{}, bearing{}, range_rate{}, closing_speed{}, los_rate{}; };

double wrapAngle(double angle);
Geometry geometry(const RelativeState& s);
double timeToGo(const RelativeState& s,double max_time);
double zeroEffortMiss(const RelativeState& s,double max_time);
double clampAccel(double u,double max_accel);
double clampSlew(double u,double previous,double max_slew,double dt);
double proportionalNavigation(const RelativeState& s,const PnConfig& pn,double max_accel);

class PredictiveGuidance {
public:
  PredictiveGuidance(PnConfig pn,PredictiveConfig cfg,InterceptorConfig vehicle);
  double command(const RelativeState& rel,const OwnshipState& own,double previous,Vec2 target_accel={},
                 std::array<double,4> variances={},double measurement_age=0.0,double process_accel_sigma=0.0);
  const std::vector<double>& lastSequence() const { return sequence_; }
private:
  double cost(const RelativeState&,const OwnshipState&,const std::vector<double>&,double,Vec2,
              std::array<double,4>,double,double) const;
  PnConfig pn_{}; PredictiveConfig cfg_{}; InterceptorConfig vehicle_{};
  std::vector<double> sequence_;
};

} // namespace intercept
