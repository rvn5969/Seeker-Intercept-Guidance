#pragma once
#include "config.hpp"
#include <array>

namespace intercept {

class RelativeEkf {
public:
  RelativeEkf(EstimatorConfig estimator,SeekerConfig seeker);
  void reset(const SeekerMeasurement& z);
  void predict(double dt,Vec2 known_relative_accel={});
  void update(const SeekerMeasurement& z);
  RelativeState state() const;
  std::array<double,4> variances() const;
  bool initialized() const { return initialized_; }
private:
  using V4=std::array<double,4>;
  using M4=std::array<std::array<double,4>,4>;
  EstimatorConfig estimator_{}; SeekerConfig seeker_{};
  V4 x_{}; M4 P_{}; bool initialized_{false};
};

RelativeState propagateRelative(RelativeState s,double dt,Vec2 accel={});

} // namespace intercept
