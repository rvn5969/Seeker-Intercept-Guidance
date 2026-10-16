#pragma once
#include <array>
#include <cmath>

namespace intercept {

struct Vec2 {
  double x{}, y{};
  Vec2 operator+(Vec2 b) const { return {x+b.x,y+b.y}; }
  Vec2 operator-(Vec2 b) const { return {x-b.x,y-b.y}; }
  Vec2 operator*(double s) const { return {x*s,y*s}; }
};
inline Vec2 operator-(Vec2 a){ return {-a.x,-a.y}; }
inline double dot(Vec2 a,Vec2 b){ return a.x*b.x+a.y*b.y; }
inline double cross(Vec2 a,Vec2 b){ return a.x*b.y-a.y*b.x; }
inline double norm(Vec2 a){ return std::hypot(a.x,a.y); }
inline Vec2 fromPolar(double r,double a){ return {r*std::cos(a),r*std::sin(a)}; }

struct RelativeState { Vec2 r{}, v{}; };
struct OwnshipState { Vec2 p{}; double speed{}, heading{}, lateral_accel{}; };

struct SeekerMeasurement {
  double timestamp{}, range{}, bearing{};
  double own_speed{}, own_heading{}, own_lateral_accel{};
  bool valid{true};
};

struct GuidanceCommand { double timestamp{}, lateral_accel{}; bool valid{true}; };
struct Range { double lo{}, hi{}; };

} // namespace intercept
