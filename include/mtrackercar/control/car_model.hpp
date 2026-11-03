#pragma once

#include <cmath>
#include "mtrackercar/control/car_kinematics.hpp"

namespace mtrackercar
{
struct CarState
{
  double x{}, y{}, theta{};
};

// Ideal rear-axle bicycle model, exact integration for constant inputs.
inline void integrate_car(CarState & state, double speed, double yaw_rate, double dt)
{
  if (!std::isfinite(dt) || dt <= 0) return;
  const double turn = yaw_rate * dt;
  const double half = turn / 2;
  const double sinc = std::abs(half) < 1e-8 ? 1.0 : std::sin(half) / half;
  const double distance = speed * dt * sinc;
  state.x += distance * std::cos(state.theta + half);
  state.y += distance * std::sin(state.theta + half);
  state.theta = std::atan2(std::sin(state.theta + turn), std::cos(state.theta + turn));
}
}  // namespace mtrackercar
