#pragma once
#include <algorithm>
#include <cmath>

namespace mtrackercar
{
struct CarInput
{
  double rear_velocity{};
  double servo{};
};

// Twist uses yaw rate, including in reverse. A car cannot turn in place.
inline CarInput car_input(double speed, double yaw_rate, double wheelbase,
  double radius, double max_steering, double servo_angle)
{
  if (!std::isfinite(speed) || !std::isfinite(yaw_rate) || std::abs(speed) < 1e-6) {
    return {};
  }
  const double steering = std::clamp(std::atan(wheelbase * yaw_rate / speed),
    -max_steering, max_steering);
  return {std::clamp(speed / radius, -127.0, 127.0), steering / servo_angle};
}
}  // namespace mtrackercar
