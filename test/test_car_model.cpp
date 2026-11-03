#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include "mtrackercar/control/car_model.hpp"

namespace
{
bool near(double a, double b) {return std::abs(a - b) < 1e-9;}
}

int main()
{
  using namespace mtrackercar;
  CarState state;
  integrate_car(state, 0.2, 0, 5);
  assert(near(state.x, 1) && near(state.y, 0) && near(state.theta, 0));
  integrate_car(state, -0.2, 0, 5);
  assert(near(state.x, 0));
  state = {};
  const double pi = std::acos(-1.0);
  integrate_car(state, 0.1, 0.2, pi / 0.4);
  assert(near(state.x, 0.5) && near(state.y, 0.5) && near(state.theta, pi / 2));
  state = {};
  for (int i = 0; i < 1000; ++i) integrate_car(state, 0.1, 0.2, 2 * pi / 200);
  assert(near(state.x, 0) && near(state.y, 0) && near(state.theta, 0));
  state = {1, 2, pi / 2};
  integrate_car(state, 0.1, 0, 2);
  assert(near(state.x, 1) && near(state.y, 2.2));
  integrate_car(state, 100, 100, 0);
  assert(near(state.x, 1) && near(state.y, 2.2));
  state = {};
  const auto input = car_input(-0.1, 100, 0.19, 0.033, 0.6, 1.047);
  const double speed = input.rear_velocity * 0.033;
  const double yaw = speed * std::tan(input.servo * 1.047) / 0.19;
  assert(speed < 0 && yaw > 0 && yaw < 1);
  integrate_car(state, speed, yaw, 1);
  assert(state.x < 0 && state.y < 0 && state.theta > 0);
}
