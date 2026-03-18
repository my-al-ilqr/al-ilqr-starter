#include "dynamics/unicycle_model.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

Vector UnicycleModel::NextState(const Vector& state, const Vector& control, double dt) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("UnicycleModel received an input with invalid dimensions.");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive.");
  }

  const double x = state(0);
  const double y = state(1);
  const double heading = state(2);
  const double linear_velocity = control(0);
  const double yaw_rate = control(1);

  Vector next_state(StateDim());
  next_state(0) = x + dt * linear_velocity * std::cos(heading);
  next_state(1) = y + dt * linear_velocity * std::sin(heading);
  next_state(2) = heading + dt * yaw_rate;
  return next_state;
}

}  // namespace my_al_ilqr
