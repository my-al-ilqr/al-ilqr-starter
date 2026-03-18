#include "dynamics/linear_point_mass_model.hpp"

#include <stdexcept>

namespace my_al_ilqr {

Vector LinearPointMassModel::NextState(const Vector& state,
                                       const Vector& control,
                                       double dt) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("LinearPointMassModel received an input with invalid dimensions.");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive.");
  }

  Vector next_state(StateDim());
  const double position = state(0);
  const double velocity = state(1);
  const double acceleration = control(0);

  next_state(0) = position + dt * velocity + 0.5 * dt * dt * acceleration;
  next_state(1) = velocity + dt * acceleration;
  return next_state;
}

}  // namespace my_al_ilqr
