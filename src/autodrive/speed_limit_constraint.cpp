#include "autodrive/speed_limit_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

SpeedLimitConstraint::SpeedLimitConstraint(double min_speed, double max_speed)
    : min_speed_(min_speed), max_speed_(max_speed) {
  if (min_speed_ > max_speed_) {
    throw std::invalid_argument("SpeedLimitConstraint bounds are inconsistent.");
  }
}

Vector SpeedLimitConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("SpeedLimitConstraint received an input with invalid dimensions.");
  }

  Vector values(OutputDim());
  values(0) = min_speed_ - state(3);
  values(1) = state(3) - max_speed_;
  return values;
}

}  // namespace my_al_ilqr
