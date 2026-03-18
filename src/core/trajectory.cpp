#include "core/trajectory.hpp"

#include <stdexcept>

namespace my_al_ilqr {

Trajectory::Trajectory(int state_dim, int control_dim, int horizon)
    : state_dim_(state_dim),
      control_dim_(control_dim),
      horizon_(horizon),
      states_(horizon + 1, Vector::Zero(state_dim)),
      controls_(horizon, Vector::Zero(control_dim)) {
  if (state_dim <= 0 || control_dim <= 0 || horizon <= 0) {
    throw std::invalid_argument("Trajectory dimensions and horizon must be positive.");
  }
}

Vector& Trajectory::State(int k) {
  return states_.at(k);
}

const Vector& Trajectory::State(int k) const {
  return states_.at(k);
}

Vector& Trajectory::Control(int k) {
  return controls_.at(k);
}

const Vector& Trajectory::Control(int k) const {
  return controls_.at(k);
}

}  // namespace my_al_ilqr
