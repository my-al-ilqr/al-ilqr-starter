#include "constraints/terminal_goal_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

TerminalGoalConstraint::TerminalGoalConstraint(Vector target_state)
    : target_state_(std::move(target_state)) {
  if (target_state_.size() == 0) {
    throw std::invalid_argument("TerminalGoalConstraint target state must be non-empty.");
  }
}

Vector TerminalGoalConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim()) {
    throw std::invalid_argument("TerminalGoalConstraint received a state with invalid dimensions.");
  }
  if (control.size() != 0 && control.size() != ControlDim()) {
    throw std::invalid_argument("TerminalGoalConstraint received an unexpected control vector.");
  }
  return state - target_state_;
}

}  // namespace my_al_ilqr
