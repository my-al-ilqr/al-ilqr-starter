#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class TerminalGoalConstraint final : public ConstraintFunction {
 public:
  explicit TerminalGoalConstraint(Vector target_state);

  ConstraintType Type() const override { return ConstraintType::kEquality; }
  int StateDim() const override { return target_state_.size(); }
  int ControlDim() const override { return 0; }
  int OutputDim() const override { return target_state_.size(); }
  std::string Name() const override { return "terminal_goal"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  Vector target_state_;
};

}  // namespace my_al_ilqr
