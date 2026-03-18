#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class ControlBoxConstraint final : public ConstraintFunction {
 public:
  ControlBoxConstraint(int state_dim, Vector lower_bound, Vector upper_bound);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return state_dim_; }
  int ControlDim() const override { return lower_bound_.size(); }
  int OutputDim() const override { return 2 * lower_bound_.size(); }
  std::string Name() const override { return "control_box"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  int state_dim_ = 0;
  Vector lower_bound_;
  Vector upper_bound_;
};

}  // namespace my_al_ilqr
