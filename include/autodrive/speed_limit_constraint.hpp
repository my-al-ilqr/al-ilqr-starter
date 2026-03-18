#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class SpeedLimitConstraint final : public ConstraintFunction {
 public:
  SpeedLimitConstraint(double min_speed, double max_speed);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 2; }
  std::string Name() const override { return "speed_limit"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  double min_speed_ = 0.0;
  double max_speed_ = 0.0;
};

}  // namespace my_al_ilqr
