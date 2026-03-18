#pragma once

#include "cost/cost_function.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

class QuadraticCost final : public CostFunction {
 public:
  QuadraticCost(Matrix Q, Matrix R, Matrix Qf, Vector state_reference, Vector control_reference);

  int StateDim() const override { return state_reference_.size(); }
  int ControlDim() const override { return control_reference_.size(); }

  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override;

 private:
  Matrix Q_;
  Matrix R_;
  Matrix Qf_;
  Vector state_reference_;
  Vector control_reference_;
};

}  // namespace my_al_ilqr
