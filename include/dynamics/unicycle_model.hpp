#pragma once

#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

class UnicycleModel final : public DynamicsModel {
 public:
  int StateDim() const override { return 3; }
  int ControlDim() const override { return 2; }
  Vector NextState(const Vector& state, const Vector& control, double dt) const override;
};

}  // namespace my_al_ilqr
