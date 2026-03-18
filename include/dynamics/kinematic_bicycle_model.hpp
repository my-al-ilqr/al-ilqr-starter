#pragma once

#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

class KinematicBicycleModel final : public DynamicsModel {
 public:
  explicit KinematicBicycleModel(double wheelbase = 2.8) : wheelbase_(wheelbase) {}

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double Wheelbase() const { return wheelbase_; }

  Vector NextState(const Vector& state, const Vector& control, double dt) const override;

 private:
  double wheelbase_;
};

}  // namespace my_al_ilqr
