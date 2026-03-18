#pragma once

#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

class Trajectory {
 public:
  Trajectory(int state_dim, int control_dim, int horizon);

  int StateDim() const { return state_dim_; }
  int ControlDim() const { return control_dim_; }
  int Horizon() const { return horizon_; }

  void SetTimeStep(double dt) { dt_ = dt; }
  double TimeStep() const { return dt_; }

  Vector& State(int k);
  const Vector& State(int k) const;

  Vector& Control(int k);
  const Vector& Control(int k) const;

 private:
  int state_dim_;
  int control_dim_;
  int horizon_;
  double dt_ = 0.0;
  std::vector<Vector> states_;
  std::vector<Vector> controls_;
};

}  // namespace my_al_ilqr
