#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

class DynamicsModel {
 public:
  virtual ~DynamicsModel() = default;

  virtual int StateDim() const = 0;
  virtual int ControlDim() const = 0;
  virtual Vector NextState(const Vector& state, const Vector& control, double dt) const = 0;
};

}  // namespace my_al_ilqr
