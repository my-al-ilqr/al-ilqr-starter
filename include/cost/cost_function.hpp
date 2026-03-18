#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

class CostFunction {
 public:
  virtual ~CostFunction() = default;

  virtual int StateDim() const = 0;
  virtual int ControlDim() const = 0;
  virtual double StageCost(const Vector& state, const Vector& control) const = 0;
  virtual double TerminalCost(const Vector& state) const = 0;
};

}  // namespace my_al_ilqr
