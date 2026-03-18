#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr {

enum class ConstraintType {
  kEquality,
  kInequality,
};

class ConstraintFunction {
 public:
  virtual ~ConstraintFunction() = default;

  virtual ConstraintType Type() const = 0;
  virtual int StateDim() const = 0;
  virtual int ControlDim() const = 0;
  virtual int OutputDim() const = 0;
  virtual std::string Name() const = 0;
  virtual Vector Evaluate(const Vector& state, const Vector& control) const = 0;
};

using ConstraintPtr = std::shared_ptr<ConstraintFunction>;

struct ConstraintEvaluation {
  std::string name;
  ConstraintType type = ConstraintType::kEquality;
  int knot_point = 0;
  Vector values;
  double max_violation = 0.0;
};

double MaxViolation(const ConstraintFunction& constraint, const Vector& values);

}  // namespace my_al_ilqr
