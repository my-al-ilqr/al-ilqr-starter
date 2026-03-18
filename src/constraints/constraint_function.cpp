#include "constraints/constraint_function.hpp"

#include <algorithm>
#include <cmath>

namespace my_al_ilqr {

double MaxViolation(const ConstraintFunction& constraint, const Vector& values) {
  if (values.size() == 0) {
    return 0.0;
  }

  if (constraint.Type() == ConstraintType::kEquality) {
    return values.cwiseAbs().maxCoeff();
  }

  double max_violation = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    max_violation = std::max(max_violation, values(i));
  }
  return std::max(0.0, max_violation);
}

}  // namespace my_al_ilqr
