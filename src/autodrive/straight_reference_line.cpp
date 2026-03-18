#include "autodrive/straight_reference_line.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

StraightReferenceLine::StraightReferenceLine(double origin_x, double origin_y, double heading)
    : origin_x_(origin_x), origin_y_(origin_y), heading_(heading) {}

double StraightReferenceLine::LongitudinalPosition(const Vector& state) const {
  if (state.size() < 2) {
    throw std::invalid_argument("StraightReferenceLine requires at least x-y state coordinates.");
  }
  const double dx = state(0) - origin_x_;
  const double dy = state(1) - origin_y_;
  return dx * std::cos(heading_) + dy * std::sin(heading_);
}

double StraightReferenceLine::LateralError(const Vector& state) const {
  if (state.size() < 2) {
    throw std::invalid_argument("StraightReferenceLine requires at least x-y state coordinates.");
  }
  const double dx = state(0) - origin_x_;
  const double dy = state(1) - origin_y_;
  return -dx * std::sin(heading_) + dy * std::cos(heading_);
}

double StraightReferenceLine::HeadingError(const Vector& state) const {
  if (state.size() < 3) {
    throw std::invalid_argument("StraightReferenceLine requires a heading state.");
  }
  return state(2) - heading_;
}

}  // namespace my_al_ilqr
