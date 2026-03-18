#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

class StraightReferenceLine {
 public:
  StraightReferenceLine(double origin_x = 0.0,
                        double origin_y = 0.0,
                        double heading = 0.0);

  double LongitudinalPosition(const Vector& state) const;
  double LateralError(const Vector& state) const;
  double HeadingError(const Vector& state) const;

  double OriginX() const { return origin_x_; }
  double OriginY() const { return origin_y_; }
  double Heading() const { return heading_; }

 private:
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double heading_ = 0.0;
};

}  // namespace my_al_ilqr
