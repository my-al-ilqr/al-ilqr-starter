#pragma once

#include "autodrive/vehicle_bicycle_config.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class CircularObstacleConstraint final : public ConstraintFunction {
 public:
  CircularObstacleConstraint(double obstacle_center_x,
                             double obstacle_center_y,
                             double obstacle_radius,
                             VehicleCollisionCircle vehicle_circle);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 1; }
  std::string Name() const override { return "circular_obstacle"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  double ObstacleCenterX() const { return obstacle_center_x_; }
  double ObstacleCenterY() const { return obstacle_center_y_; }
  double ObstacleRadius() const { return obstacle_radius_; }
  const VehicleCollisionCircle& VehicleCircle() const { return vehicle_circle_; }

 private:
  double obstacle_center_x_ = 0.0;
  double obstacle_center_y_ = 0.0;
  double obstacle_radius_ = 0.0;
  VehicleCollisionCircle vehicle_circle_;
};

}  // namespace my_al_ilqr
