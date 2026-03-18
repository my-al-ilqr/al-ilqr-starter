#include "autodrive/circular_obstacle_constraint.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

CircularObstacleConstraint::CircularObstacleConstraint(double obstacle_center_x,
                                                       double obstacle_center_y,
                                                       double obstacle_radius,
                                                       VehicleCollisionCircle vehicle_circle)
    : obstacle_center_x_(obstacle_center_x),
      obstacle_center_y_(obstacle_center_y),
      obstacle_radius_(obstacle_radius),
      vehicle_circle_(vehicle_circle) {
  if (obstacle_radius_ <= 0.0) {
    throw std::invalid_argument("CircularObstacleConstraint obstacle_radius must be positive.");
  }
  if (vehicle_circle_.radius <= 0.0) {
    throw std::invalid_argument("CircularObstacleConstraint vehicle circle radius must be positive.");
  }
}

Vector CircularObstacleConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "CircularObstacleConstraint received an input with invalid dimensions.");
  }

  const double c = std::cos(state(2));
  const double s = std::sin(state(2));
  const double vehicle_center_x =
      state(0) + c * vehicle_circle_.center_x_body - s * vehicle_circle_.center_y_body;
  const double vehicle_center_y =
      state(1) + s * vehicle_circle_.center_x_body + c * vehicle_circle_.center_y_body;
  const double dx = vehicle_center_x - obstacle_center_x_;
  const double dy = vehicle_center_y - obstacle_center_y_;
  const double safe_radius = obstacle_radius_ + vehicle_circle_.radius;
  Vector values(OutputDim());
  values(0) = safe_radius * safe_radius - (dx * dx + dy * dy);
  return values;
}

}  // namespace my_al_ilqr
