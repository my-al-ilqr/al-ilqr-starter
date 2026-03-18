#pragma once

#include <filesystem>

namespace my_al_ilqr {

struct BicycleKinematicConfig {
  double wheelbase = 2.8;
  double min_acceleration = -1.0;
  double max_acceleration = 1.0;
  double min_steering = -0.45;
  double max_steering = 0.45;
  double min_speed = 0.0;
  double max_speed = 6.0;
};

struct VehicleBodyConfig {
  double length = 4.6;
  double width = 1.9;
  double rear_axle_to_rear = 1.0;
};

struct VehicleCollisionCircle {
  double center_x_body = 0.0;
  double center_y_body = 0.0;
  double radius = 0.0;
};

struct VehicleBicycleConfig {
  BicycleKinematicConfig model;
  VehicleBodyConfig body;
};

std::filesystem::path DefaultVehicleBicycleConfigPath();
VehicleBicycleConfig LoadVehicleBicycleConfig(const std::filesystem::path& path);
VehicleCollisionCircle BuildSingleCircleVehicleApproximation(const VehicleBodyConfig& config);

}  // namespace my_al_ilqr
