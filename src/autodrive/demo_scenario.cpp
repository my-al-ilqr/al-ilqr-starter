#include "autodrive/demo_scenario.hpp"

#include <memory>

#include "autodrive/circular_obstacle_constraint.hpp"
#include "autodrive/lane_tracking_cost.hpp"
#include "autodrive/speed_limit_constraint.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "constraints/control_box_constraint.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"

namespace my_al_ilqr {

namespace {

Vector ObstacleCenter(double x, double y) {
  Vector center(2);
  center << x, y;
  return center;
}

AutodriveDemoScenario CreateScenarioSkeleton(const DynamicObstacleScenarioConfig& config,
                                            const VehicleBicycleConfig& vehicle_config) {
  AutodriveDemoScenario scenario;
  scenario.vehicle_config = vehicle_config;
  scenario.vehicle_collision_circle = BuildSingleCircleVehicleApproximation(vehicle_config.body);
  scenario.reference_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);
  scenario.road_lower_bound = -config.road_half_width;
  scenario.road_upper_bound = config.road_half_width;
  scenario.obstacle_radius = config.obstacle_radius;
  scenario.vehicle_length = vehicle_config.body.length;
  scenario.vehicle_width = vehicle_config.body.width;
  scenario.rear_axle_to_rear = vehicle_config.body.rear_axle_to_rear;

  auto dynamics = std::make_shared<KinematicBicycleModel>(vehicle_config.model.wheelbase);
  auto cost = std::make_shared<LaneTrackingCost>(scenario.reference_line,
                                                 3.0,
                                                 12.0,
                                                 config.stage_lateral_weight,
                                                 config.stage_heading_weight,
                                                 config.stage_speed_weight,
                                                 config.stage_accel_weight,
                                                 config.stage_steering_weight,
                                                 config.terminal_longitudinal_weight,
                                                 config.terminal_lateral_weight,
                                                 config.terminal_heading_weight,
                                                 config.terminal_speed_weight);
  scenario.base_problem = std::make_shared<OptimalControlProblem>(dynamics, cost, 55, 0.1);
  scenario.constrained_problem =
      std::make_shared<ConstrainedOptimalControlProblem>(scenario.base_problem);

  Vector u_lb(2);
  u_lb << vehicle_config.model.min_acceleration, vehicle_config.model.min_steering;
  Vector u_ub(2);
  u_ub << vehicle_config.model.max_acceleration, vehicle_config.model.max_steering;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    scenario.constrained_problem->AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(scenario.base_problem->StateDim(), u_lb, u_ub));
    scenario.constrained_problem->AddStageConstraint(
        k, std::make_shared<SpeedLimitConstraint>(vehicle_config.model.min_speed,
                                                  vehicle_config.model.max_speed));
  }

  scenario.initial_state = Vector(4);
  scenario.initial_state << 0.0, 0.0, 0.0, 2.5;

  ILQROptions inner_options;
  inner_options.max_iterations = 35;
  inner_options.cost_tolerance = 1e-5;
  inner_options.derivative_epsilon = 1e-4;
  inner_options.regularization_init = 1e-5;
  inner_options.regularization_min = 1e-8;
  inner_options.regularization_max = 1e5;
  inner_options.regularization_increase_factor = 10.0;
  inner_options.regularization_decrease_factor = 5.0;
  inner_options.line_search_max_iterations = 10;

  scenario.solver_options.inner_options = inner_options;
  scenario.solver_options.max_outer_iterations = 18;
  scenario.solver_options.constraint_tolerance = 1e-1;
  scenario.solver_options.initial_penalty = 20.0;
  scenario.solver_options.penalty_scaling = 5.0;
  scenario.solver_options.penalty_update_ratio = 0.4;
  scenario.solver_options.max_penalty = 1e8;

  return scenario;
}

void AddObstacleConstraints(AutodriveDemoScenario* scenario) {
  if (!scenario) {
    return;
  }
  if (scenario->obstacle_centers.empty()) {
    return;
  }
  if (static_cast<int>(scenario->obstacle_centers.size()) < scenario->base_problem->Horizon()) {
    throw std::invalid_argument("Obstacle prediction must provide at least one center per stage.");
  }
  for (int k = 0; k < scenario->base_problem->Horizon(); ++k) {
    scenario->constrained_problem->AddStageConstraint(
        k, std::make_shared<CircularObstacleConstraint>(
               scenario->obstacle_centers[k](0),
               scenario->obstacle_centers[k](1),
               scenario->obstacle_radius,
               scenario->vehicle_collision_circle));
  }
}

}  // namespace

AutodriveDemoScenario CreateAutodriveDemoScenario() {
  return CreateSingleStaticObstacleTestScenario();
}

AutodriveDemoScenario CreateSingleStaticObstacleTestScenario() {
  const VehicleBicycleConfig vehicle_config =
      LoadVehicleBicycleConfig(DefaultVehicleBicycleConfigPath());
  StaticObstacleScenarioConfig config;
  return CreateSingleStaticObstacleTestScenario(vehicle_config, config);
}

AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& static_config) {
  DynamicObstacleScenarioConfig config;
  config.obstacle_center_x = static_config.obstacle_center_x;
  config.obstacle_initial_y = static_config.obstacle_center_y;
  config.obstacle_speed_y = 0.0;
  config.road_half_width = static_config.road_half_width;
  config.obstacle_radius = static_config.obstacle_radius;
  config.stage_lateral_weight = static_config.stage_lateral_weight;
  config.stage_heading_weight = static_config.stage_heading_weight;
  config.stage_speed_weight = static_config.stage_speed_weight;
  config.stage_accel_weight = static_config.stage_accel_weight;
  config.stage_steering_weight = static_config.stage_steering_weight;
  config.terminal_longitudinal_weight = static_config.terminal_longitudinal_weight;
  config.terminal_lateral_weight = static_config.terminal_lateral_weight;
  config.terminal_heading_weight = static_config.terminal_heading_weight;
  config.terminal_speed_weight = static_config.terminal_speed_weight;
  AutodriveDemoScenario scenario = CreateScenarioSkeleton(config, vehicle_config);
  scenario.obstacle_center_x = static_config.obstacle_center_x;
  scenario.obstacle_center_y = static_config.obstacle_center_y;
  scenario.obstacle_centers.assign(
      scenario.base_problem->Horizon() + 1, ObstacleCenter(scenario.obstacle_center_x, scenario.obstacle_center_y));
  AddObstacleConstraints(&scenario);

  scenario.initial_state << static_config.initial_x,
      static_config.initial_y,
      static_config.initial_yaw,
      static_config.initial_speed;

  scenario.initial_controls.assign(scenario.base_problem->Horizon(), Vector::Zero(2));
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    scenario.initial_controls[k](0) = (k < 8) ? 0.2 : 0.0;
    if (k < 15) {
      scenario.initial_controls[k](1) = 0.28;
    } else if (k < 30) {
      scenario.initial_controls[k](1) = -0.24;
    } else {
      scenario.initial_controls[k](1) = 0.08;
    }
  }

  return scenario;
}

AutodriveDemoScenario CreateDynamicObstacleDemoScenario() {
  DynamicObstacleScenarioConfig config;
  config.obstacle_radius = 0.75;
  config.road_half_width = 2.3;
  config.stage_lateral_weight = 1.5;
  config.terminal_lateral_weight = 16.0;
  config.initial_steering_guess = 0.04;
  return CreateDynamicObstacleDemoScenario(config);
}

AutodriveDemoScenario CreateDynamicObstacleDemoScenario(const DynamicObstacleScenarioConfig& config) {
  const VehicleBicycleConfig vehicle_config =
      LoadVehicleBicycleConfig(DefaultVehicleBicycleConfigPath());
  AutodriveDemoScenario scenario = CreateScenarioSkeleton(config, vehicle_config);
  scenario.obstacle_center_x = config.obstacle_center_x;
  scenario.obstacle_center_y = config.obstacle_initial_y;
  scenario.obstacle_centers.reserve(scenario.base_problem->Horizon() + 1);
  for (int k = 0; k <= scenario.base_problem->Horizon(); ++k) {
    const double time = k * scenario.base_problem->TimeStep();
    const double center_x = config.obstacle_center_x;
    const double center_y = config.obstacle_initial_y + config.obstacle_speed_y * time;
    scenario.obstacle_centers.push_back(ObstacleCenter(center_x, center_y));
  }
  AddObstacleConstraints(&scenario);

  scenario.initial_controls.assign(scenario.base_problem->Horizon(), Vector::Zero(2));
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    scenario.initial_controls[k](0) = (k < config.accel_guess_steps) ? config.initial_accel_guess : 0.0;
    scenario.initial_controls[k](1) = config.initial_steering_guess;
  }

  scenario.solver_options.max_outer_iterations = 14;
  scenario.solver_options.constraint_tolerance = 1e-1;
  scenario.solver_options.initial_penalty = 15.0;
  scenario.solver_options.penalty_scaling = 5.0;
  scenario.solver_options.penalty_update_ratio = 0.45;
  scenario.solver_options.max_outer_iterations = 18;

  return scenario;
}

AutodriveDemoScenario CreateDynamicObstacleSpeedStudyScenario(double obstacle_speed_y) {
  DynamicObstacleScenarioConfig config;
  config.obstacle_initial_y = -0.35;
  config.obstacle_speed_y = obstacle_speed_y;
  config.obstacle_radius = 0.7;
  config.road_half_width = 2.0;
  config.stage_lateral_weight = 9.0;
  config.stage_heading_weight = 1.0;
  config.stage_speed_weight = 1.2;
  config.terminal_lateral_weight = 28.0;
  config.terminal_speed_weight = 10.0;
  config.initial_accel_guess = 0.0;
  config.accel_guess_steps = 0;
  config.initial_steering_guess = 0.0;
  return CreateDynamicObstacleDemoScenario(config);
}

}  // namespace my_al_ilqr
