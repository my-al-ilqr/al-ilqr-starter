#include <cassert>
#include <cmath>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  const auto scenario = my_al_ilqr::CreateDynamicObstacleDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  assert(!result.outer_logs.empty());
  assert(result.best_violation < initial_violation);
  assert(result.best_violation <= 0.12);

  const auto vehicle_circle =
      my_al_ilqr::BuildSingleCircleVehicleApproximation(scenario.vehicle_config.body);
  double min_clearance_margin = 1e9;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    const auto& obstacle = scenario.obstacle_centers[k];
    const auto state = result.trajectory.State(k);
    const double c = std::cos(state(2));
    const double s = std::sin(state(2));
    const double circle_x =
        state(0) + c * vehicle_circle.center_x_body - s * vehicle_circle.center_y_body;
    const double circle_y =
        state(1) + s * vehicle_circle.center_x_body + c * vehicle_circle.center_y_body;
    const double dx = circle_x - obstacle(0);
    const double dy = circle_y - obstacle(1);
    const double safe_distance = scenario.obstacle_radius + vehicle_circle.radius;
    const double margin = dx * dx + dy * dy - safe_distance * safe_distance;
    min_clearance_margin = std::min(min_clearance_margin, margin);
  }

  assert(min_clearance_margin >= -0.12);
  assert(result.trajectory.State(scenario.base_problem->Horizon())(0) >= 10.0);

  return 0;
}
