#include <cassert>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::Vector;

  const auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);
  const double initial_violation = scenario.constrained_problem->MaxViolation(initial_trajectory);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  assert(!result.outer_logs.empty());
  const double returned_violation = scenario.constrained_problem->MaxViolation(result.trajectory);
  assert(returned_violation <= initial_violation + 1e-9);
  assert(returned_violation <= 0.2);

  const Vector terminal_state = result.trajectory.State(scenario.base_problem->Horizon());
  assert(terminal_state(0) >= 10.5);
  assert(terminal_state(3) >= 2.0);

  return 0;
}
