#include <cassert>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  scenario.solver_options.max_outer_iterations = 10;
  scenario.solver_options.constraint_tolerance = 1e-2;
  scenario.solver_options.initial_penalty = 2.0;
  scenario.solver_options.penalty_scaling = 4.0;
  scenario.solver_options.penalty_update_ratio = 0.92;
  scenario.solver_options.max_penalty = 1e8;
  scenario.solver_options.return_best_trajectory = true;

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  assert(!result.outer_logs.empty());
  assert(result.best_violation <= result.final_violation + 1e-12);

  bool has_penalty_update = false;
  double previous_best = result.outer_logs.front().best_violation_so_far;
  for (const auto& log : result.outer_logs) {
    assert(log.best_violation_so_far <= previous_best + 1e-12);
    previous_best = log.best_violation_so_far;
    has_penalty_update = has_penalty_update || log.penalty_updated;
  }
  assert(has_penalty_update);

  const double returned_violation = scenario.constrained_problem->MaxViolation(result.trajectory);
  assert(std::abs(returned_violation - result.best_violation) <= 1e-10);

  return 0;
}
