#include <iostream>

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

  std::cout << "Phase 10 numerical enhancement demo\n";
  std::cout << "converged = " << (result.converged ? "true" : "false") << "\n";
  std::cout << "final violation (last iterate) = " << result.final_violation << "\n";
  std::cout << "best violation (returned) = " << result.best_violation << "\n";
  std::cout << "outer iteration summary:\n";
  for (const auto& log : result.outer_logs) {
    std::cout << "  outer " << log.outer_iteration << ": violation=" << log.max_violation
              << ", best_so_far=" << log.best_violation_so_far
              << ", penalty=" << log.max_penalty
              << ", penalty_updated=" << (log.penalty_updated ? "true" : "false")
              << ", inner_iters=" << log.inner_iterations << "\n";
  }

  return 0;
}
