#include <filesystem>
#include <iostream>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;
  const auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  const double initial_violation = scenario.constrained_problem->MaxViolation(
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls));

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  const std::filesystem::path csv_path = "build/phase8_autodrive_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, result.trajectory);

  std::cout << "Phase 8 autodrive demo\n";
  std::cout << "converged = " << (result.converged ? "true" : "false") << "\n";
  std::cout << "initial violation = " << initial_violation << "\n";
  if (!result.outer_logs.empty()) {
    std::cout << "final logged violation = " << result.outer_logs.back().max_violation << "\n";
  }
  std::cout << "returned violation = " << result.best_violation << "\n";
  std::cout << "terminal state = "
            << result.trajectory.State(scenario.base_problem->Horizon()).transpose()
            << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";
  std::cout << "outer iteration summary:\n";
  for (const auto& log : result.outer_logs) {
    std::cout << "  outer " << log.outer_iteration << ": base_cost=" << log.base_cost
              << ", violation=" << log.max_violation
              << ", penalty=" << log.max_penalty
              << ", inner_iters=" << log.inner_iterations << "\n";
  }

  return 0;
}
