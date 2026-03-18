#include <filesystem>
#include <iostream>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "visualization/al_ilqr_log_writer.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;

  const auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  const std::filesystem::path initial_csv = "build/phase9_autodrive_initial_trajectory.csv";
  const std::filesystem::path optimized_csv = "build/phase9_autodrive_optimized_trajectory.csv";
  const std::filesystem::path outer_log_csv = "build/phase9_autodrive_outer_log.csv";

  my_al_ilqr::WriteTrajectoryCsv(initial_csv, initial_trajectory);
  my_al_ilqr::WriteTrajectoryCsv(optimized_csv, result.trajectory);
  my_al_ilqr::WriteALILQROuterLogCsv(outer_log_csv, result.outer_logs);

  std::cout << "Phase 9 autodrive visualization demo\n";
  std::cout << "initial trajectory csv = " << initial_csv << "\n";
  std::cout << "optimized trajectory csv = " << optimized_csv << "\n";
  std::cout << "outer log csv = " << outer_log_csv << "\n";
  std::cout << "converged = " << (result.converged ? "true" : "false") << "\n";
  if (!result.outer_logs.empty()) {
    std::cout << "initial logged violation = " << result.outer_logs.front().max_violation << "\n";
    std::cout << "final logged violation = " << result.outer_logs.back().max_violation << "\n";
  }

  return 0;
}
