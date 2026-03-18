#include <cassert>
#include <filesystem>

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

  const std::filesystem::path initial_csv = "build/phase9_test_initial.csv";
  const std::filesystem::path optimized_csv = "build/phase9_test_optimized.csv";
  const std::filesystem::path outer_log_csv = "build/phase9_test_outer_log.csv";

  my_al_ilqr::WriteTrajectoryCsv(initial_csv, initial_trajectory);
  my_al_ilqr::WriteTrajectoryCsv(optimized_csv, result.trajectory);
  my_al_ilqr::WriteALILQROuterLogCsv(outer_log_csv, result.outer_logs);

  assert(std::filesystem::exists(initial_csv));
  assert(std::filesystem::exists(optimized_csv));
  assert(std::filesystem::exists(outer_log_csv));
  assert(std::filesystem::file_size(initial_csv) > 0);
  assert(std::filesystem::file_size(optimized_csv) > 0);
  assert(std::filesystem::file_size(outer_log_csv) > 0);
  assert(!result.outer_logs.empty());

  return 0;
}
