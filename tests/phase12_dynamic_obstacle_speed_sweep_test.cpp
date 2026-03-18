#include <cassert>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/trajectory_response_analysis.hpp"

int main() {
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::AnalyzeTrajectoryResponse;
  using my_al_ilqr::CrossingTimeAtX;
  using my_al_ilqr::TrajectoryResponseType;

  const auto baseline_scenario = my_al_ilqr::CreateDynamicObstacleSpeedStudyScenario(1.55);
  const auto baseline_trajectory =
      baseline_scenario.base_problem->Rollout(baseline_scenario.initial_state,
                                              baseline_scenario.initial_controls);
  const double baseline_crossing_time = CrossingTimeAtX(baseline_trajectory, 6.0);

  struct CaseExpectation {
    double speed;
    TrajectoryResponseType expected;
  };
  const std::vector<CaseExpectation> cases = {
      {0.25, TrajectoryResponseType::kBypass},
      {1.15, TrajectoryResponseType::kYield},
      {1.55, TrajectoryResponseType::kGoStraight},
  };

  for (const auto& test_case : cases) {
    const auto scenario = my_al_ilqr::CreateDynamicObstacleSpeedStudyScenario(test_case.speed);
    ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
    const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);
    const auto metrics = AnalyzeTrajectoryResponse(result.trajectory, 6.0, baseline_crossing_time);

    assert(result.best_violation <= 0.6);
    assert(metrics.response == test_case.expected);
  }

  return 0;
}
