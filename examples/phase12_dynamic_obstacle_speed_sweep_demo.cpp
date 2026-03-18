#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "autodrive/trajectory_response_analysis.hpp"
#include "visualization/obstacle_prediction_writer.hpp"
#include "visualization/trajectory_writer.hpp"

namespace {

std::string SpeedTag(double speed) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << speed;
  std::string tag = oss.str();
  for (char& c : tag) {
    if (c == '.') {
      c = 'p';
    }
  }
  return tag;
}

}  // namespace

int main() {
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::AnalyzeTrajectoryResponse;
  using my_al_ilqr::CrossingTimeAtX;
  using my_al_ilqr::ToString;

  const std::vector<double> obstacle_speeds = {0.25, 1.15, 1.55};
  const auto baseline_scenario = my_al_ilqr::CreateDynamicObstacleSpeedStudyScenario(1.55);
  const auto baseline_initial_trajectory = baseline_scenario.base_problem->Rollout(
      baseline_scenario.initial_state, baseline_scenario.initial_controls);
  const double baseline_crossing_time = CrossingTimeAtX(baseline_initial_trajectory, 6.0);

  const std::filesystem::path summary_csv = "build/phase12_dynamic_speed_sweep_summary.csv";
  std::ofstream summary(summary_csv);
  summary << "obstacle_speed,response,crossing_time,crossing_delay,max_abs_lateral,min_speed,best_violation\n";

  std::cout << "Phase 12 dynamic obstacle speed sweep\n";
  std::cout << "baseline crossing time = " << baseline_crossing_time << "\n";

  for (double speed : obstacle_speeds) {
    const auto scenario = my_al_ilqr::CreateDynamicObstacleSpeedStudyScenario(speed);
    ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
    const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);
    const auto metrics = AnalyzeTrajectoryResponse(result.trajectory, 6.0, baseline_crossing_time);

    const std::string tag = SpeedTag(speed);
    const std::filesystem::path traj_csv =
        "build/phase12_speed_" + tag + "_optimized_trajectory.csv";
    const std::filesystem::path obstacle_csv =
        "build/phase12_speed_" + tag + "_obstacle_prediction.csv";
    my_al_ilqr::WriteTrajectoryCsv(traj_csv, result.trajectory);
    my_al_ilqr::WriteObstaclePredictionCsv(
        obstacle_csv, scenario.obstacle_centers, scenario.obstacle_radius);

    summary << speed << "," << ToString(metrics.response) << "," << metrics.crossing_time << ","
            << metrics.crossing_delay << "," << metrics.max_abs_lateral << ","
            << metrics.min_speed << "," << result.best_violation << "\n";

    std::cout << "  speed=" << speed << ": response=" << ToString(metrics.response)
              << ", crossing_time=" << metrics.crossing_time
              << ", delay=" << metrics.crossing_delay
              << ", max_abs_y=" << metrics.max_abs_lateral
              << ", min_speed=" << metrics.min_speed
              << ", best_violation=" << result.best_violation << "\n";
  }

  std::cout << "summary csv = " << summary_csv << "\n";
  return 0;
}
