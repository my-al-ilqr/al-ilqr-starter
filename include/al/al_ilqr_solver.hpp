#pragma once

#include <vector>

#include "al/augmented_lagrangian_cost.hpp"
#include "ilqr/ilqr_solver.hpp"

namespace my_al_ilqr {

struct ALILQROptions {
  ILQROptions inner_options;
  int max_outer_iterations = 10;
  double constraint_tolerance = 1e-3;
  double initial_penalty = 10.0;
  double penalty_scaling = 5.0;
  double penalty_update_ratio = 0.25;
  double max_penalty = 1e8;
  bool return_best_trajectory = true;
};

struct ALILQROuterIterationLog {
  int outer_iteration = 0;
  int inner_iterations = 0;
  double base_cost = 0.0;
  double augmented_cost = 0.0;
  double max_violation = 0.0;
  double best_violation_so_far = 0.0;
  double max_penalty = 0.0;
  bool penalty_updated = false;
};

struct ALILQRResult {
  Trajectory trajectory;
  bool converged = false;
  double final_violation = 0.0;
  double best_violation = 0.0;
  std::vector<ALILQROuterIterationLog> outer_logs;
};

class ALILQRSolver {
 public:
  explicit ALILQRSolver(const ConstrainedOptimalControlProblem& problem, ALILQROptions options = {});

  const std::vector<ALILQROuterIterationLog>& OuterLog() const { return outer_log_; }

  ALILQRResult Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls);

 private:
  const ConstrainedOptimalControlProblem& problem_;
  ALILQROptions options_;
  std::vector<ALILQROuterIterationLog> outer_log_;
};

}  // namespace my_al_ilqr
