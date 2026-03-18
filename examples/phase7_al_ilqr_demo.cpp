#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  using my_al_ilqr::ALILQROptions;
  using my_al_ilqr::ALILQRSolver;
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<UnicycleModel>();

  Matrix Q = Matrix::Zero(3, 3);
  Q.diagonal() << 0.8, 0.8, 0.1;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.05, 0.03;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 20.0, 20.0, 2.0;

  Vector x_ref(3);
  x_ref << 1.5, 0.6, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, cost, 45, 0.1);

  ConstrainedOptimalControlProblem constrained_problem(base_problem);
  Vector u_lb(2);
  u_lb << -0.5, -0.35;
  Vector u_ub(2);
  u_ub << 0.5, 0.35;
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    constrained_problem.AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  }
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  Vector x0(3);
  x0 << 0.0, 0.0, 0.15;

  std::vector<Vector> initial_controls(base_problem->Horizon(), Vector::Zero(2));
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    initial_controls[k](0) = 0.65;
    initial_controls[k](1) = (k < 15) ? 0.38 : -0.12;
  }

  ILQROptions inner_options;
  inner_options.max_iterations = 30;
  inner_options.cost_tolerance = 1e-5;
  inner_options.derivative_epsilon = 1e-4;
  inner_options.regularization_init = 1e-5;
  inner_options.regularization_min = 1e-8;
  inner_options.regularization_max = 1e5;
  inner_options.regularization_increase_factor = 10.0;
  inner_options.regularization_decrease_factor = 5.0;
  inner_options.line_search_max_iterations = 10;
  inner_options.line_search_decrease_factor = 0.5;
  inner_options.line_search_accept_lower = 1e-4;
  inner_options.line_search_accept_upper = 10.0;

  ALILQROptions options;
  options.inner_options = inner_options;
  options.max_outer_iterations = 8;
  options.constraint_tolerance = 2e-2;
  options.initial_penalty = 10.0;
  options.penalty_scaling = 3.0;
  options.penalty_update_ratio = 0.8;
  options.max_penalty = 1e6;

  ALILQRSolver solver(constrained_problem, options);
  const auto result = solver.Solve(x0, initial_controls);

  const std::filesystem::path csv_path = "build/phase7_al_ilqr_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, result.trajectory);

  std::cout << "Phase 7 AL-iLQR demo\n";
  std::cout << "converged = " << (result.converged ? "true" : "false") << "\n";
  std::cout << "outer iterations = " << result.outer_logs.size() << "\n";
  if (!result.outer_logs.empty()) {
    std::cout << "initial logged violation = " << result.outer_logs.front().max_violation << "\n";
    std::cout << "final logged violation = " << result.outer_logs.back().max_violation << "\n";
  }
  std::cout << "terminal state = " << result.trajectory.State(base_problem->Horizon()).transpose()
            << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";
  std::cout << "outer iteration summary:\n";
  for (const auto& log : result.outer_logs) {
    std::cout << "  outer " << log.outer_iteration << ": base_cost=" << log.base_cost
              << ", aug_cost=" << log.augmented_cost
              << ", violation=" << log.max_violation
              << ", penalty=" << log.max_penalty
              << ", inner_iters=" << log.inner_iterations << "\n";
  }

  return 0;
}
