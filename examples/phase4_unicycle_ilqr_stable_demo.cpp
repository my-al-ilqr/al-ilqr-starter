#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "ilqr/ilqr_solver.hpp"
#include "problems/optimal_control_problem.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::ILQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<UnicycleModel>();

  Matrix Q = Matrix::Zero(3, 3);
  Q.diagonal() << 1.0, 1.0, 0.2;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.15, 0.08;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 30.0, 30.0, 3.0;

  Vector x_ref(3);
  x_ref << 2.5, 1.2, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 45, 0.1);

  Vector x0(3);
  x0 << 0.0, 0.0, 0.2;

  std::vector<Vector> initial_controls(problem.Horizon(), Vector::Zero(2));
  for (int k = 0; k < problem.Horizon(); ++k) {
    initial_controls[k](0) = 0.6;
    initial_controls[k](1) = (k < 15) ? 0.2 : -0.15;
  }

  ILQROptions options;
  options.max_iterations = 40;
  options.cost_tolerance = 1e-5;
  options.derivative_epsilon = 1e-4;
  options.regularization_init = 1e-6;
  options.regularization_min = 1e-8;
  options.regularization_max = 1e4;
  options.regularization_increase_factor = 10.0;
  options.regularization_decrease_factor = 5.0;
  options.line_search_max_iterations = 10;
  options.line_search_decrease_factor = 0.5;
  options.line_search_accept_lower = 1e-4;
  options.line_search_accept_upper = 10.0;

  ILQRSolver solver(problem, options);
  const auto trajectory = solver.Solve(x0, initial_controls);

  const std::filesystem::path csv_path = "build/phase4_unicycle_ilqr_stable_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, trajectory);

  std::cout << "Phase 4 stable iLQR demo\n";
  std::cout << "iterations = " << solver.AlphaHistory().size() << "\n";
  std::cout << "initial cost = " << solver.CostHistory().front() << "\n";
  std::cout << "final cost = " << solver.CostHistory().back() << "\n";
  std::cout << "terminal state = " << trajectory.State(problem.Horizon()).transpose() << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";
  std::cout << "iteration summary:\n";
  for (size_t i = 0; i < solver.AlphaHistory().size(); ++i) {
    std::cout << "  iter " << i + 1 << ": cost=" << solver.CostHistory()[i + 1]
              << ", alpha=" << solver.AlphaHistory()[i]
              << ", rho=" << solver.RegularizationHistory()[i]
              << ", z=" << solver.ImprovementRatioHistory()[i] << "\n";
  }

  return 0;
}
