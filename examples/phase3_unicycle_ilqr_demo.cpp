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
  Q.diagonal() << 1.0, 1.0, 0.1;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.2, 0.05;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 25.0, 25.0, 2.0;

  Vector x_ref(3);
  x_ref << 2.0, 1.0, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 40, 0.1);

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> initial_controls(problem.Horizon(), Vector::Zero(2));

  ILQROptions options;
  options.max_iterations = 35;
  options.cost_tolerance = 1e-5;
  options.derivative_epsilon = 1e-4;
  options.line_search_max_iterations = 8;
  options.line_search_decrease_factor = 0.5;

  ILQRSolver solver(problem, options);
  const auto trajectory = solver.Solve(x0, initial_controls);
  const auto& cost_history = solver.CostHistory();

  const std::filesystem::path csv_path = "build/phase3_unicycle_ilqr_trajectory.csv";
  my_al_ilqr::WriteTrajectoryCsv(csv_path, trajectory);

  std::cout << "Phase 3 unicycle iLQR demo\n";
  std::cout << "iterations = " << static_cast<int>(cost_history.size()) - 1 << "\n";
  std::cout << "initial cost = " << cost_history.front() << "\n";
  std::cout << "final cost = " << cost_history.back() << "\n";
  std::cout << "terminal state = " << trajectory.State(problem.Horizon()).transpose() << "\n";
  std::cout << "trajectory csv = " << csv_path << "\n";
  std::cout << "cost history:\n";
  for (size_t i = 0; i < cost_history.size(); ++i) {
    std::cout << "  iter " << i << ": " << cost_history[i] << "\n";
  }

  return 0;
}
