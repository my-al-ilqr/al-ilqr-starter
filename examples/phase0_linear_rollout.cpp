#include <iostream>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/linear_point_mass_model.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::LinearPointMassModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<LinearPointMassModel>();

  Matrix Q = Matrix::Zero(2, 2);
  Q(0, 0) = 1.0;
  Q(1, 1) = 0.1;

  Matrix R = Matrix::Constant(1, 1, 0.05);

  Matrix Qf = Matrix::Zero(2, 2);
  Qf(0, 0) = 10.0;
  Qf(1, 1) = 1.0;

  Vector x_ref(2);
  x_ref << 1.5, 0.0;
  Vector u_ref = Vector::Zero(1);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 20, 0.1);

  Vector x0 = Vector::Zero(2);
  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(1));
  for (int k = 0; k < problem.Horizon(); ++k) {
    controls[k](0) = (k < 10) ? 0.4 : -0.2;
  }

  const auto trajectory = problem.Rollout(x0, controls);
  const double total_cost = problem.TotalCost(trajectory);

  std::cout << "Phase 0 linear rollout example\n";
  std::cout << "horizon = " << problem.Horizon() << ", dt = " << problem.TimeStep() << "\n";
  std::cout << "first states:\n";
  for (int k = 0; k < 5; ++k) {
    std::cout << "  x[" << k << "] = " << trajectory.State(k).transpose() << "\n";
  }
  std::cout << "terminal state = " << trajectory.State(problem.Horizon()).transpose() << "\n";
  std::cout << "total cost = " << total_cost << "\n";

  return 0;
}
