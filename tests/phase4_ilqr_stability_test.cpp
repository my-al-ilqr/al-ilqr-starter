#include <cassert>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "ilqr/ilqr_solver.hpp"
#include "problems/optimal_control_problem.hpp"

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
  R.diagonal() << 0.2, 0.08;
  Matrix Qf = Matrix::Zero(3, 3);
  Qf.diagonal() << 25.0, 25.0, 2.0;

  Vector x_ref(3);
  x_ref << 2.0, 1.0, 0.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 35, 0.1);

  Vector x0(3);
  x0 << 0.0, 0.0, 0.3;
  std::vector<Vector> initial_controls(problem.Horizon(), Vector::Zero(2));
  for (int k = 0; k < problem.Horizon(); ++k) {
    initial_controls[k](0) = 0.4;
    initial_controls[k](1) = (k < 10) ? 0.15 : -0.05;
  }

  ILQROptions options;
  options.max_iterations = 30;
  options.cost_tolerance = 1e-5;
  options.regularization_max = 1e4;
  options.line_search_max_iterations = 10;

  ILQRSolver solver(problem, options);
  const auto trajectory = solver.Solve(x0, initial_controls);

  assert(solver.CostHistory().size() >= 2);
  assert(solver.AlphaHistory().size() + 1 == solver.CostHistory().size());
  assert(solver.RegularizationHistory().size() == solver.AlphaHistory().size());
  assert(solver.ImprovementRatioHistory().size() == solver.AlphaHistory().size());
  assert(solver.CostHistory().back() < solver.CostHistory().front());
  assert((trajectory.State(problem.Horizon()).head(2) - x_ref.head(2)).norm() < 0.2);

  return 0;
}
