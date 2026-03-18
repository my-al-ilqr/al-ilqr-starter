#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/linear_point_mass_model.hpp"
#include "problems/optimal_control_problem.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::LinearPointMassModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<LinearPointMassModel>();

  Matrix Q = Matrix::Identity(2, 2);
  Matrix R = Matrix::Identity(1, 1);
  Matrix Qf = 5.0 * Matrix::Identity(2, 2);
  Vector x_ref = Vector::Zero(2);
  Vector u_ref = Vector::Zero(1);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 5, 0.2);

  Vector x0 = Vector::Zero(2);
  std::vector<Vector> controls(problem.Horizon(), Vector::Ones(1));

  const auto trajectory = problem.Rollout(x0, controls);
  const double total_cost = problem.TotalCost(trajectory);

  assert(trajectory.Horizon() == 5);
  assert(trajectory.StateDim() == 2);
  assert(trajectory.ControlDim() == 1);
  assert(NearlyEqual(trajectory.State(problem.Horizon())(0), 0.5));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(1), 1.0));
  assert(total_cost > 0.0);

  return 0;
}
