#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"
#include "problems/optimal_control_problem.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-9) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::KinematicBicycleModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<KinematicBicycleModel>(2.5);
  Matrix Q = Matrix::Identity(4, 4);
  Matrix R = Matrix::Identity(2, 2);
  Matrix Qf = 2.0 * Matrix::Identity(4, 4);
  Vector x_ref = Vector::Zero(4);
  Vector u_ref = Vector::Zero(2);
  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 5, 0.2);

  Vector x0(4);
  x0 << 0.0, 0.0, 0.0, 2.0;

  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(2));
  for (Vector& u : controls) {
    u(0) = 0.0;
    u(1) = 0.0;
  }

  const auto trajectory = problem.Rollout(x0, controls);
  assert(NearlyEqual(trajectory.State(problem.Horizon())(0), 2.0));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(1), 0.0));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(2), 0.0));
  assert(NearlyEqual(trajectory.State(problem.Horizon())(3), 2.0));
  assert(problem.TotalCost(trajectory) > 0.0);

  return 0;
}
