#include <cassert>
#include <cmath>

#include "core/types.hpp"
#include "lqr/finite_horizon_lqr_solver.hpp"

namespace {

bool NearlyEqual(double a, double b, double tol = 1e-6) {
  return std::abs(a - b) <= tol;
}

}  // namespace

int main() {
  using my_al_ilqr::FiniteHorizonLQRProblem;
  using my_al_ilqr::FiniteHorizonLQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::Vector;

  const double dt = 0.1;

  Matrix A(2, 2);
  A << 1.0, dt,
       0.0, 1.0;
  Matrix B(2, 1);
  B << 0.5 * dt * dt,
       dt;

  Matrix Q = Matrix::Identity(2, 2);
  Matrix R = Matrix::Constant(1, 1, 0.2);
  Matrix Qf = 5.0 * Matrix::Identity(2, 2);

  Vector x_ref = Vector::Zero(2);
  Vector u_ref = Vector::Zero(1);

  FiniteHorizonLQRProblem problem{A, B, Q, R, Qf, x_ref, u_ref, 50};
  FiniteHorizonLQRSolver solver(problem);
  solver.Solve();

  Vector x0(2);
  x0 << 1.0, 0.0;
  const auto trajectory = solver.Simulate(x0);

  assert(static_cast<int>(solver.FeedbackGains().size()) == problem.horizon);
  assert(static_cast<int>(solver.RiccatiMatrices().size()) == problem.horizon + 1);
  assert(trajectory.Horizon() == problem.horizon);
  assert(trajectory.Control(0)(0) < 0.0);
  assert(std::abs(trajectory.State(problem.horizon)(0)) < 1e-2);
  assert(std::abs(trajectory.State(problem.horizon)(1)) < 1e-2);
  assert(!NearlyEqual(solver.FeedbackGains().front().norm(), 0.0));

  return 0;
}
