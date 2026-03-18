#include <iostream>
#include <memory>
#include <vector>

#include "cost/quadratic_cost.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::KinematicBicycleModel;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<KinematicBicycleModel>(2.7);

  Matrix Q = Matrix::Zero(4, 4);
  Q.diagonal() << 1.0, 1.0, 0.2, 0.3;
  Matrix R = Matrix::Zero(2, 2);
  R.diagonal() << 0.1, 0.05;
  Matrix Qf = Matrix::Zero(4, 4);
  Qf.diagonal() << 10.0, 10.0, 1.0, 1.0;

  Vector x_ref(4);
  x_ref << 8.0, 3.0, 0.0, 2.0;
  Vector u_ref = Vector::Zero(2);

  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  OptimalControlProblem problem(dynamics, cost, 40, 0.1);

  Vector x0(4);
  x0 << 0.0, 0.0, 0.0, 1.5;

  std::vector<Vector> controls(problem.Horizon(), Vector::Zero(2));
  for (int k = 0; k < problem.Horizon(); ++k) {
    controls[k](0) = (k < 12) ? 0.4 : -0.1;
    controls[k](1) = (k < 18) ? 0.12 : -0.04;
  }

  const auto trajectory = problem.Rollout(x0, controls);

  std::cout << "Phase 1 bicycle rollout example\n";
  std::cout << "wheelbase = " << dynamics->Wheelbase()
            << ", horizon = " << problem.Horizon()
            << ", dt = " << problem.TimeStep() << "\n";
  std::cout << "sampled states:\n";
  for (int k : {0, 10, 20, 30, 40}) {
    std::cout << "  x[" << k << "] = " << trajectory.State(k).transpose() << "\n";
  }
  std::cout << "total cost = " << problem.TotalCost(trajectory) << "\n";

  return 0;
}
