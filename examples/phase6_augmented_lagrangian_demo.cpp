#include <iostream>
#include <memory>
#include <vector>

#include "al/augmented_lagrangian_cost.hpp"
#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "ilqr/ilqr_solver.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::AugmentedLagrangianProblem;
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::ILQROptions;
  using my_al_ilqr::ILQRSolver;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
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
  auto base_cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, base_cost, 35, 0.1);

  ConstrainedOptimalControlProblem constrained_problem(base_problem);
  Vector u_lb(2);
  u_lb << -0.8, -0.6;
  Vector u_ub(2);
  u_ub << 0.8, 0.6;
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    constrained_problem.AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  }
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  AugmentedLagrangianProblem al_problem(constrained_problem, 10.0, 5.0);

  Vector x0(3);
  x0 << 0.0, 0.0, 0.3;
  std::vector<Vector> initial_controls(base_problem->Horizon(), Vector::Zero(2));
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    initial_controls[k](0) = 0.4;
    initial_controls[k](1) = (k < 10) ? 0.15 : -0.05;
  }

  const auto initial_trajectory = base_problem->Rollout(x0, initial_controls);
  std::cout << "Phase 6 augmented Lagrangian demo\n";
  std::cout << "initial base max violation = " << constrained_problem.MaxViolation(initial_trajectory)
            << "\n";
  std::cout << "initial AL max penalty = " << al_problem.MaxPenalty() << "\n";

  ILQROptions options;
  options.max_iterations = 20;
  ILQRSolver solver(*al_problem.UnconstrainedSubproblem(), options);
  const auto optimized_trajectory = solver.Solve(x0, initial_controls);

  std::cout << "subproblem initial cost = " << solver.CostHistory().front() << "\n";
  std::cout << "subproblem final cost = " << solver.CostHistory().back() << "\n";
  std::cout << "post-solve max violation = " << al_problem.MaxViolation(optimized_trajectory) << "\n";

  al_problem.UpdateDuals(optimized_trajectory);
  al_problem.UpdatePenalties();
  std::cout << "updated max penalty = " << al_problem.MaxPenalty() << "\n";
  std::cout << "terminal state = "
            << optimized_trajectory.State(base_problem->Horizon()).transpose() << "\n";

  return 0;
}
