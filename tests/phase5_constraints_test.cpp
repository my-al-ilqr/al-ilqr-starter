#include <cassert>
#include <memory>
#include <vector>

#include "constraints/control_box_constraint.hpp"
#include "constraints/terminal_goal_constraint.hpp"
#include "cost/quadratic_cost.hpp"
#include "dynamics/unicycle_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

int main() {
  using my_al_ilqr::ConstrainedOptimalControlProblem;
  using my_al_ilqr::ControlBoxConstraint;
  using my_al_ilqr::Matrix;
  using my_al_ilqr::OptimalControlProblem;
  using my_al_ilqr::QuadraticCost;
  using my_al_ilqr::TerminalGoalConstraint;
  using my_al_ilqr::UnicycleModel;
  using my_al_ilqr::Vector;

  auto dynamics = std::make_shared<UnicycleModel>();
  Matrix Q = Matrix::Identity(3, 3);
  Matrix R = Matrix::Identity(2, 2);
  Matrix Qf = 2.0 * Matrix::Identity(3, 3);
  Vector x_ref = Vector::Zero(3);
  Vector u_ref = Vector::Zero(2);
  auto cost = std::make_shared<QuadraticCost>(Q, R, Qf, x_ref, u_ref);
  auto base_problem = std::make_shared<OptimalControlProblem>(dynamics, cost, 4, 0.1);

  ConstrainedOptimalControlProblem constrained_problem(base_problem);
  Vector u_lb(2);
  u_lb << -0.5, -0.2;
  Vector u_ub(2);
  u_ub << 0.5, 0.2;
  constrained_problem.AddStageConstraint(
      0, std::make_shared<ControlBoxConstraint>(base_problem->StateDim(), u_lb, u_ub));
  constrained_problem.AddTerminalConstraint(std::make_shared<TerminalGoalConstraint>(x_ref));

  Vector x0 = Vector::Zero(3);
  std::vector<Vector> controls(base_problem->Horizon(), Vector::Zero(2));
  controls[0](0) = 0.8;
  controls[0](1) = 0.0;

  const auto trajectory = base_problem->Rollout(x0, controls);
  const auto evaluations = constrained_problem.EvaluateTrajectory(trajectory);

  assert(evaluations.size() == 2);
  assert(evaluations[0].name == "control_box");
  assert(evaluations[0].max_violation > 0.0);
  assert(evaluations[1].name == "terminal_goal");
  assert(evaluations[1].knot_point == base_problem->Horizon());
  assert(constrained_problem.MaxViolation(trajectory) >= evaluations[0].max_violation);

  return 0;
}
