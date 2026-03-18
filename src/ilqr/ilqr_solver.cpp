#include "ilqr/ilqr_solver.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

double StageCostAt(const OptimalControlProblem& problem, int k, const Vector& x, const Vector& u) {
  return problem.StageCostFunction(k).StageCost(x, u);
}

double TerminalCostAt(const OptimalControlProblem& problem, const Vector& x) {
  return problem.TerminalCostFunction().TerminalCost(x);
}

Vector CentralDifferenceGradientState(const OptimalControlProblem& problem,
                                      int k,
                                      const Vector& x,
                                      const Vector& u,
                                      double eps) {
  Vector grad = Vector::Zero(x.size());
  for (int i = 0; i < x.size(); ++i) {
    Vector x_plus = x;
    Vector x_minus = x;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    grad(i) = (StageCostAt(problem, k, x_plus, u) - StageCostAt(problem, k, x_minus, u)) /
              (2.0 * eps);
  }
  return grad;
}

Vector CentralDifferenceGradientControl(const OptimalControlProblem& problem,
                                        int k,
                                        const Vector& x,
                                        const Vector& u,
                                        double eps) {
  Vector grad = Vector::Zero(u.size());
  for (int i = 0; i < u.size(); ++i) {
    Vector u_plus = u;
    Vector u_minus = u;
    u_plus(i) += eps;
    u_minus(i) -= eps;
    grad(i) = (StageCostAt(problem, k, x, u_plus) - StageCostAt(problem, k, x, u_minus)) /
              (2.0 * eps);
  }
  return grad;
}

Matrix CentralDifferenceHessianState(const OptimalControlProblem& problem,
                                     int k,
                                     const Vector& x,
                                     const Vector& u,
                                     double eps) {
  Matrix hess = Matrix::Zero(x.size(), x.size());
  for (int i = 0; i < x.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_pp = x;
      Vector x_pm = x;
      Vector x_mp = x;
      Vector x_mm = x;
      x_pp(i) += eps;
      x_pp(j) += eps;
      x_pm(i) += eps;
      x_pm(j) -= eps;
      x_mp(i) -= eps;
      x_mp(j) += eps;
      x_mm(i) -= eps;
      x_mm(j) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x_pp, u) - StageCostAt(problem, k, x_pm, u) -
           StageCostAt(problem, k, x_mp, u) + StageCostAt(problem, k, x_mm, u)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

Matrix CentralDifferenceHessianControl(const OptimalControlProblem& problem,
                                       int k,
                                       const Vector& x,
                                       const Vector& u,
                                       double eps) {
  Matrix hess = Matrix::Zero(u.size(), u.size());
  for (int i = 0; i < u.size(); ++i) {
    for (int j = 0; j < u.size(); ++j) {
      Vector u_pp = u;
      Vector u_pm = u;
      Vector u_mp = u;
      Vector u_mm = u;
      u_pp(i) += eps;
      u_pp(j) += eps;
      u_pm(i) += eps;
      u_pm(j) -= eps;
      u_mp(i) -= eps;
      u_mp(j) += eps;
      u_mm(i) -= eps;
      u_mm(j) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x, u_pp) - StageCostAt(problem, k, x, u_pm) -
           StageCostAt(problem, k, x, u_mp) + StageCostAt(problem, k, x, u_mm)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

Matrix CentralDifferenceHessianCross(const OptimalControlProblem& problem,
                                     int k,
                                     const Vector& x,
                                     const Vector& u,
                                     double eps) {
  Matrix hess = Matrix::Zero(u.size(), x.size());
  for (int i = 0; i < u.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_plus = x;
      Vector x_minus = x;
      Vector u_plus = u;
      Vector u_minus = u;
      x_plus(j) += eps;
      x_minus(j) -= eps;
      u_plus(i) += eps;
      u_minus(i) -= eps;
      hess(i, j) =
          (StageCostAt(problem, k, x_plus, u_plus) - StageCostAt(problem, k, x_plus, u_minus) -
           StageCostAt(problem, k, x_minus, u_plus) + StageCostAt(problem, k, x_minus, u_minus)) /
          (4.0 * eps * eps);
    }
  }
  return hess;
}

Vector CentralDifferenceTerminalGradient(const OptimalControlProblem& problem,
                                         const Vector& x,
                                         double eps) {
  Vector grad = Vector::Zero(x.size());
  for (int i = 0; i < x.size(); ++i) {
    Vector x_plus = x;
    Vector x_minus = x;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    grad(i) = (TerminalCostAt(problem, x_plus) - TerminalCostAt(problem, x_minus)) / (2.0 * eps);
  }
  return grad;
}

Matrix CentralDifferenceTerminalHessian(const OptimalControlProblem& problem,
                                        const Vector& x,
                                        double eps) {
  Matrix hess = Matrix::Zero(x.size(), x.size());
  for (int i = 0; i < x.size(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      Vector x_pp = x;
      Vector x_pm = x;
      Vector x_mp = x;
      Vector x_mm = x;
      x_pp(i) += eps;
      x_pp(j) += eps;
      x_pm(i) += eps;
      x_pm(j) -= eps;
      x_mp(i) -= eps;
      x_mp(j) += eps;
      x_mm(i) -= eps;
      x_mm(j) -= eps;
      hess(i, j) = (TerminalCostAt(problem, x_pp) - TerminalCostAt(problem, x_pm) -
                    TerminalCostAt(problem, x_mp) + TerminalCostAt(problem, x_mm)) /
                   (4.0 * eps * eps);
    }
  }
  return hess;
}

}  // namespace

ILQRSolver::ILQRSolver(const OptimalControlProblem& problem, ILQROptions options)
    : problem_(problem), options_(std::move(options)) {
  EnsureWorkspaceSizes();
}

void ILQRSolver::EnsureWorkspaceSizes() {
  const int N = problem_.Horizon();
  const int nx = problem_.StateDim();
  const int nu = problem_.ControlDim();
  A_.assign(N, Matrix::Zero(nx, nx));
  B_.assign(N, Matrix::Zero(nx, nu));
  stage_expansions_.assign(N, CostExpansion{
                                  Vector::Zero(nx),
                                  Vector::Zero(nu),
                                  Matrix::Zero(nx, nx),
                                  Matrix::Zero(nu, nx),
                                  Matrix::Zero(nu, nu),
                              });
  feedback_gains_.assign(N, Matrix::Zero(nu, nx));
  feedforward_terms_.assign(N, Vector::Zero(nu));
}

Matrix ILQRSolver::DynamicsJacobianState(const Vector& state, const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  Matrix jac = Matrix::Zero(problem_.StateDim(), problem_.StateDim());
  for (int i = 0; i < problem_.StateDim(); ++i) {
    Vector x_plus = state;
    Vector x_minus = state;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    const Vector f_plus = problem_.Dynamics().NextState(x_plus, control, problem_.TimeStep());
    const Vector f_minus = problem_.Dynamics().NextState(x_minus, control, problem_.TimeStep());
    jac.col(i) = (f_plus - f_minus) / (2.0 * eps);
  }
  return jac;
}

Matrix ILQRSolver::DynamicsJacobianControl(const Vector& state, const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  Matrix jac = Matrix::Zero(problem_.StateDim(), problem_.ControlDim());
  for (int i = 0; i < problem_.ControlDim(); ++i) {
    Vector u_plus = control;
    Vector u_minus = control;
    u_plus(i) += eps;
    u_minus(i) -= eps;
    const Vector f_plus = problem_.Dynamics().NextState(state, u_plus, problem_.TimeStep());
    const Vector f_minus = problem_.Dynamics().NextState(state, u_minus, problem_.TimeStep());
    jac.col(i) = (f_plus - f_minus) / (2.0 * eps);
  }
  return jac;
}

ILQRSolver::CostExpansion ILQRSolver::StageCostExpansion(int k,
                                                         const Vector& state,
                                                         const Vector& control) const {
  const double eps = options_.derivative_epsilon;
  return CostExpansion{
      CentralDifferenceGradientState(problem_, k, state, control, eps),
      CentralDifferenceGradientControl(problem_, k, state, control, eps),
      CentralDifferenceHessianState(problem_, k, state, control, eps),
      CentralDifferenceHessianCross(problem_, k, state, control, eps),
      CentralDifferenceHessianControl(problem_, k, state, control, eps),
  };
}

std::pair<Vector, Matrix> ILQRSolver::TerminalCostExpansion(const Vector& state) const {
  const double eps = options_.derivative_epsilon;
  return {CentralDifferenceTerminalGradient(problem_, state, eps),
          CentralDifferenceTerminalHessian(problem_, state, eps)};
}

void ILQRSolver::UpdateExpansions(const Trajectory& trajectory) {
  for (int k = 0; k < problem_.Horizon(); ++k) {
    A_[k] = DynamicsJacobianState(trajectory.State(k), trajectory.Control(k));
    B_[k] = DynamicsJacobianControl(trajectory.State(k), trajectory.Control(k));
    stage_expansions_[k] = StageCostExpansion(k, trajectory.State(k), trajectory.Control(k));
  }
}

double ILQRSolver::EvaluateTrajectoryCost(const Trajectory& trajectory) const {
  return problem_.TotalCost(trajectory);
}

bool ILQRSolver::BackwardPass(const Trajectory& trajectory,
                              double regularization,
                              double* expected_linear,
                              double* expected_quadratic) {
  const auto [terminal_gradient, terminal_hessian] =
      TerminalCostExpansion(trajectory.State(problem_.Horizon()));
  Vector Vx = terminal_gradient;
  Matrix Vxx = terminal_hessian;
  *expected_linear = 0.0;
  *expected_quadratic = 0.0;

  for (int k = problem_.Horizon() - 1; k >= 0; --k) {
    const auto& expansion = stage_expansions_[k];
    const Matrix& A = A_[k];
    const Matrix& B = B_[k];

    const Vector Qx = expansion.lx + A.transpose() * Vx;
    const Vector Qu = expansion.lu + B.transpose() * Vx;
    const Matrix Qxx = expansion.lxx + A.transpose() * Vxx * A;
    const Matrix Qux = expansion.lux + B.transpose() * Vxx * A;
    const Matrix Quu = expansion.luu + B.transpose() * Vxx * B;
    const Matrix Quu_reg =
        Quu + regularization * Matrix::Identity(problem_.ControlDim(), problem_.ControlDim());

    Eigen::LLT<Matrix> llt(Quu_reg);
    if (llt.info() != Eigen::Success) {
      return false;
    }

    feedback_gains_[k] = -llt.solve(Qux);
    feedforward_terms_[k] = -llt.solve(Qu);

    *expected_linear += Qu.dot(feedforward_terms_[k]);
    *expected_quadratic += 0.5 * feedforward_terms_[k].dot(Quu * feedforward_terms_[k]);

    Vx = Qx + feedback_gains_[k].transpose() * Quu * feedforward_terms_[k] +
         feedback_gains_[k].transpose() * Qu + Qux.transpose() * feedforward_terms_[k];
    Vxx = Qxx + feedback_gains_[k].transpose() * Quu * feedback_gains_[k] +
          feedback_gains_[k].transpose() * Qux + Qux.transpose() * feedback_gains_[k];
    Vxx = 0.5 * (Vxx + Vxx.transpose());
  }

  return true;
}

bool ILQRSolver::ForwardPass(const Vector& initial_state,
                             const Trajectory& nominal_trajectory,
                             double current_cost,
                             double expected_linear,
                             double expected_quadratic,
                             Trajectory* accepted_trajectory,
                             double* accepted_cost,
                             double* accepted_alpha,
                             double* accepted_ratio) {
  double alpha = 1.0;

  for (int iter = 0; iter < options_.line_search_max_iterations; ++iter) {
    Trajectory candidate(problem_.StateDim(), problem_.ControlDim(), problem_.Horizon());
    candidate.SetTimeStep(problem_.TimeStep());
    candidate.State(0) = initial_state;

    for (int k = 0; k < problem_.Horizon(); ++k) {
      const Vector dx = candidate.State(k) - nominal_trajectory.State(k);
      candidate.Control(k) =
          nominal_trajectory.Control(k) + alpha * feedforward_terms_[k] + feedback_gains_[k] * dx;
      candidate.State(k + 1) =
          problem_.Dynamics().NextState(candidate.State(k), candidate.Control(k), problem_.TimeStep());
    }

    const double candidate_cost = EvaluateTrajectoryCost(candidate);
    const double actual_decrease = current_cost - candidate_cost;
    const double expected_decrease = -(alpha * expected_linear + alpha * alpha * expected_quadratic);

    if (expected_decrease > 0.0) {
      const double ratio = actual_decrease / expected_decrease;
      const bool acceptable_ratio = options_.line_search_accept_lower <= ratio &&
                                    ratio <= options_.line_search_accept_upper;
      if (candidate_cost < current_cost && acceptable_ratio) {
        *accepted_trajectory = candidate;
        *accepted_cost = candidate_cost;
        *accepted_alpha = alpha;
        *accepted_ratio = ratio;
        return true;
      }
    }

    alpha *= options_.line_search_decrease_factor;
  }

  return false;
}

Trajectory ILQRSolver::Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls) {
  if (initial_state.size() != problem_.StateDim()) {
    throw std::invalid_argument("iLQR received an initial state with invalid dimensions.");
  }
  if (static_cast<int>(initial_controls.size()) != problem_.Horizon()) {
    throw std::invalid_argument("iLQR received a control sequence with invalid length.");
  }

  cost_history_.clear();
  alpha_history_.clear();
  regularization_history_.clear();
  improvement_ratio_history_.clear();
  Trajectory trajectory = problem_.Rollout(initial_state, initial_controls);
  cost_history_.push_back(EvaluateTrajectoryCost(trajectory));
  double regularization = std::max(options_.regularization_init, options_.regularization_min);

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    const double previous_cost = cost_history_.back();
    bool accepted = false;

    while (!accepted) {
      UpdateExpansions(trajectory);

      double expected_linear = 0.0;
      double expected_quadratic = 0.0;
      const bool backward_ok =
          BackwardPass(trajectory, regularization, &expected_linear, &expected_quadratic);
      if (!backward_ok) {
        regularization *= options_.regularization_increase_factor;
        if (regularization > options_.regularization_max) {
          return trajectory;
        }
        continue;
      }

      Trajectory accepted_trajectory(problem_.StateDim(), problem_.ControlDim(), problem_.Horizon());
      double accepted_cost = previous_cost;
      double accepted_alpha = 0.0;
      double accepted_ratio = 0.0;
      accepted = ForwardPass(initial_state, trajectory, previous_cost, expected_linear,
                             expected_quadratic, &accepted_trajectory, &accepted_cost,
                             &accepted_alpha, &accepted_ratio);
      if (accepted) {
        trajectory = accepted_trajectory;
        cost_history_.push_back(accepted_cost);
        alpha_history_.push_back(accepted_alpha);
        regularization_history_.push_back(regularization);
        improvement_ratio_history_.push_back(accepted_ratio);
        regularization =
            std::max(options_.regularization_min,
                     regularization / options_.regularization_decrease_factor);
      } else {
        regularization *= options_.regularization_increase_factor;
        if (regularization > options_.regularization_max) {
          return trajectory;
        }
      }
    }

    if (std::abs(previous_cost - cost_history_.back()) < options_.cost_tolerance) {
      break;
    }
  }

  return trajectory;
}

}  // namespace my_al_ilqr
