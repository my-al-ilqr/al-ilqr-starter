#include "lqr/finite_horizon_lqr_solver.hpp"

#include <stdexcept>

namespace my_al_ilqr {

FiniteHorizonLQRSolver::FiniteHorizonLQRSolver(FiniteHorizonLQRProblem problem)
    : problem_(std::move(problem)) {
  ValidateProblem();
  feedback_gains_.assign(problem_.horizon, Matrix::Zero(problem_.ControlDim(), problem_.StateDim()));
  riccati_matrices_.assign(problem_.horizon + 1,
                           Matrix::Zero(problem_.StateDim(), problem_.StateDim()));
}

void FiniteHorizonLQRSolver::ValidateProblem() const {
  if (problem_.horizon <= 0) {
    throw std::invalid_argument("LQR horizon must be positive.");
  }
  if (problem_.A.rows() == 0 || problem_.A.cols() == 0 || problem_.B.rows() == 0 ||
      problem_.B.cols() == 0) {
    throw std::invalid_argument("LQR system matrices must be non-empty.");
  }
  if (problem_.A.rows() != problem_.A.cols()) {
    throw std::invalid_argument("LQR matrix A must be square.");
  }
  if (problem_.B.rows() != problem_.A.rows()) {
    throw std::invalid_argument("LQR matrix B must have the same number of rows as A.");
  }
  const int nx = problem_.StateDim();
  const int nu = problem_.ControlDim();
  if (problem_.Q.rows() != nx || problem_.Q.cols() != nx || problem_.Qf.rows() != nx ||
      problem_.Qf.cols() != nx) {
    throw std::invalid_argument("LQR state cost matrices have incompatible dimensions.");
  }
  if (problem_.R.rows() != nu || problem_.R.cols() != nu) {
    throw std::invalid_argument("LQR control cost matrix has incompatible dimensions.");
  }
  if (problem_.state_reference.size() != nx || problem_.control_reference.size() != nu) {
    throw std::invalid_argument("LQR reference vectors have incompatible dimensions.");
  }
}

void FiniteHorizonLQRSolver::Solve() {
  riccati_matrices_.back() = problem_.Qf;

  for (int k = problem_.horizon - 1; k >= 0; --k) {
    const Matrix& P_next = riccati_matrices_[k + 1];
    const Matrix S = problem_.R + problem_.B.transpose() * P_next * problem_.B;
    const Matrix rhs = problem_.B.transpose() * P_next * problem_.A;
    feedback_gains_[k] = -S.ldlt().solve(rhs);
    riccati_matrices_[k] =
        problem_.Q + problem_.A.transpose() * P_next * (problem_.A + problem_.B * feedback_gains_[k]);
  }

  is_solved_ = true;
}

Vector FiniteHorizonLQRSolver::Control(const Vector& state, int k) const {
  if (!is_solved_) {
    throw std::logic_error("LQR solver must be solved before querying controls.");
  }
  if (k < 0 || k >= problem_.horizon) {
    throw std::out_of_range("LQR control index is out of range.");
  }
  if (state.size() != problem_.StateDim()) {
    throw std::invalid_argument("LQR control query received a state with invalid dimensions.");
  }

  const Vector state_error = state - problem_.state_reference;
  return problem_.control_reference + feedback_gains_[k] * state_error;
}

Trajectory FiniteHorizonLQRSolver::Simulate(const Vector& initial_state) const {
  if (!is_solved_) {
    throw std::logic_error("LQR solver must be solved before simulation.");
  }
  if (initial_state.size() != problem_.StateDim()) {
    throw std::invalid_argument("LQR simulation received an initial state with invalid dimensions.");
  }

  Trajectory trajectory(problem_.StateDim(), problem_.ControlDim(), problem_.horizon);
  trajectory.State(0) = initial_state;

  for (int k = 0; k < problem_.horizon; ++k) {
    trajectory.Control(k) = Control(trajectory.State(k), k);
    trajectory.State(k + 1) = problem_.A * trajectory.State(k) + problem_.B * trajectory.Control(k);
  }

  return trajectory;
}

}  // namespace my_al_ilqr
