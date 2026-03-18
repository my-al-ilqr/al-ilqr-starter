#pragma once

#include <vector>

#include "core/trajectory.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

struct FiniteHorizonLQRProblem {
  Matrix A;
  Matrix B;
  Matrix Q;
  Matrix R;
  Matrix Qf;
  Vector state_reference;
  Vector control_reference;
  int horizon = 0;

  int StateDim() const { return A.rows(); }
  int ControlDim() const { return B.cols(); }
};

class FiniteHorizonLQRSolver {
 public:
  explicit FiniteHorizonLQRSolver(FiniteHorizonLQRProblem problem);

  void Solve();

  const std::vector<Matrix>& FeedbackGains() const { return feedback_gains_; }
  const std::vector<Matrix>& RiccatiMatrices() const { return riccati_matrices_; }
  const FiniteHorizonLQRProblem& Problem() const { return problem_; }

  Vector Control(const Vector& state, int k) const;
  Trajectory Simulate(const Vector& initial_state) const;

 private:
  void ValidateProblem() const;

  FiniteHorizonLQRProblem problem_;
  std::vector<Matrix> feedback_gains_;
  std::vector<Matrix> riccati_matrices_;
  bool is_solved_ = false;
};

}  // namespace my_al_ilqr
