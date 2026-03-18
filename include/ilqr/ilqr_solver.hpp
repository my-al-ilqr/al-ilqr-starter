#pragma once

#include <vector>

#include "core/trajectory.hpp"
#include "core/types.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

struct ILQROptions {
  int max_iterations = 50;
  double cost_tolerance = 1e-6;
  double derivative_epsilon = 1e-4;
  double regularization_init = 1e-6;
  double regularization_min = 1e-8;
  double regularization_max = 1e6;
  double regularization_increase_factor = 10.0;
  double regularization_decrease_factor = 10.0;
  int line_search_max_iterations = 8;
  double line_search_decrease_factor = 0.5;
  double line_search_accept_lower = 1e-4;
  double line_search_accept_upper = 10.0;
};

class ILQRSolver {
 public:
  explicit ILQRSolver(const OptimalControlProblem& problem, ILQROptions options = {});

  const std::vector<double>& CostHistory() const { return cost_history_; }
  const std::vector<double>& AlphaHistory() const { return alpha_history_; }
  const std::vector<double>& RegularizationHistory() const { return regularization_history_; }
  const std::vector<double>& ImprovementRatioHistory() const { return improvement_ratio_history_; }
  const std::vector<Matrix>& FeedbackGains() const { return feedback_gains_; }
  const std::vector<Vector>& FeedforwardTerms() const { return feedforward_terms_; }

  Trajectory Solve(const Vector& initial_state, const std::vector<Vector>& initial_controls);

 private:
  struct CostExpansion {
    Vector lx;
    Vector lu;
    Matrix lxx;
    Matrix lux;
    Matrix luu;
  };

  void EnsureWorkspaceSizes();
  void UpdateExpansions(const Trajectory& trajectory);
  bool BackwardPass(const Trajectory& trajectory,
                    double regularization,
                    double* expected_linear,
                    double* expected_quadratic);
  bool ForwardPass(const Vector& initial_state,
                   const Trajectory& nominal_trajectory,
                   double current_cost,
                   double expected_linear,
                   double expected_quadratic,
                   Trajectory* accepted_trajectory,
                   double* accepted_cost,
                   double* accepted_alpha,
                   double* accepted_ratio);

  Matrix DynamicsJacobianState(const Vector& state, const Vector& control) const;
  Matrix DynamicsJacobianControl(const Vector& state, const Vector& control) const;
  CostExpansion StageCostExpansion(int k, const Vector& state, const Vector& control) const;
  std::pair<Vector, Matrix> TerminalCostExpansion(const Vector& state) const;
  double EvaluateTrajectoryCost(const Trajectory& trajectory) const;

  const OptimalControlProblem& problem_;
  ILQROptions options_;
  std::vector<Matrix> A_;
  std::vector<Matrix> B_;
  std::vector<CostExpansion> stage_expansions_;
  std::vector<Matrix> feedback_gains_;
  std::vector<Vector> feedforward_terms_;
  std::vector<double> cost_history_;
  std::vector<double> alpha_history_;
  std::vector<double> regularization_history_;
  std::vector<double> improvement_ratio_history_;
};

}  // namespace my_al_ilqr
