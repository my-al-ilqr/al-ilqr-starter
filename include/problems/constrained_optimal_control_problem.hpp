#pragma once

#include <memory>
#include <vector>

#include "constraints/constraint_function.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

class ConstrainedOptimalControlProblem {
 public:
  explicit ConstrainedOptimalControlProblem(std::shared_ptr<OptimalControlProblem> problem);

  const OptimalControlProblem& BaseProblem() const { return *problem_; }
  std::shared_ptr<OptimalControlProblem> SharedBaseProblem() const { return problem_; }
  int Horizon() const { return problem_->Horizon(); }
  const std::vector<ConstraintPtr>& StageConstraints(int k) const { return stage_constraints_.at(k); }
  const std::vector<ConstraintPtr>& TerminalConstraints() const { return terminal_constraints_; }

  void AddStageConstraint(int k, ConstraintPtr constraint);
  void AddTerminalConstraint(ConstraintPtr constraint);

  std::vector<ConstraintEvaluation> EvaluateTrajectory(const Trajectory& trajectory) const;
  double MaxViolation(const Trajectory& trajectory) const;

 private:
  std::shared_ptr<OptimalControlProblem> problem_;
  std::vector<std::vector<ConstraintPtr>> stage_constraints_;
  std::vector<ConstraintPtr> terminal_constraints_;
};

}  // namespace my_al_ilqr
