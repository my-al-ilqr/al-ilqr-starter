#pragma once

#include <memory>
#include <vector>

#include "constraints/constraint_function.hpp"
#include "cost/cost_function.hpp"
#include "problems/constrained_optimal_control_problem.hpp"

namespace my_al_ilqr {

struct AugmentedLagrangianConstraintData {
  ConstraintPtr constraint;
  Vector lambda;
  Vector mu;
  double penalty_scaling = 10.0;
};

class AugmentedLagrangianKnotCost final : public CostFunction {
 public:
  AugmentedLagrangianKnotCost(std::shared_ptr<CostFunction> base_cost,
                              std::vector<AugmentedLagrangianConstraintData> constraints);

  int StateDim() const override { return base_cost_->StateDim(); }
  int ControlDim() const override { return base_cost_->ControlDim(); }
  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override;

  void UpdateDuals(const Vector& state, const Vector& control);
  void UpdatePenalties();
  double MaxViolation(const Vector& state, const Vector& control) const;
  double MaxPenalty() const;

  const std::vector<AugmentedLagrangianConstraintData>& Constraints() const { return constraints_; }
  std::vector<AugmentedLagrangianConstraintData>& Constraints() { return constraints_; }

 private:
  double ConstraintAugmentedTerm(const AugmentedLagrangianConstraintData& data,
                                 const Vector& state,
                                 const Vector& control) const;
  double TerminalAugmentedTerm(const AugmentedLagrangianConstraintData& data,
                               const Vector& state) const;

  std::shared_ptr<CostFunction> base_cost_;
  std::vector<AugmentedLagrangianConstraintData> constraints_;
};

class AugmentedLagrangianProblem {
 public:
  AugmentedLagrangianProblem(const ConstrainedOptimalControlProblem& constrained_problem,
                             double initial_penalty,
                             double penalty_scaling);

  std::shared_ptr<OptimalControlProblem> UnconstrainedSubproblem() const { return subproblem_; }
  std::shared_ptr<AugmentedLagrangianKnotCost> StageCost(int k) const { return stage_costs_.at(k); }
  std::shared_ptr<AugmentedLagrangianKnotCost> TerminalCost() const { return terminal_cost_; }

  void UpdateDuals(const Trajectory& trajectory);
  void UpdatePenalties();
  double MaxViolation(const Trajectory& trajectory) const;
  double MaxPenalty() const;

 private:
  std::shared_ptr<OptimalControlProblem> subproblem_;
  std::vector<std::shared_ptr<AugmentedLagrangianKnotCost>> stage_costs_;
  std::shared_ptr<AugmentedLagrangianKnotCost> terminal_cost_;
};

}  // namespace my_al_ilqr
