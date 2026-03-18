#include "problems/constrained_optimal_control_problem.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

ConstrainedOptimalControlProblem::ConstrainedOptimalControlProblem(
    std::shared_ptr<OptimalControlProblem> problem)
    : problem_(std::move(problem)),
      stage_constraints_(problem_ ? problem_->Horizon() : 0) {
  if (!problem_) {
    throw std::invalid_argument("ConstrainedOptimalControlProblem requires a valid base problem.");
  }
}

void ConstrainedOptimalControlProblem::AddStageConstraint(int k, ConstraintPtr constraint) {
  if (k < 0 || k >= Horizon()) {
    throw std::out_of_range("Stage constraint index is out of range.");
  }
  if (!constraint) {
    throw std::invalid_argument("Stage constraint pointer cannot be null.");
  }
  if (constraint->StateDim() != problem_->StateDim() || constraint->ControlDim() != problem_->ControlDim()) {
    throw std::invalid_argument("Stage constraint dimensions do not match the base problem.");
  }
  stage_constraints_[k].push_back(std::move(constraint));
}

void ConstrainedOptimalControlProblem::AddTerminalConstraint(ConstraintPtr constraint) {
  if (!constraint) {
    throw std::invalid_argument("Terminal constraint pointer cannot be null.");
  }
  if (constraint->StateDim() != problem_->StateDim()) {
    throw std::invalid_argument("Terminal constraint state dimensions do not match the base problem.");
  }
  terminal_constraints_.push_back(std::move(constraint));
}

std::vector<ConstraintEvaluation> ConstrainedOptimalControlProblem::EvaluateTrajectory(
    const Trajectory& trajectory) const {
  if (trajectory.Horizon() != Horizon() || trajectory.StateDim() != problem_->StateDim() ||
      trajectory.ControlDim() != problem_->ControlDim()) {
    throw std::invalid_argument("Trajectory dimensions do not match the constrained problem.");
  }

  std::vector<ConstraintEvaluation> evaluations;
  for (int k = 0; k < Horizon(); ++k) {
    for (const auto& constraint : stage_constraints_[k]) {
      const Vector values = constraint->Evaluate(trajectory.State(k), trajectory.Control(k));
      evaluations.push_back(ConstraintEvaluation{
          constraint->Name(), constraint->Type(), k, values, my_al_ilqr::MaxViolation(*constraint, values)});
    }
  }

  const Vector empty_control = Vector::Zero(0);
  for (const auto& constraint : terminal_constraints_) {
    const Vector values = constraint->Evaluate(trajectory.State(Horizon()), empty_control);
    evaluations.push_back(ConstraintEvaluation{
        constraint->Name(), constraint->Type(), Horizon(), values, my_al_ilqr::MaxViolation(*constraint, values)});
  }

  return evaluations;
}

double ConstrainedOptimalControlProblem::MaxViolation(const Trajectory& trajectory) const {
  const auto evaluations = EvaluateTrajectory(trajectory);
  double max_violation = 0.0;
  for (const auto& evaluation : evaluations) {
    max_violation = std::max(max_violation, evaluation.max_violation);
  }
  return max_violation;
}

}  // namespace my_al_ilqr
