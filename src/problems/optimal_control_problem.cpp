#include "problems/optimal_control_problem.hpp"

#include <stdexcept>

namespace my_al_ilqr {

OptimalControlProblem::OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                                             std::shared_ptr<CostFunction> cost,
                                             int horizon,
                                             double dt)
    : dynamics_(std::move(dynamics)),
      stage_costs_(horizon, cost),
      terminal_cost_(cost),
      horizon_(horizon),
      dt_(dt) {
  if (!dynamics_ || !terminal_cost_) {
    throw std::invalid_argument("Problem requires valid dynamics and cost objects.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (!stage_cost) {
      throw std::invalid_argument("All stage costs must be valid.");
    }
  }
  if (stage_costs_.empty()) {
    throw std::invalid_argument("Problem horizon and cost sequence must be positive.");
  }
  if (dynamics_->StateDim() != terminal_cost_->StateDim() ||
      dynamics_->ControlDim() != terminal_cost_->ControlDim()) {
    throw std::invalid_argument("Dynamics and terminal cost dimensions are inconsistent.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (dynamics_->StateDim() != stage_cost->StateDim() ||
        dynamics_->ControlDim() != stage_cost->ControlDim()) {
      throw std::invalid_argument("Dynamics and stage cost dimensions are inconsistent.");
    }
  }
  if (horizon_ <= 0 || dt_ <= 0.0) {
    throw std::invalid_argument("Problem horizon and time step must be positive.");
  }
}

OptimalControlProblem::OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                                             std::vector<std::shared_ptr<CostFunction>> stage_costs,
                                             std::shared_ptr<CostFunction> terminal_cost,
                                             double dt)
    : dynamics_(std::move(dynamics)),
      stage_costs_(std::move(stage_costs)),
      terminal_cost_(std::move(terminal_cost)),
      horizon_(static_cast<int>(stage_costs_.size())),
      dt_(dt) {
  if (!dynamics_ || !terminal_cost_) {
    throw std::invalid_argument("Problem requires valid dynamics and cost objects.");
  }
  if (horizon_ <= 0 || dt_ <= 0.0) {
    throw std::invalid_argument("Problem horizon and time step must be positive.");
  }
  if (stage_costs_.empty()) {
    throw std::invalid_argument("Stage cost sequence must be non-empty.");
  }
  if (dynamics_->StateDim() != terminal_cost_->StateDim() ||
      dynamics_->ControlDim() != terminal_cost_->ControlDim()) {
    throw std::invalid_argument("Dynamics and terminal cost dimensions are inconsistent.");
  }
  for (const auto& stage_cost : stage_costs_) {
    if (!stage_cost) {
      throw std::invalid_argument("All stage costs must be valid.");
    }
    if (dynamics_->StateDim() != stage_cost->StateDim() ||
        dynamics_->ControlDim() != stage_cost->ControlDim()) {
      throw std::invalid_argument("Dynamics and stage cost dimensions are inconsistent.");
    }
  }
}

int OptimalControlProblem::StateDim() const {
  return dynamics_->StateDim();
}

int OptimalControlProblem::ControlDim() const {
  return dynamics_->ControlDim();
}

const CostFunction& OptimalControlProblem::StageCostFunction(int k) const {
  return *stage_costs_.at(k);
}

Trajectory OptimalControlProblem::Rollout(const Vector& initial_state,
                                          const std::vector<Vector>& control_sequence) const {
  if (initial_state.size() != StateDim()) {
    throw std::invalid_argument("Initial state has invalid dimensions.");
  }
  if (static_cast<int>(control_sequence.size()) != horizon_) {
    throw std::invalid_argument("Control sequence length must match the problem horizon.");
  }

  Trajectory trajectory(StateDim(), ControlDim(), horizon_);
  trajectory.SetTimeStep(dt_);
  trajectory.State(0) = initial_state;

  for (int k = 0; k < horizon_; ++k) {
    if (control_sequence[k].size() != ControlDim()) {
      throw std::invalid_argument("Control input has invalid dimensions.");
    }
    trajectory.Control(k) = control_sequence[k];
    trajectory.State(k + 1) = dynamics_->NextState(trajectory.State(k), trajectory.Control(k), dt_);
  }
  return trajectory;
}

double OptimalControlProblem::TotalCost(const Trajectory& trajectory) const {
  if (trajectory.StateDim() != StateDim() || trajectory.ControlDim() != ControlDim() ||
      trajectory.Horizon() != horizon_) {
    throw std::invalid_argument("Trajectory dimensions do not match the problem definition.");
  }

  double total_cost = 0.0;
  for (int k = 0; k < horizon_; ++k) {
    total_cost += stage_costs_.at(k)->StageCost(trajectory.State(k), trajectory.Control(k));
  }
  total_cost += terminal_cost_->TerminalCost(trajectory.State(horizon_));
  return total_cost;
}

}  // namespace my_al_ilqr
