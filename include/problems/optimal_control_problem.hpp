#pragma once

#include <memory>
#include <vector>

#include "core/trajectory.hpp"
#include "cost/cost_function.hpp"
#include "dynamics/dynamics_model.hpp"

namespace my_al_ilqr {

class OptimalControlProblem {
 public:
  OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                        std::shared_ptr<CostFunction> cost,
                        int horizon,
                        double dt);
  OptimalControlProblem(std::shared_ptr<DynamicsModel> dynamics,
                        std::vector<std::shared_ptr<CostFunction>> stage_costs,
                        std::shared_ptr<CostFunction> terminal_cost,
                        double dt);

  int StateDim() const;
  int ControlDim() const;
  int Horizon() const { return horizon_; }
  double TimeStep() const { return dt_; }
  const DynamicsModel& Dynamics() const { return *dynamics_; }
  std::shared_ptr<DynamicsModel> SharedDynamics() const { return dynamics_; }
  const CostFunction& StageCostFunction(int k) const;
  std::shared_ptr<CostFunction> SharedStageCostFunction(int k) const { return stage_costs_.at(k); }
  const CostFunction& TerminalCostFunction() const { return *terminal_cost_; }
  std::shared_ptr<CostFunction> SharedTerminalCostFunction() const { return terminal_cost_; }

  Trajectory Rollout(const Vector& initial_state,
                     const std::vector<Vector>& control_sequence) const;

  double TotalCost(const Trajectory& trajectory) const;

 private:
  std::shared_ptr<DynamicsModel> dynamics_;
  std::vector<std::shared_ptr<CostFunction>> stage_costs_;
  std::shared_ptr<CostFunction> terminal_cost_;
  int horizon_;
  double dt_;
};

}  // namespace my_al_ilqr
