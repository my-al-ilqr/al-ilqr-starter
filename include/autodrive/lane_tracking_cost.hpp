#pragma once

#include <memory>

#include "autodrive/straight_reference_line.hpp"
#include "cost/cost_function.hpp"

namespace my_al_ilqr {

class LaneTrackingCost final : public CostFunction {
 public:
  LaneTrackingCost(std::shared_ptr<StraightReferenceLine> reference_line,
                   double target_speed,
                   double target_longitudinal_position,
                   double stage_lateral_weight,
                   double stage_heading_weight,
                   double stage_speed_weight,
                   double stage_accel_weight,
                   double stage_steering_weight,
                   double terminal_longitudinal_weight,
                   double terminal_lateral_weight,
                   double terminal_heading_weight,
                   double terminal_speed_weight);

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }

  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override;

 private:
  std::shared_ptr<StraightReferenceLine> reference_line_;
  double target_speed_ = 0.0;
  double target_longitudinal_position_ = 0.0;
  double stage_lateral_weight_ = 0.0;
  double stage_heading_weight_ = 0.0;
  double stage_speed_weight_ = 0.0;
  double stage_accel_weight_ = 0.0;
  double stage_steering_weight_ = 0.0;
  double terminal_longitudinal_weight_ = 0.0;
  double terminal_lateral_weight_ = 0.0;
  double terminal_heading_weight_ = 0.0;
  double terminal_speed_weight_ = 0.0;
};

}  // namespace my_al_ilqr
