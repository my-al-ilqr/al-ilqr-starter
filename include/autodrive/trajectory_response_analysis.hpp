#pragma once

#include <cmath>

#include "core/trajectory.hpp"

namespace my_al_ilqr {

enum class TrajectoryResponseType {
  kGoStraight,
  kYield,
  kBypass,
};

struct TrajectoryResponseMetrics {
  double crossing_time = 0.0;
  double crossing_delay = 0.0;
  double max_abs_lateral = 0.0;
  double min_speed = 0.0;
  TrajectoryResponseType response = TrajectoryResponseType::kGoStraight;
};

inline double CrossingTimeAtX(const Trajectory& trajectory, double target_x) {
  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    if (trajectory.State(k)(0) >= target_x) {
      return k * trajectory.TimeStep();
    }
  }
  return trajectory.Horizon() * trajectory.TimeStep();
}

inline TrajectoryResponseMetrics AnalyzeTrajectoryResponse(const Trajectory& trajectory,
                                                          double obstacle_x,
                                                          double baseline_crossing_time,
                                                          double bypass_lateral_threshold = 0.15,
                                                          double yield_delay_threshold = 0.35) {
  TrajectoryResponseMetrics metrics;
  metrics.crossing_time = CrossingTimeAtX(trajectory, obstacle_x);
  metrics.crossing_delay = metrics.crossing_time - baseline_crossing_time;
  metrics.min_speed = trajectory.State(0)(3);

  for (int k = 0; k <= trajectory.Horizon(); ++k) {
    metrics.max_abs_lateral = std::max(metrics.max_abs_lateral, std::abs(trajectory.State(k)(1)));
    metrics.min_speed = std::min(metrics.min_speed, trajectory.State(k)(3));
  }

  if (metrics.max_abs_lateral >= bypass_lateral_threshold) {
    metrics.response = TrajectoryResponseType::kBypass;
  } else if (metrics.crossing_delay >= yield_delay_threshold) {
    metrics.response = TrajectoryResponseType::kYield;
  } else {
    metrics.response = TrajectoryResponseType::kGoStraight;
  }
  return metrics;
}

inline const char* ToString(TrajectoryResponseType response) {
  switch (response) {
    case TrajectoryResponseType::kGoStraight:
      return "go_straight";
    case TrajectoryResponseType::kYield:
      return "yield";
    case TrajectoryResponseType::kBypass:
      return "bypass";
  }
  return "unknown";
}

}  // namespace my_al_ilqr
