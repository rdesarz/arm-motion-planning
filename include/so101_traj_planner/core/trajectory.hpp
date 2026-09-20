#pragma once

// std
#include <array>
#include <vector>

// so101_traj_planner
#include "so101_traj_planner/core/so101_model.hpp"

namespace so101_traj_planner {

using JointVector = std::array<double, kSo101JointCount>;

struct TrajectoryKnot {
  double time_s;
  JointVector q;
  JointVector velocity;
};

struct PlanningReport {
  double final_position_error_m;
  double final_frame_speed_mps;
  double max_joint_limit_violation_rad;
  double computation_time_s;
};

struct JointTrajectory {
  std::vector<TrajectoryKnot> knots;
  PlanningReport report;
};

}  // namespace so101_traj_planner
