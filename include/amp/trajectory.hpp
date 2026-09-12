#pragma once

#include <array>
#include <vector>

namespace amp {

using JointVector = std::array<double, 6>;

enum class ReachPlannerKind {
  aligator,
};

struct TrajectoryKnot {
  double time_s;
  JointVector q;
  JointVector velocity;
};

struct PlanningReport {
  ReachPlannerKind strategy;
  double final_position_error_m;
  double final_frame_speed_mps;
  double max_joint_limit_violation_rad;
  double computation_time_s;
};

struct JointTrajectory {
  std::vector<TrajectoryKnot> knots;
  PlanningReport report;
};

}  // namespace amp
