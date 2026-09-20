#pragma once

#include <optional>
#include <string>

namespace so101_traj_planner::detail {

inline constexpr double kFinalPositionToleranceM = 0.005;
inline constexpr double kFinalFrameSpeedToleranceMps = 0.02;
inline constexpr double kTerminalJointVelocityToleranceRadS = 1e-3;
inline constexpr double kJointLimitToleranceRad = 1e-6;
inline constexpr double kVelocityLimitToleranceRadS = 1e-6;
inline constexpr double kEffortLimitToleranceNm = 1e-6;
inline constexpr double kDynamicsDefectTolerance = 1e-5;

struct AligatorValidationMetrics {
  double final_position_error_m;
  double final_frame_speed_mps;
  double max_terminal_joint_velocity_rad_s;
  double max_joint_limit_violation_rad;
  double max_velocity_limit_violation_rad_s;
  double max_effort_limit_violation_nm;
  double max_dynamics_defect;
};

[[nodiscard]] std::optional<std::string> validate_aligator_metrics(
    const AligatorValidationMetrics& metrics);

}  // namespace so101_traj_planner::detail
