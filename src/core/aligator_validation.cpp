#include "aligator_validation.hpp"

#include <cmath>
#include <string>

namespace so101_traj_planner::detail {

std::optional<std::string> validate_aligator_metrics(const AligatorValidationMetrics& metrics) {
  if (std::isfinite(metrics.final_position_error_m) &&
      std::isfinite(metrics.final_frame_speed_mps) &&
      std::isfinite(metrics.max_terminal_joint_velocity_rad_s) &&
      std::isfinite(metrics.max_joint_limit_violation_rad) &&
      std::isfinite(metrics.max_velocity_limit_violation_rad_s) &&
      std::isfinite(metrics.max_effort_limit_violation_nm) &&
      std::isfinite(metrics.max_dynamics_defect) &&
      metrics.final_position_error_m <= kFinalPositionToleranceM &&
      metrics.final_frame_speed_mps <= kFinalFrameSpeedToleranceMps &&
      metrics.max_terminal_joint_velocity_rad_s <= kTerminalJointVelocityToleranceRadS &&
      metrics.max_joint_limit_violation_rad <= kJointLimitToleranceRad &&
      metrics.max_velocity_limit_violation_rad_s <= kVelocityLimitToleranceRadS &&
      metrics.max_effort_limit_violation_nm <= kEffortLimitToleranceNm &&
      metrics.max_dynamics_defect <= kDynamicsDefectTolerance) {
    return std::nullopt;
  }

  return "Aligator result violated a postcondition (position_error=" +
         std::to_string(metrics.final_position_error_m) +
         ", frame_speed=" + std::to_string(metrics.final_frame_speed_mps) +
         ", terminal_joint_velocity=" + std::to_string(metrics.max_terminal_joint_velocity_rad_s) +
         ", joint_violation=" + std::to_string(metrics.max_joint_limit_violation_rad) +
         ", velocity_violation=" + std::to_string(metrics.max_velocity_limit_violation_rad_s) +
         ", effort_violation=" + std::to_string(metrics.max_effort_limit_violation_nm) +
         ", dynamics_defect=" + std::to_string(metrics.max_dynamics_defect) + ")";
}

}  // namespace so101_traj_planner::detail
