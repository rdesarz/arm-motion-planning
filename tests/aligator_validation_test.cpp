#include "aligator_validation.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

}  // namespace

int main() {
  amp::detail::AligatorValidationMetrics metrics{
      .final_position_error_m = 0.001,
      .final_frame_speed_mps = 0.001,
      .max_joint_limit_violation_rad = 0.0,
      .max_velocity_limit_violation_rad_s = 0.0,
      .max_effort_limit_violation_nm = 0.0,
      .max_dynamics_defect = 0.0,
  };

  bool passed = true;
  passed &= require(!amp::detail::validate_aligator_metrics(metrics),
                    "metrics inside every planner threshold must pass");

  metrics.max_effort_limit_violation_nm = amp::detail::kEffortLimitToleranceNm * 2.0;
  passed &= require(amp::detail::validate_aligator_metrics(metrics).has_value(),
                    "an effort-limit violation must fail closed");

  metrics.max_effort_limit_violation_nm = 0.0;
  metrics.max_dynamics_defect = amp::detail::kDynamicsDefectTolerance * 2.0;
  passed &= require(amp::detail::validate_aligator_metrics(metrics).has_value(),
                    "a dynamics defect must fail closed");

  metrics.max_dynamics_defect = std::numeric_limits<double>::quiet_NaN();
  passed &= require(amp::detail::validate_aligator_metrics(metrics).has_value(),
                    "a non-finite dynamics defect must fail closed");

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
