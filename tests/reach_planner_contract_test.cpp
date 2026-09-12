#include <Eigen/Core>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>

#include "amp/reach_planner.hpp"

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

bool require_error(const std::expected<amp::JointTrajectory, amp::PlanningError>& result,
                   const amp::PlanningErrorCode expected, const char* message) {
  return require(!result && result.error().code == expected, message);
}

}  // namespace

int main() {
  bool passed = true;

  const auto unavailable =
      amp::ReachPlannerFactory::create(static_cast<amp::ReachPlannerKind>(99), AMP_TEST_ROBOT_PATH);
  passed &= require(
      !unavailable && unavailable.error().code == amp::PlanningErrorCode::strategy_unavailable,
      "an unavailable strategy must fail before model loading");

  const auto missing = amp::ReachPlannerFactory::create(
      amp::ReachPlannerKind::aligator,
      std::filesystem::path(AMP_TEST_ROBOT_PATH).parent_path() / "does-not-exist.xml");
  passed &= require(!missing && missing.error().code == amp::PlanningErrorCode::model_mismatch,
                    "a missing robot model must be a model mismatch");

  auto planner =
      amp::ReachPlannerFactory::create(amp::ReachPlannerKind::aligator, AMP_TEST_ROBOT_PATH);
  passed &= require(planner.has_value(), "the Aligator strategy must load the pinned SO-101 model");
  if (!planner) {
    std::cerr << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  const amp::ReachRequest nominal{
      .q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25},
      .target_world_m = Eigen::Vector3d{0.2, 0.0, 0.2},
      .duration_s = 2.0,
  };

  auto invalid_duration = nominal;
  invalid_duration.duration_s = 0.0;
  passed &=
      require_error((*planner)->plan(invalid_duration), amp::PlanningErrorCode::invalid_request,
                    "zero duration must be rejected");

  invalid_duration.duration_s = -1.0;
  passed &=
      require_error((*planner)->plan(invalid_duration), amp::PlanningErrorCode::invalid_request,
                    "negative duration must be rejected");

  invalid_duration.duration_s = std::numeric_limits<double>::infinity();
  passed &=
      require_error((*planner)->plan(invalid_duration), amp::PlanningErrorCode::invalid_request,
                    "a non-finite duration must be rejected");

  auto nonfinite_target = nominal;
  nonfinite_target.target_world_m.x() = std::numeric_limits<double>::quiet_NaN();
  passed &=
      require_error((*planner)->plan(nonfinite_target), amp::PlanningErrorCode::invalid_request,
                    "a non-finite target must be rejected");

  auto nonfinite_start = nominal;
  nonfinite_start.q_start[2] = std::numeric_limits<double>::infinity();
  passed &=
      require_error((*planner)->plan(nonfinite_start), amp::PlanningErrorCode::invalid_request,
                    "a non-finite start configuration must be rejected");

  auto out_of_limits = nominal;
  out_of_limits.q_start[0] = 2.0;
  passed &= require_error((*planner)->plan(out_of_limits), amp::PlanningErrorCode::invalid_request,
                          "a start configuration outside a joint limit must be rejected");

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
