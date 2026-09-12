#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>

#include "amp/reach_planner.hpp"
#include "amp/so101_model.hpp"

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

Eigen::Vector3d end_effector_position(const pinocchio::Model& model, pinocchio::Data& data,
                                      const amp::JointVector& configuration) {
  const Eigen::Map<const Eigen::VectorXd> q(configuration.data(), configuration.size());
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[model.getFrameId(amp::kSo101EndEffectorFrame.data())].translation();
}

bool valid_common_trajectory(const amp::JointTrajectory& trajectory,
                             const amp::ReachRequest& request) {
  constexpr std::array<double, amp::kSo101JointCount> lower_limits = {
      -1.91986, -1.7453293, -1.69, -1.658063, -2.7438473, -0.174533};
  constexpr std::array<double, amp::kSo101JointCount> upper_limits = {
      1.91986, 1.7453293, 1.69, 1.658063, 2.7438473, 1.7453292};
  bool passed = true;
  passed &= require(trajectory.knots.size() == 101, "a two-second plan must contain 101 knots");
  passed &= require(trajectory.report.strategy == amp::ReachPlannerKind::aligator,
                    "the report must identify the selected strategy");
  passed &= require(trajectory.report.final_position_error_m <= 0.005,
                    "the final position error must satisfy the planner contract");
  passed &= require(trajectory.report.final_frame_speed_mps <= 0.02,
                    "the final frame speed must satisfy the planner contract");
  passed &= require(trajectory.report.max_joint_limit_violation_rad <= 1e-6,
                    "the plan must satisfy joint limits");
  if (trajectory.knots.empty()) {
    return false;
  }

  passed &= require(std::abs(trajectory.knots.front().time_s) <= 1e-12,
                    "the first knot must start at zero");
  passed &= require(std::abs(trajectory.knots.back().time_s - request.duration_s) <= 1e-12,
                    "the final knot must end at the requested duration");
  for (std::size_t knot = 0; knot < trajectory.knots.size(); ++knot) {
    const auto& value = trajectory.knots[knot];
    if (knot > 0) {
      passed &= require(value.time_s > trajectory.knots[knot - 1].time_s,
                        "trajectory timestamps must be strictly increasing");
    }
    for (std::size_t joint = 0; joint < value.q.size(); ++joint) {
      passed &= require(std::isfinite(value.q[joint]) && std::isfinite(value.velocity[joint]),
                        "trajectory states must be finite");
      passed &= require(value.q[joint] >= lower_limits[joint] - 1e-6 &&
                            value.q[joint] <= upper_limits[joint] + 1e-6,
                        "every returned joint position must satisfy its model limit");
    }
    passed &= require(std::abs(value.q[amp::kSo101GripperIndex] -
                               request.q_start[amp::kSo101GripperIndex]) <= 1e-12,
                      "the gripper position must remain locked");
    passed &= require(std::abs(value.velocity[amp::kSo101GripperIndex]) <= 1e-12,
                      "the gripper velocity must remain zero");
  }
  for (std::size_t joint = 0; joint < request.q_start.size(); ++joint) {
    passed &= require(std::abs(trajectory.knots.front().q[joint] - request.q_start[joint]) <= 1e-9,
                      "the first knot must match q_start");
  }
  return passed;
}

}  // namespace

int main() {
  pinocchio::Model model;
  try {
    pinocchio::mjcf::buildModel(AMP_TEST_ROBOT_PATH, model, false);
  } catch (const std::exception& error) {
    std::cerr << "FAILED: could not load test model: " << error.what() << "\n";
    return EXIT_FAILURE;
  }
  pinocchio::Data data(model);

  auto planner =
      amp::ReachPlannerFactory::create(amp::ReachPlannerKind::aligator, AMP_TEST_ROBOT_PATH);
  if (!planner) {
    std::cerr << "FAILED: " << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  const amp::JointVector q_reference = {0.35, -0.45, 0.55, -0.30, 0.40, 0.25};
  const amp::ReachRequest request{
      .q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25},
      .target_world_m = end_effector_position(model, data, q_reference),
      .duration_s = 2.0,
  };
  const auto reachable = (*planner)->plan(request);
  if (!reachable) {
    std::cerr << "FAILED: reachable request was rejected: " << reachable.error().message << "\n";
    return EXIT_FAILURE;
  }
  bool passed = valid_common_trajectory(*reachable, request);

  const amp::ReachRequest no_op_request{
      .q_start = request.q_start,
      .target_world_m = end_effector_position(model, data, request.q_start),
      .duration_s = 2.0,
  };
  const auto no_op = (*planner)->plan(no_op_request);
  passed &=
      require(no_op.has_value(), "a target at the initial end-effector position must succeed");
  if (no_op) {
    passed &= valid_common_trajectory(*no_op, no_op_request);
    double maximum_displacement = 0.0;
    for (const auto& knot : no_op->knots) {
      for (std::size_t joint = 0; joint < amp::kSo101ArmJointCount; ++joint) {
        maximum_displacement =
            std::max(maximum_displacement, std::abs(knot.q[joint] - no_op_request.q_start[joint]));
      }
    }
    if (maximum_displacement > 1e-3) {
      std::cerr << "Observed no-op displacement: " << maximum_displacement << " rad\n";
    }
    passed &=
        require(maximum_displacement <= 1e-3, "a no-op target must not create visible arm motion");
  } else {
    std::cerr << "FAILED: no-op request was rejected: " << no_op.error().message << "\n";
  }

  auto unreachable_request = request;
  unreachable_request.target_world_m = Eigen::Vector3d{3.0, 3.0, 3.0};
  const auto unreachable = (*planner)->plan(unreachable_request);
  passed &= require(!unreachable, "an unreachable target must fail closed");
  if (!unreachable) {
    passed &= require(unreachable.error().code == amp::PlanningErrorCode::planning_failed ||
                          unreachable.error().code == amp::PlanningErrorCode::validation_failed,
                      "an unreachable target must return a planning or validation error");
  }

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
