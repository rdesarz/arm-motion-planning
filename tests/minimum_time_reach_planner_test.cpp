#include "so101_traj_planner/core/minimum_time_reach_planner.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>

#include "so101_traj_planner/core/so101_model.hpp"

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

Eigen::Vector3d end_effector_position(const so101_traj_planner::JointVector& configuration) {
  pinocchio::Model model;
  pinocchio::mjcf::buildModel(AMP_TEST_ROBOT_PATH, model, false);
  pinocchio::Data data(model);
  const Eigen::Map<const Eigen::VectorXd> q(configuration.data(), configuration.size());
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[model.getFrameId(so101_traj_planner::kSo101EndEffectorFrame.data())]
      .translation();
}

}  // namespace

int main() {
  const auto excessive_workers =
      so101_traj_planner::MinimumTimeReachPlanner::load(AMP_TEST_ROBOT_PATH, 33);
  bool passed =
      require(!excessive_workers && excessive_workers.error().code ==
                                        so101_traj_planner::PlanningErrorCode::invalid_request,
              "an excessive worker count must fail closed");

  auto planner = so101_traj_planner::MinimumTimeReachPlanner::load(AMP_TEST_ROBOT_PATH, 2);
  if (!planner) {
    std::cerr << "FAILED: " << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  const so101_traj_planner::JointVector q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25};
  const so101_traj_planner::MinimumTimeReachRequest request{
      .q_start = q_start,
      .target_world_m = end_effector_position(q_start),
      .minimum_duration_s = 0.04,
      .maximum_duration_s = 0.08,
      .duration_resolution_s = 0.02,
  };

  const auto result = planner->plan(request);
  passed &= require(result.has_value(), "the parallel search must find the no-op trajectory");
  if (result) {
    passed &= require(std::abs(result->trajectory.knots.back().time_s - 0.04) <= 1e-12,
                      "the search must return the shortest validated duration");
    passed &= require(result->trajectory.report.max_terminal_joint_velocity_rad_s <= 1e-3,
                      "the selected trajectory must stop with zero terminal joint velocity");
    passed &= require(result->search_report.candidate_horizon_count == 3,
                      "the inclusive duration grid must contain three candidates");
    passed &= require(result->search_report.worker_count == 2,
                      "the requested two-worker search must run with two workers");
    passed &= require(result->search_report.evaluated_horizon_count >= 1 &&
                          result->search_report.evaluated_horizon_count <= 3,
                      "early stopping must evaluate a bounded number of candidates");
    passed &= require(result->search_report.validated_horizon_count >= 1,
                      "at least one horizon must be independently validated");
  } else {
    std::cerr << result.error().message << "\n";
  }

  const so101_traj_planner::MinimumTimeReachRequest moving_request{
      .q_start = q_start,
      .target_world_m = Eigen::Vector3d{0.31741606, -0.09770090, 0.26459835},
      .minimum_duration_s = 1.6,
      .maximum_duration_s = 2.0,
      .duration_resolution_s = 0.2,
  };
  const auto moving_result = planner->plan(moving_request);
  passed &= require(moving_result.has_value(), "the search must find a moving trajectory");
  if (moving_result) {
    passed &= require(std::abs(moving_result->trajectory.knots.back().time_s - 1.8) <= 1e-12,
                      "the search must reject the shorter non-stopping horizon");
    passed &=
        require(moving_result->trajectory.report.max_terminal_joint_velocity_rad_s <= 1e-3,
                "the minimum-time moving trajectory must stop with zero terminal joint velocity");
  } else {
    std::cerr << moving_result.error().message << "\n";
  }

  auto invalid = request;
  invalid.duration_resolution_s = 0.0;
  const auto invalid_result = planner->plan(invalid);
  passed &= require(!invalid_result && invalid_result.error().code ==
                                           so101_traj_planner::PlanningErrorCode::invalid_request,
                    "a zero search resolution must be rejected");

  invalid = request;
  invalid.maximum_duration_s = std::numeric_limits<double>::infinity();
  const auto infinite_result = planner->plan(invalid);
  passed &= require(!infinite_result && infinite_result.error().code ==
                                            so101_traj_planner::PlanningErrorCode::invalid_request,
                    "a non-finite maximum duration must be rejected");

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
