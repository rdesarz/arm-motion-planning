#pragma once

#include <Eigen/Core>
#include <expected>
#include <filesystem>
#include <memory>
#include <string>

#include "so101_traj_planner/core/trajectory.hpp"

namespace so101_traj_planner {

struct ReachRequest {
  JointVector q_start;
  Eigen::Vector3d target_world_m;
  double duration_s;
};

enum class PlanningErrorCode {
  invalid_request,
  model_mismatch,
  frame_missing,
  planning_failed,
  validation_failed,
};

struct PlanningError {
  PlanningErrorCode code;
  std::string message;
};

class AligatorReachPlanner {
 public:
  [[nodiscard]] static std::expected<AligatorReachPlanner, PlanningError> load(
      const std::filesystem::path& robot_mjcf);

  ~AligatorReachPlanner();
  AligatorReachPlanner(AligatorReachPlanner&&) noexcept;
  AligatorReachPlanner& operator=(AligatorReachPlanner&&) noexcept;
  AligatorReachPlanner(const AligatorReachPlanner&) = delete;
  AligatorReachPlanner& operator=(const AligatorReachPlanner&) = delete;

  [[nodiscard]] std::expected<JointTrajectory, PlanningError> plan(
      const ReachRequest& request) const;

 private:
  class Impl;
  explicit AligatorReachPlanner(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace so101_traj_planner
