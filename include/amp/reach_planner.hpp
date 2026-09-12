#pragma once

#include <Eigen/Core>
#include <expected>
#include <filesystem>
#include <memory>
#include <string>

#include "amp/trajectory.hpp"

namespace amp {

struct ReachRequest {
  JointVector q_start;
  Eigen::Vector3d target_world_m;
  double duration_s;
};

enum class PlanningErrorCode {
  invalid_request,
  strategy_unavailable,
  model_mismatch,
  frame_missing,
  planning_failed,
  validation_failed,
};

struct PlanningError {
  PlanningErrorCode code;
  std::string message;
};

class ReachPlanner {
 public:
  virtual ~ReachPlanner() = default;

  [[nodiscard]] virtual std::expected<JointTrajectory, PlanningError> plan(
      const ReachRequest& request) const = 0;
};

class ReachPlannerFactory {
 public:
  [[nodiscard]] static std::expected<std::unique_ptr<ReachPlanner>, PlanningError> create(
      ReachPlannerKind kind, const std::filesystem::path& robot_mjcf);
};

}  // namespace amp
