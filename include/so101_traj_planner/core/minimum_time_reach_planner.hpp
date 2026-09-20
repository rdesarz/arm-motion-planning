#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <memory>

#include "so101_traj_planner/core/aligator_reach_planner.hpp"

namespace so101_traj_planner {

struct MinimumTimeReachRequest {
  JointVector q_start;
  Eigen::Vector3d target_world_m;
  double minimum_duration_s;
  double maximum_duration_s;
  double duration_resolution_s = 0.02;
};

struct MinimumTimeSearchReport {
  std::size_t candidate_horizon_count;
  std::size_t evaluated_horizon_count;
  std::size_t validated_horizon_count;
  std::size_t worker_count;
  double computation_time_s;
};

struct MinimumTimeReachResult {
  JointTrajectory trajectory;
  MinimumTimeSearchReport search_report;
};

class MinimumTimeReachPlanner {
 public:
  [[nodiscard]] static std::expected<MinimumTimeReachPlanner, PlanningError> load(
      const std::filesystem::path& robot_mjcf, std::size_t worker_count = 0);

  ~MinimumTimeReachPlanner();
  MinimumTimeReachPlanner(MinimumTimeReachPlanner&&) noexcept;
  MinimumTimeReachPlanner& operator=(MinimumTimeReachPlanner&&) noexcept;
  MinimumTimeReachPlanner(const MinimumTimeReachPlanner&) = delete;
  MinimumTimeReachPlanner& operator=(const MinimumTimeReachPlanner&) = delete;

  [[nodiscard]] std::expected<MinimumTimeReachResult, PlanningError> plan(
      const MinimumTimeReachRequest& request) const;

 private:
  class Impl;
  explicit MinimumTimeReachPlanner(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace so101_traj_planner
