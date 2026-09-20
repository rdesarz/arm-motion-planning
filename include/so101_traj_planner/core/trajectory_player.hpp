#pragma once

#include <array>
#include <expected>
#include <optional>
#include <string>

#include "so101_traj_planner/core/simulation.hpp"
#include "so101_traj_planner/core/trajectory.hpp"

namespace so101_traj_planner {

struct PlaybackReport {
  double planned_terminal_error_m;
  double simulated_terminal_tracking_error_m;
  double max_joint_tracking_error_rad;
};

class TrajectoryPlaybackController {
 public:
  [[nodiscard]] static std::expected<TrajectoryPlaybackController, std::string> create(
      const Simulation& simulation);

  [[nodiscard]] std::expected<void, std::string> start(const JointTrajectory& trajectory,
                                                       Simulation& simulation);
  [[nodiscard]] std::expected<bool, std::string> before_step(Simulation& simulation);
  void stop();

 private:
  explicit TrajectoryPlaybackController(std::array<int, kSo101JointCount> actuator_ids);

  std::array<int, kSo101JointCount> actuator_ids_;
  std::optional<JointTrajectory> trajectory_;
  double start_time_s_ = 0.0;
};

class TrajectoryPlayer {
 public:
  [[nodiscard]] static std::expected<PlaybackReport, std::string> play(
      const JointTrajectory& trajectory, Simulation& simulation);

  // Opens an interactive viewer, plays in real time, and pauses on the final pose.
  [[nodiscard]] static std::expected<void, std::string> visualize(const JointTrajectory& trajectory,
                                                                  Simulation& simulation);
};

}  // namespace so101_traj_planner
