#pragma once

#include <expected>
#include <string>

#include "amp/simulation.hpp"
#include "amp/trajectory.hpp"

namespace amp {

struct PlaybackReport {
  double planned_terminal_error_m;
  double simulated_terminal_tracking_error_m;
  double max_joint_tracking_error_rad;
};

class TrajectoryPlayer {
 public:
  [[nodiscard]] static std::expected<PlaybackReport, std::string> play(
      const JointTrajectory& trajectory, Simulation& simulation);

  // Opens an interactive viewer, plays in real time, and pauses on the final pose.
  [[nodiscard]] static std::expected<void, std::string> visualize(const JointTrajectory& trajectory,
                                                                  Simulation& simulation);
};

}  // namespace amp
