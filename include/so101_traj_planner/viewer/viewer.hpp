#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "so101_traj_planner/core/simulation.hpp"

namespace so101_traj_planner {

using WorldPoint = std::array<double, 3>;

struct ViewerOptions {
  std::string title = "SO-101 - MuJoCo";
  std::function<std::expected<void, std::string>(Simulation&)> on_reset;
  std::function<std::expected<bool, std::string>(Simulation&)> before_step;
  std::function<std::expected<void, std::string>(Simulation&, const WorldPoint&)>
      on_target_submitted;
  std::optional<WorldPoint> target_editor_initial;
  bool start_paused = false;
  bool enable_target_editor = false;
};

struct ViewerResult {
  std::optional<WorldPoint> selected_world_point;
};

// Runs the interactive MuJoCo viewer until its window is closed.
// Returns an error string if a window or rendering context cannot be created.
// The target editor previews XYZ changes immediately. If on_target_submitted is set, sending a
// target invokes it without closing the viewer.
std::expected<ViewerResult, std::string> run_viewer(Simulation& simulation,
                                                    ViewerOptions options = {});

}  // namespace so101_traj_planner
