#pragma once

#include <functional>
#include <string>

#include "amp/simulation.hpp"

namespace amp {

struct ViewerOptions {
  std::string title = "SO-101 - MuJoCo";
  std::function<std::expected<void, std::string>(Simulation&)> on_reset;
  std::function<std::expected<bool, std::string>(Simulation&)> before_step;
};

// Runs the interactive MuJoCo viewer until its window is closed.
// Returns an error string if a window or rendering context cannot be created.
std::expected<void, std::string> run_viewer(Simulation& simulation, ViewerOptions options = {});

}  // namespace amp
