#pragma once

#include <string>

#include "amp/simulation.hpp"

namespace amp {

// Runs the interactive MuJoCo viewer until its window is closed.
// Returns an error string if a window or rendering context cannot be created.
std::expected<void, std::string> run_viewer(Simulation& simulation);

}  // namespace amp
