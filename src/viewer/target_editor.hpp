#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <string_view>

#include "so101_traj_planner/viewer/viewer.hpp"

namespace so101_traj_planner::viewer_detail {

std::expected<WorldPoint, std::string> validated_target(const std::array<mjtNum, 3>& values);
std::expected<std::array<mjtNum, 3>, std::string> nudged_target(std::array<mjtNum, 3> values,
                                                                std::size_t axis, mjtNum delta);
std::expected<WorldPoint, std::string> preview_target(std::array<mjtNum, 3> values,
                                                      std::optional<std::size_t> editing_axis,
                                                      std::string_view edit_text);

}  // namespace so101_traj_planner::viewer_detail
