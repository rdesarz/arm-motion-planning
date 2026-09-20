#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "target_editor.hpp"

int main() {
  const std::array<mjtNum, 3> values{0.31, -0.08, 0.27};
  const auto target = so101_traj_planner::viewer_detail::validated_target(values);
  if (!target || std::abs((*target)[0] - values[0]) > 1e-12 ||
      std::abs((*target)[1] - values[1]) > 1e-12 || std::abs((*target)[2] - values[2]) > 1e-12) {
    std::cerr << "FAILED: finite XYZ editor values were not preserved\n";
    return EXIT_FAILURE;
  }

  auto invalid = values;
  invalid[1] = std::numeric_limits<mjtNum>::quiet_NaN();
  if (so101_traj_planner::viewer_detail::validated_target(invalid)) {
    std::cerr << "FAILED: a non-finite GUI target was accepted\n";
    return EXIT_FAILURE;
  }

  const auto coarse = so101_traj_planner::viewer_detail::nudged_target(values, 0, -0.01);
  if (!coarse || std::abs((*coarse)[0] - 0.30) > 1e-12 || (*coarse)[1] != values[1] ||
      (*coarse)[2] != values[2]) {
    std::cerr << "FAILED: the X decrement button did not apply a 1 cm step\n";
    return EXIT_FAILURE;
  }

  const auto fine = so101_traj_planner::viewer_detail::nudged_target(values, 2, 0.001);
  if (!fine || std::abs((*fine)[2] - 0.271) > 1e-12 || (*fine)[0] != values[0] ||
      (*fine)[1] != values[1]) {
    std::cerr << "FAILED: the shifted Z increment did not apply a 1 mm step\n";
    return EXIT_FAILURE;
  }

  if (so101_traj_planner::viewer_detail::nudged_target(values, values.size(), 0.01)) {
    std::cerr << "FAILED: an invalid target axis was accepted\n";
    return EXIT_FAILURE;
  }

  const auto preview =
      so101_traj_planner::viewer_detail::preview_target(values, std::size_t{1}, "-0.125");
  if (!preview || std::abs((*preview)[1] + 0.125) > 1e-12 || (*preview)[0] != values[0] ||
      (*preview)[2] != values[2]) {
    std::cerr << "FAILED: valid text being edited did not update the target preview\n";
    return EXIT_FAILURE;
  }
  if (so101_traj_planner::viewer_detail::preview_target(values, std::size_t{1}, "-")) {
    std::cerr << "FAILED: incomplete editor text was accepted as a target preview\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
