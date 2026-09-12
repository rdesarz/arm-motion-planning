#include "amp/trajectory_player.hpp"

#include <Eigen/Core>
#include <cstdlib>
#include <iostream>

#include "amp/reach_planner.hpp"
#include "amp/simulation.hpp"

namespace {

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

}  // namespace

int main() {
  auto planner =
      amp::ReachPlannerFactory::create(amp::ReachPlannerKind::aligator, AMP_TEST_ROBOT_PATH);
  if (!planner) {
    std::cerr << "FAILED: " << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  const amp::ReachRequest request{
      .q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25},
      .target_world_m = Eigen::Vector3d{0.31741606, -0.09770090, 0.26459835},
      .duration_s = 2.0,
  };
  const auto trajectory = (*planner)->plan(request);
  if (!trajectory) {
    std::cerr << "FAILED: " << trajectory.error().message << "\n";
    return EXIT_FAILURE;
  }

  auto simulation = amp::Simulation::load(AMP_TEST_SCENE_PATH);
  if (!simulation) {
    std::cerr << "FAILED: " << simulation.error() << "\n";
    return EXIT_FAILURE;
  }

  const auto playback = amp::TrajectoryPlayer::play(*trajectory, *simulation);
  if (!playback) {
    std::cerr << "FAILED: " << playback.error() << "\n";
    return EXIT_FAILURE;
  }

  bool passed = true;
  passed &= require(playback->planned_terminal_error_m == trajectory->report.final_position_error_m,
                    "playback must preserve the planner's terminal error metric");
  passed &= require(std::isfinite(playback->simulated_terminal_tracking_error_m),
                    "MuJoCo terminal tracking error must be finite");
  passed &= require(std::isfinite(playback->max_joint_tracking_error_rad),
                    "maximum joint tracking error must be finite");
  passed &= require(simulation->data().time >= request.duration_s,
                    "playback must cover the complete trajectory duration");
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
