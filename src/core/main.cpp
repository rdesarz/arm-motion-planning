#include <Eigen/Core>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "amp/core/aligator_reach_planner.hpp"
#include "amp/core/simulation.hpp"
#include "amp/core/trajectory_player.hpp"

namespace {

struct Options {
  std::filesystem::path robot_mjcf = AMP_DEFAULT_ROBOT_PATH;
  std::filesystem::path scene = AMP_DEFAULT_SCENE_PATH;
  amp::JointVector q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25};
  std::optional<Eigen::Vector3d> target;
  double duration_s = 2.0;
  bool playback = false;
  bool visual = false;
  bool csv = false;
};

void print_usage(const std::string_view executable) {
  std::cout << "Usage: " << executable
            << " --target X Y Z [--q-start Q0 Q1 Q2 Q3 Q4 Q5] [options]\n\n"
            << "Options:\n"
            << "  --duration SECONDS  Requested duration (default: 2.0)\n"
            << "  --robot PATH        SO-101 robot MJCF\n"
            << "  --scene PATH        MuJoCo scene used by --playback\n"
            << "  --playback          Track the joint trajectory in MuJoCo\n"
            << "  --visual            Show real-time playback (implies --playback)\n"
            << "  --csv               Print every trajectory knot as CSV\n";
}

std::expected<double, std::string> parse_number(const std::string_view text,
                                                const std::string_view option) {
  double value = 0.0;
  std::istringstream input(std::string{text});
  input.imbue(std::locale::classic());
  if (!(input >> value)) {
    return std::unexpected("invalid numeric value for " + std::string(option) + ": " +
                           std::string(text));
  }
  input >> std::ws;
  if (!input.eof()) {
    return std::unexpected("invalid numeric value for " + std::string(option) + ": " +
                           std::string(text));
  }
  return value;
}

std::expected<Options, std::string> parse_options(const int argc, char* argv[]) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument = argv[index];
    if (argument == "--target") {
      if (index + 3 >= argc) {
        return std::unexpected("--target requires three coordinates");
      }
      Eigen::Vector3d target;
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        const auto value = parse_number(argv[++index], "--target");
        if (!value) {
          return std::unexpected(value.error());
        }
        target[axis] = *value;
      }
      options.target = target;
    } else if (argument == "--q-start") {
      if (index + 6 >= argc) {
        return std::unexpected("--q-start requires six joint positions");
      }
      for (double& position : options.q_start) {
        const auto value = parse_number(argv[++index], "--q-start");
        if (!value) {
          return std::unexpected(value.error());
        }
        position = *value;
      }
    } else if (argument == "--duration") {
      if (++index >= argc) {
        return std::unexpected("--duration requires a value");
      }
      const auto value = parse_number(argv[index], "--duration");
      if (!value) {
        return std::unexpected(value.error());
      }
      options.duration_s = *value;
    } else if (argument == "--robot") {
      if (++index >= argc) {
        return std::unexpected("--robot requires a path");
      }
      options.robot_mjcf = argv[index];
    } else if (argument == "--scene") {
      if (++index >= argc) {
        return std::unexpected("--scene requires a path");
      }
      options.scene = argv[index];
    } else if (argument == "--playback") {
      options.playback = true;
    } else if (argument == "--visual") {
      options.playback = true;
      options.visual = true;
    } else if (argument == "--csv") {
      options.csv = true;
    } else if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      return std::unexpected("unknown option: " + std::string(argument));
    }
  }
  if (!options.target) {
    return std::unexpected("--target is required");
  }
  return options;
}

std::string_view error_code_name(const amp::PlanningErrorCode code) {
  switch (code) {
    case amp::PlanningErrorCode::invalid_request:
      return "invalid_request";
    case amp::PlanningErrorCode::model_mismatch:
      return "model_mismatch";
    case amp::PlanningErrorCode::frame_missing:
      return "frame_missing";
    case amp::PlanningErrorCode::planning_failed:
      return "planning_failed";
    case amp::PlanningErrorCode::validation_failed:
      return "validation_failed";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto options = parse_options(argc, argv);
  if (!options) {
    std::cerr << "Error: " << options.error() << "\n";
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  auto planner = amp::AligatorReachPlanner::load(options->robot_mjcf);
  if (!planner) {
    std::cerr << "Planning error [" << error_code_name(planner.error().code)
              << "]: " << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  const amp::ReachRequest request{
      .q_start = options->q_start,
      .target_world_m = *options->target,
      .duration_s = options->duration_s,
  };
  const auto trajectory = planner->plan(request);
  if (!trajectory) {
    std::cerr << "Planning error [" << error_code_name(trajectory.error().code)
              << "]: " << trajectory.error().message << "\n";
    return EXIT_FAILURE;
  }

  std::cout << "planner=aligator\n"
            << "knots=" << trajectory->knots.size() << "\n"
            << "duration_s=" << trajectory->knots.back().time_s << "\n"
            << "final_position_error_m=" << trajectory->report.final_position_error_m << "\n"
            << "final_frame_speed_mps=" << trajectory->report.final_frame_speed_mps << "\n"
            << "max_joint_limit_violation_rad=" << trajectory->report.max_joint_limit_violation_rad
            << "\n"
            << "computation_time_s=" << trajectory->report.computation_time_s << "\n";

  if (options->csv) {
    std::cout << "time_s,shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll,gripper\n";
    for (const auto& knot : trajectory->knots) {
      std::cout << knot.time_s;
      for (const double position : knot.q) {
        std::cout << ',' << position;
      }
      std::cout << '\n';
    }
  }

  if (options->playback) {
    auto simulation = amp::Simulation::load(options->scene);
    if (!simulation) {
      std::cerr << "Playback error: " << simulation.error() << "\n";
      return EXIT_FAILURE;
    }
    const auto playback = amp::TrajectoryPlayer::play(*trajectory, *simulation);
    if (!playback) {
      std::cerr << "Playback error: " << playback.error() << "\n";
      return EXIT_FAILURE;
    }
    std::cout
        << "simulated_terminal_tracking_error_m=" << playback->simulated_terminal_tracking_error_m
        << "\n"
        << "max_joint_tracking_error_rad=" << playback->max_joint_tracking_error_rad << "\n"
        << "playback_note=MuJoCo tracked position references; optimized torques were not sent\n";

    if (options->visual) {
      std::cout << "viewer_note=Close the window to exit; press R to replay the trajectory\n";
      if (const auto visual = amp::TrajectoryPlayer::visualize(*trajectory, *simulation); !visual) {
        std::cerr << "Visual playback error: " << visual.error() << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  return EXIT_SUCCESS;
}
