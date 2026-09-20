#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "so101_traj_planner/core/aligator_reach_planner.hpp"
#include "so101_traj_planner/core/simulation.hpp"
#include "so101_traj_planner/core/so101_model.hpp"
#include "so101_traj_planner/core/trajectory_player.hpp"
#include "so101_traj_planner/viewer/viewer.hpp"

namespace {

struct Options {
  std::filesystem::path robot_mjcf = AMP_DEFAULT_ROBOT_PATH;
  std::filesystem::path scene = AMP_DEFAULT_SCENE_PATH;
  so101_traj_planner::JointVector q_start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.25};
  std::optional<Eigen::Vector3d> target;
  double duration_s = 2.0;
  bool playback = false;
  bool visual = false;
  bool csv = false;
  bool pick_target = false;
};

void print_usage(const std::string_view executable) {
  std::cout << "Usage: " << executable
            << " (--target X Y Z | --pick-target)"
               " [--q-start Q0 Q1 Q2 Q3 Q4 Q5] [options]\n\n"
            << "Options:\n"
            << "  --duration SECONDS  Requested duration (default: 2.0)\n"
            << "  --robot PATH        SO-101 robot MJCF\n"
            << "  --scene PATH        MuJoCo scene used for picking and playback\n"
            << "  --pick-target       Open the persistent MuJoCo target editor\n"
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
    } else if (argument == "--pick-target") {
      options.pick_target = true;
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
  if (options.target.has_value() == options.pick_target) {
    return std::unexpected("provide exactly one of --target or --pick-target");
  }
  return options;
}

std::expected<void, std::string> set_joint_configuration(
    so101_traj_planner::Simulation& simulation,
    const so101_traj_planner::JointVector& configuration) {
  simulation.reset();
  auto& model = simulation.model();
  auto& data = simulation.data();
  for (std::size_t index = 0; index < so101_traj_planner::kSo101JointNames.size(); ++index) {
    const std::string name{so101_traj_planner::kSo101JointNames[index]};
    const int joint_id = mj_name2id(&model, mjOBJ_JOINT, name.c_str());
    const int actuator_id = mj_name2id(&model, mjOBJ_ACTUATOR, name.c_str());
    if (joint_id < 0 || actuator_id < 0) {
      return std::unexpected("MuJoCo scene is missing joint or actuator " + name);
    }
    data.qpos[model.jnt_qposadr[joint_id]] = configuration[index];
    data.ctrl[actuator_id] = configuration[index];
  }
  mj_forward(&model, &data);
  return {};
}

std::expected<so101_traj_planner::JointVector, std::string> current_joint_configuration(
    const so101_traj_planner::Simulation& simulation) {
  so101_traj_planner::JointVector configuration{};
  const auto& model = simulation.model();
  const auto& data = simulation.data();
  for (std::size_t index = 0; index < so101_traj_planner::kSo101JointNames.size(); ++index) {
    const std::string name{so101_traj_planner::kSo101JointNames[index]};
    const int joint_id = mj_name2id(&model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      return std::unexpected("MuJoCo scene is missing joint " + name);
    }
    configuration[index] = data.qpos[model.jnt_qposadr[joint_id]];
  }
  return configuration;
}

void print_plan_report(const so101_traj_planner::JointTrajectory& trajectory) {
  std::cout << "planner=aligator\n"
            << "knots=" << trajectory.knots.size() << "\n"
            << "duration_s=" << trajectory.knots.back().time_s << "\n"
            << "final_position_error_m=" << trajectory.report.final_position_error_m << "\n"
            << "final_frame_speed_mps=" << trajectory.report.final_frame_speed_mps << "\n"
            << "max_terminal_joint_velocity_rad_s="
            << trajectory.report.max_terminal_joint_velocity_rad_s << "\n"
            << "max_joint_limit_violation_rad=" << trajectory.report.max_joint_limit_violation_rad
            << "\n"
            << "computation_time_s=" << trajectory.report.computation_time_s << "\n";
}

std::expected<void, std::string> run_target_editor(
    const std::filesystem::path& scene_path, const so101_traj_planner::JointVector& q_start,
    const double duration_s, const so101_traj_planner::AligatorReachPlanner& planner) {
  auto simulation = so101_traj_planner::Simulation::load(scene_path);
  if (!simulation) {
    return std::unexpected(simulation.error());
  }
  if (const auto configured = set_joint_configuration(*simulation, q_start); !configured) {
    return std::unexpected(configured.error());
  }
  const int end_effector_id = mj_name2id(&simulation->model(), mjOBJ_SITE,
                                         so101_traj_planner::kSo101EndEffectorFrame.data());
  if (end_effector_id < 0) {
    return std::unexpected("MuJoCo scene is missing gripperframe");
  }
  const so101_traj_planner::WorldPoint initial_target{
      simulation->data().site_xpos[3 * end_effector_id],
      simulation->data().site_xpos[3 * end_effector_id + 1],
      simulation->data().site_xpos[3 * end_effector_id + 2],
  };

  auto controller = so101_traj_planner::TrajectoryPlaybackController::create(*simulation);
  if (!controller) {
    return std::unexpected(controller.error());
  }
  auto playback_controller = std::move(*controller);

  so101_traj_planner::ViewerOptions viewer_options{
      .title = "SO-101 - Reach Target Editor",
      .on_reset =
          [q_start, &playback_controller](so101_traj_planner::Simulation& current) {
            playback_controller.stop();
            return set_joint_configuration(current, q_start);
          },
      .before_step =
          [&playback_controller](so101_traj_planner::Simulation& current) {
            return playback_controller.before_step(current);
          },
      .on_target_submitted =
          [&planner, &playback_controller, duration_s](
              so101_traj_planner::Simulation& current,
              const so101_traj_planner::WorldPoint& point) -> std::expected<void, std::string> {
        const auto current_q = current_joint_configuration(current);
        if (!current_q) {
          return std::unexpected(current_q.error());
        }
        const so101_traj_planner::ReachRequest request{
            .q_start = *current_q,
            .target_world_m = Eigen::Vector3d{point[0], point[1], point[2]},
            .duration_s = duration_s,
        };
        const auto trajectory = planner.plan(request);
        if (!trajectory) {
          return std::unexpected(trajectory.error().message);
        }
        if (const auto started = playback_controller.start(*trajectory, current); !started) {
          return std::unexpected(started.error());
        }
        std::cout << "selected_target_world_m=" << point[0] << ',' << point[1] << ',' << point[2]
                  << "\n";
        print_plan_report(*trajectory);
        return {};
      },
      .target_editor_initial = initial_target,
      .start_paused = true,
      .enable_target_editor = true,
  };
  const auto selection = so101_traj_planner::run_viewer(*simulation, std::move(viewer_options));
  if (!selection) {
    return std::unexpected(selection.error());
  }
  return {};
}

std::string_view error_code_name(const so101_traj_planner::PlanningErrorCode code) {
  switch (code) {
    case so101_traj_planner::PlanningErrorCode::invalid_request:
      return "invalid_request";
    case so101_traj_planner::PlanningErrorCode::model_mismatch:
      return "model_mismatch";
    case so101_traj_planner::PlanningErrorCode::frame_missing:
      return "frame_missing";
    case so101_traj_planner::PlanningErrorCode::planning_failed:
      return "planning_failed";
    case so101_traj_planner::PlanningErrorCode::validation_failed:
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

  auto planner = so101_traj_planner::AligatorReachPlanner::load(options->robot_mjcf);
  if (!planner) {
    std::cerr << "Planning error [" << error_code_name(planner.error().code)
              << "]: " << planner.error().message << "\n";
    return EXIT_FAILURE;
  }

  if (options->pick_target) {
    const auto editor =
        run_target_editor(options->scene, options->q_start, options->duration_s, *planner);
    if (!editor) {
      std::cerr << "Target editor error: " << editor.error() << "\n";
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }

  const so101_traj_planner::ReachRequest request{
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

  print_plan_report(*trajectory);

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
    auto simulation = so101_traj_planner::Simulation::load(options->scene);
    if (!simulation) {
      std::cerr << "Playback error: " << simulation.error() << "\n";
      return EXIT_FAILURE;
    }
    const auto playback = so101_traj_planner::TrajectoryPlayer::play(*trajectory, *simulation);
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
      if (const auto visual =
              so101_traj_planner::TrajectoryPlayer::visualize(*trajectory, *simulation);
          !visual) {
        std::cerr << "Visual playback error: " << visual.error() << "\n";
        return EXIT_FAILURE;
      }
    }
  }

  return EXIT_SUCCESS;
}
