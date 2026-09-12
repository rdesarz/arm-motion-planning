#include "amp/trajectory_player.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include "amp/so101_model.hpp"

namespace amp {

namespace {

std::expected<JointVector, std::string> sample_positions(const JointTrajectory& trajectory,
                                                         const double time_s) {
  if (trajectory.knots.empty()) {
    return std::unexpected("trajectory has no knots");
  }
  if (time_s <= trajectory.knots.front().time_s) {
    return trajectory.knots.front().q;
  }
  if (time_s >= trajectory.knots.back().time_s) {
    return trajectory.knots.back().q;
  }

  const auto upper = std::upper_bound(
      trajectory.knots.begin(), trajectory.knots.end(), time_s,
      [](const double value, const TrajectoryKnot& knot) { return value < knot.time_s; });
  const auto lower = std::prev(upper);
  const double interval = upper->time_s - lower->time_s;
  if (!std::isfinite(interval) || interval <= 0.0) {
    return std::unexpected("trajectory timestamps are not strictly increasing");
  }
  const double alpha = (time_s - lower->time_s) / interval;
  JointVector positions{};
  for (std::size_t joint = 0; joint < positions.size(); ++joint) {
    positions[joint] = (1.0 - alpha) * lower->q[joint] + alpha * upper->q[joint];
  }
  return positions;
}

}  // namespace

std::expected<PlaybackReport, std::string> TrajectoryPlayer::play(const JointTrajectory& trajectory,
                                                                  Simulation& simulation) {
  if (trajectory.knots.size() < 2 || trajectory.knots.front().time_s != 0.0 ||
      !std::isfinite(trajectory.knots.back().time_s) || trajectory.knots.back().time_s <= 0.0) {
    return std::unexpected("trajectory must contain a finite, positive-duration timeline");
  }
  double previous_time = -1.0;
  const double gripper_reference = trajectory.knots.front().q[kSo101GripperIndex];
  for (const auto& knot : trajectory.knots) {
    if (!std::isfinite(knot.time_s) || knot.time_s <= previous_time) {
      return std::unexpected("trajectory timestamps must be finite and strictly increasing");
    }
    for (const double position : knot.q) {
      if (!std::isfinite(position)) {
        return std::unexpected("trajectory contains a non-finite joint position");
      }
    }
    if (std::abs(knot.q[kSo101GripperIndex] - gripper_reference) > 1e-12 ||
        std::abs(knot.velocity[kSo101GripperIndex]) > 1e-12) {
      return std::unexpected("trajectory must keep the gripper position locked");
    }
    previous_time = knot.time_s;
  }

  const auto& model = simulation.model();
  auto& data = simulation.data();
  if (model.nq != static_cast<int>(kSo101JointCount) ||
      model.nu != static_cast<int>(kSo101JointCount) || model.opt.timestep <= 0.0) {
    return std::unexpected("MuJoCo model does not satisfy the six-joint playback contract");
  }

  std::array<int, kSo101JointCount> qpos_addresses{};
  std::array<int, kSo101JointCount> dof_addresses{};
  std::array<int, kSo101JointCount> actuator_ids{};
  for (std::size_t joint = 0; joint < kSo101JointNames.size(); ++joint) {
    const auto name = std::string(kSo101JointNames[joint]);
    const int joint_id = mj_name2id(&model, mjOBJ_JOINT, name.c_str());
    const int actuator_id = mj_name2id(&model, mjOBJ_ACTUATOR, name.c_str());
    if (joint_id < 0 || actuator_id < 0) {
      return std::unexpected("MuJoCo model is missing playback mapping for " + name);
    }
    qpos_addresses[joint] = model.jnt_qposadr[joint_id];
    dof_addresses[joint] = model.jnt_dofadr[joint_id];
    actuator_ids[joint] = actuator_id;
  }

  const int site_id = mj_name2id(&model, mjOBJ_SITE, kSo101EndEffectorFrame.data());
  if (site_id < 0) {
    return std::unexpected("MuJoCo model is missing gripperframe");
  }

  simulation.reset();
  for (std::size_t joint = 0; joint < kSo101JointNames.size(); ++joint) {
    const double initial_position = trajectory.knots.front().q[joint];
    if (!std::isfinite(initial_position)) {
      return std::unexpected("trajectory contains a non-finite initial position");
    }
    data.qpos[qpos_addresses[joint]] = initial_position;
    data.ctrl[actuator_ids[joint]] = initial_position;
  }
  mj_forward(&model, &data);

  double maximum_joint_tracking_error = 0.0;
  const double duration = trajectory.knots.back().time_s;
  while (data.time < duration) {
    const double previous_simulation_time = data.time;
    const double command_time = std::min(duration, data.time + model.opt.timestep);
    const auto command = sample_positions(trajectory, command_time);
    if (!command) {
      return std::unexpected(command.error());
    }
    for (std::size_t joint = 0; joint < kSo101JointNames.size(); ++joint) {
      if (!std::isfinite((*command)[joint])) {
        return std::unexpected("trajectory interpolation produced a non-finite command");
      }
      data.ctrl[actuator_ids[joint]] = (*command)[joint];
    }

    simulation.step();
    if (!std::isfinite(data.time) || data.time <= previous_simulation_time) {
      return std::unexpected("MuJoCo playback time did not advance");
    }
    const auto reference = sample_positions(trajectory, std::min(data.time, duration));
    if (!reference) {
      return std::unexpected(reference.error());
    }
    for (std::size_t joint = 0; joint < kSo101JointNames.size(); ++joint) {
      const double actual = data.qpos[qpos_addresses[joint]];
      if (!std::isfinite(actual) || !std::isfinite(data.qvel[dof_addresses[joint]])) {
        return std::unexpected("MuJoCo playback produced a non-finite state");
      }
      maximum_joint_tracking_error =
          std::max(maximum_joint_tracking_error, std::abs(actual - (*reference)[joint]));
    }
  }

  auto reference_data =
      std::unique_ptr<mjData, decltype(&mj_deleteData)>(mj_makeData(&model), &mj_deleteData);
  if (!reference_data) {
    return std::unexpected("MuJoCo could not allocate terminal reference data");
  }
  for (std::size_t joint = 0; joint < kSo101JointNames.size(); ++joint) {
    reference_data->qpos[qpos_addresses[joint]] = trajectory.knots.back().q[joint];
  }
  mj_forward(&model, reference_data.get());

  double terminal_tracking_error_squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double difference =
        data.site_xpos[3 * site_id + axis] - reference_data->site_xpos[3 * site_id + axis];
    terminal_tracking_error_squared += difference * difference;
  }

  return PlaybackReport{
      .planned_terminal_error_m = trajectory.report.final_position_error_m,
      .simulated_terminal_tracking_error_m = std::sqrt(terminal_tracking_error_squared),
      .max_joint_tracking_error_rad = maximum_joint_tracking_error,
  };
}

}  // namespace amp
