#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <string_view>

#include "amp/simulation.hpp"

namespace {

constexpr std::array<std::string_view, 6> kJointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};

bool require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
  }
  return condition;
}

bool check_configuration(amp::Simulation& simulation, const pinocchio::Model& pin_model,
                         pinocchio::Data& pin_data, const Eigen::VectorXd& q) {
  auto& mj_model = simulation.model();
  auto& mj_data = simulation.data();

  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const auto name = std::string(kJointNames[index]);
    const int mj_joint = mj_name2id(&mj_model, mjOBJ_JOINT, name.c_str());
    if (mj_joint < 0) {
      return false;
    }
    mj_data.qpos[mj_model.jnt_qposadr[mj_joint]] = q[static_cast<Eigen::Index>(index)];
  }
  mj_forward(&mj_model, &mj_data);

  pinocchio::forwardKinematics(pin_model, pin_data, q);
  pinocchio::updateFramePlacements(pin_model, pin_data);

  const int mj_site = mj_name2id(&mj_model, mjOBJ_SITE, "gripperframe");
  const auto pin_frame = pin_model.getFrameId("gripperframe");
  if (mj_site < 0 || pin_frame >= pin_model.nframes) {
    return false;
  }

  const Eigen::Map<const Eigen::Vector3d> mj_position(&mj_data.site_xpos[3 * mj_site]);
  const Eigen::Vector3d pin_position = pin_data.oMf[pin_frame].translation();
  return (mj_position - pin_position).norm() <= 1e-9;
}

}  // namespace

int main() {
  bool passed = true;

  auto simulation = amp::Simulation::load(AMP_TEST_SCENE_PATH);
  passed &= require(simulation.has_value(), "MuJoCo must load the pinned SO-101 scene");
  if (!simulation) {
    std::cerr << simulation.error() << "\n";
    return EXIT_FAILURE;
  }

  pinocchio::Model pin_model;
  try {
    pinocchio::mjcf::buildModel(AMP_TEST_ROBOT_PATH, pin_model, false);
  } catch (const std::exception& error) {
    std::cerr << "FAILED: Pinocchio could not load the pinned SO-101 MJCF: " << error.what()
              << "\n";
    return EXIT_FAILURE;
  }

  passed &= require(pin_model.nq == 6, "Pinocchio must expose six generalized coordinates");
  passed &= require(pin_model.nv == 6, "Pinocchio must expose six velocities");
  passed &= require(pin_model.existFrame("gripperframe"),
                    "Pinocchio must expose the MuJoCo gripperframe site as a frame");

  const auto& mj_model = simulation->model();
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    const auto name = std::string(kJointNames[index]);
    const int mj_joint = mj_name2id(&mj_model, mjOBJ_JOINT, name.c_str());
    passed &= require(mj_joint >= 0, "MuJoCo joint mapping must be complete");
    passed &= require(pin_model.existJointName(name), "Pinocchio joint mapping must be complete");
    if (mj_joint < 0 || !pin_model.existJointName(name)) {
      continue;
    }

    const auto pin_joint = pin_model.getJointId(name);
    passed &= require(pin_model.idx_qs[pin_joint] == static_cast<int>(index),
                      "Pinocchio joint order must match the canonical trajectory order");
    passed &= require(mj_model.jnt_qposadr[mj_joint] == static_cast<int>(index),
                      "MuJoCo joint order must match the canonical trajectory order");
    passed &=
        require(std::abs(mj_model.jnt_range[2 * mj_joint] -
                         pin_model.lowerPositionLimit[static_cast<Eigen::Index>(index)]) <= 1e-9,
                "lower joint limits must match across engines");
    passed &=
        require(std::abs(mj_model.jnt_range[2 * mj_joint + 1] -
                         pin_model.upperPositionLimit[static_cast<Eigen::Index>(index)]) <= 1e-9,
                "upper joint limits must match across engines");
  }

  pinocchio::Data pin_data(pin_model);
  passed &= require(check_configuration(*simulation, pin_model, pin_data, Eigen::VectorXd::Zero(6)),
                    "neutral gripperframe FK must match across engines");

  Eigen::VectorXd nonzero(6);
  nonzero << 0.35, -0.45, 0.55, -0.30, 0.40, 0.25;
  passed &= require(check_configuration(*simulation, pin_model, pin_data, nonzero),
                    "nonzero gripperframe FK must match across engines");

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
