#pragma once

// std
#include <array>
#include <cstddef>
#include <string_view>

namespace so101_traj_planner {

inline constexpr std::size_t kSo101JointCount = 6;
inline constexpr std::size_t kSo101ArmJointCount = 5;
inline constexpr std::size_t kSo101GripperIndex = 5;
inline constexpr double kSo101ExperimentalVelocityLimitRadS = 4.0;
inline constexpr double kSo101ActuatorEffortLimitNm = 2.94;
inline constexpr std::array<std::string_view, kSo101JointCount> kSo101JointNames = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
inline constexpr std::string_view kSo101EndEffectorFrame = "gripperframe";

}  // namespace so101_traj_planner
