# SO-101 motion planning

A C++23 experiment that generates a finite-duration SO-101 joint trajectory to a target `gripperframe` position. The first reach-planner strategy uses Aligator ProxDDP and Pinocchio; optional playback tracks the resulting position references in MuJoCo.

## Requirements

- [Pixi](https://pixi.sh/) 0.80 or newer
- an internet connection for the first environment and robot-model download
- OpenGL and a graphical display only for the interactive viewer

`pixi.lock` pins the complete Apple Silicon environment, including Aligator 0.19.0, Pinocchio 4.0.0, MuJoCo 3.7.0, GLFW 3.4, CMake, Ninja, and the C++ compiler. CMake downloads only the SO-101 runtime model files and verifies them against hashes from an exact MuJoCo Menagerie revision.

## Build and test

```sh
pixi run build
pixi run test
```

CMake writes the compilation database to `build/pixi/compile_commands.json`; `.clangd` already points there.

## Generate a reach trajectory

Run the canonical reachable example:

```sh
pixi run reach-demo
```

Or provide a world-frame target in metres:

```sh
pixi run ./build/pixi/so101_reach \
  --target 0.31741606 -0.09770090 0.26459835 \
  --duration 2.0 \
  --q-start 0 0 0 0 0 0.25 \
  --csv
```

The default planner is selected through the project-owned `ReachPlanner` strategy interface. `--planner aligator` is explicit and is currently the only available implementation. A successful plan contains 101 position-and-velocity knots for the default two-second horizon and reports its terminal position error, terminal frame speed, joint-limit violation, and computation time.

The planner fails closed for invalid inputs, unavailable strategies, model mismatches, solver failure, or violated postconditions. It does not silently return an invalid last iterate.

## MuJoCo playback

```sh
pixi run playback-demo
```

Playback linearly resamples the planned joint positions at MuJoCo's timestep and reports Cartesian terminal tracking error and maximum joint tracking error. It sends position references to the current MuJoCo actuators; it does not execute Aligator's optimized torques and is not evidence of hardware readiness.

## Simulator

```sh
pixi run sim
pixi run headless
```

Viewer controls:

- left mouse: rotate the camera
- right mouse: pan the camera
- middle mouse or scroll: zoom
- `Space`: pause/resume
- `R` or `Backspace`: reset
- `Esc`: quit

To use a local copy of the pinned robot model, configure CMake explicitly:

```sh
pixi run cmake -S . -B build/pixi -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DAMP_SO101_MODEL_DIR=/path/to/mujoco_menagerie/robotstudio_so101
```

## Scope and architecture

`ReachPlanner` accepts a start joint configuration, a world-frame position target, and a duration. It returns a strategy-neutral `JointTrajectory`; Aligator and Pinocchio types remain private to the adapter. `TrajectoryPlayer` separately consumes the trajectory, so simulation does not depend on the planner.

The current optimizer uses five arm joints and locks the gripper at its initial position. It enforces experimental velocity limits, the model's 2.94 Nm actuator evidence, and model joint limits. Collision avoidance, orientation targets, automatic duration optimization, online replanning, and hardware execution are not implemented.

The full contract, acceptance thresholds, and class diagram are in [specification 001](specs/001-aligator-reach-trajectory.md).

The SO-101 model is sourced from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101) and is licensed under Apache-2.0. Its license is downloaded alongside the model assets.
