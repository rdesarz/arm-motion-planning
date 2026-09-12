# SO-101 motion planning

A C++23 workspace for developing motion planners and controllers for the SO-101 arm. The first executable loads the curated SO-101 model from MuJoCo Menagerie and runs it either in an interactive MuJoCo viewer or headlessly.

## Requirements

- CMake 3.24 or newer
- Conan 2.1 or newer
- A compiler with C++23 support
- An internet connection for the first dependency installation
- OpenGL and a graphical display for the interactive viewer

Conan installs the pinned MuJoCo 3.7.0 and GLFW 3.4 packages. CMake downloads only the SO-101 runtime model files—not the complete Menagerie repository—and verifies them against hashes from an exact Menagerie revision.

## Build and run

```sh
# Required only when Conan has no default profile yet:
conan profile detect

conan install . --lockfile=conan.lock --build=missing \
  -s build_type=Release \
  -s compiler.cppstd=23
cmake --preset conan-release
cmake --build --preset conan-release -j
./build/Release/so101_sim
```

## clangd

CMake generates `build/Release/compile_commands.json`, containing the real C++23 compiler flags and Conan dependency paths. The repository's `.clangd` file directs clangd to that database, so editors require no machine-specific include-path configuration.

After changing CMake targets or dependencies, refresh the database with:

```sh
conan install . --lockfile=conan.lock --build=missing \
  -s build_type=Release \
  -s compiler.cppstd=23
cmake --preset conan-release
```

Then restart clangd from VS Code's command palette if it does not reload the database automatically.

Viewer controls:

- left mouse: rotate the camera
- right mouse: pan the camera
- middle mouse or scroll: zoom
- `Space`: pause/resume
- `R` or `Backspace`: reset
- `Esc`: quit

Run without creating a window:

```sh
./build/Release/so101_sim --headless --steps 1000
ctest --preset conan-release --output-on-failure
```

Pass another MJCF scene as the final argument when needed:

```sh
./build/Release/so101_sim path/to/scene.xml
```

For an offline build, first ensure Conan's package cache is populated and point CMake at an existing `robotstudio_so101` directory:

```sh
conan install . --lockfile=conan.lock --no-remote \
  -s build_type=Release \
  -s compiler.cppstd=23
cmake --preset conan-release \
  -DAMP_SO101_MODEL_DIR=/path/to/mujoco_menagerie/robotstudio_so101
```

## Current scope

This establishes model loading, deterministic stepping, actuator-control access, reset behavior, basic rendering, and a headless test. It does not yet implement a motion planner, controller, collision-query API, or claim simulation-to-hardware fidelity. Those should be added as separate layers over `amp::Simulation`.

The SO-101 model is sourced from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101) and is licensed under Apache-2.0. Its license is downloaded alongside the model assets.
