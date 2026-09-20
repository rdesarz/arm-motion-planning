cmake_minimum_required(VERSION 3.24)

project(
  arm_motion_planning
  VERSION 0.1.0
  DESCRIPTION "Motion planning and control experiments for the SO-101 arm"
  LANGUAGES C CXX
)

if(POLICY CMP0167)
  set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AMP_BUILD_TESTS "Build the headless simulation tests" ON)
set(AMP_SO101_MODEL_DIR "" CACHE PATH "Path to a local robotstudio_so101 model directory")

find_package(mujoco 3.1.3 CONFIG REQUIRED)
find_package(glfw3 3.4 CONFIG REQUIRED)
find_package(pinocchio 4 CONFIG REQUIRED)
find_package(aligator 0.19 CONFIG REQUIRED)

if(TARGET glfw)
  set(AMP_GLFW_TARGET glfw)
elseif(TARGET glfw3::glfw)
  set(AMP_GLFW_TARGET glfw3::glfw)
else()
  message(FATAL_ERROR "The glfw3 package did not define a supported CMake target")
endif()

if(AMP_SO101_MODEL_DIR)
  cmake_path(ABSOLUTE_PATH AMP_SO101_MODEL_DIR NORMALIZE OUTPUT_VARIABLE AMP_SO101_MODEL_PATH)
else()
  include(cmake/FetchSo101Model.cmake)
  amp_fetch_so101_model(AMP_SO101_MODEL_PATH)
endif()

set(AMP_SO101_SCENE "${AMP_SO101_MODEL_PATH}/scene.xml")
if(NOT EXISTS "${AMP_SO101_SCENE}")
  message(FATAL_ERROR "SO-101 scene not found: ${AMP_SO101_SCENE}")
endif()

add_library(amp_simulation src/core/simulation.cpp)
add_library(amp::simulation ALIAS amp_simulation)
target_include_directories(amp_simulation PUBLIC include)
target_link_libraries(amp_simulation PUBLIC mujoco::mujoco)
target_compile_features(amp_simulation PUBLIC cxx_std_23)

add_library(amp_viewer src/viewer/viewer.cpp)
add_library(amp::viewer ALIAS amp_viewer)
target_include_directories(amp_viewer PUBLIC include)
target_link_libraries(amp_viewer PUBLIC amp::simulation PRIVATE ${AMP_GLFW_TARGET})
target_compile_features(amp_viewer PUBLIC cxx_std_23)

add_library(amp_planning
            src/core/aligator_reach_planner.cpp
            src/core/aligator_validation.cpp)
add_library(amp::planning ALIAS amp_planning)
target_include_directories(amp_planning PUBLIC include)
target_link_libraries(amp_planning
                      PUBLIC Eigen3::Eigen
                      PRIVATE aligator::aligator pinocchio::pinocchio_parsers)
target_compile_features(amp_planning PUBLIC cxx_std_23)

add_library(amp_playback src/core/trajectory_player.cpp)
add_library(amp::playback ALIAS amp_playback)
target_include_directories(amp_playback PUBLIC include)
target_link_libraries(amp_playback PUBLIC amp::simulation PRIVATE amp::viewer)
target_compile_features(amp_playback PUBLIC cxx_std_23)

add_executable(so101_sim src/viewer/main.cpp)
target_link_libraries(so101_sim PRIVATE amp::viewer)
target_compile_features(so101_sim PRIVATE cxx_std_23)
target_compile_definitions(so101_sim PRIVATE AMP_DEFAULT_SCENE_PATH="${AMP_SO101_SCENE}")

add_executable(so101_reach src/core/main.cpp)
target_link_libraries(so101_reach PRIVATE amp::planning amp::playback amp::simulation)
target_compile_features(so101_reach PRIVATE cxx_std_23)
target_compile_definitions(
  so101_reach
  PRIVATE
    AMP_DEFAULT_SCENE_PATH="${AMP_SO101_SCENE}"
    AMP_DEFAULT_ROBOT_PATH="${AMP_SO101_MODEL_PATH}/so101.xml"
)

if(MSVC)
  target_compile_options(amp_simulation PRIVATE /W4 /permissive-)
  target_compile_options(amp_viewer PRIVATE /W4 /permissive-)
  target_compile_options(amp_planning PRIVATE /W4 /permissive-)
  target_compile_options(amp_playback PRIVATE /W4 /permissive-)
  target_compile_options(so101_sim PRIVATE /W4 /permissive-)
  target_compile_options(so101_reach PRIVATE /W4 /permissive-)
else()
  target_compile_options(amp_simulation PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(amp_viewer PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(amp_planning PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(amp_playback PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(so101_sim PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(so101_reach PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(AMP_BUILD_TESTS)
  enable_testing()
  add_executable(simulation_test tests/simulation_test.cpp)
  target_link_libraries(simulation_test PRIVATE amp::simulation)
  target_compile_features(simulation_test PRIVATE cxx_std_23)
  target_compile_definitions(simulation_test PRIVATE AMP_TEST_SCENE_PATH="${AMP_SO101_SCENE}")
  add_test(NAME simulation_loads_and_steps COMMAND simulation_test)

  add_executable(model_parity_test tests/model_parity_test.cpp)
  target_link_libraries(model_parity_test PRIVATE amp::simulation pinocchio::pinocchio_parsers)
  target_compile_features(model_parity_test PRIVATE cxx_std_23)
  target_compile_definitions(
    model_parity_test
    PRIVATE
      AMP_TEST_SCENE_PATH="${AMP_SO101_SCENE}"
      AMP_TEST_ROBOT_PATH="${AMP_SO101_MODEL_PATH}/so101.xml"
  )
  add_test(NAME model_parity COMMAND model_parity_test)

  add_executable(aligator_reach_planner_contract_test
                 tests/aligator_reach_planner_contract_test.cpp)
  target_link_libraries(aligator_reach_planner_contract_test PRIVATE amp::planning)
  target_compile_features(aligator_reach_planner_contract_test PRIVATE cxx_std_23)
  target_compile_definitions(aligator_reach_planner_contract_test
                             PRIVATE AMP_TEST_ROBOT_PATH="${AMP_SO101_MODEL_PATH}/so101.xml")
  add_test(NAME aligator_reach_planner_contract COMMAND aligator_reach_planner_contract_test)

  add_executable(aligator_reach_planner_test tests/aligator_reach_planner_test.cpp)
  target_link_libraries(aligator_reach_planner_test
                        PRIVATE amp::planning pinocchio::pinocchio_parsers)
  target_compile_features(aligator_reach_planner_test PRIVATE cxx_std_23)
  target_compile_definitions(aligator_reach_planner_test
                             PRIVATE AMP_TEST_ROBOT_PATH="${AMP_SO101_MODEL_PATH}/so101.xml")
  add_test(NAME aligator_reachable_target COMMAND aligator_reach_planner_test)

  add_executable(aligator_validation_test tests/aligator_validation_test.cpp)
  target_include_directories(aligator_validation_test PRIVATE src/core)
  target_link_libraries(aligator_validation_test PRIVATE amp::planning)
  target_compile_features(aligator_validation_test PRIVATE cxx_std_23)
  add_test(NAME aligator_internal_validation COMMAND aligator_validation_test)

  add_executable(trajectory_player_test tests/trajectory_player_test.cpp)
  target_link_libraries(trajectory_player_test
                        PRIVATE amp::planning amp::playback amp::simulation)
  target_compile_features(trajectory_player_test PRIVATE cxx_std_23)
  target_compile_definitions(
    trajectory_player_test
    PRIVATE
      AMP_TEST_SCENE_PATH="${AMP_SO101_SCENE}"
      AMP_TEST_ROBOT_PATH="${AMP_SO101_MODEL_PATH}/so101.xml"
  )
  add_test(NAME trajectory_playback COMMAND trajectory_player_test)

  add_test(NAME reach_cli_demo
           COMMAND so101_reach --target 0.31741606 -0.09770090 0.26459835)
endif()

message(STATUS "SO-101 scene: ${AMP_SO101_SCENE}")
