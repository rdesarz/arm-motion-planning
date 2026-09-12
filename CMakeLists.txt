cmake_minimum_required(VERSION 3.24)

project(
  arm_motion_planning
  VERSION 0.1.0
  DESCRIPTION "Motion planning and control experiments for the SO-101 arm"
  LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(AMP_BUILD_TESTS "Build the headless simulation tests" ON)
set(AMP_SO101_MODEL_DIR "" CACHE PATH "Path to a local robotstudio_so101 model directory")

find_package(mujoco 3.1.3 CONFIG REQUIRED)
find_package(glfw3 3.4 CONFIG REQUIRED)

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

add_library(amp_simulation src/simulation.cpp)
add_library(amp::simulation ALIAS amp_simulation)
target_include_directories(amp_simulation PUBLIC include)
target_link_libraries(amp_simulation PUBLIC mujoco::mujoco)
target_compile_features(amp_simulation PUBLIC cxx_std_23)

add_executable(so101_sim src/main.cpp src/viewer.cpp)
target_link_libraries(so101_sim PRIVATE amp::simulation ${AMP_GLFW_TARGET})
target_compile_features(so101_sim PRIVATE cxx_std_23)
target_compile_definitions(so101_sim PRIVATE AMP_DEFAULT_SCENE_PATH="${AMP_SO101_SCENE}")

if(MSVC)
  target_compile_options(amp_simulation PRIVATE /W4 /permissive-)
  target_compile_options(so101_sim PRIVATE /W4 /permissive-)
else()
  target_compile_options(amp_simulation PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(so101_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(AMP_BUILD_TESTS)
  enable_testing()
  add_executable(simulation_test tests/simulation_test.cpp)
  target_link_libraries(simulation_test PRIVATE amp::simulation)
  target_compile_features(simulation_test PRIVATE cxx_std_23)
  target_compile_definitions(simulation_test PRIVATE AMP_TEST_SCENE_PATH="${AMP_SO101_SCENE}")
  add_test(NAME simulation_loads_and_steps COMMAND simulation_test)
endif()

message(STATUS "SO-101 scene: ${AMP_SO101_SCENE}")
