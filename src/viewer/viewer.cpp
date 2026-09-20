#include "amp/viewer/viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>

namespace amp {
namespace {

struct ViewerState {
  ViewerState(Simulation& simulation, ViewerOptions viewer_options)
      : simulation(simulation), viewer_options(std::move(viewer_options)) {
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&options);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    camera.lookat[0] = -0.12;
    camera.lookat[1] = 0.0;
    camera.lookat[2] = 0.18;
    camera.distance = 0.85;
    camera.azimuth = 145.0;
    camera.elevation = -25.0;
  }

  Simulation& simulation;
  ViewerOptions viewer_options;
  mjvCamera camera{};
  mjvOption options{};
  mjvScene scene{};
  mjrContext context{};
  bool paused = false;
  bool completed = false;
  bool left_button = false;
  bool middle_button = false;
  bool right_button = false;
  double last_x = 0.0;
  double last_y = 0.0;
  std::optional<std::string> error;
};

ViewerState& state_from(GLFWwindow* window) {
  return *static_cast<ViewerState*>(glfwGetWindowUserPointer(window));
}

std::expected<void, std::string> reset(ViewerState& state) {
  state.completed = false;
  state.paused = false;
  if (state.viewer_options.on_reset) {
    return state.viewer_options.on_reset(state.simulation);
  }
  state.simulation.reset();
  return {};
}

void key_callback(GLFWwindow* window, int key, int, int action, int) {
  if (action != GLFW_PRESS) {
    return;
  }

  auto& state = state_from(window);
  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  } else if (key == GLFW_KEY_SPACE) {
    state.paused = !state.paused;
  } else if (key == GLFW_KEY_BACKSPACE || key == GLFW_KEY_R) {
    if (const auto result = reset(state); !result) {
      state.error = result.error();
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
  }
}

void mouse_button_callback(GLFWwindow* window, int, int, int) {
  auto& state = state_from(window);
  state.left_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  state.middle_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  state.right_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &state.last_x, &state.last_y);
}

void cursor_callback(GLFWwindow* window, const double x, const double y) {
  auto& state = state_from(window);
  if (!state.left_button && !state.middle_button && !state.right_button) {
    return;
  }

  const double delta_x = x - state.last_x;
  const double delta_y = y - state.last_y;
  state.last_x = x;
  state.last_y = y;

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if (height <= 0) {
    return;
  }

  const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse action = mjMOUSE_ZOOM;
  if (state.right_button) {
    action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (state.left_button) {
    action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }

  mjv_moveCamera(&state.simulation.model(), action, delta_x / height, delta_y / height,
                 &state.scene, &state.camera);
}

void scroll_callback(GLFWwindow* window, double, const double y_offset) {
  auto& state = state_from(window);
  mjv_moveCamera(&state.simulation.model(), mjMOUSE_ZOOM, 0.0, -0.05 * y_offset, &state.scene,
                 &state.camera);
}

void glfw_error_callback(int code, const char* description) {
  std::fprintf(stderr, "GLFW error %d: %s\n", code, description);
}

}  // namespace

std::expected<void, std::string> run_viewer(Simulation& simulation, ViewerOptions options) {
  if (options.on_reset) {
    if (const auto result = options.on_reset(simulation); !result) {
      return std::unexpected(result.error());
    }
  }

  glfwSetErrorCallback(glfw_error_callback);
  if (glfwInit() != GLFW_TRUE) {
    return std::unexpected("GLFW initialization failed; is a graphical display available?");
  }

  GLFWwindow* window = glfwCreateWindow(1200, 900, options.title.c_str(), nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    return std::unexpected("Could not create the MuJoCo viewer window");
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  ViewerState state(simulation, std::move(options));
  glfwSetWindowUserPointer(window, &state);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_callback);
  glfwSetScrollCallback(window, scroll_callback);

  mjv_makeScene(&simulation.model(), &state.scene, 2000);
  mjr_makeContext(&simulation.model(), &state.context, mjFONTSCALE_150);

  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    if (!state.paused) {
      const mjtNum frame_start = simulation.data().time;
      while (simulation.data().time - frame_start < 1.0 / 60.0) {
        if (state.viewer_options.before_step) {
          const auto should_step = state.viewer_options.before_step(simulation);
          if (!should_step) {
            state.error = should_step.error();
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
          }
          if (!*should_step) {
            state.completed = true;
            state.paused = true;
            break;
          }
        }
        simulation.step();
      }
    }

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(&simulation.model(), &simulation.data(), &state.options, nullptr, &state.camera,
                    mjCAT_ALL, &state.scene);
    mjr_render(viewport, &state.scene, &state.context);

    const std::string status =
        "Time: " + std::to_string(simulation.data().time) +
        (state.completed ? "  [COMPLETE - R TO REPLAY]" : (state.paused ? "  [PAUSED]" : ""));
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                "Space: pause   R/Backspace: reset or replay   Esc: quit", status.c_str(),
                &state.context);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&state.scene);
  mjr_freeContext(&state.context);
  glfwDestroyWindow(window);
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
  if (state.error) {
    return std::unexpected(*state.error);
  }
  return {};
}

}  // namespace amp
