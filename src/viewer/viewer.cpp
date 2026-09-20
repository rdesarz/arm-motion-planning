#include "so101_traj_planner/viewer/viewer.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "target_editor.hpp"

namespace so101_traj_planner {

std::expected<WorldPoint, std::string> viewer_detail::validated_target(
    const std::array<mjtNum, 3>& values) {
  WorldPoint point{};
  for (std::size_t axis = 0; axis < point.size(); ++axis) {
    if (!std::isfinite(values[axis])) {
      return std::unexpected("Target coordinates must be finite");
    }
    point[axis] = values[axis];
  }
  return point;
}

std::expected<std::array<mjtNum, 3>, std::string> viewer_detail::nudged_target(
    std::array<mjtNum, 3> values, const std::size_t axis, const mjtNum delta) {
  if (axis >= values.size()) {
    return std::unexpected("Target axis is out of range");
  }
  values[axis] += delta;
  if (!validated_target(values)) {
    return std::unexpected("Target coordinates must be finite");
  }
  return values;
}

std::expected<WorldPoint, std::string> viewer_detail::preview_target(
    std::array<mjtNum, 3> values, const std::optional<std::size_t> editing_axis,
    const std::string_view edit_text) {
  if (editing_axis) {
    if (*editing_axis >= values.size()) {
      return std::unexpected("Target axis is out of range");
    }
    const std::string input{edit_text};
    char* end = nullptr;
    const double value = std::strtod(input.c_str(), &end);
    while (end != nullptr && std::isspace(static_cast<unsigned char>(*end)) != 0) {
      ++end;
    }
    if (end == input.c_str() || end == nullptr || *end != '\0' || !std::isfinite(value)) {
      return std::unexpected("Target coordinates must be valid finite numbers");
    }
    values[*editing_axis] = value;
  }
  return validated_target(values);
}

namespace {

constexpr int kTargetEditorRect = 1;
constexpr int kSendTargetUserId = 1;
constexpr int kFirstNudgeUserId = 10;
constexpr int kLastNudgeUserId = kFirstNudgeUserId + 5;
constexpr mjtNum kNudgeStep = 0.01;
constexpr mjtNum kFineNudgeStep = 0.001;

bool initialize_glfw() {
  static const bool initialized = [] {
    if (glfwInit() != GLFW_TRUE) {
      return false;
    }
    std::atexit(glfwTerminate);
    return true;
  }();
  return initialized;
}

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
    paused = this->viewer_options.start_paused;

    if (this->viewer_options.target_editor_initial) {
      for (std::size_t axis = 0; axis < target_editor_values.size(); ++axis) {
        target_editor_values[axis] = (*this->viewer_options.target_editor_initial)[axis];
      }
    }
  }

  Simulation& simulation;
  ViewerOptions viewer_options;
  mjvCamera camera{};
  mjvOption options{};
  mjvScene scene{};
  mjrContext context{};
  mjUI target_ui{};
  mjuiState ui_state{};
  std::array<mjtNum, 3> target_editor_values{};
  bool paused = false;
  bool completed = false;
  bool left_button = false;
  bool middle_button = false;
  bool right_button = false;
  double last_x = 0.0;
  double last_y = 0.0;
  std::optional<WorldPoint> selected_world_point;
  bool selection_confirmed = false;
  std::string selection_status = "Enter X, Y, and Z, then click Send target";
  std::optional<std::string> error;
};

ViewerState& state_from(GLFWwindow* window) {
  return *static_cast<ViewerState*>(glfwGetWindowUserPointer(window));
}

std::string target_status(const WorldPoint& point) {
  return "Target: " + std::to_string(point[0]) + ", " + std::to_string(point[1]) + ", " +
         std::to_string(point[2]) + " m";
}

std::expected<void, std::string> reset(ViewerState& state) {
  state.completed = false;
  state.paused = state.viewer_options.start_paused;
  state.selection_confirmed = false;
  state.selection_status = "Enter X, Y, and Z, then click Send target";
  if (state.viewer_options.on_reset) {
    return state.viewer_options.on_reset(state.simulation);
  }
  state.simulation.reset();
  return {};
}

void update_layout(GLFWwindow* window, ViewerState& state) {
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  state.ui_state.nrect = state.viewer_options.enable_target_editor ? 2 : 1;
  state.ui_state.rect[0] = {0, 0, framebuffer_width, framebuffer_height};
  if (state.viewer_options.enable_target_editor) {
    const int panel_width = std::min(state.target_ui.width, framebuffer_width);
    state.ui_state.rect[kTargetEditorRect] = {
        framebuffer_width - panel_width,
        0,
        panel_width,
        framebuffer_height,
    };
  }
}

bool update_ui_state(GLFWwindow* window, ViewerState& state) {
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  if (window_width <= 0 || window_height <= 0 || framebuffer_width <= 0 ||
      framebuffer_height <= 0) {
    return false;
  }

  double cursor_x = 0.0;
  double cursor_y = 0.0;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  cursor_x *= framebuffer_width / static_cast<double>(window_width);
  cursor_y =
      framebuffer_height - cursor_y * framebuffer_height / static_cast<double>(window_height);

  state.ui_state.dx = cursor_x - state.ui_state.x;
  state.ui_state.dy = cursor_y - state.ui_state.y;
  state.ui_state.x = cursor_x;
  state.ui_state.y = cursor_y;
  state.ui_state.left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  state.ui_state.right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  state.ui_state.middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  state.ui_state.control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                           glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
  state.ui_state.shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  state.ui_state.alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
  state.ui_state.mouserect =
      mjr_findRect(static_cast<int>(std::lround(cursor_x)), static_cast<int>(std::lround(cursor_y)),
                   state.ui_state.nrect - 1, state.ui_state.rect + 1) +
      1;
  return true;
}

mjtButton mujoco_button(const int glfw_button) {
  if (glfw_button == GLFW_MOUSE_BUTTON_LEFT) {
    return mjBUTTON_LEFT;
  }
  if (glfw_button == GLFW_MOUSE_BUTTON_RIGHT) {
    return mjBUTTON_RIGHT;
  }
  if (glfw_button == GLFW_MOUSE_BUTTON_MIDDLE) {
    return mjBUTTON_MIDDLE;
  }
  return mjBUTTON_NONE;
}

std::optional<WorldPoint> current_preview_target(const ViewerState& state);

void apply_ui_change(ViewerState& state, const mjuiItem* changed) {
  if (changed == nullptr) {
    return;
  }

  if (changed->type == mjITEM_EDITNUM) {
    const auto target = viewer_detail::validated_target(state.target_editor_values);
    if (!target) {
      state.selection_status = target.error();
      return;
    }
    state.selected_world_point = *target;
    state.selection_status = target_status(*target) + "   Click Send target";
    return;
  }

  if (changed->type == mjITEM_BUTTON && changed->userid >= kFirstNudgeUserId &&
      changed->userid <= kLastNudgeUserId) {
    const int nudge_id = changed->userid - kFirstNudgeUserId;
    const std::size_t axis = static_cast<std::size_t>(nudge_id / 2);
    const mjtNum direction = nudge_id % 2 == 0 ? -1.0 : 1.0;
    const mjtNum step = state.ui_state.shift ? kFineNudgeStep : kNudgeStep;
    const auto values =
        viewer_detail::nudged_target(state.target_editor_values, axis, direction * step);
    if (!values) {
      state.selection_status = values.error();
      return;
    }
    state.target_editor_values = *values;
    state.selected_world_point = *viewer_detail::validated_target(*values);

    state.selection_status = std::string("Editor ") + "XYZ"[axis] + ": " +
                             std::to_string(state.target_editor_values[axis]) +
                             " m   Click Send target";
    mjui_update(-1, -1, &state.target_ui, &state.ui_state, &state.context);
    return;
  }

  if (changed->type != mjITEM_BUTTON || changed->userid != kSendTargetUserId) {
    return;
  }

  const auto target = current_preview_target(state);
  if (!target) {
    state.selection_status = "Target coordinates must be valid finite numbers";
    return;
  }
  state.selected_world_point = *target;
  state.selection_status = target_status(*target) + "   Planning...";
  if (state.viewer_options.on_target_submitted) {
    const auto submitted =
        state.viewer_options.on_target_submitted(state.simulation, *state.selected_world_point);
    if (!submitted) {
      state.selection_status = "Target rejected: " + submitted.error();
      return;
    }
    state.completed = false;
    state.paused = false;
    state.selection_confirmed = true;
    state.selection_status = target_status(*target) + "   Motion started";
    return;
  }

  state.selection_confirmed = true;
}

void submit_current_target(ViewerState& state) {
  mjuiItem submission{};
  submission.type = mjITEM_BUTTON;
  submission.userid = kSendTargetUserId;
  apply_ui_change(state, &submission);
}

std::optional<WorldPoint> current_preview_target(const ViewerState& state) {
  std::optional<std::size_t> editing_axis;
  if (state.target_ui.editsect > 0 && state.target_ui.editsect <= state.target_ui.nsect) {
    const auto& section = state.target_ui.sect[state.target_ui.editsect - 1];
    if (state.target_ui.edititem >= 0 && state.target_ui.edititem < section.nitem) {
      const auto& item = section.item[state.target_ui.edititem];
      if (item.type == mjITEM_EDITNUM) {
        for (std::size_t axis = 0; axis < state.target_editor_values.size(); ++axis) {
          if (item.pdata == &state.target_editor_values[axis]) {
            editing_axis = axis;
            break;
          }
        }
      }
    }
  }

  const auto target = viewer_detail::preview_target(state.target_editor_values, editing_axis,
                                                    state.target_ui.edittext);
  return target ? std::optional<WorldPoint>{*target} : std::nullopt;
}

void sync_target_preview(ViewerState& state) {
  const auto target = current_preview_target(state);
  if (!target || state.selected_world_point == target) {
    return;
  }
  state.selected_world_point = *target;
  state.selection_status = target_status(*target) + "   Click Send target";
}

void initialize_target_editor(GLFWwindow* window, ViewerState& state) {
  if (!state.viewer_options.enable_target_editor) {
    return;
  }

  state.target_ui.spacing = mjui_themeSpacing(0);
  state.target_ui.color = mjui_themeColor(0);
  state.target_ui.rectid = kTargetEditorRect;
  state.target_ui.auxid = 0;

  const mjuiDef definitions[] = {
      {mjITEM_SECTION, "Reach target (metres)", mjSECT_FIXED, nullptr, "", 0},
      {mjITEM_EDITNUM, "X", 1, &state.target_editor_values[0], "1", 0},
      {mjITEM_BUTTON, "X <  (-1 cm)", 1, nullptr, "", kFirstNudgeUserId},
      {mjITEM_BUTTON, "X >  (+1 cm)", 1, nullptr, "", kFirstNudgeUserId + 1},
      {mjITEM_EDITNUM, "Y", 1, &state.target_editor_values[1], "1", 0},
      {mjITEM_BUTTON, "Y <  (-1 cm)", 1, nullptr, "", kFirstNudgeUserId + 2},
      {mjITEM_BUTTON, "Y >  (+1 cm)", 1, nullptr, "", kFirstNudgeUserId + 3},
      {mjITEM_EDITNUM, "Z", 1, &state.target_editor_values[2], "1", 0},
      {mjITEM_BUTTON, "Z <  (-1 cm)", 1, nullptr, "", kFirstNudgeUserId + 4},
      {mjITEM_BUTTON, "Z >  (+1 cm)", 1, nullptr, "", kFirstNudgeUserId + 5},
      {mjITEM_BUTTON, "Send target", 1, nullptr, "", kSendTargetUserId},
      {mjITEM_END, "", 0, nullptr, "", 0},
  };
  mjui_add(&state.target_ui, definitions);
  mjui_resize(&state.target_ui, &state.context);
  mjr_addAux(state.target_ui.auxid, state.target_ui.width, state.target_ui.maxheight,
             state.target_ui.spacing.samples, &state.context);
  update_layout(window, state);
  state.ui_state.type = mjEVENT_RESIZE;
  mjui_update(-1, -1, &state.target_ui, &state.ui_state, &state.context);
  sync_target_preview(state);
}

void key_callback(GLFWwindow* window, int key, int, int action, int) {
  if (action != GLFW_PRESS) {
    return;
  }

  auto& state = state_from(window);
  if (state.viewer_options.enable_target_editor && update_ui_state(window, state)) {
    const bool editor_had_focus = state.target_ui.editsect != 0;
    state.ui_state.type = mjEVENT_KEY;
    state.ui_state.key = key;
    state.ui_state.keytime = glfwGetTime();
    mjuiItem* changed = mjui_event(&state.target_ui, &state.ui_state, &state.context);
    apply_ui_change(state, changed);
    if (editor_had_focus || state.target_ui.editsect != 0 || changed != nullptr) {
      return;
    }
  }

  if (key == GLFW_KEY_ESCAPE) {
    state.selected_world_point.reset();
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  } else if (state.viewer_options.enable_target_editor &&
             (key == GLFW_KEY_ENTER || key == GLFW_KEY_KP_ENTER)) {
    submit_current_target(state);
  } else if (!state.viewer_options.enable_target_editor && key == GLFW_KEY_SPACE) {
    state.paused = !state.paused;
  } else if (key == GLFW_KEY_BACKSPACE ||
             (!state.viewer_options.enable_target_editor && key == GLFW_KEY_R)) {
    if (const auto result = reset(state); !result) {
      state.error = result.error();
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
  }
}

void mouse_button_callback(GLFWwindow* window, const int button, const int action, int) {
  auto& state = state_from(window);

  if (state.viewer_options.enable_target_editor && update_ui_state(window, state)) {
    state.ui_state.type = action == GLFW_PRESS ? mjEVENT_PRESS : mjEVENT_RELEASE;
    state.ui_state.button = mujoco_button(button);
    state.ui_state.buttontime = glfwGetTime();
    if (action == GLFW_PRESS && state.ui_state.mouserect != 0) {
      state.ui_state.dragbutton = state.ui_state.button;
      state.ui_state.dragrect = state.ui_state.mouserect;
    }
    const bool handled_by_ui = state.ui_state.mouserect == state.target_ui.rectid ||
                               state.ui_state.dragrect == state.target_ui.rectid;
    if (handled_by_ui) {
      apply_ui_change(state, mjui_event(&state.target_ui, &state.ui_state, &state.context));
    }
    if (action == GLFW_RELEASE) {
      state.ui_state.dragbutton = 0;
      state.ui_state.dragrect = 0;
    }
    if (handled_by_ui) {
      state.left_button = false;
      state.middle_button = false;
      state.right_button = false;
      return;
    }
  }

  state.left_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  state.middle_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  state.right_button = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &state.last_x, &state.last_y);
}

void cursor_callback(GLFWwindow* window, const double x, const double y) {
  auto& state = state_from(window);
  if (state.viewer_options.enable_target_editor && update_ui_state(window, state) &&
      (state.ui_state.mouserect == state.target_ui.rectid ||
       state.ui_state.dragrect == state.target_ui.rectid)) {
    if (state.ui_state.left || state.ui_state.right || state.ui_state.middle) {
      state.ui_state.type = mjEVENT_MOVE;
      apply_ui_change(state, mjui_event(&state.target_ui, &state.ui_state, &state.context));
    }
    return;
  }

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

void scroll_callback(GLFWwindow* window, const double x_offset, const double y_offset) {
  auto& state = state_from(window);
  if (state.viewer_options.enable_target_editor && update_ui_state(window, state) &&
      state.ui_state.mouserect == state.target_ui.rectid) {
    int window_width = 0;
    int window_height = 0;
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetWindowSize(window, &window_width, &window_height);
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    const double scale =
        window_width > 0 ? framebuffer_width / static_cast<double>(window_width) : 1.0;
    state.ui_state.type = mjEVENT_SCROLL;
    state.ui_state.sx = x_offset * scale;
    state.ui_state.sy = y_offset * scale;
    apply_ui_change(state, mjui_event(&state.target_ui, &state.ui_state, &state.context));
    return;
  }

  mjv_moveCamera(&state.simulation.model(), mjMOUSE_ZOOM, 0.0, -0.05 * y_offset, &state.scene,
                 &state.camera);
}

void glfw_error_callback(int code, const char* description) {
  std::fprintf(stderr, "GLFW error %d: %s\n", code, description);
}

void add_target_marker(ViewerState& state) {
  constexpr int marker_geometry_count = 4;
  if (!state.selected_world_point ||
      state.scene.ngeom + marker_geometry_count > state.scene.maxgeom) {
    return;
  }

  const mjtNum size[3] = {0.012, 0.012, 0.012};
  const mjtNum position[3] = {(*state.selected_world_point)[0], (*state.selected_world_point)[1],
                              (*state.selected_world_point)[2]};
  const float color[4] = {1.0F, 0.75F, 0.05F, 1.0F};
  mjv_initGeom(&state.scene.geoms[state.scene.ngeom], mjGEOM_SPHERE, size, position, nullptr,
               color);
  ++state.scene.ngeom;

  constexpr std::array<std::array<float, 4>, 3> axis_colors = {
      std::array<float, 4>{0.95F, 0.10F, 0.10F, 1.0F},
      std::array<float, 4>{0.10F, 0.85F, 0.15F, 1.0F},
      std::array<float, 4>{0.15F, 0.35F, 1.0F, 1.0F},
  };
  for (std::size_t axis = 0; axis < 3; ++axis) {
    mjtNum start[3] = {position[0], position[1], position[2]};
    mjtNum end[3] = {position[0], position[1], position[2]};
    start[axis] -= 0.04;
    end[axis] += 0.04;
    auto& geometry = state.scene.geoms[state.scene.ngeom];
    mjv_initGeom(&geometry, mjGEOM_LINE, nullptr, nullptr, nullptr, axis_colors[axis].data());
    mjv_connector(&geometry, mjGEOM_LINE, 0.002, start, end);
    ++state.scene.ngeom;
  }
}

}  // namespace

std::expected<ViewerResult, std::string> run_viewer(Simulation& simulation, ViewerOptions options) {
  if (options.on_reset) {
    if (const auto result = options.on_reset(simulation); !result) {
      return std::unexpected(result.error());
    }
  }

  glfwSetErrorCallback(glfw_error_callback);
  if (!initialize_glfw()) {
    return std::unexpected("GLFW initialization failed; is a graphical display available?");
  }

  GLFWwindow* window = glfwCreateWindow(1200, 900, options.title.c_str(), nullptr, nullptr);
  if (window == nullptr) {
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
  initialize_target_editor(window, state);

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
            if (state.viewer_options.enable_target_editor) {
              state.selection_status = "Motion complete   Edit XYZ and send another target";
            }
            break;
          }
        }
        simulation.step();
      }
    }

    update_layout(window, state);
    if (state.viewer_options.enable_target_editor) {
      sync_target_preview(state);
    }
    mjrRect viewport = state.ui_state.rect[0];
    if (state.viewer_options.enable_target_editor) {
      viewport.width = std::max(1, state.ui_state.rect[kTargetEditorRect].left);
    }

    mjv_updateScene(&simulation.model(), &simulation.data(), &state.options, nullptr, &state.camera,
                    mjCAT_ALL, &state.scene);
    add_target_marker(state);
    mjr_render(viewport, &state.scene, &state.context);

    const std::string status =
        "Time: " + std::to_string(simulation.data().time) +
        (state.completed ? "  [COMPLETE - R TO REPLAY]" : (state.paused ? "  [PAUSED]" : ""));
    const std::string controls = state.viewer_options.enable_target_editor
                                     ? "Edit or nudge XYZ; Shift = 1 mm   Send target   Esc: quit"
                                     : "Space: pause   R/Backspace: reset or replay   Esc: quit";
    const std::string detail =
        state.viewer_options.enable_target_editor ? state.selection_status : status;
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, controls.c_str(), detail.c_str(),
                &state.context);

    if (state.viewer_options.enable_target_editor) {
      mjui_render(&state.target_ui, &state.ui_state, &state.context);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&state.scene);
  mjr_freeContext(&state.context);
  glfwDestroyWindow(window);
  if (state.error) {
    return std::unexpected(*state.error);
  }
  return ViewerResult{
      .selected_world_point = state.selection_confirmed ? state.selected_world_point : std::nullopt,
  };
}

}  // namespace so101_traj_planner
