/*
 * viewer_base.cpp
 *
 * Created on: Jul 27, 2021 08:59
 * Description:
 *
 * Note: how to generate compressed data from ttf file
 *  $ cd src/fonts && g++ binary_to_compressed_c.cpp
 *  $ ./a.out OpenSans-Regular.ttf OpenSansRegular > opensans_regular.hpp
 *
 * In the above command, "xxx.ttf" is the original ttf file, "OpenSansRegular"
 * will be part of the variable name, e.g. OpenSansRegular_compressed_data,
 * "xxx.hpp" will be the source file you can include to load compressed data
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer.hpp"

#include <iostream>
#include <functional>

#include "implot/implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace quickviz {
namespace {
template <typename T>
struct FramebufferSizeCallback;

template <typename Ret, typename... Params>
struct FramebufferSizeCallback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args) {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

// Initialize the static member.
template <typename Ret, typename... Params>
std::function<Ret(Params...)> FramebufferSizeCallback<Ret(Params...)>::func;

template <typename T>
struct JoystickCallback;

template <typename Ret, typename... Params>
struct JoystickCallback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args) {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

// Initialize the static member.
template <typename Ret, typename... Params>
std::function<Ret(Params...)> JoystickCallback<Ret(Params...)>::func;
}  // namespace

Viewer::Viewer(std::string title, uint32_t width, uint32_t height,
               uint32_t window_hints)
    : Window(title, width, height, window_hints) {
  // setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  // setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(win_, true);
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCallbacks(window, "#canvas");
#endif

  // decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  const char *glsl_version = "#version 100";
#elif defined(__APPLE__)
  // GL 3.2 + GLSL 150
  const char *glsl_version = "#version 150";
#else
  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
#endif
  ImGui_ImplOpenGL3_Init(glsl_version);

  // set up callbacks
  // convert callback-function to c-pointer first
  FramebufferSizeCallback<void(GLFWwindow *, int, int)>::func =
      std::bind(&Viewer::OnResize, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3);
  void (*resize_callback_func)(GLFWwindow *, int, int) =
      static_cast<decltype(resize_callback_func)>(
          FramebufferSizeCallback<void(GLFWwindow *, int, int)>::callback);
  glfwSetFramebufferSizeCallback(win_, resize_callback_func);

  // load fonts
  Fonts::LoadFonts();

  // load default style
  LoadDefaultStyle();
  ApplyDarkColorScheme();

  // enable/disable features
  EnableDocking(true);
  EnableKeyboardNav(true);
  EnableGamepadNav(false);
}

Viewer::~Viewer() {
  Fonts::UnloadFonts();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
}

void Viewer::LoadDefaultStyle() {
  // additional variable initialization
  SetBackgroundColor(1.0f, 1.0f, 1.0f, 1.0f);

  // disable roundings
  ImGui::GetStyle().WindowRounding = 0.0f;
  ImGui::GetStyle().ChildRounding = 0.0f;
  ImGui::GetStyle().FrameRounding = 0.0f;
  ImGui::GetStyle().GrabRounding = 0.0f;
  ImGui::GetStyle().PopupRounding = 0.0f;
  ImGui::GetStyle().ScrollbarRounding = 0.0f;
  ImGui::GetStyle().WindowPadding = {0.0f, 0.0f};
}

void Viewer::ApplyDarkColorScheme() {
  ImGui::StyleColorsDark();

  SetBackgroundColor(0.31f, 0.31f, 0.31f, 1.0f);

  auto &colors = ImGui::GetStyle().Colors;
  colors[ImGuiCol_WindowBg] = ImVec4{0.1f, 0.105f, 0.11f, 1.0f};

  // Headers
  colors[ImGuiCol_Header] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_HeaderHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_HeaderActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Buttons
  colors[ImGuiCol_Button] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_ButtonHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_ButtonActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Frame BG
  colors[ImGuiCol_FrameBg] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};
  colors[ImGuiCol_FrameBgHovered] = ImVec4{0.3f, 0.305f, 0.31f, 1.0f};
  colors[ImGuiCol_FrameBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};

  // Tabs
  colors[ImGuiCol_Tab] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabHovered] = ImVec4{0.38f, 0.3805f, 0.381f, 1.0f};
  colors[ImGuiCol_TabActive] = ImVec4{0.28f, 0.2805f, 0.281f, 1.0f};
  colors[ImGuiCol_TabUnfocused] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TabUnfocusedActive] = ImVec4{0.2f, 0.205f, 0.21f, 1.0f};

  // Title
  colors[ImGuiCol_TitleBg] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgActive] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4{0.15f, 0.1505f, 0.151f, 1.0f};
}

void Viewer::ApplyLightColorScheme() {
  ImGui::StyleColorsClassic();
  SetBackgroundColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void Viewer::SetBackgroundColor(float r, float g, float b, float a) {
  bg_color_[0] = r;
  bg_color_[1] = g;
  bg_color_[2] = b;
  bg_color_[3] = a;
}

void Viewer::SetWindowSizeLimits(int min_x, int min_y, int max_x, int max_y) {
  int minx = min_x == -1 ? GLFW_DONT_CARE : min_x;
  int miny = min_y == -1 ? GLFW_DONT_CARE : min_y;
  int maxx = max_x == -1 ? GLFW_DONT_CARE : max_x;
  int maxy = max_y == -1 ? GLFW_DONT_CARE : max_y;
  glfwSetWindowSizeLimits(win_, minx, miny, maxx, maxy);
}

void Viewer::EnableDocking(bool enable) {
  if (enable) {
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // enable docking with shift key
    ImGui::GetIO().ConfigDockingWithShift = true;
  } else {
    ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
  }
}

void Viewer::EnableKeyboardNav(bool enable) {
  ImGuiIO &io = ImGui::GetIO();
  if (enable) {
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  } else {
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
  }
}

void Viewer::EnableGamepadNav(bool enable) {
  ImGuiIO &io = ImGui::GetIO();
  if (enable) {
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  } else {
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad;
  }
}

void Viewer::EnumerateJoysticks() {
  joysticks_.clear();
  for (int i = GLFW_JOYSTICK_1; i < GLFW_JOYSTICK_LAST; i++) {
    if (glfwJoystickPresent(i)) {
      JoystickDevice js;
      js.id = i;
      js.name = glfwGetJoystickName(i);
      joysticks_[i] = js;
    }
  }
}

void Viewer::OnJoystickEvent(int id, int event) {
  if (event == GLFW_CONNECTED) {
    EnumerateJoysticks();
  } else if (event == GLFW_DISCONNECTED) {
    if (current_joystick_input_.device.id == id) {
      StopJoystickInputMonitoring();
    }
    joysticks_.erase(id);
  }

  std::vector<JoystickDevice> js;
  for (const auto &pair : joysticks_) {
    js.push_back(pair.second);
  }
  for (auto &obj : scene_objects_) {
    obj->OnJoystickDeviceChange(js);
  }
}

bool Viewer::StartJoystickInputMonitoring(int id) {
  if (joysticks_.find(id) == joysticks_.end()) {
    std::cerr << "[ERROR] Viewer::RegisterJoystickInputUpdateCallback(): "
                 "Joystick with ID "
              << id << " not found" << std::endl;
    return false;
  }
  current_joystick_input_.device.id = id;
  current_joystick_input_.device.name = joysticks_[id].name;
  current_joystick_input_.axes.clear();
  current_joystick_input_.buttons.clear();
  current_joystick_input_.hats.clear();
  return true;
}

void Viewer::StopJoystickInputMonitoring() {
  current_joystick_input_.device.id = -1;
  current_joystick_input_.device.name = "";
  current_joystick_input_.axes.clear();
  current_joystick_input_.buttons.clear();
  current_joystick_input_.hats.clear();
}

bool Viewer::IsJoystickInputMonitoringActive() const {
  return current_joystick_input_.device.id >= GLFW_JOYSTICK_1 &&
         current_joystick_input_.device.id < GLFW_JOYSTICK_LAST;
}

void Viewer::EnableJoystickInput(bool enable) {
  handle_joystick_input_ = enable;
  if (handle_joystick_input_) {
    EnumerateJoysticks();

    JoystickCallback<void(int, int)>::func =
        std::bind(&Viewer::OnJoystickEvent, this, std::placeholders::_1,
                  std::placeholders::_2);

    void (*joystick_callback)(int, int) =
        static_cast<decltype(joystick_callback)>(
            JoystickCallback<void(int, int)>::callback);

    glfwSetJoystickCallback(joystick_callback);
  }
}

std::vector<JoystickDevice> Viewer::GetListOfJoysticks() {
  if (!handle_joystick_input_) {
    EnumerateJoysticks();
  }
  std::vector<JoystickDevice> js;
  for (const auto &pair : joysticks_) {
    js.push_back(pair.second);
  }
  return js;
}

bool Viewer::GetJoystickInput(int id, JoystickInput &input) {
  input.device.id = id;
  input.device.name = joysticks_[id].name;
  if (glfwJoystickPresent(id)) {
    // clear previous data
    input.axes.clear();
    input.buttons.clear();
    input.hats.clear();

    // read input data
    int axis_count, button_count;
    const float *axes = glfwGetJoystickAxes(id, &axis_count);
    const unsigned char *buttons = glfwGetJoystickButtons(id, &button_count);

    input.axes.assign(axes, axes + axis_count);
    input.buttons.assign(buttons, buttons + button_count);

#if ((GLFW_VERSION_MAJOR >= 3) && (GLFW_VERSION_MINOR >= 3))
    int hat_count;
    const unsigned char *hats = glfwGetJoystickHats(id, &hat_count);
    input.hats.assign(hats, hats + hat_count);
#endif
    return true;
  }
  return false;
}

void Viewer::SetWindowShouldClose() {
  glfwSetWindowShouldClose(win_, GLFW_TRUE);
}

void Viewer::SetupOpenGL() {
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Viewer::ClearBackground() {
  int display_w, display_h;
  glfwGetFramebufferSize(win_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], bg_color_[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Viewer::CreateNewImGuiFrame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Viewer::RenderImGuiFrame() {
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Viewer::RenderSceneObjects() {
  for (auto &obj : scene_objects_) {
    if (obj->IsVisible()) obj->OnRender();
  }
}

void Viewer::HandleJoystickInput() {
  if (handle_joystick_input_ &&
      current_joystick_input_.device.id >= GLFW_JOYSTICK_1 &&
      current_joystick_input_.device.id < GLFW_JOYSTICK_LAST) {
    JoystickInput input;
    if (GetJoystickInput(current_joystick_input_.device.id, input)) {
      if (input != current_joystick_input_) {
        for (auto &obj : scene_objects_) {
          obj->OnJoystickUpdate(input);
        }
        current_joystick_input_ = input;
      }
    }
  }
}

bool Viewer::AddSceneObject(std::shared_ptr<SceneObject> obj) {
  if (obj == nullptr) {
    std::cerr << "[ERROR] Viewer::AddSceneObject(): object is nullptr"
              << std::endl;
    return false;
  }
  scene_objects_.push_back(obj);
  return true;
}

void Viewer::OnResize(GLFWwindow *window, int width, int height) {
  //  std::cout << "-- Viewer::OnResize: " << width << "x" << height <<
  //  std::endl;
  for (auto &obj : scene_objects_) {
    obj->OnResize(width, height);
  }
}

void Viewer::Show() {
  // initialize layers
  int display_w, display_h;
  glfwGetFramebufferSize(win_, &display_w, &display_h);
  OnResize(win_, display_w, display_h);

  SetupOpenGL();

  // main loop
  while (!ShouldClose()) {
    // handle events
    PollEvents();
    HandleJoystickInput();

    // draw stuff
    ClearBackground();
    CreateNewImGuiFrame();
    RenderSceneObjects();
    RenderImGuiFrame();

    // swap buffers
    SwapBuffers();
  }
}
}  // namespace quickviz