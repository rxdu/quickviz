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

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace quickviz {
namespace {
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args) {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

// Initialize the static member.
template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;
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
  Callback<void(GLFWwindow *, int, int)>::func =
      std::bind(&Viewer::OnResize, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3);
  void (*resize_callback_func)(GLFWwindow *, int, int) =
      static_cast<decltype(resize_callback_func)>(
          Callback<void(GLFWwindow *, int, int)>::callback);
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

  // set up default layer
  auto default_layer = std::make_shared<Layer>("DefaultLayer");
  default_layer->SetVisible(false);
  layers_.push_back(default_layer);
}

Viewer::~Viewer() {
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

void Viewer::EnableDocking(bool enable) {
  if (enable) {
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
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

void Viewer::DockSpaceOverMainViewport() {
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
}

void Viewer::StartNewFrame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Viewer::RenderFrame() {
  ImGui::Render();

  int display_w, display_h;
  glfwGetFramebufferSize(win_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], bg_color_[3]);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool Viewer::AddRenderable(std::shared_ptr<Renderable> renderable) {
  if (renderable == nullptr) {
    std::cerr << "[ERROR] Viewer::AddRenderable: renderable is nullptr"
              << std::endl;
    return false;
  }
  if (renderable->IsContainer()) {
    layers_.push_back(std::dynamic_pointer_cast<Layer>(renderable));
  } else {
    std::cout << "[INFO] Added renderable object to default layer" << std::endl;
    layers_.front()->AddRenderable(renderable);
    layers_.front()->SetVisible(true);
  }
  return true;
}

void Viewer::OnResize(GLFWwindow *window, int width, int height) {
  std::cout << "-- Viewer::OnResize: " << width << "x" << height << std::endl;
  for (auto &layer : layers_) {
    layer->OnResize(width, height);
    layer->PrintLayout();
  }
}

void Viewer::Show() {
  while (!ShouldClose()) {
    // handle events
    PollEvents();

    // draw stuff
    StartNewFrame();
    //    std::cout << "--" << std::endl;
    for (auto &layer : layers_) {
      if (layer->IsVisible()) {
        //        std::cout << "layer: " << layer->GetName() << std::endl;
        layer->OnRender();
      }
    }
    RenderFrame();

    // swap buffers
    SwapBuffers();
  }
}
}  // namespace quickviz