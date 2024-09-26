/*
 * window.cpp
 *
 * Created on: Mar 04, 2021 16:01
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

#include "imview/window.hpp"

#include <stdexcept>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace quickviz {
void HandleGlfwError(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

//-------------------------------------------------------------------//

Window::Window(std::string title, uint32_t width, uint32_t height,
               uint32_t window_hints) {
  // initialize GLFW
  glfwSetErrorCallback(HandleGlfwError);
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW library");
  }

  // apply window hints
  ApplyWindowHints(window_hints);

  // create GLFW window
  win_ = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
  if (win_ == NULL) {
    throw std::runtime_error("Failed to create GLFW window");
  }
  glfwMakeContextCurrent(win_);
  glfwSwapInterval(1);

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

  // load default style
  LoadDefaultStyle();
  ApplyDarkColorScheme();

  // enable/disable features
  EnableDocking(true);
  EnableKeyboardNav(true);
  EnableGamepadNav(false);
}

Window::~Window() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(win_);
  glfwTerminate();
}

void Window::ApplyWindowHints(uint32_t window_hints) {
  // default hints
#if defined(IMGUI_IMPL_OPENGL_ES2)
  // GL ES 2.0 + GLSL 100
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
  // GL 3.2 + GLSL 150
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac
#else
  // GL 3.0 + GLSL 130
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
// glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
// glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

  // optional hints
  if (window_hints & WIN_FOCUSED) {
    glfwWindowHint(GLFW_FOCUSED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
  }
  if (window_hints & WIN_RESIZABLE) {
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  }
  if (window_hints & WIN_DECORATED) {
    glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
  }
  if (window_hints & WIN_AUTO_ICONIFY) {
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
  }
  if (window_hints & WIN_FLOATING) {
    glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_FLOATING, GLFW_FALSE);
  }
  if (window_hints & WIN_MAXIMIZED) {
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
  } else {
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_FALSE);
  }
}

void Window::LoadDefaultStyle() {
  // additional variable initialization
  bg_color_[0] = 1.0f;
  bg_color_[1] = 1.0f;
  bg_color_[2] = 1.0f;
  bg_color_[3] = 1.0f;

  // disable roundings
  ImGui::GetStyle().WindowRounding = 0.0f;
  ImGui::GetStyle().ChildRounding = 0.0f;
  ImGui::GetStyle().FrameRounding = 0.0f;
  ImGui::GetStyle().GrabRounding = 0.0f;
  ImGui::GetStyle().PopupRounding = 0.0f;
  ImGui::GetStyle().ScrollbarRounding = 0.0f;
  ImGui::GetStyle().WindowPadding = {0.0f, 0.0f};
}

void Window::ApplyDarkColorScheme() {
  ImGui::StyleColorsDark();

  bg_color_[0] = 0.31f;
  bg_color_[1] = 0.31f;
  bg_color_[2] = 0.31f;
  bg_color_[3] = 1.00f;

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

void Window::ApplyLightColorScheme() {
  ImGui::StyleColorsClassic();

  bg_color_[0] = 1.0f;
  bg_color_[1] = 1.0f;
  bg_color_[2] = 1.0f;
  bg_color_[3] = 1.0f;
}

void Window::EnableDocking(bool enable) {
  if (enable) {
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  } else {
    ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_DockingEnable;
  }
}

void Window::SetBackgroundColor(float r, float g, float b, float a) {
  bg_color_[0] = r;
  bg_color_[1] = g;
  bg_color_[2] = b;
  bg_color_[3] = a;
}

void Window::EnableKeyboardNav(bool enable) {
  ImGuiIO &io = ImGui::GetIO();
  if (enable) {
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  } else {
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
  }
}

void Window::EnableGamepadNav(bool enable) {
  ImGuiIO &io = ImGui::GetIO();
  if (enable) {
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  } else {
    io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableGamepad;
  }
}

uint32_t Window::GetWidth() const {
  int width, height;
  glfwGetWindowSize(win_, &width, &height);
  return width;
  //   return ImGui::GetIO().DisplaySize.x;
}

uint32_t Window::GetHeight() const {
  int width, height;
  glfwGetWindowSize(win_, &width, &height);
  return height;
  // return ImGui::GetIO().DisplaySize.y;
}

void Window::PollEvents() {
  // Poll and handle events (inputs, window resize, etc.)
  glfwPollEvents();
}

void Window::CloseWindow() { glfwSetWindowShouldClose(win_, 1); }

bool Window::ShouldClose() { return glfwWindowShouldClose(win_); }

void Window::StartNewFrame() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Window::RenderFrame() {
  ImGui::Render();
  int display_w, display_h;
  glfwGetFramebufferSize(win_, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], bg_color_[3]);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(win_);
}

void Window::SampleLoop() {
  while (!ShouldClose()) {
    // Poll and handle events (inputs, window resize, etc.)
    glfwPollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) break;

    // Start the Dear ImGui frame
    StartNewFrame();

    // you can draw whatever here

    // Rendering
    RenderFrame();
  }
}
}  // namespace quickviz