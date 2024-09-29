/*
 * window.cpp
 *
 * Created on: Mar 04, 2021 16:01
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/window.hpp"

#include <stdexcept>

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

#ifdef IMVIEW_WITH_GLAD
  // initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Failed to initialize GLAD");
  }
#endif
}

Window::~Window() {
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

uint32_t Window::GetWidth() const {
  int width, height;
  glfwGetWindowSize(win_, &width, &height);
  return width;
}

uint32_t Window::GetHeight() const {
  int width, height;
  glfwGetWindowSize(win_, &width, &height);
  return height;
}

void Window::PollEvents() {
  // Poll and handle events (inputs, window resize, etc.)
  glfwPollEvents();

  // exit if ESC is pressed
  if (glfwGetKey(win_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    CloseWindow();
  }
}

void Window::SwapBuffers() { glfwSwapBuffers(win_); }

void Window::CloseWindow() { glfwSetWindowShouldClose(win_, 1); }

bool Window::ShouldClose() { return glfwWindowShouldClose(win_); }
}  // namespace quickviz