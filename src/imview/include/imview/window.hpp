/*
 * window.hpp
 *
 * Created on: Mar 04, 2021 15:05
 * Description: a light wrapper around GLFW
 *  Window <- Layer <- Panel <- Renderable
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_WINDOW_HPP
#define IMVIEW_WINDOW_HPP

#ifdef IMVIEW_WITH_GLAD
#include <glad/glad.h>
#endif
#include <GLFW/glfw3.h>

#include <string>

namespace quickviz {
class Window {
 public:
  enum WINDOW_HINT {
    WIN_FOCUSED = 0x00000001,
    WIN_RESIZABLE = 0x00000004,
    WIN_DECORATED = 0x00000010,
    WIN_AUTO_ICONIFY = 0x00000020,
    WIN_FLOATING = 0x00000040,
    WIN_MAXIMIZED = 0x00000080
  };

 public:
  Window(std::string title, uint32_t width, uint32_t height,
         uint32_t window_hints = WIN_RESIZABLE | WIN_DECORATED);
  ~Window();

  // do not allow copy
  Window(const Window &other) = delete;
  Window &operator=(const Window &other) = delete;
  Window(const Window &&other) = delete;
  Window &operator=(const Window &&other) = delete;

  // public methods
  uint32_t GetWidth() const;
  uint32_t GetHeight() const;

  bool ShouldClose() const;
  void CloseWindow();
  void PollEvents();
  void SwapBuffers();

 protected:
  void ApplyWindowHints(uint32_t window_hints);
  void LoadDefaultStyle();

  GLFWwindow *win_;
};
}  // namespace quickviz

#endif /* IMVIEW_WINDOW_HPP */
