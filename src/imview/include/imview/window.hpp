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
#include <memory>
#include <vector>

#include "imview/input/input_manager.hpp"

namespace quickviz {
class Panel;  // Forward declaration

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
  void DisableExitOnEsc();

  // for testing purposes, not recommended for normal use
  GLFWwindow *GetWindowObject();

  // Input management
  InputManager& GetInputManager() { return *input_manager_; }
  const InputManager& GetInputManager() const { return *input_manager_; }

  // Panel registration for centralized input
  void RegisterPanel(std::shared_ptr<Panel> panel);
  void UnregisterPanel(const std::string& panel_name);
  void ClearPanels();

 protected:
  void ApplyWindowHints(uint32_t window_hints);
  void LoadDefaultStyle();

  GLFWwindow *win_;
  std::unique_ptr<InputManager> input_manager_;
  std::vector<std::weak_ptr<Panel>> registered_panels_;
  bool exit_on_esc_ = true;
};
}  // namespace quickviz

#endif /* IMVIEW_WINDOW_HPP */
