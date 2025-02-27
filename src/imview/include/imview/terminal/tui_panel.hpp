/**
 * @file tui_panel.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_TERMINAL_TUI_PANEL_HPP
#define IMVIEW_TERMINAL_TUI_PANEL_HPP

#include <ncurses.h>

#include <memory>
#include <unordered_map>

#include "imview/scene_object.hpp"

namespace quickviz {
class TuiPanel : public SceneObject {
 public:
  TuiPanel(const std::string &title = "TUI Panel");
  virtual ~TuiPanel();

  // public API
  void OnRender() override;
  void OnResize(float width, float height) override;
  void OnJoystickUpdate(const JoystickInput &input) override {};

  void SetNoBorder(bool value);

  // for derived classes
  virtual void Draw() = 0;

 protected:
  bool has_border_ = true;
  WINDOW *window_;
};
}  // namespace quickviz

#endif /* IMVIEW_TERMINAL_TUI_PANEL_HPP */
