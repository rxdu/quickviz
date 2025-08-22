/*
 * @file menu_bar.hpp
 * @date 10/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MENU_BAR_HPP
#define XMOTION_MENU_BAR_HPP

#include <functional>

#include "imview/panel.hpp"

namespace quickviz {
class MenuBar : public Panel {
 public:
  MenuBar(std::string name = "Menu");

  void Draw() override;

  using ChangeDebugPanelVisibilityCallback = std::function<void(bool)>;
  void SetChangeDebugPanelVisibilityCallback(
      std::function<void(bool)> callback) {
    change_debug_panel_visibility_callback_ = callback;
  }

  using WindowShouldCloseCallback = std::function<void()>;
  void SetWindowShouldCloseCallback(std::function<void()> callback) {
    window_should_close_callback_ = callback;
  }

 private:
  ChangeDebugPanelVisibilityCallback change_debug_panel_visibility_callback_;
  WindowShouldCloseCallback window_should_close_callback_;
};
}  // namespace quickviz

#endif  // XMOTION_MENU_BAR_HPP