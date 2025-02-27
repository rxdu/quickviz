/**
 * @file tui.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-26
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_TERMINAL_TUI_COMPOSER_HPP
#define IMVIEW_TERMINAL_TUI_COMPOSER_HPP

#include <ncurses.h>

#include <string>
#include <vector>
#include <memory>

#include "imview/terminal/tui_panel.hpp"

namespace quickviz {
class TuiComposer {
 public:
  TuiComposer(const std::string &title = "TUI Composer",
              bool has_border = true);
  ~TuiComposer();

  bool AddSceneObject(std::shared_ptr<SceneObject> obj);
  void Show();

 private:
  void Resize();

  std::string title_;
  bool has_border_;
  int term_size_x_ = 0;
  int term_size_y_ = 0;
  std::vector<std::shared_ptr<SceneObject>> scene_objects_;

  bool keep_running_{false};
};
}  // namespace quickviz

#endif /* IMVIEW_TERMINAL_TUI_COMPOSER_HPP */
