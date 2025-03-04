/**
 * @file tui_viewer.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-26
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_TERMINAL_TUI_VIEWER_HPP
#define IMVIEW_TERMINAL_TUI_VIEWER_HPP

#include <ncurses.h>

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include "imview/terminal/tui_text.hpp"
#include "imview/terminal/tui_panel.hpp"

namespace quickviz {
class TuiViewer {
 public:
  TuiViewer(const std::string &title = "TUI Composer", bool has_border = true);
  ~TuiViewer();

  bool AddSceneObject(std::shared_ptr<SceneObject> obj);
  void Show();

 private:
  void Init();
  void Deinit();
  void Resize();
  void ForceRefresh();

  std::string title_;
  bool has_border_;
  int term_size_x_ = 0;
  int term_size_y_ = 0;
  std::vector<std::shared_ptr<SceneObject>> scene_objects_;

  bool keep_running_{false};

  // for debugging
  // $ tail -f /tmp/tui_viewer.log
  std::ofstream log_file_;
};
}  // namespace quickviz

#endif /* IMVIEW_TERMINAL_TUI_VIEWER_HPP */
