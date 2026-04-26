/*
 * @file history_panel.hpp
 * @brief Minimal history panel listing the editor command stack
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_HISTORY_PANEL_HPP
#define QUICKVIZ_EDITOR_HISTORY_PANEL_HPP

#include <string>

#include "imview/panel.hpp"

namespace quickviz::editor {

class EditorState;

class HistoryPanel : public Panel {
 public:
  HistoryPanel(const std::string& name, EditorState* state);

  void Draw() override;

 private:
  EditorState* state_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_HISTORY_PANEL_HPP
