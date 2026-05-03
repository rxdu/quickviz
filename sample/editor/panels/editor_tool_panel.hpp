/*
 * @file editor_tool_panel.hpp
 * @brief Minimal tool panel: shows live/selected counts and a Delete button
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_TOOL_PANEL_HPP
#define QUICKVIZ_EDITOR_TOOL_PANEL_HPP

#include <string>

#include "viewer/panel.hpp"

namespace quickviz::editor {

class EditorState;

class EditorToolPanel : public Panel {
 public:
  EditorToolPanel(const std::string& name, EditorState* state);

  void Draw() override;

 private:
  EditorState* state_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_TOOL_PANEL_HPP
