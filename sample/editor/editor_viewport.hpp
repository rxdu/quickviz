/*
 * @file editor_viewport.hpp
 * @brief 3D viewport with editor-side keyboard shortcuts
 *
 * Extends GlScenePanel — the library's ImGui+GL scene host — with shortcuts
 * for Undo/Redo and selection-driven deletion. The selection tool is set up
 * in the constructor and exposed for the rest of the app to wire commands.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EDITOR_VIEWPORT_HPP
#define QUICKVIZ_EDITOR_VIEWPORT_HPP

#include <memory>
#include <string>

#include "scene/gl_scene_panel.hpp"
#include "scene/tools/point_selection_tool.hpp"

namespace quickviz::editor {

class EditorState;

class EditorViewport : public GlScenePanel {
 public:
  EditorViewport(const std::string& name, EditorState* state);

  // Pointer to the selection tool that this viewport owns and registered with
  // its SceneManager. Non-owning — the SceneManager keeps the tool alive.
  PointSelectionTool* GetTool() { return tool_.get(); }

  void Draw() override;

 private:
  void HandleShortcuts();

  EditorState* state_;
  std::shared_ptr<PointSelectionTool> tool_;
};

}  // namespace quickviz::editor

#endif  // QUICKVIZ_EDITOR_VIEWPORT_HPP
