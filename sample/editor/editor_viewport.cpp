/*
 * @file editor_viewport.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "editor_viewport.hpp"

#include <utility>

#include <imgui.h>

#include "commands/delete_points_command.hpp"
#include "editor_state.hpp"

namespace quickviz::editor {

EditorViewport::EditorViewport(const std::string& name, EditorState* state)
    : GlScenePanel(name), state_(state) {
  // Built-in scene-input selection conflicts with PointSelectionTool. Disable
  // it here, same pattern used by sample/pointcloud_viewer.
  GetSceneInputHandler()->SetSelectionEnabled(false);

  tool_ = PointSelectionToolFactory::CreateForPointCloud(
      GetSceneManager(), kEditableCloudName, "editor_point_select");
  GetSceneManager()->RegisterTool(tool_);
  GetSceneManager()->ActivateTool("editor_point_select");
}

void EditorViewport::Draw() {
  // Mirror GlScenePanel::Draw() but slot HandleShortcuts() inside the
  // ImGui window scope so IsWindowFocused/IsWindowHovered work correctly.
  Begin();
  HandleShortcuts();
  RenderInsideWindow();
  End();
}

void EditorViewport::HandleShortcuts() {
  if (!state_ || !state_->IsInitialized()) return;

  // We are inside ImGui's frame; ImGuiIO is the authoritative input source.
  ImGuiIO& io = ImGui::GetIO();
  const bool ctrl = io.KeyCtrl;
  const bool shift = io.KeyShift;

  // Only consume keys when this panel is focused; otherwise the shortcut
  // would fire even while typing in (future) text inputs elsewhere.
  if (!ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows |
                              ImGuiFocusedFlags_DockHierarchy) &&
      !ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows)) {
    return;
  }

  if (ctrl && !shift && ImGui::IsKeyPressed(ImGuiKey_Z, /*repeat=*/false)) {
    state_->History().Undo();
    return;
  }
  if (ctrl && (shift && ImGui::IsKeyPressed(ImGuiKey_Z, /*repeat=*/false))) {
    state_->History().Redo();
    return;
  }
  if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Y, /*repeat=*/false)) {
    state_->History().Redo();
    return;
  }

  if (ImGui::IsKeyPressed(ImGuiKey_Delete, /*repeat=*/false) ||
      ImGui::IsKeyPressed(ImGuiKey_Backspace, /*repeat=*/false)) {
    auto indices = state_->CurrentSelectionIndices();
    if (!indices.empty()) {
      state_->History().Exec(std::make_unique<DeletePointsCommand>(
          state_, std::move(indices)));
    }
    return;
  }

  if (ImGui::IsKeyPressed(ImGuiKey_Escape, /*repeat=*/false)) {
    if (state_->Tool()) state_->Tool()->ClearSelection();
  }
}

}  // namespace quickviz::editor
