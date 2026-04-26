/*
 * @file editor_tool_panel.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "editor_tool_panel.hpp"

#include <utility>

#include <imgui.h>

#include "scene/tools/point_selection_tool.hpp"

#include "../commands/delete_points_command.hpp"
#include "../editor_state.hpp"

namespace quickviz::editor {

EditorToolPanel::EditorToolPanel(const std::string& name, EditorState* state)
    : Panel(name), state_(state) {}

void EditorToolPanel::Draw() {
  Begin();

  if (!state_ || !state_->IsInitialized()) {
    ImGui::TextDisabled("Editor not initialized");
    End();
    return;
  }

  ImGui::Text("Live points:    %zu / %zu", state_->LivePointCount(),
              state_->TotalPointCount());
  ImGui::Text("Selected:       %zu", state_->SelectedPointCount());

  ImGui::Separator();

  auto indices = state_->CurrentSelectionIndices();
  ImGui::BeginDisabled(indices.empty());
  if (ImGui::Button("Delete Selected")) {
    state_->History().Exec(std::make_unique<DeletePointsCommand>(
        state_, std::move(indices)));
  }
  ImGui::EndDisabled();

  ImGui::SameLine();
  ImGui::BeginDisabled(state_->SelectedPointCount() == 0);
  if (ImGui::Button("Clear Selection")) {
    if (state_->Tool()) state_->Tool()->ClearSelection();
  }
  ImGui::EndDisabled();

  ImGui::Separator();

  ImGui::TextWrapped(
      "Ctrl+Click to select points. Shift to add. Ctrl+Right-Click clears.\n"
      "Press Delete or Backspace to remove the selected points.\n"
      "Ctrl+Z to undo, Ctrl+Shift+Z or Ctrl+Y to redo.");

  End();
}

}  // namespace quickviz::editor
