/*
 * @file history_panel.cpp
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#include "history_panel.hpp"

#include <imgui.h>

#include "../editor_state.hpp"

namespace quickviz::editor {

HistoryPanel::HistoryPanel(const std::string& name, EditorState* state)
    : Panel(name), state_(state) {}

void HistoryPanel::Draw() {
  Begin();

  if (!state_ || !state_->IsInitialized()) {
    ImGui::TextDisabled("Editor not initialized");
    End();
    return;
  }

  auto& history = state_->History();

  ImGui::BeginDisabled(!history.CanUndo());
  if (ImGui::Button("Undo")) history.Undo();
  ImGui::EndDisabled();
  ImGui::SameLine();
  ImGui::BeginDisabled(!history.CanRedo());
  if (ImGui::Button("Redo")) history.Redo();
  ImGui::EndDisabled();

  ImGui::Separator();
  ImGui::Text("Done: %zu  Redo: %zu", history.DoneCount(),
              history.UndoneCount());
  ImGui::Separator();

  if (ImGui::BeginChild("history_list", ImVec2(0, 0), true)) {
    auto done = history.DoneDescriptions();
    for (std::size_t i = 0; i < done.size(); ++i) {
      const bool is_top = (i + 1 == done.size());
      if (is_top) {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.4f, 1.0f), "* %s",
                           done[i].c_str());
      } else {
        ImGui::Text("  %s", done[i].c_str());
      }
    }
    auto undone = history.UndoneDescriptions();
    if (!undone.empty()) {
      ImGui::Separator();
      ImGui::TextDisabled("redoable:");
      for (const auto& d : undone) {
        ImGui::TextDisabled("  %s", d.c_str());
      }
    }
  }
  ImGui::EndChild();

  End();
}

}  // namespace quickviz::editor
