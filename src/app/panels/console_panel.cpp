/*
 * @file console_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/console_panel.hpp"

#include "imview/fonts.hpp"

namespace quickviz {
ConsolePanel::ConsolePanel(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetNoResize(true);
  this->SetNoMove(true);
  this->SetWindowNoMenuButton();
}

void ConsolePanel::Draw() {
  Begin();
  {
    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();
    ImGui::PopFont();
  }
  End();
}
}  // namespace quickviz