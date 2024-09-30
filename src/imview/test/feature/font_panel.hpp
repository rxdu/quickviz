/*
 * @file font_panel.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_FONT_PANEL_HPP
#define QUICKVIZ_FONT_PANEL_HPP

#include "imview/viewer.hpp"

namespace quickviz {
class FontPanel : public Renderable {
 public:
  bool IsVisible() const override { return true; }
  bool IsContainer() const override { return false; }
  void OnRender() override {
    ImGui::Begin("Canvas", NULL,
                 ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBackground);

    ImGui::SetCursorPos(ImVec2(10, 5));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 0, 0, 200));
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont20));
    ImGui::Text("Text Font Normal");
    ImGui::PopFont();

    ImGui::End();
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_FONT_PANEL_HPP