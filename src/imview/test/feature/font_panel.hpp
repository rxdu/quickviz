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
#include "imview/panel.hpp"

namespace quickviz {
class FontPanel : public Panel {
 public:
  FontPanel(std::string text = "Text Font Normal")
      : Panel("FontPanel"), text_(text) {}
  bool IsVisible() const override { return true; }
  bool IsContainer() const override { return false; }

  void OnRender() override {
    ImGui::SetNextWindowPos(ImVec2(x_, y_));
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
//    std::cout << "Next window: " << x_ << ", " << y_ << ", " << width_ << ", "
//              << height_ << std::endl;
    ImGui::Begin("Canvas", NULL,
                 ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoBackground);

    ImGui::SetCursorPos(ImVec2(0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 0, 0, 200));
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont20));
    ImGui::Text(text_.c_str());
    ImGui::PopFont();

    ImGui::End();
  }

  std::string text_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_FONT_PANEL_HPP