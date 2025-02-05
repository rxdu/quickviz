/*
 * @file imgui_fixed_panel.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMGUI_FIXED_PANEL_HPP
#define QUICKVIZ_IMGUI_FIXED_PANEL_HPP

#include "imview/viewer.hpp"
#include "imview/panel.hpp"

namespace quickviz {
class ImGuiFixedPanel : public Panel {
 public:
  ImGuiFixedPanel(std::string name = "ImGuiFixedPanel") : Panel(name) {
    this->SetAutoLayout(true);
    this->SetNoMove(true);
    this->SetNoBringToFrontOnFocus(true);
    this->SetNoInputs();
    this->SetNoCollapse(true);
    this->SetNoResize(true);
    this->SetNoScrollbar(true);
    // this->SetNoBackground(true);
  }

  void Draw() override {
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
};
}  // namespace quickviz

#endif  // QUICKVIZ_IMGUI_FIXED_PANEL_HPP