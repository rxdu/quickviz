/*
 * @file imtext_scene_object.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_IMTEXT_PANEL_HPP
#define QUICKVIZ_IMTEXT_PANEL_HPP

#include "imview/panel.hpp"

namespace quickviz {
class ImTextPanel : public Panel {
 public:
  ImTextPanel(std::string name = "ImTextPanel") : Panel(name) {
    this->SetAutoLayout(false);
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

#endif  // QUICKVIZ_IMTEXT_PANEL_HPP