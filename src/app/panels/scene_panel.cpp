/*
 * @file scene_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/scene_panel.hpp"

#include "glad/glad.h"
#include "imview/fonts.hpp"

namespace quickviz {
ScenePanel::ScenePanel(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetNoResize(true);
//  this->SetNoMove(true);
  this->SetWindowNoMenuButton();
}

void ScenePanel::Draw() {
  Begin();
  {
    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();
    ImGui::PopFont();

    glEnable(GL_SCISSOR_TEST);
    glViewport(x_, y_, width_, height_);
    glScissor(x_, y_, width_, height_);

    glClearColor(0.5, 0, 0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_SCISSOR_TEST);
  }
  End();
}
}  // namespace quickviz