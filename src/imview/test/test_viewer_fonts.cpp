/*
 * test_viewer.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/viewer.hpp"

using namespace quickviz;

class FontPanel : public Renderable {
 public:
  void Render() {
    ImGui::Begin("Canvas", NULL,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
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

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont16));
    ImGui::Text("Test Font Tiny");
    ImGui::PopFont();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    ImGui::Text("Text Font Small");
    ImGui::PopFont();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont28));
    ImGui::Text("Text Font Big");
    ImGui::PopFont();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont32));
    ImGui::Text("Text Font Large");
    ImGui::PopFont();

    ImGui::PushFont(Fonts::GetFont(FontSize::kFont40));
    ImGui::Text("Text Font ExtraLarge");
    ImGui::PopFont();

    ImGui::SetCursorPos(ImVec2(180, 200));
    ImGui::PushFont(Fonts::GetFont(FontSize::kFont20));
    ImGui::Text("Canvas");
    ImGui::PopFont();

    ImGui::End();
  }
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto panel = std::make_shared<FontPanel>();
  viewer.AddRenderable(0, panel);
  //  viewer.AddRenderable(2, panel);

  viewer.Show();
  return 0;
}