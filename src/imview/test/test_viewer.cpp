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

class MyViewer : public Viewer {
  void Draw() {
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

    ImGui::SetCursorPos(ImVec2(180, 200));
    ImGui::PushFont(GetFont(FontSize::Normal));
    ImGui::Text("Canvas");
    ImGui::PopFont();

    ImGui::End();
  }
};

int main(int argc, char* argv[]) {
  MyViewer viewer;
  viewer.Show();
  return 0;
}