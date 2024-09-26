/*
 * test_wgui.cpp
 *
 * Created on: Jul 22, 2021 14:50
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "imview/window.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Window win("Test Window", 1920, 1080);

  while (!win.ShouldClose()) {
    // handle events
    win.PollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) {
      win.CloseWindow();
    }

    // draw stuff
    win.StartNewFrame();

    {
      ImGui::Begin("Canvas ");
      //      ImGui::Begin("Canvas ", NULL,
      //                   ImGuiWindowFlags_NoTitleBar |
      //                   ImGuiWindowFlags_NoResize |
      //                       ImGuiWindowFlags_NoMove);
      ImGui::Text("Text Sample");
      //      ImGui::PushFont(win.GetFont(FontSize::Normal));
      //      ImGui::Text("Text Font Normal");
      //      ImGui::PopFont();
      //
      //      ImGui::PushFont(win.GetFont(FontSize::Tiny));
      //      ImGui::Text("Test Font Tiny");
      //      ImGui::PopFont();
      //
      //      ImGui::PushFont(win.GetFont(FontSize::Small));
      //      ImGui::Text("Text Font Small");
      //      ImGui::PopFont();
      //
      //      ImGui::PushFont(win.GetFont(FontSize::Big));
      //      ImGui::Text("Text Font Big");
      //      ImGui::PopFont();
      //
      //      ImGui::PushFont(win.GetFont(FontSize::Large));
      //      ImGui::Text("Text Font Large");
      //      ImGui::PopFont();
      //
      //      ImGui::PushFont(win.GetFont(FontSize::ExtraLarge));
      //      ImGui::Text("Text Font ExtraLarge");
      //      ImGui::PopFont();

      ImGui::End();
    }

    win.RenderFrame();
  }

  return 0;
}