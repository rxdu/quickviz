/*
 * @file config_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/config_panel.hpp"

#include "imview/fonts.hpp"

namespace quickviz {
ConfigPanel::ConfigPanel(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetNoResize(true);
  this->SetNoMove(true);
  this->SetWindowNoMenuButton();
  this->SetWindowNoTabBar();
}

void ConfigPanel::Draw() {
  Begin();
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  ImGui::PushFont(Fonts::GetFont(FontSize::kFont16));
  {
    //----------------------- view settings -----------------------//
    if (ImGui::CollapsingHeader("Views")) {
      ImGui::Indent(16.0f);
      ImGui::PushItemWidth(80);
      {
        ImGui::SetNextItemWidth(content_size.x - 80);
        static int item_current_2 = 0;
        ImGui::Combo("##view-selection-combo", &item_current_2,
                     "Obit\0TopDown\0\0");
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
        }
      }
      ImGui::PopItemWidth();
      ImGui::Unindent(16.0f);
    }

    //----------------------- help -----------------------//
    if (ImGui::CollapsingHeader("Help")) {
      ImGui::SeparatorText("ABOUT THIS DEMO:");
      ImGui::BulletText(
          "Sections below are demonstrating many aspects of the library.");
      ImGui::BulletText(
          "The \"Examples\" menu above leads to more demo contents.");
      ImGui::BulletText(
          "The \"Tools\" menu above gives access to: About Box, Style Editor,\n"
          "and Metrics/Debugger (general purpose Dear ImGui debugging tool).");

      ImGui::SeparatorText("PROGRAMMER GUIDE:");
      ImGui::BulletText(
          "See the ShowDemoWindow() code in imgui_demo.cpp. <- you are here!");
      ImGui::BulletText("See comments in imgui.cpp.");
      ImGui::BulletText("See example applications in the examples/ folder.");
      ImGui::BulletText("Read the FAQ at ");
      ImGui::SameLine(0, 0);
      ImGui::TextLinkOpenURL("https://www.dearimgui.com/faq/");
      ImGui::BulletText(
          "Set 'io.ConfigFlags |= NavEnableKeyboard' for keyboard controls.");
      ImGui::BulletText(
          "Set 'io.ConfigFlags |= NavEnableGamepad' for gamepad controls.");

      ImGui::SeparatorText("USER GUIDE:");
      ImGui::ShowUserGuide();
    }

    //    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    //    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
    ImGui::SetCursorPos(ImVec2(0, ImGui::GetWindowHeight() - 35));
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::Indent(16.0f);
    ImGui::Text("Frame update rate: %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Unindent(16.0f);
    //    ImGui::PopStyleColor();
    //    ImGui::PopFont();
    //    ImGui::Spacing();
  }
  ImGui::PopFont();
  End();
}
}  // namespace quickviz