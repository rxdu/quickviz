/*
 * @file menu_bar.cpp
 * @date 10/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/menu_bar.hpp"

namespace quickviz {
MenuBar::MenuBar(std::string name) : Panel(name) {
  this->SetAutoLayout(true);
  this->SetNoResize(false);
}

void MenuBar::Draw() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      //      ShowExampleMenuFile();
      if (ImGui::MenuItem("Exit", "ESC")) {
        if (window_should_close_callback_) window_should_close_callback_();
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "CTRL+Z")) {
      }
      if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {
      }  // Disabled item
      ImGui::Separator();
      if (ImGui::MenuItem("Cut", "CTRL+X")) {
      }
      if (ImGui::MenuItem("Copy", "CTRL+C")) {
      }
      if (ImGui::MenuItem("Paste", "CTRL+V")) {
      }
      ImGui::EndMenu();
    }
    static bool show_debug_panel = true;
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("  Show Console ", NULL, show_debug_panel)) {
        show_debug_panel = !show_debug_panel;
        if (change_debug_panel_visibility_callback_) {
          change_debug_panel_visibility_callback_(show_debug_panel);
        }
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}
}  // namespace quickviz