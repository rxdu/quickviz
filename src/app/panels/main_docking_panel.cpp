/*
 * @file main_docking_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/main_docking_panel.hpp"

#include "imgui.h"
#include "imgui_internal.h"

namespace quickviz {
MainDockingPanel::MainDockingPanel(std::string name) : Panel(name) {
  this->SetAutoLayout(true);
  this->SetNoResize(true);
  this->SetNoMove(true);
  this->SetNoTitleBar(true);
  this->SetNoBackground(true);

  scene_panel_.SetNoMove(true);
  scene_panel_.SetWindowNoTabBar();
}

void MainDockingPanel::Draw() {
  if (!IsAutoLayout()) {
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    ImGui::SetNextWindowPos(ImVec2(0, 0));
  }

  // set up layout
  Begin();

  // set up dockspace
  dockspace_id_ = ImGui::GetID("MainDockingPanel");
  ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_NoResize;
  ImGui::DockSpace(dockspace_id_, ImGui::GetWindowSize(), dockspace_flags);
  {
    if (!layout_initialized_) {
      dockspace_id_ = ImGui::DockBuilderAddNode(dockspace_id_, dockspace_flags);

      ImGui::DockBuilderSplitNode(dockspace_id_, ImGuiDir_Left, 0.2f,
                                  &config_panel_node_, &gl_scene_widget_node_);
      ImGui::DockBuilderSplitNode(gl_scene_widget_node_, ImGuiDir_Up, 0.8f,
                                  &gl_scene_widget_node_, &console_panel_node_);

      ImGui::DockBuilderDockWindow(config_panel_.GetName().c_str(),
                                   config_panel_node_);
      ImGui::DockBuilderDockWindow(scene_panel_.GetName().c_str(),
                                   gl_scene_widget_node_);
      ImGui::DockBuilderDockWindow(console_panel_.GetName().c_str(),
                                   console_panel_node_);

      layout_initialized_ = true;
    }

    ImGui::DockBuilderSetNodePos(dockspace_id_, ImGui::GetWindowPos());
    ImGui::DockBuilderSetNodeSize(dockspace_id_, ImGui::GetWindowSize());
  }

  // draw child panels
  if (config_panel_.IsVisible()) config_panel_.Draw();
  if (console_panel_.IsVisible()) console_panel_.Draw();
  if (scene_panel_.IsVisible()) scene_panel_.Draw();

  End();
}

void MainDockingPanel::ChangeDebugPanelVisibility(bool visible) {
  console_panel_.SetVisibility(visible);
}
}  // namespace quickviz