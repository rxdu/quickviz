/*
 * @file point_cloud_tool_panel.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "point_cloud_tool_panel.hpp"
#include "interactive_scene_manager.hpp"
#include <iostream>
#include <imgui.h>

namespace quickviz {

InteractiveSceneManager* PointCloudToolPanel::GetInteractiveSceneManager() const {
  return scene_manager_;
}
void PointCloudToolPanel::Draw() {
  // Use explicit window begin/end to control the title
  ImGui::Begin("Point Cloud Tools");
  
  auto* interactive_sm = GetInteractiveSceneManager();
  // TODO: Need to get point cloud reference from scene manager
  PointCloud* point_cloud = nullptr;
  // For now, try to get the point cloud from the scene manager directly
  if (interactive_sm) {
    auto* gl_object = interactive_sm->GetOpenGLObject("point_cloud");
    point_cloud = dynamic_cast<PointCloud*>(gl_object);
  }

  // === APPEARANCE CONTROLS SECTION ===
  ImGui::Text("Appearance Controls");
  ImGui::Separator();
  
  if (point_cloud) {
    // Synchronize slider with point cloud's current point size
    point_size_ = point_cloud->GetPointSize();
    
    if (ImGui::SliderFloat("Point Size", &point_size_, 0.5f, 10.0f, "%.1f")) {
      point_cloud->SetPointSize(point_size_);
    }
  } else {
    ImGui::Text("No point cloud loaded");
  }
  
  ImGui::Separator();

  // === SELECTION TOOLS SECTION ===
  DrawToolSelectionUI();
  DrawPointSelectionControls();

  // === MOUSE TRACKING SECTION (Collapsible) ===
  ImGui::Separator();
  if (ImGui::CollapsingHeader("Mouse Tracking", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (mouse_info_.valid) {
      ImGui::Text("Screen Position: (%.1f, %.1f)", mouse_info_.screen_pos.x,
                  mouse_info_.screen_pos.y);
      // ImGui::Text("World Position: (%.3f, %.3f, %.3f)",
      //             mouse_info_.world_pos.x, mouse_info_.world_pos.y, mouse_info_.world_pos.z);
    } else {
      ImGui::Text("Mouse not in scene");
    }
  }

  ImGui::End();
}

void PointCloudToolPanel::DrawToolSelectionUI() {
  ImGui::Text("Interactive Tools");
  ImGui::Separator();
  
  auto* interactive_sm = GetInteractiveSceneManager();
  if (!interactive_sm) {
    ImGui::Text("Scene manager not available");
    return;
  }
  
  // Check if point selection tool is active
  auto active_tool = interactive_sm->GetSceneManager()->GetActiveTool();
  auto point_tool = GetPointSelectionTool();
  point_selection_tool_active_ = (active_tool == point_tool);
  
  // Toggle button for point selection tool
  if (ImGui::Checkbox("Point Selection Tool", &point_selection_tool_active_)) {
    if (point_selection_tool_active_) {
      // Activate point selection tool
      if (point_tool) {
        interactive_sm->GetSceneManager()->ActivateTool("point_select");
      } else {
        std::cerr << "Point selection tool not found!" << std::endl;
        point_selection_tool_active_ = false;
      }
    } else {
      // Deactivate current tool
      interactive_sm->GetSceneManager()->GetTools().DeactivateCurrentTool();
    }
  }
  
  // Show active tool status
  if (active_tool) {
    ImGui::SameLine();
    ImGui::Text("(Active: %s)", active_tool->GetDisplayName().c_str());
  }
}

void PointCloudToolPanel::DrawPointSelectionControls() {
  ImGui::Separator();
  ImGui::Text("Point Selection Settings");
  
  auto point_tool = GetPointSelectionTool();
  if (!point_tool) {
    ImGui::Text("Point selection tool not available");
    return;
  }
  
  // Selection radius control
  selection_radius_ = point_tool->GetSelectionRadius();
  if (ImGui::SliderInt("Selection Radius (px)", &selection_radius_, 1, 20)) {
    point_tool->SetSelectionRadius(selection_radius_);
  }
  
  // Selection mode control
  const char* selection_modes[] = {"Single", "Add", "Toggle", "Subtract"};
  selection_mode_index_ = static_cast<int>(point_tool->GetSelectionMode());
  
  if (ImGui::Combo("Selection Mode", &selection_mode_index_, selection_modes, IM_ARRAYSIZE(selection_modes))) {
    point_tool->SetSelectionMode(static_cast<PointSelectionTool::SelectionMode>(selection_mode_index_));
  }
  
  // Visual feedback controls
  auto feedback = point_tool->GetVisualFeedback();
  bool feedback_changed = false;
  
  if (ImGui::Checkbox("Show Hover Highlight", &feedback.show_hover_highlight)) {
    feedback_changed = true;
  }
  
  if (ImGui::Checkbox("Show Selection Count", &feedback.show_selection_count)) {
    feedback_changed = true;
  }
  
  if (ImGui::ColorEdit3("Hover Color", &feedback.hover_color[0])) {
    feedback_changed = true;
  }
  
  if (ImGui::ColorEdit3("Selection Color", &feedback.selection_color[0])) {
    feedback_changed = true;
  }
  
  if (feedback_changed) {
    point_tool->SetVisualFeedback(feedback);
  }
  
  // Selection status and controls
  ImGui::Separator();
  ImGui::Text("Selection Status");
  
  auto* interactive_sm = GetInteractiveSceneManager();
  if (interactive_sm) {
    const auto& multi_selection = interactive_sm->GetMultiSelection();
    size_t selected_count = multi_selection.Count();
    
    ImGui::Text("Selected Points: %zu", selected_count);
    
    if (selected_count > 0) {
      // Show selection statistics
      glm::vec3 centroid = multi_selection.GetCentroid();
      auto [min_pt, max_pt] = multi_selection.GetBounds();
      
      ImGui::Text("Centroid: (%.3f, %.3f, %.3f)", centroid.x, centroid.y, centroid.z);
      ImGui::Text("Min: (%.2f, %.2f, %.2f)", min_pt.x, min_pt.y, min_pt.z);
      ImGui::Text("Max: (%.2f, %.2f, %.2f)", max_pt.x, max_pt.y, max_pt.z);
      
      // Clear selection button
      if (ImGui::Button("Clear Selection")) {
        point_tool->ClearSelection();
      }
      
      ImGui::SameLine();
      if (ImGui::Button("Print Selection")) {
        auto point_selections = multi_selection.GetPoints();
        std::cout << "\n=== Point Selection Details ===" << std::endl;
        for (const auto& sel : point_selections) {
          std::cout << "Cloud: " << sel.cloud_name 
                    << ", Point: " << sel.point_index 
                    << ", Position: (" << sel.world_position.x 
                    << ", " << sel.world_position.y 
                    << ", " << sel.world_position.z << ")" << std::endl;
        }
      }
    }
  }
  
  // Usage instructions
  ImGui::Separator();
  ImGui::Text("Usage:");
  ImGui::BulletText("Left Click: Select point (mode dependent)");
  ImGui::BulletText("Ctrl + Left Click: Add to selection");
  ImGui::BulletText("Shift + Left Click: Toggle selection");
  ImGui::BulletText("Alt + Left Click: Remove from selection");
  ImGui::BulletText("Escape: Clear all selections");
}

std::shared_ptr<PointSelectionTool> PointCloudToolPanel::GetPointSelectionTool() const {
  auto* interactive_sm = GetInteractiveSceneManager();
  if (!interactive_sm) {
    return nullptr;
  }
  
  auto tool = interactive_sm->GetSceneManager()->GetTools().GetTool("point_select");
  return std::dynamic_pointer_cast<PointSelectionTool>(tool);
}

}  // namespace quickviz