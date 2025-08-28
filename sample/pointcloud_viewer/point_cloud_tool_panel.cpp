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
  ImGui::Text("Selection Tools");
  ImGui::Separator();
  
  if (interactive_sm && point_cloud) {
    const auto& multi_selection = interactive_sm->GetMultiSelection();
    size_t selected_count = multi_selection.Count();
    ImGui::Text("Selected Items: %zu", selected_count);
    
    if (selected_count > 0) {
      glm::vec3 centroid = multi_selection.GetCentroid();
      auto [min_pt, max_pt] = multi_selection.GetBounds();
      
      ImGui::Text("Centroid: (%.3f, %.3f, %.3f)", centroid.x, centroid.y, centroid.z);
      ImGui::Text("Bounds: (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", 
                  min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);
      
      if (ImGui::Button("Clear Selection")) {
        interactive_sm->ClearSelection();
      }
    }
    
  } else {
    ImGui::Text("No point cloud loaded");
  }
  
  ImGui::Separator();
  ImGui::Text("Point Selection Controls:");
  ImGui::BulletText("Ctrl + Left Click: Select point");
  ImGui::BulletText("Ctrl + Shift + Left Click: Add to selection");
  ImGui::BulletText("Ctrl + Alt + Left Click: Toggle point selection");
  ImGui::BulletText("Ctrl + Right Click: Clear selection");
  
  ImGui::Text("Keyboard Shortcuts:");
  ImGui::BulletText("C: Clear selection");
  ImGui::BulletText("Space: Print selection statistics");
  ImGui::BulletText("T: Toggle selection mode");

  // === MOUSE TRACKING SECTION (Collapsible) ===
  ImGui::Separator();
  if (ImGui::CollapsingHeader("Mouse Tracking", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (mouse_info_.valid) {
      ImGui::Text("Screen Position: (%.1f, %.1f)", mouse_info_.screen_pos.x,
                  mouse_info_.screen_pos.y);
      ImGui::Text("World Position: (%.3f, %.3f, %.3f)", 
                  mouse_info_.world_pos.x, mouse_info_.world_pos.y, mouse_info_.world_pos.z);
    } else {
      ImGui::Text("Mouse not in scene");
    }
  }

  ImGui::End();
}
}  // namespace quickviz