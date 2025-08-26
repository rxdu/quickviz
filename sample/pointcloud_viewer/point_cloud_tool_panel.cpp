/*
 * @file point_cloud_tool_panel.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "point_cloud_tool_panel.hpp"
#include "interactive_scene_manager.hpp"
#include "visualization/selection/point_cloud_selector.hpp"
#include <iostream>

using namespace quickviz::visualization;

namespace quickviz {

InteractiveSceneManager* PointCloudToolPanel::GetInteractiveSceneManager() const {
  return dynamic_cast<InteractiveSceneManager*>(scene_manager_);
}
void PointCloudToolPanel::Draw() {
  // Use explicit window begin/end to control the title
  ImGui::Begin("Point Cloud Tools");
  
  auto* interactive_sm = GetInteractiveSceneManager();
  auto* selector = interactive_sm ? interactive_sm->GetSelector() : nullptr;

  // === SELECTION TOOLS SECTION ===
  ImGui::Text("Selection Tools");
  ImGui::Separator();
  
  if (selector) {
    size_t selected_count = selector->GetSelectionCount();
    ImGui::Text("Selected Points: %zu", selected_count);
    
    if (selected_count > 0) {
      glm::vec3 centroid = selector->GetSelectionCentroid();
      ImGui::Text("Centroid: (%.2f, %.2f, %.2f)", centroid.x, centroid.y, centroid.z);
      
      if (ImGui::Button("Clear Selection")) {
        selector->ClearSelection();
        selector->ApplySelectionVisualization();
        std::cout << "Selection cleared from UI" << std::endl;
      }
      
      ImGui::SameLine();
      if (ImGui::Button("Print Stats")) {
        auto [min_pt, max_pt] = selector->GetSelectionBounds();
        std::cout << "\n=== Selection Statistics ===" << std::endl;
        std::cout << "Count: " << selected_count << " points" << std::endl;
        std::cout << "Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
        std::cout << "Min bound: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
        std::cout << "Max bound: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
        std::cout << "========================\n" << std::endl;
      }
    }
    
    ImGui::Separator();
    ImGui::Text("Region Selection Tools:");
    
    // Sphere selection
    ImGui::SliderFloat("Sphere Radius", &sphere_radius_, 0.1f, 10.0f);
    if (ImGui::Button("Select in Sphere") && mouse_info_.valid && selected_count > 0) {
      glm::vec3 centroid = selector->GetSelectionCentroid();
      auto indices = selector->SelectInSphere(centroid, sphere_radius_);
      selector->UpdateSelection(indices, SelectionMode::kAdditive);
      selector->ApplySelectionVisualization("sphere_selection", glm::vec3(0.0f, 1.0f, 0.0f), 1.3f);
      std::cout << "Sphere selection: " << indices.size() << " points around centroid" << std::endl;
    }
    
    // Box selection
    ImGui::SliderFloat("Box Size", &box_size_, 0.1f, 10.0f);
    if (ImGui::Button("Select in Box") && mouse_info_.valid && selected_count > 0) {
      glm::vec3 centroid = selector->GetSelectionCentroid();
      glm::vec3 half_size(box_size_ * 0.5f);
      auto indices = selector->SelectInBox(centroid - half_size, centroid + half_size);
      selector->UpdateSelection(indices, SelectionMode::kAdditive);
      selector->ApplySelectionVisualization("box_selection", glm::vec3(0.0f, 0.0f, 1.0f), 1.4f);
      std::cout << "Box selection: " << indices.size() << " points around centroid" << std::endl;
    }
    
    // Plane selection
    if (ImGui::Button("Select Above Z=0")) {
      auto indices = selector->SelectByPlane(glm::vec3(0, 0, 0), glm::vec3(0, 0, 1), true);
      selector->UpdateSelection(indices, SelectionMode::kSingle);
      selector->ApplySelectionVisualization("plane_selection", glm::vec3(1.0f, 0.0f, 1.0f), 1.2f);
      std::cout << "Plane selection: " << indices.size() << " points above Z=0" << std::endl;
    }
    
    ImGui::SameLine();
    if (ImGui::Button("Select Below Z=0")) {
      auto indices = selector->SelectByPlane(glm::vec3(0, 0, 0), glm::vec3(0, 0, 1), false);
      selector->UpdateSelection(indices, SelectionMode::kSingle);
      selector->ApplySelectionVisualization("plane_selection", glm::vec3(1.0f, 0.0f, 1.0f), 1.2f);
      std::cout << "Plane selection: " << indices.size() << " points below Z=0" << std::endl;
    }
    
  } else {
    ImGui::Text("No point cloud loaded");
  }
  
  ImGui::Separator();
  ImGui::Text("Mouse Controls:");
  ImGui::BulletText("Left Click: Pick point");
  ImGui::BulletText("Shift + Left Click: Add to selection");
  ImGui::BulletText("Ctrl + Left Click: Remove from selection");  
  ImGui::BulletText("Right Click: Clear selection");
  
  ImGui::Text("Keyboard Controls:");
  ImGui::BulletText("C: Clear selection");
  ImGui::BulletText("Space: Print statistics");
  ImGui::BulletText("T: Toggle selection mode");

  // === MOUSE TRACKING SECTION (Collapsible) ===
  ImGui::Separator();
  if (ImGui::CollapsingHeader("Mouse Tracking", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (mouse_info_.valid) {
      ImGui::Text("Screen Position: (%.1f, %.1f)", mouse_info_.screen_pos.x,
                  mouse_info_.screen_pos.y);

      ImGui::Text("Ray Origin:");
      ImGui::Text("  X: %.3f", mouse_info_.ray.origin.x);
      ImGui::Text("  Y: %.3f", mouse_info_.ray.origin.y);
      ImGui::Text("  Z: %.3f", mouse_info_.ray.origin.z);

      ImGui::Text("Ray Direction:");
      ImGui::Text("  X: %.3f", mouse_info_.ray.direction.x);
      ImGui::Text("  Y: %.3f", mouse_info_.ray.direction.y);
      ImGui::Text("  Z: %.3f", mouse_info_.ray.direction.z);

      // Calculate a point along the ray (e.g., at distance 10 units)
      float distance = 10.0f;
      glm::vec3 point_on_ray =
          mouse_info_.ray.origin + mouse_info_.ray.direction * distance;
      ImGui::Text("Point at distance %.1f:", distance);
      ImGui::Text("  (%.2f, %.2f, %.2f)", point_on_ray.x, point_on_ray.y,
                  point_on_ray.z);
    } else {
      ImGui::Text("Mouse not in scene");
    }
  }

  ImGui::End();
}
}  // namespace quickviz