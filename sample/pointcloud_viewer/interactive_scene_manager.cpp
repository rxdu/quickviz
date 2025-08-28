/*
 * @file interactive_scene_manager.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "interactive_scene_manager.hpp"

#include "point_cloud_tool_panel.hpp"
#include <iostream>
#include <imgui.h>
#include <glad/glad.h>

namespace quickviz {
// Implementation of InteractiveSceneManager::Draw
void InteractiveSceneManager::Draw() {
  Begin();

  // Store the mouse tracking state before calling RenderInsideWindow
  PointCloudToolPanel::MouseInfo mouse_info;

  // Get current mouse state directly using ImGui
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // Calculate mouse position relative to this window's content area
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
  float local_x = io.MousePos.x - window_pos.x - window_content_min.x;
  float local_y = io.MousePos.y - window_pos.y - window_content_min.y;

  // Check if mouse is inside the content area and window is hovered
  bool mouse_in_content = (local_x >= 0 && local_x <= content_size.x &&
                           local_y >= 0 && local_y <= content_size.y);
  bool window_hovered = ImGui::IsWindowHovered();

  if (window_hovered && mouse_in_content) {
    mouse_info.valid = true;
    mouse_info.screen_pos = glm::vec2(local_x, local_y);
    // Use placeholder world position (could be enhanced with depth buffer sampling)
    mouse_info.world_pos = glm::vec3(0.0f, 0.0f, 0.0f);

    // Draw crosshair overlay
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Calculate absolute position for crosshair
    float abs_x = window_pos.x + window_content_min.x + local_x;
    float abs_y = window_pos.y + window_content_min.y + local_y;

    // Crosshair parameters
    const float crosshair_size = 15.0f;
    const float gap = 3.0f;
    const ImU32 color =
        IM_COL32(0, 255, 0, 200);  // Green with some transparency
    const float thickness = 2.0f;

    // Draw horizontal line (with gap in middle)
    draw_list->AddLine(ImVec2(abs_x - crosshair_size, abs_y),
                       ImVec2(abs_x - gap, abs_y), color, thickness);
    draw_list->AddLine(ImVec2(abs_x + gap, abs_y),
                       ImVec2(abs_x + crosshair_size, abs_y), color, thickness);

    // Draw vertical line (with gap in middle)
    draw_list->AddLine(ImVec2(abs_x, abs_y - crosshair_size),
                       ImVec2(abs_x, abs_y - gap), color, thickness);
    draw_list->AddLine(ImVec2(abs_x, abs_y + gap),
                       ImVec2(abs_x, abs_y + crosshair_size), color, thickness);

    // Optional: Draw center dot
    draw_list->AddCircleFilled(ImVec2(abs_x, abs_y), 2.0f,
                               IM_COL32(255, 255, 0, 255));
  }

  // Handle input if selection is enabled
  if (selection_enabled_) {
    HandleMouseInput();
    HandleKeyboardInput();
  }

  RenderInsideWindow();

  // Update tool panel with mouse information
  if (tool_panel_) {
    tool_panel_->UpdateMouseInfo(mouse_info);
  }

  End();
}

void InteractiveSceneManager::SetPointCloud(std::unique_ptr<PointCloud> point_cloud) {
  if (point_cloud) {
    std::cout << "Setting point cloud with " << point_cloud->GetPointCount() << " points" << std::endl;
    
    // Add to scene (transfer ownership)
    AddOpenGLObject("point_cloud", std::move(point_cloud));
    
    // Get back as pointer and cast to PointCloud for selection
    auto* gl_object = GetOpenGLObject("point_cloud");
    std::cout << "Retrieved OpenGL object: " << gl_object << std::endl;
    
    auto* point_cloud_ptr = dynamic_cast<PointCloud*>(gl_object);
    std::cout << "Cast to PointCloud: " << point_cloud_ptr << std::endl;
    
    if (point_cloud_ptr) {
      SetActivePointCloud(point_cloud_ptr);
      std::cout << "Active point cloud set successfully" << std::endl;
    } else {
      std::cout << "Failed to cast to PointCloud!" << std::endl;
    }
    
    // Set selection callback
    SetPointSelectionCallback([this](const std::vector<size_t>& indices) {
      std::cout << "Selection changed: " << indices.size() << " points selected" << std::endl;
      
      if (!indices.empty()) {
        glm::vec3 centroid = GetSelectionCentroid();
        auto [min_pt, max_pt] = GetSelectionBounds();
        
        std::cout << "  Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
        std::cout << "  Bounds: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ") to ("
                  << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
      }
    });
    
    std::cout << "Point selection system initialized for " << point_cloud_ptr->GetPointCount() << " points" << std::endl;
    
    std::cout << "\n=== Point Selection Controls ===" << std::endl;
    std::cout << "  Ctrl + Left Click: Select point (single selection)" << std::endl;
    std::cout << "  Ctrl + Shift + Left Click: Add point to selection" << std::endl;
    std::cout << "  Ctrl + Alt + Left Click: Toggle point selection" << std::endl;
    std::cout << "  Ctrl + Right Click: Clear selection" << std::endl;
    std::cout << "  C key: Clear selection" << std::endl;
    std::cout << "  Space key: Show selection statistics" << std::endl;
    std::cout << "  T key: Toggle selection mode on/off" << std::endl;
    std::cout << "==============================\n" << std::endl;
  }
}

void InteractiveSceneManager::HandleMouseInput() {
  if (!selection_enabled_) return;
  if (!GetActivePointCloud()) return;
  
  ImGuiIO& io = ImGui::GetIO();
  
  // Get current mouse position relative to window
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
  float local_x = io.MousePos.x - window_pos.x - window_content_min.x;
  float local_y = io.MousePos.y - window_pos.y - window_content_min.y;
  
  // Check if mouse is inside the content area
  bool mouse_in_viewport = (local_x >= 0 && local_x <= content_size.x &&
                            local_y >= 0 && local_y <= content_size.y);
  
  // Check if this window is hovered (not blocked by other ImGui windows)
  bool window_hovered = ImGui::IsWindowHovered();
  
  // Handle Ctrl+left click for point picking (to avoid interfering with camera controls)
  if (mouse_in_viewport && window_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && io.KeyCtrl) {
    // Use the integrated GlSceneManager selection API
    if (io.KeyShift) {
      // Ctrl+Shift = add to selection
      AddPointAt(local_x, local_y, 3);
    } else if (io.KeyAlt) {
      // Ctrl+Alt = toggle selection
      TogglePointAt(local_x, local_y, 3);
    } else {
      // Ctrl alone = single selection (replace)
      SelectPointAt(local_x, local_y, 3);
    }
  }
  
  // Handle Ctrl+right click to clear selection
  if (mouse_in_viewport && window_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && io.KeyCtrl) {
    ClearPointSelection();
  }
}

void InteractiveSceneManager::HandleKeyboardInput() {
  // Handle keyboard shortcuts
  if (ImGui::IsKeyPressed(ImGuiKey_C)) {
    // Clear selection
    ClearPointSelection();
  }
  
  if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
    // Print selection statistics
    if (GetSelectedPointCount() > 0) {
      size_t count = GetSelectedPointCount();
      auto [min_pt, max_pt] = GetSelectionBounds();
      glm::vec3 centroid = GetSelectionCentroid();
      
      std::cout << "\n=== Selection Statistics ===" << std::endl;
      std::cout << "Count: " << count << " points" << std::endl;
      std::cout << "Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
      std::cout << "Min bound: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
      std::cout << "Max bound: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
      std::cout << "========================\n" << std::endl;
    } else {
      std::cout << "No points selected" << std::endl;
    }
  }
  
  if (ImGui::IsKeyPressed(ImGuiKey_T)) {
    // Toggle selection mode
    selection_enabled_ = !selection_enabled_;
    std::cout << "Selection " << (selection_enabled_ ? "enabled" : "disabled") << std::endl;
  }
}

}  // namespace quickviz