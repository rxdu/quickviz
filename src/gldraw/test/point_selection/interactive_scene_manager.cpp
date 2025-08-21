/*
 * @file interactive_scene_manager.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "interactive_scene_manager.hpp"

#include "point_cloud_tool_panel.hpp"

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
    mouse_info.ray = GetMouseRayInWorldSpace(local_x, local_y, content_size.x,
                                             content_size.y);

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

  RenderInsideWindow();

  // Update tool panel with mouse information
  if (tool_panel_) {
    tool_panel_->UpdateMouseInfo(mouse_info);
  }

  End();
}
}  // namespace quickviz