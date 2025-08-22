/*
 * @file point_cloud_tool_panel.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "point_cloud_tool_panel.hpp"

namespace quickviz {
void PointCloudToolPanel::Draw() {
  // Use explicit window begin/end to control the title
  ImGui::Begin("Point Cloud Tools");

  ImGui::Text("Mouse Tracking");
  ImGui::Separator();

  if (mouse_info_.valid) {
    ImGui::Text("Screen Position: (%.1f, %.1f)", mouse_info_.screen_pos.x,
                mouse_info_.screen_pos.y);

    ImGui::Separator();
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
    ImGui::Separator();
    ImGui::Text("Point at distance %.1f:", distance);
    ImGui::Text("  (%.2f, %.2f, %.2f)", point_on_ray.x, point_on_ray.y,
                point_on_ray.z);
  } else {
    ImGui::Text("Mouse not in scene");
  }

  ImGui::Separator();
  ImGui::Text("Selection Tools");
  ImGui::Text("Coming soon...");

  ImGui::End();
}
}  // namespace quickviz