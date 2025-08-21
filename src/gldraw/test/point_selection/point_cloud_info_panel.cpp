/*
 * @file point_cloud_info_panel.cpp
 * @date 8/21/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "point_cloud_info_panel.hpp"

#include <filesystem>

namespace quickviz {
void PointCloudInfoPanel::Draw() {
  // Use explicit window begin/end to control the title
  ImGui::Begin("Point Cloud Info");

  // ImGui::Text("Point Cloud Information");
  // ImGui::Separator();
  ImGui::Dummy({0.0f, 5.0f});  // Add some space before the title

  ImGui::Text("File: %s",
              std::filesystem::path(metadata_.filename).filename().c_str());
  ImGui::Text("Format: %s", metadata_.format.c_str());
  ImGui::Text("PCL Type: %s", metadata_.detected_pcl_type.c_str());
  ImGui::Text("Points: %zu", metadata_.point_count);
  ImGui::Text("File Size: %.2f MB", metadata_.file_size_mb);

  ImGui::Separator();
  ImGui::Text("Available Fields:");
  ImGui::BulletText("XYZ: %s", metadata_.fields.HasXYZ() ? "Yes" : "No");
  ImGui::BulletText("RGB: %s", metadata_.fields.HasRGBColor() ? "Yes" : "No");
  ImGui::BulletText("RGBA: %s", metadata_.fields.HasRGBAColor() ? "Yes" : "No");
  ImGui::BulletText("Intensity: %s",
                    metadata_.fields.has_intensity ? "Yes" : "No");
  ImGui::BulletText("Normals: %s",
                    metadata_.fields.HasNormals() ? "Yes" : "No");

  ImGui::Separator();
  ImGui::Text("Bounding Box:");
  ImGui::Text("  Min: (%.2f, %.2f, %.2f)", metadata_.min_bounds.x,
              metadata_.min_bounds.y, metadata_.min_bounds.z);
  ImGui::Text("  Max: (%.2f, %.2f, %.2f)", metadata_.max_bounds.x,
              metadata_.max_bounds.y, metadata_.max_bounds.z);

  glm::vec3 panel_size = metadata_.max_bounds - metadata_.min_bounds;
  ImGui::Text("  Size: (%.2f, %.2f, %.2f)", panel_size.x, panel_size.y,
              panel_size.z);

  glm::vec3 center = (metadata_.min_bounds + metadata_.max_bounds) * 0.5f;
  ImGui::Text("  Center: (%.2f, %.2f, %.2f)", center.x, center.y, center.z);

  ImGui::End();
}
}  // namespace quickviz