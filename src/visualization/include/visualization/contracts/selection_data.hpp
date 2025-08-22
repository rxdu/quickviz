/*
 * @file selection_data.hpp
 * @date 2025-01-22
 * @brief Clean data contract for point selection visualization
 *
 * This is a pure data structure for external applications to describe
 * point selections that should be visualized. No processing logic.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SELECTION_DATA_HPP
#define VISUALIZATION_SELECTION_DATA_HPP

#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace quickviz {
namespace visualization {

/**
 * @brief Data contract for point selection visualization
 * 
 * External processing libraries should populate this structure
 * and pass it to gldraw for visualization.
 */
struct SelectionData {
  // Core selection data
  std::vector<size_t> point_indices;
  
  // Visual properties
  glm::vec3 highlight_color{1.0f, 1.0f, 0.0f};  // Default yellow
  float size_multiplier = 1.5f;
  
  // Optional visual elements
  bool show_bounding_box = true;
  bool show_centroid = false;
  glm::vec3 bbox_color{1.0f, 1.0f, 1.0f};      // Default white
  glm::vec3 centroid_color{1.0f, 0.0f, 0.0f};  // Default red
  
  // Metadata (optional)
  std::string selection_name = "selection";
  std::string description;
  
  // Utility methods
  bool IsEmpty() const { return point_indices.empty(); }
  size_t GetCount() const { return point_indices.size(); }
};

} // namespace visualization
} // namespace quickviz

#endif // VISUALIZATION_SELECTION_DATA_HPP