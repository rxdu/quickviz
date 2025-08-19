/**
 * @file selection_result.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2024-12-19
 * @brief Selection result data structures for point cloud interaction
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SELECTION_RESULT_HPP
#define QUICKVIZ_SELECTION_RESULT_HPP

#include <vector>
#include <functional>
#include <glm/glm.hpp>

namespace quickviz {

/**
 * @brief Result of a point cloud selection operation
 */
struct SelectionResult {
  // Selected point indices
  std::vector<size_t> indices;
  
  // Bounding box of selected points
  glm::vec3 min_bounds;
  glm::vec3 max_bounds;
  
  // Centroid of selected points
  glm::vec3 centroid;
  
  // Number of selected points
  size_t count = 0;
  
  // Selection method used
  enum class Method {
    kPointPick,     // Single point picking
    kRectangle,     // Rectangle selection
    kLasso,         // Lasso/polygon selection
    kRadius,        // Radius-based selection around a point
    kProgrammatic   // Programmatically set selection
  };
  Method method = Method::kProgrammatic;
  
  // Screen-space selection region (for rectangle/lasso)
  std::vector<glm::vec2> screen_region;
  
  // Clear the selection
  void Clear() {
    indices.clear();
    min_bounds = glm::vec3(0.0f);
    max_bounds = glm::vec3(0.0f);
    centroid = glm::vec3(0.0f);
    count = 0;
    screen_region.clear();
  }
  
  // Check if selection is empty
  bool IsEmpty() const {
    return indices.empty();
  }
  
  // Compute bounds and centroid from points
  void ComputeStatistics(const std::vector<glm::vec3>& points);
};

/**
 * @brief Selection operation mode
 */
enum class SelectionMode {
  kReplace,  // Replace current selection
  kAdd,      // Add to current selection
  kRemove,   // Remove from current selection
  kToggle    // Toggle selection state
};

/**
 * @brief Selection callback function type
 * 
 * Called when selection changes with the new selection result
 */
using SelectionCallback = std::function<void(const SelectionResult&)>;

/**
 * @brief Selection hover callback function type
 * 
 * Called when hovering over a point (for highlighting)
 * Returns -1 if no point is hovered
 */
using HoverCallback = std::function<void(int point_index)>;

}  // namespace quickviz

#endif  // QUICKVIZ_SELECTION_RESULT_HPP