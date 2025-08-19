/**
 * @file selection_result.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2024-12-19
 * @brief Implementation of selection result data structures
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "renderer/selection/selection_result.hpp"

#include <limits>

namespace quickviz {

void SelectionResult::ComputeStatistics(const std::vector<glm::vec3>& points) {
  if (indices.empty() || points.empty()) {
    Clear();
    return;
  }
  
  // Initialize bounds
  min_bounds = glm::vec3(std::numeric_limits<float>::max());
  max_bounds = glm::vec3(std::numeric_limits<float>::lowest());
  centroid = glm::vec3(0.0f);
  
  // Compute bounds and accumulate for centroid
  size_t valid_count = 0;
  for (size_t idx : indices) {
    if (idx >= points.size()) continue;
    
    const glm::vec3& point = points[idx];
    
    // Update bounds
    min_bounds = glm::min(min_bounds, point);
    max_bounds = glm::max(max_bounds, point);
    
    // Accumulate for centroid
    centroid += point;
    valid_count++;
  }
  
  // Compute centroid
  if (valid_count > 0) {
    centroid /= static_cast<float>(valid_count);
    count = valid_count;
  } else {
    Clear();
  }
}

}  // namespace quickviz