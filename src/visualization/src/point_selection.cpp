/**
 * @file point_selection.cpp
 * @date 2025-08-26
 * @brief Implementation of simplified point selection utilities
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "visualization/point_selection.hpp"
#include <algorithm>
#include <numeric>

namespace quickviz {
namespace visualization {

PointSelection::PointSelection(std::shared_ptr<PointCloud> cloud, GlSceneManager* scene_manager)
    : point_cloud_(cloud), scene_manager_(scene_manager) {
  if (!point_cloud_ || !scene_manager_) {
    throw std::invalid_argument("PointSelection: point_cloud and scene_manager cannot be null");
  }
}

// === Selection Operations ===

bool PointSelection::SelectPoint(float screen_x, float screen_y, int radius) {
  auto point_index = PickPointAtScreenPos(screen_x, screen_y, radius);
  if (!point_index) {
    return false;
  }
  
  // Replace current selection
  selected_indices_.clear();
  selected_indices_.push_back(*point_index);
  
  UpdateVisualization();
  NotifySelectionChanged();
  return true;
}

bool PointSelection::AddPoint(float screen_x, float screen_y, int radius) {
  auto point_index = PickPointAtScreenPos(screen_x, screen_y, radius);
  if (!point_index) {
    return false;
  }
  
  // Add to selection if not already selected
  if (!IsPointSelected(*point_index)) {
    AddToSelection(*point_index);
    UpdateVisualization();
    NotifySelectionChanged();
  }
  
  return true;
}

bool PointSelection::TogglePoint(float screen_x, float screen_y, int radius) {
  auto point_index = PickPointAtScreenPos(screen_x, screen_y, radius);
  if (!point_index) {
    return false;
  }
  
  // Toggle selection state
  if (IsPointSelected(*point_index)) {
    RemoveFromSelection(*point_index);
  } else {
    AddToSelection(*point_index);
  }
  
  UpdateVisualization();
  NotifySelectionChanged();
  return true;
}

void PointSelection::ClearSelection() {
  if (selected_indices_.empty()) {
    return;
  }
  
  selected_indices_.clear();
  UpdateVisualization();
  NotifySelectionChanged();
}

// === Point Data Access ===

std::vector<glm::vec3> PointSelection::GetSelectedPoints() const {
  std::vector<glm::vec3> selected_points;
  const auto& all_points = point_cloud_->GetPoints();
  
  selected_points.reserve(selected_indices_.size());
  for (size_t idx : selected_indices_) {
    if (idx < all_points.size()) {
      selected_points.push_back(all_points[idx]);
    }
  }
  
  return selected_points;
}

std::vector<glm::vec3> PointSelection::GetSelectedColors() const {
  std::vector<glm::vec3> selected_colors;
  const auto& all_colors = point_cloud_->GetColors();
  
  if (all_colors.empty()) {
    return selected_colors;  // No color data available
  }
  
  selected_colors.reserve(selected_indices_.size());
  for (size_t idx : selected_indices_) {
    if (idx < all_colors.size()) {
      selected_colors.push_back(all_colors[idx]);
    }
  }
  
  return selected_colors;
}

std::optional<glm::vec3> PointSelection::GetSelectedPoint(size_t selection_index) const {
  if (selection_index >= selected_indices_.size()) {
    return std::nullopt;
  }
  
  size_t cloud_index = selected_indices_[selection_index];
  const auto& all_points = point_cloud_->GetPoints();
  
  if (cloud_index >= all_points.size()) {
    return std::nullopt;
  }
  
  return all_points[cloud_index];
}

// === Convenience Statistics ===

glm::vec3 PointSelection::GetSelectionCentroid() const {
  if (selected_indices_.empty()) {
    return glm::vec3(0.0f);
  }
  
  const auto& all_points = point_cloud_->GetPoints();
  glm::vec3 sum(0.0f);
  size_t valid_count = 0;
  
  for (size_t idx : selected_indices_) {
    if (idx < all_points.size()) {
      sum += all_points[idx];
      ++valid_count;
    }
  }
  
  return valid_count > 0 ? sum / static_cast<float>(valid_count) : glm::vec3(0.0f);
}

std::pair<glm::vec3, glm::vec3> PointSelection::GetSelectionBounds() const {
  if (selected_indices_.empty()) {
    return {glm::vec3(0.0f), glm::vec3(0.0f)};
  }
  
  const auto& all_points = point_cloud_->GetPoints();
  
  // Initialize with first valid point
  glm::vec3 min_bounds(std::numeric_limits<float>::max());
  glm::vec3 max_bounds(std::numeric_limits<float>::lowest());
  
  bool found_valid = false;
  for (size_t idx : selected_indices_) {
    if (idx < all_points.size()) {
      const auto& point = all_points[idx];
      if (!found_valid) {
        min_bounds = max_bounds = point;
        found_valid = true;
      } else {
        min_bounds = glm::min(min_bounds, point);
        max_bounds = glm::max(max_bounds, point);
      }
    }
  }
  
  if (!found_valid) {
    return {glm::vec3(0.0f), glm::vec3(0.0f)};
  }
  
  return {min_bounds, max_bounds};
}

// === Visualization Control ===

void PointSelection::SetSelectionVisualization(const glm::vec3& color,
                                               float size_multiplier,
                                               const std::string& layer_name) {
  highlight_color_ = color;
  size_multiplier_ = size_multiplier;
  layer_name_ = layer_name;
  
  if (visualization_enabled_) {
    UpdateVisualization();
  }
}

void PointSelection::SetVisualizationEnabled(bool enabled) {
  visualization_enabled_ = enabled;
  UpdateVisualization();
}

// === Internal Methods ===

std::optional<size_t> PointSelection::PickPointAtScreenPos(float x, float y, int radius) {
  if (!scene_manager_) {
    return std::nullopt;
  }
  
  // Use existing GPU picking from GlSceneManager
  size_t point_index = scene_manager_->PickPointAtPixelWithRadius(
      static_cast<int>(x), static_cast<int>(y), radius);
  
  if (point_index == SIZE_MAX) {
    return std::nullopt;
  }
  
  // Validate against point cloud size
  if (point_index >= point_cloud_->GetPointCount()) {
    return std::nullopt;
  }
  
  return point_index;
}

void PointSelection::UpdateVisualization() {
  if (!visualization_enabled_) {
    // Clear visualization
    point_cloud_->ClearHighlights(layer_name_);
    return;
  }
  
  if (selected_indices_.empty()) {
    // Clear highlights if no selection
    point_cloud_->ClearHighlights(layer_name_);
  } else {
    // Apply highlights to selected points
    point_cloud_->HighlightPoints(selected_indices_, highlight_color_, 
                                 layer_name_, size_multiplier_);
  }
}

void PointSelection::NotifySelectionChanged() {
  if (callback_) {
    callback_(selected_indices_);
  }
}

bool PointSelection::IsPointSelected(size_t point_index) const {
  return std::find(selected_indices_.begin(), selected_indices_.end(), point_index) 
         != selected_indices_.end();
}

void PointSelection::RemoveFromSelection(size_t point_index) {
  auto it = std::find(selected_indices_.begin(), selected_indices_.end(), point_index);
  if (it != selected_indices_.end()) {
    selected_indices_.erase(it);
  }
}

void PointSelection::AddToSelection(size_t point_index) {
  if (!IsPointSelected(point_index)) {
    selected_indices_.push_back(point_index);
  }
}

}  // namespace visualization
}  // namespace quickviz