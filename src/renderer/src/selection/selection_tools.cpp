/**
 * @file selection_tools.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2024-12-19
 * @brief Implementation of interactive selection tools for point clouds
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "renderer/selection/selection_tools.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <glm/gtc/matrix_transform.hpp>

namespace quickviz {

SelectionTools::SelectionTools() {
  viewport_ = glm::ivec4(0, 0, 800, 600);  // Default viewport
  projection_matrix_ = glm::mat4(1.0f);
  view_matrix_ = glm::mat4(1.0f);
  coord_transform_ = glm::mat4(1.0f);
}

SelectionTools::~SelectionTools() = default;

void SelectionTools::SetPointCloud(PointCloud* point_cloud) {
  point_cloud_ = point_cloud;
  InvalidateScreenCache();
}

void SelectionTools::SetCamera(const Camera* camera) {
  camera_ = camera;
  InvalidateScreenCache();
}

void SelectionTools::SetViewport(int x, int y, int width, int height) {
  viewport_ = glm::ivec4(x, y, width, height);
  InvalidateScreenCache();
}

void SelectionTools::SetProjectionMatrix(const glm::mat4& projection) {
  projection_matrix_ = projection;
  InvalidateScreenCache();
}

void SelectionTools::SetViewMatrix(const glm::mat4& view) {
  view_matrix_ = view;
  InvalidateScreenCache();
}

void SelectionTools::SetCoordinateTransform(const glm::mat4& transform) {
  coord_transform_ = transform;
  InvalidateScreenCache();
}

int SelectionTools::PickPoint(float screen_x, float screen_y, float tolerance) {
  if (!point_cloud_ || point_cloud_->GetPointCount() == 0) {
    return -1;
  }
  
  UpdateScreenCache();
  
  float min_distance = tolerance;
  int selected_index = -1;
  
  const auto& points = point_cloud_->GetPoints();
  for (size_t i = 0; i < points.size(); ++i) {
    float distance = ComputeScreenDistance(points[i], screen_x, screen_y);
    if (distance < min_distance) {
      min_distance = distance;
      selected_index = static_cast<int>(i);
    }
  }
  
  return selected_index;
}

SelectionResult SelectionTools::SelectRectangle(float x1, float y1, float x2, float y2,
                                               SelectionMode mode) {
  if (!point_cloud_ || point_cloud_->GetPointCount() == 0) {
    return SelectionResult();
  }
  
  UpdateScreenCache();
  
  // Normalize rectangle corners
  glm::vec2 min_corner(std::min(x1, x2), std::min(y1, y2));
  glm::vec2 max_corner(std::max(x1, x2), std::max(y1, y2));
  
  std::vector<size_t> selected_indices;
  const auto& points = point_cloud_->GetPoints();
  
  for (size_t i = 0; i < points.size(); ++i) {
    if (i < screen_cache_.size()) {
      const glm::vec2& screen_pos = screen_cache_[i];
      if (IsPointInRectangle(screen_pos, min_corner, max_corner)) {
        selected_indices.push_back(i);
      }
    }
  }
  
  // Apply selection mode
  selected_indices = ApplySelectionMode(selected_indices, mode);
  
  // Update selection result
  UpdateSelectionResult(selected_indices, SelectionResult::Method::kRectangle);
  
  // Store screen region
  current_selection_.screen_region = {
    min_corner,
    glm::vec2(max_corner.x, min_corner.y),
    max_corner,
    glm::vec2(min_corner.x, max_corner.y)
  };
  
  // Trigger callback
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  // Update point cloud selection
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
  
  return current_selection_;
}

SelectionResult SelectionTools::SelectLasso(const std::vector<glm::vec2>& screen_points,
                                           SelectionMode mode) {
  if (!point_cloud_ || point_cloud_->GetPointCount() == 0 || screen_points.size() < 3) {
    return SelectionResult();
  }
  
  UpdateScreenCache();
  
  std::vector<size_t> selected_indices;
  const auto& points = point_cloud_->GetPoints();
  
  for (size_t i = 0; i < points.size(); ++i) {
    if (i < screen_cache_.size()) {
      const glm::vec2& screen_pos = screen_cache_[i];
      if (IsPointInPolygon(screen_pos, screen_points)) {
        selected_indices.push_back(i);
      }
    }
  }
  
  // Apply selection mode
  selected_indices = ApplySelectionMode(selected_indices, mode);
  
  // Update selection result
  UpdateSelectionResult(selected_indices, SelectionResult::Method::kLasso);
  current_selection_.screen_region = screen_points;
  
  // Trigger callback
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  // Update point cloud selection
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
  
  return current_selection_;
}

SelectionResult SelectionTools::SelectRadius(float center_x, float center_y, float radius,
                                            SelectionMode mode) {
  if (!point_cloud_ || point_cloud_->GetPointCount() == 0) {
    return SelectionResult();
  }
  
  UpdateScreenCache();
  
  glm::vec2 center(center_x, center_y);
  float radius_squared = radius * radius;
  
  std::vector<size_t> selected_indices;
  const auto& points = point_cloud_->GetPoints();
  
  for (size_t i = 0; i < points.size(); ++i) {
    if (i < screen_cache_.size()) {
      const glm::vec2& screen_pos = screen_cache_[i];
      float dist_squared = glm::dot(screen_pos - center, screen_pos - center);
      if (dist_squared <= radius_squared) {
        selected_indices.push_back(i);
      }
    }
  }
  
  // Apply selection mode
  selected_indices = ApplySelectionMode(selected_indices, mode);
  
  // Update selection result
  UpdateSelectionResult(selected_indices, SelectionResult::Method::kRadius);
  
  // Trigger callback
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  // Update point cloud selection
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
  
  return current_selection_;
}

void SelectionTools::GrowSelection(float distance_threshold) {
  if (!point_cloud_ || current_selection_.IsEmpty()) {
    return;
  }
  
  const auto& points = point_cloud_->GetPoints();
  std::vector<size_t> new_indices = current_selection_.indices;
  float threshold_squared = distance_threshold * distance_threshold;
  
  // For each unselected point, check if it's within threshold of any selected point
  std::vector<bool> is_selected(points.size(), false);
  for (size_t idx : current_selection_.indices) {
    is_selected[idx] = true;
  }
  
  for (size_t i = 0; i < points.size(); ++i) {
    if (is_selected[i]) continue;
    
    // Check distance to all selected points
    for (size_t selected_idx : current_selection_.indices) {
      glm::vec3 diff = points[i] - points[selected_idx];
      float dist_squared = glm::dot(diff, diff);
      if (dist_squared <= threshold_squared) {
        new_indices.push_back(i);
        break;
      }
    }
  }
  
  // Update selection
  UpdateSelectionResult(new_indices, current_selection_.method);
  
  // Trigger callback
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  // Update point cloud selection
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
}

void SelectionTools::ShrinkSelection(float distance_threshold) {
  if (!point_cloud_ || current_selection_.IsEmpty()) {
    return;
  }
  
  const auto& points = point_cloud_->GetPoints();
  std::vector<size_t> new_indices;
  float threshold_squared = distance_threshold * distance_threshold;
  
  // Mark selected points
  std::vector<bool> is_selected(points.size(), false);
  for (size_t idx : current_selection_.indices) {
    is_selected[idx] = true;
  }
  
  // Keep only points that have all neighbors within threshold also selected
  for (size_t idx : current_selection_.indices) {
    bool is_interior = true;
    
    // Check if all nearby points are also selected
    for (size_t i = 0; i < points.size(); ++i) {
      if (i == idx) continue;
      
      glm::vec3 diff = points[i] - points[idx];
      float dist_squared = glm::dot(diff, diff);
      
      if (dist_squared <= threshold_squared && !is_selected[i]) {
        is_interior = false;
        break;
      }
    }
    
    if (is_interior) {
      new_indices.push_back(idx);
    }
  }
  
  // Update selection
  UpdateSelectionResult(new_indices, current_selection_.method);
  
  // Trigger callback
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  // Update point cloud selection
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
}

void SelectionTools::ClearSelection() {
  current_selection_.Clear();
  
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  if (point_cloud_) {
    point_cloud_->ClearSelection();
  }
}

void SelectionTools::InvertSelection() {
  if (!point_cloud_) return;
  
  std::vector<bool> is_selected(point_cloud_->GetPointCount(), false);
  for (size_t idx : current_selection_.indices) {
    is_selected[idx] = true;
  }
  
  std::vector<size_t> new_indices;
  for (size_t i = 0; i < is_selected.size(); ++i) {
    if (!is_selected[i]) {
      new_indices.push_back(i);
    }
  }
  
  UpdateSelectionResult(new_indices, current_selection_.method);
  
  if (selection_callback_) {
    selection_callback_(current_selection_);
  }
  
  if (point_cloud_) {
    point_cloud_->SetSelectedPoints(current_selection_.indices);
  }
}

glm::vec2 SelectionTools::ScreenToNDC(float screen_x, float screen_y) const {
  return glm::vec2(
    2.0f * screen_x / viewport_[2] - 1.0f,
    1.0f - 2.0f * screen_y / viewport_[3]  // Flip Y coordinate
  );
}

glm::vec2 SelectionTools::WorldToScreen(const glm::vec3& world_point) const {
  // Transform to clip space
  glm::vec4 point_4d(world_point, 1.0f);
  glm::vec4 clip_pos = projection_matrix_ * view_matrix_ * coord_transform_ * point_4d;
  
  // Perspective division to get NDC
  if (std::abs(clip_pos.w) > 0.001f) {
    clip_pos /= clip_pos.w;
  }
  
  // Convert NDC to screen coordinates
  glm::vec2 screen_pos;
  screen_pos.x = viewport_[0] + viewport_[2] * (clip_pos.x + 1.0f) * 0.5f;
  screen_pos.y = viewport_[1] + viewport_[3] * (1.0f - clip_pos.y) * 0.5f;
  
  return screen_pos;
}

bool SelectionTools::IsPointInPolygon(const glm::vec2& point,
                                      const std::vector<glm::vec2>& polygon) {
  if (polygon.size() < 3) return false;
  
  // Ray casting algorithm
  int crossings = 0;
  size_t n = polygon.size();
  
  for (size_t i = 0; i < n; ++i) {
    size_t j = (i + 1) % n;
    
    // Check if ray from point to +X crosses edge
    if ((polygon[i].y <= point.y && point.y < polygon[j].y) ||
        (polygon[j].y <= point.y && point.y < polygon[i].y)) {
      // Compute X coordinate of intersection
      float t = (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y);
      float x_intersect = polygon[i].x + t * (polygon[j].x - polygon[i].x);
      
      if (point.x < x_intersect) {
        crossings++;
      }
    }
  }
  
  return (crossings % 2) == 1;
}

bool SelectionTools::IsPointInRectangle(const glm::vec2& point,
                                        const glm::vec2& min_corner,
                                        const glm::vec2& max_corner) {
  return point.x >= min_corner.x && point.x <= max_corner.x &&
         point.y >= min_corner.y && point.y <= max_corner.y;
}

void SelectionTools::UpdateHover(float screen_x, float screen_y) {
  int new_hovered = PickPoint(screen_x, screen_y, 10.0f);  // 10 pixel tolerance for hover
  
  if (new_hovered != hovered_point_index_) {
    hovered_point_index_ = new_hovered;
    
    if (hover_callback_) {
      hover_callback_(hovered_point_index_);
    }
    
    // Highlight hovered point
    if (point_cloud_ && hovered_point_index_ >= 0) {
      point_cloud_->HighlightPoint(
        static_cast<size_t>(hovered_point_index_),
        glm::vec3(1.0f, 1.0f, 0.0f),  // Yellow for hover
        "hover",
        1.2f
      );
    } else if (point_cloud_) {
      point_cloud_->ClearHighlights("hover");
    }
  }
}

void SelectionTools::UpdateSelectionResult(const std::vector<size_t>& indices,
                                          SelectionResult::Method method) {
  current_selection_.indices = indices;
  current_selection_.method = method;
  
  if (point_cloud_) {
    current_selection_.ComputeStatistics(point_cloud_->GetPoints());
  }
}

std::vector<size_t> SelectionTools::ApplySelectionMode(const std::vector<size_t>& new_indices,
                                                       SelectionMode mode) {
  switch (mode) {
    case SelectionMode::kReplace:
      return new_indices;
      
    case SelectionMode::kAdd: {
      std::vector<size_t> combined = current_selection_.indices;
      combined.insert(combined.end(), new_indices.begin(), new_indices.end());
      
      // Remove duplicates
      std::sort(combined.begin(), combined.end());
      combined.erase(std::unique(combined.begin(), combined.end()), combined.end());
      
      return combined;
    }
    
    case SelectionMode::kRemove: {
      std::vector<bool> keep(point_cloud_->GetPointCount(), false);
      for (size_t idx : current_selection_.indices) {
        keep[idx] = true;
      }
      for (size_t idx : new_indices) {
        keep[idx] = false;
      }
      
      std::vector<size_t> result;
      for (size_t i = 0; i < keep.size(); ++i) {
        if (keep[i]) {
          result.push_back(i);
        }
      }
      return result;
    }
    
    case SelectionMode::kToggle: {
      std::vector<bool> selected(point_cloud_->GetPointCount(), false);
      for (size_t idx : current_selection_.indices) {
        selected[idx] = true;
      }
      for (size_t idx : new_indices) {
        selected[idx] = !selected[idx];
      }
      
      std::vector<size_t> result;
      for (size_t i = 0; i < selected.size(); ++i) {
        if (selected[i]) {
          result.push_back(i);
        }
      }
      return result;
    }
  }
  
  return new_indices;
}

float SelectionTools::ComputeScreenDistance(const glm::vec3& world_point,
                                           float screen_x, float screen_y) const {
  glm::vec2 screen_pos = WorldToScreen(world_point);
  glm::vec2 diff = screen_pos - glm::vec2(screen_x, screen_y);
  return glm::length(diff);
}

void SelectionTools::UpdateScreenCache() const {
  if (screen_cache_valid_ || !point_cloud_) return;
  
  const auto& points = point_cloud_->GetPoints();
  screen_cache_.resize(points.size());
  
  for (size_t i = 0; i < points.size(); ++i) {
    screen_cache_[i] = WorldToScreen(points[i]);
  }
  
  screen_cache_valid_ = true;
}

}  // namespace quickviz