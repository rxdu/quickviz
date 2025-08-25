/**
 * @file point_cloud_selector.cpp
 * @date 2025-08-25
 * @brief Implementation of point cloud selection utilities
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "visualization/selection/point_cloud_selector.hpp"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace quickviz {
namespace visualization {

PointCloudSelector::PointCloudSelector() 
  : pcl_cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
    kdtree_(std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>()) {
}

PointCloudSelector::~PointCloudSelector() = default;

void PointCloudSelector::SetPointCloud(std::shared_ptr<PointCloud> point_cloud) {
  point_cloud_ = point_cloud;
  if (point_cloud_) {
    BuildPCLCloud();
    UpdateSpatialIndex();
  }
}

void PointCloudSelector::BuildPCLCloud() {
  if (!point_cloud_) return;

  const auto& points = point_cloud_->GetPoints();
  pcl_cloud_->clear();
  pcl_cloud_->reserve(points.size());

  for (const auto& pt : points) {
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pt.x;
    pcl_pt.y = pt.y;
    pcl_pt.z = pt.z;
    pcl_cloud_->push_back(pcl_pt);
  }

  pcl_cloud_->width = pcl_cloud_->size();
  pcl_cloud_->height = 1;
  pcl_cloud_->is_dense = true;
}

void PointCloudSelector::UpdateSpatialIndex() {
  if (pcl_cloud_ && !pcl_cloud_->empty()) {
    kdtree_->setInputCloud(pcl_cloud_);
  }
}

float PointCloudSelector::PointToRayDistance(const glm::vec3& point, 
                                             const Ray& ray) const {
  // Vector from ray origin to point
  glm::vec3 op = point - ray.origin;
  
  // Project onto ray direction
  float t = glm::dot(op, ray.direction);
  
  // Point on ray closest to the point
  glm::vec3 closest_on_ray = ray.origin + t * ray.direction;
  
  // Distance from point to closest point on ray
  return glm::length(point - closest_on_ray);
}

std::optional<PickResult> PointCloudSelector::PickPoint(const Ray& ray, 
                                                        float max_distance) const {
  if (!point_cloud_ || pcl_cloud_->empty()) {
    return std::nullopt;
  }

  const auto& points = point_cloud_->GetPoints();
  
  std::optional<PickResult> best_result;
  float best_score = std::numeric_limits<float>::max();

  // Check all points (could be optimized with spatial partitioning)
  for (size_t i = 0; i < points.size(); ++i) {
    float dist_to_ray = PointToRayDistance(points[i], ray);
    
    if (dist_to_ray <= max_distance) {
      // Use combination of ray distance and distance from origin as score
      float dist_from_origin = glm::length(points[i] - ray.origin);
      float score = dist_to_ray + dist_from_origin * 0.01f; // Prefer closer points
      
      if (score < best_score) {
        best_score = score;
        best_result = PickResult{
          i,
          points[i],
          dist_from_origin,
          glm::vec3(0) // Screen point not computed here
        };
      }
    }
  }

  return best_result;
}

std::optional<PickResult> PointCloudSelector::PickNearestPoint(
    const glm::vec3& position, float max_distance) const {
  if (!point_cloud_ || pcl_cloud_->empty()) {
    return std::nullopt;
  }

  pcl::PointXYZ search_point;
  search_point.x = position.x;
  search_point.y = position.y;
  search_point.z = position.z;

  std::vector<int> indices(1);
  std::vector<float> distances(1);

  if (kdtree_->nearestKSearch(search_point, 1, indices, distances) > 0) {
    float dist = std::sqrt(distances[0]);
    if (dist <= max_distance) {
      const auto& points = point_cloud_->GetPoints();
      return PickResult{
        static_cast<size_t>(indices[0]),
        points[indices[0]],
        dist,
        glm::vec3(0)
      };
    }
  }

  return std::nullopt;
}

std::vector<PickResult> PointCloudSelector::PickPointsAlongRay(
    const Ray& ray, float max_distance, size_t max_points) const {
  std::vector<PickResult> results;
  
  if (!point_cloud_ || pcl_cloud_->empty()) {
    return results;
  }

  const auto& points = point_cloud_->GetPoints();
  
  // Collect all points within distance threshold
  std::vector<std::pair<float, PickResult>> candidates;
  
  for (size_t i = 0; i < points.size(); ++i) {
    float dist_to_ray = PointToRayDistance(points[i], ray);
    
    if (dist_to_ray <= max_distance) {
      float dist_from_origin = glm::length(points[i] - ray.origin);
      candidates.push_back({
        dist_from_origin,
        PickResult{i, points[i], dist_from_origin, glm::vec3(0)}
      });
    }
  }

  // Sort by distance from ray origin
  std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // Return up to max_points results
  size_t count = std::min(candidates.size(), max_points);
  results.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    results.push_back(candidates[i].second);
  }

  return results;
}

std::vector<size_t> PointCloudSelector::SelectInSphere(
    const glm::vec3& center, float radius) const {
  std::vector<size_t> indices;
  
  if (!point_cloud_ || pcl_cloud_->empty()) {
    return indices;
  }

  pcl::PointXYZ search_point;
  search_point.x = center.x;
  search_point.y = center.y;
  search_point.z = center.z;

  std::vector<int> pcl_indices;
  std::vector<float> distances;

  kdtree_->radiusSearch(search_point, radius, pcl_indices, distances);

  indices.reserve(pcl_indices.size());
  for (int idx : pcl_indices) {
    indices.push_back(static_cast<size_t>(idx));
  }

  return indices;
}

bool PointCloudSelector::IsPointInBox(const glm::vec3& point,
                                      const glm::vec3& min,
                                      const glm::vec3& max) const {
  return point.x >= min.x && point.x <= max.x &&
         point.y >= min.y && point.y <= max.y &&
         point.z >= min.z && point.z <= max.z;
}

std::vector<size_t> PointCloudSelector::SelectInBox(
    const glm::vec3& min, const glm::vec3& max) const {
  std::vector<size_t> indices;
  
  if (!point_cloud_) {
    return indices;
  }

  const auto& points = point_cloud_->GetPoints();
  for (size_t i = 0; i < points.size(); ++i) {
    if (IsPointInBox(points[i], min, max)) {
      indices.push_back(i);
    }
  }

  return indices;
}

std::vector<size_t> PointCloudSelector::SelectByPlane(
    const glm::vec3& point, const glm::vec3& normal, bool select_positive) const {
  std::vector<size_t> indices;
  
  if (!point_cloud_) {
    return indices;
  }

  const auto& points = point_cloud_->GetPoints();
  glm::vec3 n = glm::normalize(normal);

  for (size_t i = 0; i < points.size(); ++i) {
    float distance = glm::dot(points[i] - point, n);
    if ((select_positive && distance > 0) || (!select_positive && distance <= 0)) {
      indices.push_back(i);
    }
  }

  return indices;
}

bool PointCloudSelector::IsPointInCylinder(const glm::vec3& point,
                                           const glm::vec3& base,
                                           const glm::vec3& axis,
                                           float height,
                                           float radius) const {
  // Vector from base to point
  glm::vec3 bp = point - base;
  
  // Project onto axis to get height along cylinder
  float h = glm::dot(bp, axis);
  
  // Check if within height bounds
  if (h < 0 || h > height) {
    return false;
  }
  
  // Get perpendicular distance from axis
  glm::vec3 projection = h * axis;
  glm::vec3 perpendicular = bp - projection;
  float dist = glm::length(perpendicular);
  
  return dist <= radius;
}

std::vector<size_t> PointCloudSelector::SelectInCylinder(
    const glm::vec3& base, const glm::vec3& axis,
    float height, float radius) const {
  std::vector<size_t> indices;
  
  if (!point_cloud_) {
    return indices;
  }

  const auto& points = point_cloud_->GetPoints();
  glm::vec3 normalized_axis = glm::normalize(axis);

  for (size_t i = 0; i < points.size(); ++i) {
    if (IsPointInCylinder(points[i], base, normalized_axis, height, radius)) {
      indices.push_back(i);
    }
  }

  return indices;
}

void PointCloudSelector::SetSelectedIndices(const std::vector<size_t>& indices) {
  selected_indices_ = indices;
  NotifySelectionChanged();
}

void PointCloudSelector::ClearSelection() {
  selected_indices_.clear();
  NotifySelectionChanged();
}

void PointCloudSelector::UpdateSelection(const std::vector<size_t>& indices,
                                         SelectionMode mode) {
  switch (mode) {
    case SelectionMode::kSingle:
      selected_indices_ = indices;
      break;
      
    case SelectionMode::kAdditive: {
      std::unordered_set<size_t> current_set(selected_indices_.begin(), 
                                             selected_indices_.end());
      for (size_t idx : indices) {
        current_set.insert(idx);
      }
      selected_indices_.assign(current_set.begin(), current_set.end());
      std::sort(selected_indices_.begin(), selected_indices_.end());
      break;
    }
    
    case SelectionMode::kSubtractive: {
      std::unordered_set<size_t> to_remove(indices.begin(), indices.end());
      selected_indices_.erase(
        std::remove_if(selected_indices_.begin(), selected_indices_.end(),
                      [&to_remove](size_t idx) { 
                        return to_remove.count(idx) > 0; 
                      }),
        selected_indices_.end()
      );
      break;
    }
    
    case SelectionMode::kToggle: {
      std::unordered_set<size_t> current_set(selected_indices_.begin(),
                                             selected_indices_.end());
      for (size_t idx : indices) {
        if (current_set.count(idx)) {
          current_set.erase(idx);
        } else {
          current_set.insert(idx);
        }
      }
      selected_indices_.assign(current_set.begin(), current_set.end());
      std::sort(selected_indices_.begin(), selected_indices_.end());
      break;
    }
  }
  
  NotifySelectionChanged();
}

void PointCloudSelector::ApplySelectionVisualization(
    const std::string& layer_name,
    const glm::vec3& color,
    float size_multiplier) {
  if (!point_cloud_) return;
  
  if (selected_indices_.empty()) {
    point_cloud_->ClearHighlights(layer_name);
  } else {
    point_cloud_->HighlightPoints(selected_indices_, color, 
                                 layer_name, size_multiplier);
  }
}

std::pair<glm::vec3, glm::vec3> PointCloudSelector::GetSelectionBounds() const {
  if (!point_cloud_ || selected_indices_.empty()) {
    return {glm::vec3(0), glm::vec3(0)};
  }

  const auto& points = point_cloud_->GetPoints();
  glm::vec3 min_pt(std::numeric_limits<float>::max());
  glm::vec3 max_pt(std::numeric_limits<float>::lowest());

  for (size_t idx : selected_indices_) {
    if (idx < points.size()) {
      min_pt = glm::min(min_pt, points[idx]);
      max_pt = glm::max(max_pt, points[idx]);
    }
  }

  return {min_pt, max_pt};
}

glm::vec3 PointCloudSelector::GetSelectionCentroid() const {
  if (!point_cloud_ || selected_indices_.empty()) {
    return glm::vec3(0);
  }

  const auto& points = point_cloud_->GetPoints();
  glm::vec3 centroid(0);
  size_t valid_count = 0;

  for (size_t idx : selected_indices_) {
    if (idx < points.size()) {
      centroid += points[idx];
      valid_count++;
    }
  }

  if (valid_count > 0) {
    centroid /= static_cast<float>(valid_count);
  }

  return centroid;
}

void PointCloudSelector::NotifySelectionChanged() {
  if (selection_callback_) {
    selection_callback_(selected_indices_);
  }
}

}  // namespace visualization
}  // namespace quickviz