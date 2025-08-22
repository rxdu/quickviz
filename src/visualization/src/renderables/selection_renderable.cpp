/*
 * @file selection_renderable.cpp
 * @date 2025-01-22
 * @brief Implementation of SelectionRenderable class
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "visualization/renderables/selection_renderable.hpp"
#include <sstream>
#include <chrono>

namespace quickviz {
namespace visualization {

std::unique_ptr<SelectionRenderable> SelectionRenderable::FromData(
    const SelectionData& selection_data,
    PointCloud& target_cloud) {
  
  std::string layer_name = GenerateLayerName(selection_data.selection_name);
  
  return std::make_unique<SelectionRenderable>(
      selection_data, target_cloud, layer_name);
}

SelectionRenderable::SelectionRenderable(const SelectionData& selection_data,
                                       PointCloud& target_cloud,
                                       const std::string& layer_name)
    : selection_data_(selection_data),
      target_cloud_(target_cloud),
      layer_name_(layer_name) {
}

SelectionRenderable::~SelectionRenderable() {
  if (is_applied_) {
    RemoveHighlight();
  }
}

void SelectionRenderable::AllocateGpuResources() {
  if (!IsGpuResourcesAllocated()) {
    target_cloud_.AllocateGpuResources();
    ApplyHighlight();
  }
}

void SelectionRenderable::ReleaseGpuResources() noexcept {
  if (IsGpuResourcesAllocated()) {
    RemoveHighlight();
    // Note: Don't release target_cloud_ resources as it may be shared
  }
}

void SelectionRenderable::OnDraw(const glm::mat4& projection, 
                                const glm::mat4& view,
                                const glm::mat4& coord_transform) {
  if (IsGpuResourcesAllocated() && !selection_data_.IsEmpty()) {
    target_cloud_.OnDraw(projection, view, coord_transform);
  }
}

bool SelectionRenderable::IsGpuResourcesAllocated() const noexcept {
  return target_cloud_.IsGpuResourcesAllocated() && is_applied_;
}

void SelectionRenderable::UpdateSelection(const SelectionData& new_selection_data) {
  if (is_applied_) {
    RemoveHighlight();
  }
  
  selection_data_ = new_selection_data;
  
  if (target_cloud_.IsGpuResourcesAllocated()) {
    ApplyHighlight();
  }
}

void SelectionRenderable::ClearSelection() {
  if (is_applied_) {
    RemoveHighlight();
  }
  
  selection_data_.point_indices.clear();
}

void SelectionRenderable::ApplyHighlight() {
  if (selection_data_.IsEmpty() || !target_cloud_.IsGpuResourcesAllocated()) {
    return;
  }
  
  // Create or get highlight layer
  auto layer = target_cloud_.CreateLayer(layer_name_, 100); // High priority to render on top
  if (!layer) {
    return;
  }
  
  // Configure the layer with selection data
  layer->SetPoints(selection_data_.point_indices);
  layer->SetColor(selection_data_.highlight_color);
  layer->SetPointSizeMultiplier(selection_data_.size_multiplier);
  layer->SetHighlightMode(PointLayer::HighlightMode::kColorAndSize);
  
  is_applied_ = true;
}

void SelectionRenderable::RemoveHighlight() {
  if (is_applied_) {
    target_cloud_.RemoveLayer(layer_name_);
    is_applied_ = false;
  }
}

std::string SelectionRenderable::GenerateLayerName(const std::string& base_name) {
  // Generate unique layer name with timestamp
  auto now = std::chrono::high_resolution_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()).count();
  
  std::ostringstream oss;
  oss << "selection_" << base_name << "_" << timestamp;
  return oss.str();
}

} // namespace visualization
} // namespace quickviz