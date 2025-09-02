/*
 * @file point_cloud_feedback_handler.cpp
 * @date Sept 2, 2025
 * @brief Implementation of specialized feedback handler for point clouds
 *
 * This handler leverages the existing PointLayerManager system to provide
 * visual feedback for point clouds while preserving the 60-100x performance
 * optimizations of the index-based rendering approach.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/feedback/point_cloud_feedback_handler.hpp"

#include <sstream>
#include <algorithm>
#include <iostream>

#include "gldraw/renderable/point_cloud.hpp"

namespace quickviz {

// Static feedback layer specifications for different feedback types
const std::unordered_map<FeedbackType, PointCloudFeedbackHandler::FeedbackLayerSpec> 
PointCloudFeedbackHandler::FEEDBACK_SPECS = {
  {FeedbackType::kHover, {
    "__feedback_hover_",
    200,  // High priority for immediate visibility
    PointLayer::HighlightMode::kSphereSurface,
    glm::vec3(0.0f, 0.8f, 1.0f),  // Cyan
    1.3f
  }},
  {FeedbackType::kSelection, {
    "__feedback_selection_",
    150,  // Medium-high priority
    PointLayer::HighlightMode::kSphereSurface,
    glm::vec3(1.0f, 0.6f, 0.0f),  // Orange
    1.5f
  }},
  {FeedbackType::kManipulation, {
    "__feedback_manipulation_",
    180,  // High priority for active manipulation
    PointLayer::HighlightMode::kSphereSurface,
    glm::vec3(0.0f, 1.0f, 0.0f),  // Green
    1.4f
  }},
  {FeedbackType::kError, {
    "__feedback_error_",
    250,  // Highest priority for errors
    PointLayer::HighlightMode::kOutline,
    glm::vec3(1.0f, 0.0f, 0.0f),  // Red
    1.2f
  }}
};

PointCloudFeedbackHandler::PointCloudFeedbackHandler() {
  // Initialize with empty state - layers will be created on demand
}

void PointCloudFeedbackHandler::ShowPointFeedback(PointCloud* point_cloud, 
                                                 const std::vector<size_t>& point_indices,
                                                 FeedbackType type,
                                                 const FeedbackTheme& theme) {
  if (!point_cloud) {
    return;
  }
  

  // Get or create the feedback layer for this point cloud and type
  auto feedback_layer = GetOrCreateFeedbackLayer(point_cloud, type, theme);
  if (!feedback_layer) {
    return;
  }

  // Set the points to highlight on this layer
  if (!point_indices.empty()) {
    // Specific point feedback (e.g., selection)
    feedback_layer->SetPoints(point_indices);
  } else {
    // Object-level feedback (e.g., hover) - highlight all points
    std::vector<size_t> all_indices;
    size_t point_count = point_cloud->GetPointCount();
    all_indices.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
      all_indices.push_back(i);
    }
    feedback_layer->SetPoints(all_indices);
  }
  feedback_layer->SetVisible(true);

  // Apply theme-specific styling
  ApplyThemeToLayer(feedback_layer, type, theme);

  // Track this feedback as active
  std::string layer_name = GenerateLayerName(point_cloud, type);
  auto it = std::find_if(active_feedback_.begin(), active_feedback_.end(),
                        [point_cloud, type](const ActiveFeedback& feedback) {
                          return feedback.point_cloud == point_cloud && feedback.type == type;
                        });
  
  if (it == active_feedback_.end()) {
    active_feedback_.push_back({point_cloud, type, layer_name});
  }
}

void PointCloudFeedbackHandler::RemovePointFeedback(PointCloud* point_cloud, FeedbackType type) {
  if (!point_cloud) {
    return;
  }

  std::string layer_name = GenerateLayerName(point_cloud, type);
  
  // Remove the layer from the point cloud
  point_cloud->RemoveLayer(layer_name);

  // Remove from our active tracking
  active_feedback_.erase(
    std::remove_if(active_feedback_.begin(), active_feedback_.end(),
                  [point_cloud, type](const ActiveFeedback& feedback) {
                    return feedback.point_cloud == point_cloud && feedback.type == type;
                  }),
    active_feedback_.end()
  );
}

void PointCloudFeedbackHandler::ClearFeedback(FeedbackType type) {
  // Remove all layers of the specified type from all point clouds
  auto it = active_feedback_.begin();
  while (it != active_feedback_.end()) {
    if (it->type == type) {
      // Remove the layer from the point cloud
      it->point_cloud->RemoveLayer(it->layer_name);
      it = active_feedback_.erase(it);
    } else {
      ++it;
    }
  }
}

void PointCloudFeedbackHandler::ClearAllFeedback() {
  // Remove all feedback layers from all point clouds
  for (const auto& feedback : active_feedback_) {
    feedback.point_cloud->RemoveLayer(feedback.layer_name);
  }
  active_feedback_.clear();
}

void PointCloudFeedbackHandler::Update(float delta_time) {
  // Currently no animations implemented, but this is where we would
  // update any time-based effects like blinking, pulsing, etc.
  
  // Clean up any feedback for point clouds that may have been destroyed
  // (This is a defensive measure - proper lifecycle management should
  //  call RemovePointFeedback before destroying point clouds)
  active_feedback_.erase(
    std::remove_if(active_feedback_.begin(), active_feedback_.end(),
                  [](const ActiveFeedback& feedback) {
                    // Note: This is a simple check - in a more robust system
                    // we might use weak_ptr or a proper notification system
                    return feedback.point_cloud == nullptr;
                  }),
    active_feedback_.end()
  );
}

bool PointCloudFeedbackHandler::HasFeedback(PointCloud* point_cloud, FeedbackType type) const {
  if (!point_cloud) {
    return false;
  }

  auto it = std::find_if(active_feedback_.begin(), active_feedback_.end(),
                        [point_cloud, type](const ActiveFeedback& feedback) {
                          return feedback.point_cloud == point_cloud && feedback.type == type;
                        });
  
  return it != active_feedback_.end();
}

std::string PointCloudFeedbackHandler::GenerateLayerName(PointCloud* point_cloud, 
                                                        FeedbackType type) const {
  auto spec_it = FEEDBACK_SPECS.find(type);
  if (spec_it == FEEDBACK_SPECS.end()) {
    return "__feedback_unknown_";
  }

  // Generate unique name using point cloud pointer and feedback type
  std::ostringstream oss;
  oss << spec_it->second.layer_name_prefix << std::hex << reinterpret_cast<uintptr_t>(point_cloud);
  return oss.str();
}

std::shared_ptr<PointLayer> PointCloudFeedbackHandler::GetOrCreateFeedbackLayer(PointCloud* point_cloud,
                                                                               FeedbackType type, 
                                                                               const FeedbackTheme& theme) {
  if (!point_cloud) {
    return nullptr;
  }

  std::string layer_name = GenerateLayerName(point_cloud, type);
  
  // Try to get existing layer first
  auto existing_layer = point_cloud->GetLayer(layer_name);
  if (existing_layer) {
    return existing_layer;
  }

  // Create new layer with appropriate priority
  auto spec_it = FEEDBACK_SPECS.find(type);
  if (spec_it == FEEDBACK_SPECS.end()) {
    return nullptr;
  }

  const auto& spec = spec_it->second;
  auto new_layer = point_cloud->CreateLayer(layer_name, spec.priority);
  
  if (new_layer) {
    // Configure the layer with default settings
    new_layer->SetHighlightMode(spec.highlight_mode);
    new_layer->SetColor(spec.default_color);
    new_layer->SetPointSizeMultiplier(spec.default_size_multiplier);
    
    // Apply theme overrides
    ApplyThemeToLayer(new_layer, type, theme);
  }

  return new_layer;
}

void PointCloudFeedbackHandler::ApplyThemeToLayer(std::shared_ptr<PointLayer> layer, 
                                                 FeedbackType type, 
                                                 const FeedbackTheme& theme) {
  if (!layer) {
    return;
  }

  // Apply theme colors and settings based on feedback type
  glm::vec3 color;
  float size_multiplier = 1.0f;
  
  switch (type) {
    case FeedbackType::kHover:
      color = glm::vec3(theme.hover_color.r, theme.hover_color.g, theme.hover_color.b);
      size_multiplier = theme.point_size_multiplier_hover;
      break;
    case FeedbackType::kSelection:
      color = glm::vec3(theme.selection_color.r, theme.selection_color.g, theme.selection_color.b);
      size_multiplier = theme.point_size_multiplier_selection;
      break;
    case FeedbackType::kManipulation:
      color = glm::vec3(theme.manipulation_color.r, theme.manipulation_color.g, theme.manipulation_color.b);
      size_multiplier = theme.point_size_multiplier_selection; // Use selection multiplier as fallback
      break;
    case FeedbackType::kError:
      color = glm::vec3(theme.error_color.r, theme.error_color.g, theme.error_color.b);
      size_multiplier = theme.point_size_multiplier_error;
      break;
    default:
      // Use default spec values for other feedback types
      auto spec_it = FEEDBACK_SPECS.find(type);
      if (spec_it != FEEDBACK_SPECS.end()) {
        color = spec_it->second.default_color;
        size_multiplier = spec_it->second.default_size_multiplier;
      } else {
        return; // Unknown type, bail out
      }
      break;
  }
  
  layer->SetColor(color);
  layer->SetPointSizeMultiplier(size_multiplier);
  
  // Note: Opacity handling would require extending PointLayer to support it
  // The alpha component in theme colors is available but not currently used
}

} // namespace quickviz