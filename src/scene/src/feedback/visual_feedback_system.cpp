/*
 * @file visual_feedback_system.cpp
 * @date Sept 2, 2025
 * @brief Implementation of the visual feedback system coordination layer
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/feedback/visual_feedback_system.hpp"
#include "scene/feedback/point_cloud_feedback_handler.hpp"
#include "scene/feedback/object_feedback_handler.hpp"

#include "scene/scene_manager.hpp"
#include "scene/selection_manager.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "scene/interface/opengl_object.hpp"

#include <algorithm>
#include <iostream>

namespace quickviz {

// === FeedbackTheme Implementation ===

FeedbackTheme FeedbackTheme::Default() {
  return FeedbackTheme{};  // Uses default values from header
}

FeedbackTheme FeedbackTheme::HighContrast() {
  FeedbackTheme theme;
  theme.hover_color = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);           // Bright yellow
  theme.selection_color = glm::vec4(1.0f, 0.0f, 1.0f, 1.0f);       // Bright magenta
  theme.pre_selection_color = glm::vec4(0.0f, 1.0f, 1.0f, 0.8f);   // Bright cyan
  theme.error_color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);           // Bright red
  theme.success_color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);         // Bright green
  
  theme.point_size_multiplier_hover = 1.5f;
  theme.point_size_multiplier_selection = 2.0f;
  theme.outline_width = 3.0f;
  
  theme.hover_intensity = 1.0f;
  theme.selection_intensity = 1.0f;
  
  return theme;
}

FeedbackTheme FeedbackTheme::Subtle() {
  FeedbackTheme theme;
  theme.hover_color = glm::vec4(0.9f, 0.9f, 0.9f, 0.3f);          // Light gray
  theme.selection_color = glm::vec4(0.8f, 0.8f, 1.0f, 0.5f);      // Light blue
  theme.pre_selection_color = glm::vec4(0.8f, 1.0f, 0.8f, 0.4f);  // Light green
  theme.error_color = glm::vec4(1.0f, 0.6f, 0.6f, 0.5f);          // Light red
  theme.success_color = glm::vec4(0.6f, 1.0f, 0.6f, 0.5f);        // Light green
  
  theme.point_size_multiplier_hover = 1.1f;
  theme.point_size_multiplier_selection = 1.2f;
  theme.outline_width = 1.0f;
  
  theme.hover_intensity = 0.5f;
  theme.selection_intensity = 0.7f;
  
  return theme;
}

FeedbackTheme FeedbackTheme::CADStyle() {
  FeedbackTheme theme;
  theme.hover_color = glm::vec4(0.0f, 0.8f, 1.0f, 0.8f);          // CAD blue
  theme.selection_color = glm::vec4(1.0f, 0.5f, 0.0f, 1.0f);      // CAD orange  
  theme.pre_selection_color = glm::vec4(0.5f, 1.0f, 0.5f, 0.6f);  // Light green
  theme.error_color = glm::vec4(1.0f, 0.0f, 0.0f, 0.9f);          // CAD red
  theme.success_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.8f);        // CAD green
  
  theme.point_size_multiplier_hover = 1.2f;
  theme.point_size_multiplier_selection = 1.8f;
  theme.outline_width = 2.5f;
  
  theme.pulse_frequency = 1.5f;
  theme.fade_duration = 0.2f;
  
  return theme;
}

// === VisualFeedbackSystem Implementation ===

VisualFeedbackSystem::VisualFeedbackSystem(SceneManager* scene_manager)
    : scene_manager_(scene_manager)
    , theme_(FeedbackTheme::Default())
    , enabled_(true)
    , currently_hovered_object_("")
{
  if (!scene_manager_) {
    throw std::invalid_argument("VisualFeedbackSystem: scene_manager cannot be null");
  }
  
  // Create specialized handlers
  point_cloud_handler_ = std::make_unique<PointCloudFeedbackHandler>();
  object_handler_ = std::make_unique<ObjectFeedbackHandler>(scene_manager_);
  
  // Visual feedback system initialized successfully
}

VisualFeedbackSystem::~VisualFeedbackSystem() {
  ClearAllFeedback();
}

void VisualFeedbackSystem::ShowFeedback(const std::string& object_name, 
                                       FeedbackType type, 
                                       FeedbackStyle style) {
  if (!enabled_ || object_name.empty()) {
    return;
  }
  
  DelegateFeedback(object_name, type, style, true);
  UpdateFeedbackState(object_name, type, true);
  
  // Notify callback if registered
  if (feedback_callback_) {
    feedback_callback_(object_name, type, true);
  }
}

void VisualFeedbackSystem::RemoveFeedback(const std::string& object_name, 
                                         FeedbackType type) {
  if (object_name.empty()) {
    return;
  }
  
  DelegateFeedback(object_name, type, FeedbackStyle::kOutline, false);
  UpdateFeedbackState(object_name, type, false);
  
  // Notify callback if registered
  if (feedback_callback_) {
    feedback_callback_(object_name, type, false);
  }
}

void VisualFeedbackSystem::ClearFeedback(FeedbackType type) {
  if (!enabled_) {
    return;
  }
  
  // Clear from specialized handlers
  point_cloud_handler_->ClearFeedback(type);
  object_handler_->ClearFeedback(type);
  
  // Update internal state tracking
  for (auto& pair : active_feedback_) {
    auto& feedback_types = pair.second;
    feedback_types.erase(
      std::remove(feedback_types.begin(), feedback_types.end(), type),
      feedback_types.end()
    );
  }
  
  // Remove empty entries
  for (auto it = active_feedback_.begin(); it != active_feedback_.end();) {
    if (it->second.empty()) {
      it = active_feedback_.erase(it);
    } else {
      ++it;
    }
  }
  
  // Feedback cleared for type successfully
}

void VisualFeedbackSystem::ClearAllFeedback() {
  if (!enabled_) {
    return;
  }
  
  // Clear from specialized handlers
  point_cloud_handler_->ClearAllFeedback();
  object_handler_->ClearAllFeedback();
  
  // Clear internal state
  active_feedback_.clear();
  currently_hovered_object_.clear();
  
  // All feedback cleared successfully
}

void VisualFeedbackSystem::OnSelectionChanged(const MultiSelection& selection) {
  if (!enabled_) {
    return;
  }

  // Clear existing selection feedback
  ClearFeedback(FeedbackType::kSelection);
  
  // Add feedback for new selection
  const auto& selections = selection.GetSelections();
  for (const auto& sel : selections) {
    if (std::holds_alternative<PointSelection>(sel)) {
      // Handle point selection
      const auto& point_sel = std::get<PointSelection>(sel);
      if (point_sel.point_cloud && !point_sel.cloud_name.empty()) {
        std::vector<size_t> point_indices = {point_sel.point_index};
        point_cloud_handler_->ShowPointFeedback(point_sel.point_cloud, 
                                               point_indices,
                                               FeedbackType::kSelection, 
                                               theme_);
        UpdateFeedbackState(point_sel.cloud_name, FeedbackType::kSelection, true);
      }
    } else if (std::holds_alternative<ObjectSelection>(sel)) {
      // Handle object selection
      const auto& obj_sel = std::get<ObjectSelection>(sel);
      if (!obj_sel.object_name.empty()) {
        ShowFeedback(obj_sel.object_name, FeedbackType::kSelection);
      }
    }
  }
  
  // Selection feedback updated successfully
}

void VisualFeedbackSystem::OnObjectHovered(const std::string& object_name) {
  if (!enabled_) {
    return;
  }
  
  // Clear previous hover feedback
  if (!currently_hovered_object_.empty()) {
    RemoveFeedback(currently_hovered_object_, FeedbackType::kHover);
  }
  
  // Set new hover feedback
  currently_hovered_object_ = object_name;
  if (!object_name.empty()) {
    ShowFeedback(object_name, FeedbackType::kHover);
  }
}

void VisualFeedbackSystem::OnObjectUnhovered() {
  OnObjectHovered("");  // Clear hover by passing empty string
}

void VisualFeedbackSystem::Update(float delta_time) {
  if (!enabled_) {
    return;
  }
  
  // Update specialized handlers
  point_cloud_handler_->Update(delta_time);
  object_handler_->Update(delta_time);
}

void VisualFeedbackSystem::RenderOverlays(const glm::mat4& projection, 
                                         const glm::mat4& view) {
  if (!enabled_) {
    return;
  }
  
  // Only render non-point-cloud feedback overlays
  // Point cloud feedback is handled by LayerManager during normal rendering
  object_handler_->RenderFeedback(projection, view);
}

void VisualFeedbackSystem::SetTheme(const FeedbackTheme& theme) {
  theme_ = theme;
  
  // TODO: Apply theme changes to existing feedback
  // This would require re-applying current feedback with new theme
  // Theme updated successfully
}

bool VisualFeedbackSystem::HasFeedback(const std::string& object_name, 
                                      FeedbackType type) const {
  auto it = active_feedback_.find(object_name);
  if (it == active_feedback_.end()) {
    return false;
  }
  
  const auto& feedback_types = it->second;
  return std::find(feedback_types.begin(), feedback_types.end(), type) != feedback_types.end();
}

std::vector<std::string> VisualFeedbackSystem::GetObjectsWithFeedback(FeedbackType type) const {
  std::vector<std::string> objects;
  
  for (const auto& pair : active_feedback_) {
    const auto& feedback_types = pair.second;
    if (std::find(feedback_types.begin(), feedback_types.end(), type) != feedback_types.end()) {
      objects.push_back(pair.first);
    }
  }
  
  return objects;
}

size_t VisualFeedbackSystem::GetActiveFeedbackCount() const {
  size_t count = 0;
  for (const auto& pair : active_feedback_) {
    count += pair.second.size();
  }
  return count;
}

// === Private Methods ===

void VisualFeedbackSystem::DelegateFeedback(const std::string& object_name, 
                                           FeedbackType type,
                                           FeedbackStyle style, 
                                           bool show) {
  auto* object = GetObject(object_name);
  if (!object) {
    std::cerr << "[VisualFeedbackSystem] Warning: Object '" << object_name 
              << "' not found in scene" << std::endl;
    return;
  }
  
  if (IsPointCloud(object)) {
    // Delegate to point cloud handler
    auto* point_cloud = static_cast<PointCloud*>(object);
    
    if (show) {
      // For point clouds, we need to get the points to highlight
      // This could come from selection state or hover detection
      std::vector<size_t> point_indices = GetPointIndicesForFeedback(object_name);
      point_cloud_handler_->ShowPointFeedback(point_cloud, point_indices, type, theme_);
    } else {
      point_cloud_handler_->RemovePointFeedback(point_cloud, type);
    }
  } else {
    // Delegate to object handler
    if (show) {
      object_handler_->ShowObjectFeedback(object_name, type, style, theme_);
    } else {
      object_handler_->RemoveObjectFeedback(object_name, type);
    }
  }
}

void VisualFeedbackSystem::UpdateFeedbackState(const std::string& object_name, 
                                              FeedbackType type, 
                                              bool active) {
  if (active) {
    // Add feedback type to object's active list
    auto& feedback_types = active_feedback_[object_name];
    if (std::find(feedback_types.begin(), feedback_types.end(), type) == feedback_types.end()) {
      feedback_types.push_back(type);
    }
  } else {
    // Remove feedback type from object's active list
    auto it = active_feedback_.find(object_name);
    if (it != active_feedback_.end()) {
      auto& feedback_types = it->second;
      feedback_types.erase(
        std::remove(feedback_types.begin(), feedback_types.end(), type),
        feedback_types.end()
      );
      
      // Remove object entry if no feedback types remain
      if (feedback_types.empty()) {
        active_feedback_.erase(it);
      }
    }
  }
}

OpenGlObject* VisualFeedbackSystem::GetObject(const std::string& object_name) const {
  return scene_manager_->GetOpenGLObject(object_name);
}

bool VisualFeedbackSystem::IsPointCloud(OpenGlObject* object) const {
  return dynamic_cast<PointCloud*>(object) != nullptr;
}

std::vector<size_t> VisualFeedbackSystem::GetPointIndicesForFeedback(const std::string& object_name) const {
  // This is a placeholder implementation
  // In the full implementation, this would:
  // 1. Check with SelectionManager for selected points
  // 2. Check for hover detection results
  // 3. Return appropriate point indices
  
  // For now, return empty vector (means no specific points, might highlight all)
  return std::vector<size_t>();
}

} // namespace quickviz