/*
 * @file object_feedback_handler.cpp
 * @date Sept 2, 2025
 * @brief Implementation of specialized feedback handler for non-point-cloud objects
 *
 * This handler provides visual feedback for meshes, primitives, and other 
 * non-point-cloud objects using overlay rendering techniques such as outlines,
 * surface overlays, wireframes, and glow effects.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/feedback/object_feedback_handler.hpp"

#include <algorithm>
#include <sstream>
#include <iostream>

#include "scene/scene_manager.hpp"
#include "scene/renderable/sphere.hpp"

namespace quickviz {

ObjectFeedbackHandler::ObjectFeedbackHandler(SceneManager* scene_manager) 
    : scene_manager_(scene_manager), outline_shader_program_(0), 
      overlay_shader_program_(0), wireframe_shader_program_(0), 
      shaders_initialized_(false) {
  // Initialize with empty state - shaders and feedback will be created on demand
}

ObjectFeedbackHandler::~ObjectFeedbackHandler() {
  // Clean up any remaining feedback overlays
  ClearAllFeedback();
  // Clean up OpenGL resources
  CleanupShaders();
}

void ObjectFeedbackHandler::ShowObjectFeedback(const std::string& object_name,
                                              FeedbackType type,
                                              FeedbackStyle style,
                                              const FeedbackTheme& theme) {
  if (!scene_manager_ || object_name.empty()) {
    return;
  }

  // Check if we already have feedback for this object and type
  auto it = std::find_if(active_feedback_.begin(), active_feedback_.end(),
                        [&object_name, type](const ObjectFeedbackState& state) {
                          return state.object_name == object_name && state.type == type;
                        });
  
  if (it != active_feedback_.end()) {
    // Update existing feedback
    it->style = style;
    it->color = GetColorForFeedbackType(type, theme);
    it->animated = (style == FeedbackStyle::kPulse || style == FeedbackStyle::kGlow);
    it->animation_time = 0.0f;
  } else {
    // Create new feedback state
    ObjectFeedbackState new_state;
    new_state.object_name = object_name;
    new_state.type = type;
    new_state.style = style;
    new_state.color = GetColorForFeedbackType(type, theme);
    new_state.intensity = 1.0f;
    new_state.animated = (style == FeedbackStyle::kPulse || style == FeedbackStyle::kGlow);
    new_state.animation_time = 0.0f;
    
    active_feedback_.push_back(new_state);
  }

  // Immediately apply highlighting to the object
  if (auto* object = scene_manager_->GetOpenGLObject(object_name)) {
    bool should_highlight = (type == FeedbackType::kSelection || 
                           type == FeedbackType::kHover ||
                           type == FeedbackType::kPreSelection);
    object->SetHighlighted(should_highlight);
  }
}

void ObjectFeedbackHandler::RemoveObjectFeedback(const std::string& object_name,
                                                FeedbackType type) {
  if (object_name.empty()) {
    return;
  }

  // Check if we're removing the last feedback for this object
  auto feedbacks_for_object = std::count_if(active_feedback_.begin(), active_feedback_.end(),
                                           [&object_name](const ObjectFeedbackState& state) {
                                             return state.object_name == object_name;
                                           });

  // Remove the feedback
  active_feedback_.erase(
    std::remove_if(active_feedback_.begin(), active_feedback_.end(),
                  [&object_name, type](const ObjectFeedbackState& state) {
                    return state.object_name == object_name && state.type == type;
                  }),
    active_feedback_.end()
  );

  // If this was the only feedback for this object, unhighlight it
  if (feedbacks_for_object == 1) {
    if (auto* object = scene_manager_->GetOpenGLObject(object_name)) {
      object->SetHighlighted(false);
    }
  }
}

void ObjectFeedbackHandler::ClearFeedback(FeedbackType type) {
  // Collect objects that will lose all feedback after this removal
  std::vector<std::string> objects_to_unhighlight;
  
  for (const auto& state : active_feedback_) {
    if (state.type == type) {
      // Check if this object has any other feedback types
      bool has_other_feedback = std::any_of(active_feedback_.begin(), active_feedback_.end(),
                                           [&state](const ObjectFeedbackState& other) {
                                             return other.object_name == state.object_name && 
                                                    other.type != state.type;
                                           });
      
      if (!has_other_feedback) {
        objects_to_unhighlight.push_back(state.object_name);
      }
    }
  }
  
  // Remove all feedback of the specified type
  active_feedback_.erase(
    std::remove_if(active_feedback_.begin(), active_feedback_.end(),
                  [type](const ObjectFeedbackState& state) {
                    return state.type == type;
                  }),
    active_feedback_.end()
  );
  
  // Unhighlight objects that no longer have any feedback
  for (const auto& object_name : objects_to_unhighlight) {
    if (auto* object = scene_manager_->GetOpenGLObject(object_name)) {
      object->SetHighlighted(false);
    }
  }
}

void ObjectFeedbackHandler::ClearAllFeedback() {
  // Unhighlight all objects that currently have feedback
  for (const auto& state : active_feedback_) {
    if (auto* object = scene_manager_->GetOpenGLObject(state.object_name)) {
      object->SetHighlighted(false);
    }
  }
  
  active_feedback_.clear();
}

void ObjectFeedbackHandler::Update(float delta_time) {
  // Update any time-based animations for active feedback
  for (auto& state : active_feedback_) {
    if (state.animated) {
      UpdateAnimation(state, delta_time);
    }
  }
}

void ObjectFeedbackHandler::RenderFeedback(const glm::mat4& projection,
                                          const glm::mat4& view) {
  if (active_feedback_.empty() || !scene_manager_) {
    return;
  }

  // Initialize shaders if needed
  if (!shaders_initialized_) {
    InitializeShaders();
  }

  // Render feedback for each active state
  for (const auto& state : active_feedback_) {
    // Get the object from scene manager
    OpenGlObject* object = scene_manager_->GetOpenGLObject(state.object_name);
    if (!object) {
      std::cerr << "[ObjectFeedbackHandler] Warning: Object '" << state.object_name 
                << "' not found in scene manager" << std::endl;
      continue;  // Skip this object
    }
    
    switch (state.style) {
      case FeedbackStyle::kOutline:
        RenderOutline(state, object, projection, view);
        break;
      case FeedbackStyle::kSurfaceOverlay:
        RenderSurfaceOverlay(state, object, projection, view);
        break;
      case FeedbackStyle::kWireframe:
        RenderWireframe(state, object, projection, view);
        break;
      case FeedbackStyle::kBoundingBox:
        RenderBoundingBox(state, object, projection, view);
        break;
      default:
        // Default to outline for unknown styles
        RenderOutline(state, object, projection, view);
        break;
    }
  }
}

bool ObjectFeedbackHandler::HasFeedback(const std::string& object_name,
                                       FeedbackType type) const {
  if (object_name.empty()) {
    return false;
  }

  auto it = std::find_if(active_feedback_.begin(), active_feedback_.end(),
                        [&object_name, type](const ObjectFeedbackState& state) {
                          return state.object_name == object_name && state.type == type;
                        });
  
  return it != active_feedback_.end();
}

std::vector<std::string> ObjectFeedbackHandler::GetObjectsWithFeedback(FeedbackType type) const {
  std::vector<std::string> result;
  
  for (const auto& state : active_feedback_) {
    if (state.type == type) {
      result.push_back(state.object_name);
    }
  }
  
  return result;
}

glm::vec4 ObjectFeedbackHandler::GetColorForFeedbackType(FeedbackType type, const FeedbackTheme& theme) const {
  switch (type) {
    case FeedbackType::kHover:
      return theme.hover_color;
    case FeedbackType::kSelection:
      return theme.selection_color;
    case FeedbackType::kPreSelection:
      return theme.pre_selection_color;
    case FeedbackType::kManipulation:
      return theme.manipulation_color;
    case FeedbackType::kError:
      return theme.error_color;
    case FeedbackType::kSuccess:
      return theme.success_color;
    default:
      return glm::vec4(1.0f, 1.0f, 1.0f, 1.0f); // White fallback
  }
}

void ObjectFeedbackHandler::UpdateAnimation(ObjectFeedbackState& state, float delta_time) {
  state.animation_time += delta_time;
  
  if (state.style == FeedbackStyle::kPulse) {
    // Update pulse phase for pulsing animation
    state.pulse_phase += delta_time * state.animation_speed;
    state.intensity = 0.5f + 0.5f * std::sin(state.pulse_phase);
  }
}

float ObjectFeedbackHandler::CalculateAnimatedIntensity(const ObjectFeedbackState& state) const {
  if (state.style == FeedbackStyle::kPulse) {
    return state.intensity;
  }
  
  return 1.0f; // Full intensity for non-animated styles
}

// Shader management placeholder methods

void ObjectFeedbackHandler::InitializeShaders() {
  // Placeholder: Would initialize OpenGL shaders for feedback rendering
  // This would create outline, overlay, and wireframe shader programs
  shaders_initialized_ = true;
}

void ObjectFeedbackHandler::CleanupShaders() {
  // Placeholder: Would clean up OpenGL shader resources
  if (outline_shader_program_ != 0) {
    // glDeleteProgram(outline_shader_program_);
    outline_shader_program_ = 0;
  }
  if (overlay_shader_program_ != 0) {
    // glDeleteProgram(overlay_shader_program_);
    overlay_shader_program_ = 0;
  }
  if (wireframe_shader_program_ != 0) {
    // glDeleteProgram(wireframe_shader_program_);
    wireframe_shader_program_ = 0;
  }
  shaders_initialized_ = false;
}

// Placeholder rendering methods - these would need full OpenGL implementation

void ObjectFeedbackHandler::RenderOutline(const ObjectFeedbackState& state, OpenGlObject* object,
                                         const glm::mat4& projection, const glm::mat4& view) {
  // Use the proper SetHighlighted method from GeometricPrimitive
  if (object) {
    bool should_highlight = (state.type == FeedbackType::kSelection || 
                           state.type == FeedbackType::kHover ||
                           state.type == FeedbackType::kPreSelection);
    object->SetHighlighted(should_highlight);
  }
}

void ObjectFeedbackHandler::RenderSurfaceOverlay(const ObjectFeedbackState& state, OpenGlObject* object,
                                                const glm::mat4& projection, const glm::mat4& view) {
  // Placeholder: Would render semi-transparent colored overlay
  // 1. Enable blending
  // 2. Render object with feedback color and alpha
  // 3. Restore original blending state
}

void ObjectFeedbackHandler::RenderWireframe(const ObjectFeedbackState& state, OpenGlObject* object,
                                           const glm::mat4& projection, const glm::mat4& view) {
  // Placeholder: Would render object in wireframe mode
  // 1. Set polygon mode to GL_LINE
  // 2. Render object with feedback color
  // 3. Restore polygon mode
}

void ObjectFeedbackHandler::RenderBoundingBox(const ObjectFeedbackState& state, OpenGlObject* object,
                                             const glm::mat4& projection, const glm::mat4& view) {
  // Placeholder: Would render wireframe bounding box
  // 1. Calculate object bounding box
  // 2. Render box edges with feedback color
}

} // namespace quickviz