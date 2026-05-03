/*
 * scene_manager.cpp
 *
 * Created on 3/6/25 9:09 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "scene/scene_manager.hpp"

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cfloat>

#include <glad/glad.h>

#include "scene/coordinate_transformer.hpp"
#include "scene/selection_manager.hpp"
#include "scene/tools/interaction_tool.hpp"
#include "scene/renderable/point_cloud.hpp"
#include "scene/renderable/geometric_primitive.hpp"

namespace quickviz {
SceneManager::SceneManager(const std::string& name, Mode mode)
    : name_(name), mode_(mode) {

  camera_ = std::make_unique<Camera>();
  if (mode_ == Mode::k3D) {
    camera_controller_ = std::make_unique<CameraController>(
        *camera_, glm::vec3(0.0f, 6.0f, 8.0f), 0.0f, 25.0f);
  } else {
    // For 2D mode, position the camera above the X-Z plane looking down
    // This gives a proper top-down view with Y as the up direction
    camera_controller_ = std::make_unique<CameraController>(
        *camera_, glm::vec3(0.0f, 8.0f, 0.0f), -90.0f, -90.0f);
    camera_controller_->SetMode(CameraController::Mode::kTopDown);
  }

  // Initialize the coordinate system transformation matrix
  coord_transform_ =
      CoordinateTransformer::GetStandardToOpenGLTransform();
  
  // Initialize selection system
  selection_manager_ = std::make_unique<SelectionManager>(this);
  
  // Initialize tool system
  tool_manager_ = std::make_unique<ToolManager>(this);
}

SceneManager::~SceneManager() {
  ClearOpenGLObjects();
  
  // Clean up static shaders from GeometricPrimitive before OpenGL context is destroyed
  // This prevents segfault on exit when static shaders try to clean up after context is gone
  GeometricPrimitive::CleanupShaders();
  
  frame_buffer_.reset();
}

void SceneManager::SetBackgroundColor(float r, float g, float b, float a) {
  background_color_ = glm::vec4(r, g, b, a);
}

void SceneManager::SetClippingPlanes(float z_near, float z_far) {
  z_near_ = z_near;
  z_far_ = z_far;
}

void SceneManager::AddOpenGLObject(const std::string& name,
                                     std::unique_ptr<OpenGlObject> object) {
  if (object == nullptr) {
    throw std::invalid_argument("Object is nullptr");
  }
  
  // Register with selection system before taking ownership
  if (selection_manager_) {
    selection_manager_->RegisterObject(name, object.get());
  }
  
  drawable_objects_[name] = std::move(object);
}

void SceneManager::RemoveOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    // Unregister from selection system before removing
    if (selection_manager_) {
      selection_manager_->UnregisterObject(name);
    }
    
    drawable_objects_.erase(name);
  }
}

OpenGlObject* SceneManager::GetOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    return drawable_objects_[name].get();
  }
  return nullptr;
}

void SceneManager::ClearOpenGLObjects() { drawable_objects_.clear(); }

void SceneManager::UpdateView(const glm::mat4& projection,
                                const glm::mat4& view) {
  projection_ = projection;
  view_ = view;
}

void SceneManager::RenderToFramebuffer(float width, float height) {
  // Get view matrices from camera
  float aspect_ratio = width / height;
  glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio, z_near_, z_far_);
  glm::mat4 view = camera_->GetViewMatrix();
  UpdateView(projection, view);

  // Call pre-draw callback if set
  if (pre_draw_callback_) {
    pre_draw_callback_();
  }

  // Create or resize framebuffer as needed
  if (frame_buffer_ == nullptr) {
    frame_buffer_ = std::make_unique<FrameBuffer>(width, height);
  } else if (frame_buffer_->GetWidth() != width ||
             frame_buffer_->GetHeight() != height) {
    frame_buffer_->Resize(width, height);
  }

  // render to frame buffer
  frame_buffer_->Bind();
  frame_buffer_->Clear(background_color_.r, background_color_.g,
                       background_color_.b, background_color_.a);

  // Apply coordinate system transformation if enabled
  glm::mat4 transform =
      use_coord_transform_ ? coord_transform_ : glm::mat4(1.0f);

  for (auto& obj : drawable_objects_) {
    obj.second->OnDraw(projection_, view_, transform);
  }
  frame_buffer_->Unbind();

}

uint32_t SceneManager::GetFramebufferTexture() const {
  return frame_buffer_ ? frame_buffer_->GetTextureId() : 0;
}


// === Selection System Implementation ===

SelectionResult SceneManager::Select(float screen_x, float screen_y, const SelectionOptions& options) {
  if (!selection_enabled_) {
    return SelectionResult{};
  }
  return selection_manager_->Select(screen_x, screen_y, options);
}

bool SceneManager::AddToSelection(float screen_x, float screen_y, const SelectionOptions& options) {
  if (!selection_enabled_) {
    return false;
  }
  return selection_manager_->AddToSelection(screen_x, screen_y, options);
}

const MultiSelection& SceneManager::GetMultiSelection() const {
  return selection_manager_->GetMultiSelection();
}

// === Interactive Tools System Implementation ===

void SceneManager::RegisterTool(std::shared_ptr<InteractionTool> tool) {
  tool_manager_->RegisterTool(tool);
}

bool SceneManager::ActivateTool(const std::string& name) {
  return tool_manager_->ActivateTool(name);
}

std::shared_ptr<InteractionTool> SceneManager::GetActiveTool() const {
  return tool_manager_->GetActiveTool();
}

}  // namespace quickviz
