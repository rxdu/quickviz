/*
 * gl_scene_manager.cpp
 *
 * Created on 3/6/25 9:09 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/gl_scene_manager.hpp"

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cfloat>

#include <glad/glad.h>

#include "gldraw/coordinate_system_transformer.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/geometric_primitive.hpp"
#include "gldraw/gpu_selection.hpp"

namespace quickviz {
GlSceneManager::GlSceneManager(const std::string& name, Mode mode)
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
      CoordinateSystemTransformer::GetStandardToOpenGLTransform();
  
  // Initialize GPU selection system
  gpu_selection_ = std::make_unique<GPUSelection>(this);
}

GlSceneManager::~GlSceneManager() {
  ClearOpenGLObjects();
  
  // Clean up static shaders from GeometricPrimitive before OpenGL context is destroyed
  // This prevents segfault on exit when static shaders try to clean up after context is gone
  GeometricPrimitive::CleanupShaders();
  
  frame_buffer_.reset();
}

void GlSceneManager::SetShowRenderingInfo(bool show) {
  show_rendering_info_ = show;
}

void GlSceneManager::SetBackgroundColor(float r, float g, float b, float a) {
  background_color_ = glm::vec4(r, g, b, a);
}

void GlSceneManager::SetClippingPlanes(float z_near, float z_far) {
  z_near_ = z_near;
  z_far_ = z_far;
}

void GlSceneManager::AddOpenGLObject(const std::string& name,
                                     std::unique_ptr<OpenGlObject> object) {
  if (object == nullptr) {
    throw std::invalid_argument("Object is nullptr");
  }
  
  // Register with GPU selection system before taking ownership
  if (gpu_selection_) {
    gpu_selection_->RegisterObject(name, object.get());
  }
  
  drawable_objects_[name] = std::move(object);
}

void GlSceneManager::RemoveOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    // Unregister from GPU selection system before removing
    if (gpu_selection_) {
      gpu_selection_->UnregisterObject(name);
    }
    
    drawable_objects_.erase(name);
  }
}

OpenGlObject* GlSceneManager::GetOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    return drawable_objects_[name].get();
  }
  return nullptr;
}

void GlSceneManager::ClearOpenGLObjects() { drawable_objects_.clear(); }

void GlSceneManager::UpdateView(const glm::mat4& projection,
                                const glm::mat4& view) {
  projection_ = projection;
  view_ = view;
}

void GlSceneManager::RenderToFramebuffer(float width, float height) {
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

uint32_t GlSceneManager::GetFramebufferTexture() const {
  return frame_buffer_ ? frame_buffer_->GetTextureId() : 0;
}


// Ray casting methods removed - using GPU ID-buffer selection exclusively

// GPU ID-buffer picking implementation
void GlSceneManager::RenderIdBuffer() {
  if (!frame_buffer_) return;  // Need main framebuffer to get dimensions
  float width = frame_buffer_->GetWidth();
  float height = frame_buffer_->GetHeight();
  
  // CRITICAL FIX: Recalculate projection and view matrices using same logic as main render
  // This ensures perfect synchronization between main render and ID buffer
  float aspect_ratio = width / height;
  glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio, z_near_, z_far_);
  glm::mat4 view = camera_->GetViewMatrix();
  
  // Create or resize ID framebuffer to match main framebuffer size
  // IMPORTANT: Use 0 samples (no multisampling) for ID buffer to ensure exact pixel values
  if (id_frame_buffer_ == nullptr) {
    id_frame_buffer_ = std::make_unique<FrameBuffer>(width, height, 0);  // No multisampling for ID picking
  } else if (id_frame_buffer_->GetWidth() != width || 
             id_frame_buffer_->GetHeight() != height) {
    id_frame_buffer_->Resize(width, height);
  }
  
  // Render to ID framebuffer
  id_frame_buffer_->Bind();
  id_frame_buffer_->Clear(0.0f, 0.0f, 0.0f, 0.0f);  // Black background = no point (ID 0)
  glClear(GL_DEPTH_BUFFER_BIT);  // Ensure depth buffer is cleared
  
  // Ensure proper OpenGL state for ID buffer rendering
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDisable(GL_BLEND); // No blending for ID buffer
  
  // CRITICAL: Set viewport to match ID framebuffer dimensions exactly
  glViewport(0, 0, static_cast<int>(width), static_cast<int>(height));
  
  // Apply coordinate system transformation if enabled
  glm::mat4 transform = use_coord_transform_ ? coord_transform_ : glm::mat4(1.0f);
  
  // Render point clouds in ID mode first
  int point_cloud_count = 0;
  for (auto& obj : drawable_objects_) {
    PointCloud* point_cloud = dynamic_cast<PointCloud*>(obj.second.get());
    if (point_cloud) {
      point_cloud_count++;
      
      // Temporarily switch to ID buffer rendering mode
      PointMode original_mode = point_cloud->GetRenderMode();
      point_cloud->SetRenderMode(PointMode::kIdBuffer);
      
      // Render the point cloud with ID encoding
      point_cloud->OnDraw(projection, view, transform);
      
      // Restore original rendering mode
      point_cloud->SetRenderMode(original_mode);
    }
  }
  
  // Render objects with unique ID colors for selection  
  uint32_t object_id_base = 0x800000; // Start object IDs at 8M to avoid point ID conflicts
  uint32_t current_object_id = object_id_base;
  
  // Store ID->name mapping for decoding (using class member)
  id_to_object_name_.clear();
  
  for (const auto& [name, object] : drawable_objects_) {
    // Skip point clouds (already handled above)
    if (dynamic_cast<PointCloud*>(object.get())) continue;
    
    if (!object->SupportsSelection()) continue;
    
    // Store ID mapping
    id_to_object_name_[current_object_id] = name;
    
    // Convert ID to RGB color (24-bit)
    float r = static_cast<float>((current_object_id >> 0) & 0xFF) / 255.0f;
    float g = static_cast<float>((current_object_id >> 8) & 0xFF) / 255.0f;
    float b = static_cast<float>((current_object_id >> 16) & 0xFF) / 255.0f;
    glm::vec3 id_color(r, g, b);
    
    // Use the new ID rendering interface to render objects with solid ID colors
    if (object->SupportsIdRendering()) {
      object->SetIdRenderMode(true);
      object->SetIdColor(id_color);
      object->OnDraw(projection, view, transform);
      object->SetIdRenderMode(false); // Restore normal rendering mode
    }
    
    current_object_id += 0x100; // Use larger increments for better color separation
    if (current_object_id > 0xFFFFFF) break; // Prevent overflow
  }
  
  // Restore OpenGL state
  glEnable(GL_BLEND); // Re-enable blending for normal rendering
  
  id_frame_buffer_->Unbind();
}

size_t GlSceneManager::ReadPixelId(int x, int y) {
  if (!id_frame_buffer_) {
    return SIZE_MAX; // Invalid
  }
  
  // Flip Y coordinate (OpenGL bottom-left vs screen top-left)
  int gl_y = static_cast<int>(id_frame_buffer_->GetHeight()) - y - 1;
  
  // Bind the ID framebuffer for reading
  id_frame_buffer_->Bind();
  
  // Read pixel RGB values
  uint8_t pixel[3];
  glReadPixels(x, gl_y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);
  
  id_frame_buffer_->Unbind();
  
  // Decode ID from RGB values (works for both points and objects)
  uint32_t decoded_id = (uint32_t(pixel[0]) << 0) | (uint32_t(pixel[1]) << 8) | (uint32_t(pixel[2]) << 16);
  
  // ReadPixelId decoding complete
  return decoded_id;
}

size_t GlSceneManager::PickPointAtPixel(int x, int y, const std::string& point_cloud_name) {
  // Render the ID buffer
  RenderIdBuffer();
  
  // Read the point ID at the specified pixel
  return ReadPixelId(x, y);
}

size_t GlSceneManager::PickPointAtPixelWithRadius(int x, int y, int radius, const std::string& point_cloud_name) {
  // Render the ID buffer
  RenderIdBuffer();
  
  if (!id_frame_buffer_) {
    return SIZE_MAX;
  }
  
  // Read pixels in a small radius around the target position
  size_t closest_point = SIZE_MAX;
  float min_distance = static_cast<float>(radius * radius + 1); // Start with distance larger than radius
  
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      int px = x + dx;
      int py = y + dy;
      
      // Check bounds
      if (px < 0 || py < 0 || 
          px >= static_cast<int>(id_frame_buffer_->GetWidth()) || 
          py >= static_cast<int>(id_frame_buffer_->GetHeight())) {
        continue;
      }
      
      // Check if pixel is within circular radius
      float distance = std::sqrt(dx * dx + dy * dy);
      if (distance > radius) continue;
      
      // Read point ID at this pixel
      size_t point_id = ReadPixelId(px, py);
      if (point_id != SIZE_MAX && distance < min_distance) {
        min_distance = distance;
        closest_point = point_id;
      }
    }
  }
  
  return closest_point;
}

// === Point Selection Implementation ===

void GlSceneManager::SetActivePointCloud(PointCloud* point_cloud) {
  active_point_cloud_ = point_cloud;
  // Clear existing selection when switching point clouds
  ClearPointSelection();
}

bool GlSceneManager::SelectPointAt(float screen_x, float screen_y, int radius) {
  if (!active_point_cloud_) {
    return false;
  }
  
  size_t point_index = PickPointAtPixelWithRadius(
      static_cast<int>(screen_x), static_cast<int>(screen_y), radius);
  
  if (point_index == SIZE_MAX || point_index >= active_point_cloud_->GetPointCount()) {
    return false;
  }
  
  // Replace current selection
  selected_point_indices_.clear();
  selected_point_indices_.push_back(point_index);
  
  UpdateSelectionVisualization();
  NotifySelectionChanged();
  return true;
}

bool GlSceneManager::AddPointAt(float screen_x, float screen_y, int radius) {
  if (!active_point_cloud_) {
    return false;
  }
  
  size_t point_index = PickPointAtPixelWithRadius(
      static_cast<int>(screen_x), static_cast<int>(screen_y), radius);
  
  if (point_index == SIZE_MAX || point_index >= active_point_cloud_->GetPointCount()) {
    return false;
  }
  
  // Add to selection if not already selected
  if (!IsPointSelected(point_index)) {
    AddToSelection(point_index);
    UpdateSelectionVisualization();
    NotifySelectionChanged();
  }
  
  return true;
}

bool GlSceneManager::TogglePointAt(float screen_x, float screen_y, int radius) {
  if (!active_point_cloud_) {
    return false;
  }
  
  size_t point_index = PickPointAtPixelWithRadius(
      static_cast<int>(screen_x), static_cast<int>(screen_y), radius);
  
  if (point_index == SIZE_MAX || point_index >= active_point_cloud_->GetPointCount()) {
    return false;
  }
  
  // Toggle selection state
  if (IsPointSelected(point_index)) {
    RemoveFromSelection(point_index);
  } else {
    AddToSelection(point_index);
  }
  
  UpdateSelectionVisualization();
  NotifySelectionChanged();
  return true;
}

void GlSceneManager::ClearPointSelection() {
  if (selected_point_indices_.empty()) {
    return;
  }
  
  selected_point_indices_.clear();
  UpdateSelectionVisualization();
  NotifySelectionChanged();
}

std::vector<glm::vec3> GlSceneManager::GetSelectedPoints() const {
  std::vector<glm::vec3> selected_points;
  
  if (!active_point_cloud_) {
    return selected_points;
  }
  
  const auto& all_points = active_point_cloud_->GetPoints();
  selected_points.reserve(selected_point_indices_.size());
  
  for (size_t idx : selected_point_indices_) {
    if (idx < all_points.size()) {
      selected_points.push_back(all_points[idx]);
    }
  }
  
  return selected_points;
}

std::vector<glm::vec3> GlSceneManager::GetSelectedPointColors() const {
  std::vector<glm::vec3> selected_colors;
  
  if (!active_point_cloud_) {
    return selected_colors;
  }
  
  const auto& all_colors = active_point_cloud_->GetColors();
  if (all_colors.empty()) {
    return selected_colors;  // No color data available
  }
  
  selected_colors.reserve(selected_point_indices_.size());
  for (size_t idx : selected_point_indices_) {
    if (idx < all_colors.size()) {
      selected_colors.push_back(all_colors[idx]);
    }
  }
  
  return selected_colors;
}

glm::vec3 GlSceneManager::GetSelectionCentroid() const {
  if (selected_point_indices_.empty() || !active_point_cloud_) {
    return glm::vec3(0.0f);
  }
  
  const auto& all_points = active_point_cloud_->GetPoints();
  glm::vec3 sum(0.0f);
  size_t valid_count = 0;
  
  for (size_t idx : selected_point_indices_) {
    if (idx < all_points.size()) {
      sum += all_points[idx];
      ++valid_count;
    }
  }
  
  return valid_count > 0 ? sum / static_cast<float>(valid_count) : glm::vec3(0.0f);
}

std::pair<glm::vec3, glm::vec3> GlSceneManager::GetSelectionBounds() const {
  if (selected_point_indices_.empty() || !active_point_cloud_) {
    return {glm::vec3(0.0f), glm::vec3(0.0f)};
  }
  
  const auto& all_points = active_point_cloud_->GetPoints();
  
  // Initialize with first valid point
  glm::vec3 min_bounds(std::numeric_limits<float>::max());
  glm::vec3 max_bounds(std::numeric_limits<float>::lowest());
  
  bool found_valid = false;
  for (size_t idx : selected_point_indices_) {
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

void GlSceneManager::SetSelectionVisualization(const glm::vec3& color,
                                               float size_multiplier,
                                               const std::string& layer_name) {
  selection_color_ = color;
  selection_size_multiplier_ = size_multiplier;
  selection_layer_name_ = layer_name;
  
  if (selection_visualization_enabled_) {
    UpdateSelectionVisualization();
  }
}

void GlSceneManager::SetSelectionVisualizationEnabled(bool enabled) {
  selection_visualization_enabled_ = enabled;
  UpdateSelectionVisualization();
}

// === Private Helper Methods ===

void GlSceneManager::UpdateSelectionVisualization() {
  if (!active_point_cloud_) {
    return;
  }
  
  if (!selection_visualization_enabled_) {
    // Clear visualization
    active_point_cloud_->ClearHighlights(selection_layer_name_);
    return;
  }
  
  if (selected_point_indices_.empty()) {
    // Clear highlights if no selection
    active_point_cloud_->ClearHighlights(selection_layer_name_);
  } else {
    // Apply highlights to selected points
    active_point_cloud_->HighlightPoints(selected_point_indices_, selection_color_, 
                                         selection_layer_name_, selection_size_multiplier_);
  }
}

void GlSceneManager::NotifySelectionChanged() {
  if (point_selection_callback_) {
    point_selection_callback_(selected_point_indices_);
  }
}

bool GlSceneManager::IsPointSelected(size_t point_index) const {
  return std::find(selected_point_indices_.begin(), selected_point_indices_.end(), point_index) 
         != selected_point_indices_.end();
}

void GlSceneManager::RemoveFromSelection(size_t point_index) {
  auto it = std::find(selected_point_indices_.begin(), selected_point_indices_.end(), point_index);
  if (it != selected_point_indices_.end()) {
    selected_point_indices_.erase(it);
  }
}

void GlSceneManager::AddToSelection(size_t point_index) {
  if (!IsPointSelected(point_index)) {
    selected_point_indices_.push_back(point_index);
  }
}

// === Object Selection Implementation ===

std::string GlSceneManager::SelectObjectAt(float screen_x, float screen_y) {
  if (!frame_buffer_) return "";
  
  // Use GPU ID-buffer selection for EVERYTHING (both objects and points)
  // This is the unified approach we agreed on
  
  // Step 1: Render scene to ID buffer with unique colors for each selectable item
  RenderIdBuffer();
  
  // Step 2: Read the pixel color at the mouse position
  // Use precise pixel coordinates (round to nearest pixel center)
  int pixel_x = static_cast<int>(std::round(screen_x));
  int pixel_y = static_cast<int>(std::round(screen_y));
  
  
  uint32_t selected_id = ReadPixelId(pixel_x, pixel_y);
  
  // Coordinate conversion complete
  
  if (selected_id == 0) {
    // Background - no selection
    ClearObjectSelection();
    return "";
  }
  
  // Step 3: Decode the ID to determine what was selected
  
  // Check if this is a point ID (1 to 4M-1)
  if (selected_id >= 1 && selected_id < 0x400000) {
    size_t point_index = selected_id - 1; // Point IDs start at 1
    if (active_point_cloud_) {
      
      // Highlight selected point (TODO: use PointCloud layer system)
      if (object_selection_callback_) {
        object_selection_callback_("point_" + std::to_string(point_index));
      }
      
      return "point_" + std::to_string(point_index);
    }
  }
  
  // Check if this is an object ID (8M and above)
  else if (selected_id >= 0x800000 && selected_id <= 0xFFFFFF) {
    // Look up object name from ID mapping (stored in RenderIdBuffer)
    
    // Use the ID directly - no conversion needed if ID rendering is working properly
    uint32_t actual_object_id = selected_id;
    
    // Use the ID-to-name map that was built during rendering
    // This map is declared as static in RenderIdBuffer() function
    // We need to access it here, so let's make it a class member instead
    auto it = id_to_object_name_.find(actual_object_id);
    if (it != id_to_object_name_.end()) {
      const std::string& name = it->second;
      
      // Clear previous selection and highlight new one
      ClearObjectSelection();
      selected_object_name_ = name;
      
      auto obj_it = drawable_objects_.find(name);
      if (obj_it != drawable_objects_.end()) {
        obj_it->second->SetHighlighted(true);
      }
      
      if (object_selection_callback_) {
        object_selection_callback_(name);
      }
      
      return name;
    }
  }
  
  std::cout << "GPU Selection: Unrecognized ID " << selected_id << std::endl;
  return "";
}

void GlSceneManager::ClearObjectSelection() {
  // Clear object selection
  if (!selected_object_name_.empty()) {
    auto it = drawable_objects_.find(selected_object_name_);
    if (it != drawable_objects_.end()) {
      it->second->SetHighlighted(false);
    }
    selected_object_name_.clear();
  }
  
  // Clear point selection (TODO: implement with PointCloud layer system)
  // For now, just clear the object selection
  
  // Call callback
  if (object_selection_callback_) {
    object_selection_callback_("");
  }
}

void GlSceneManager::SetObjectHighlight(const std::string& name, bool highlighted) {
  // This is handled internally by GPU selection system
  auto it = drawable_objects_.find(name);
  if (it != drawable_objects_.end()) {
    it->second->SetHighlighted(highlighted);
  }
}

// === GPU Selection System Implementation ===

GPUSelectionResult GlSceneManager::GPUSelectAt(float screen_x, float screen_y,
                                               float screen_width, float screen_height,
                                               int radius) {
  if (gpu_selection_) {
    return gpu_selection_->SelectAtScreen(screen_x, screen_y, screen_width, screen_height, radius);
  }
  return GPUSelectionResult::None();
}

void GlSceneManager::SetGPUSelectionMode(GPUSelectionMode mode) {
  if (gpu_selection_) {
    gpu_selection_->SetMode(mode);
  }
}

GPUSelectionMode GlSceneManager::GetGPUSelectionMode() const {
  if (gpu_selection_) {
    return gpu_selection_->GetMode();
  }
  return GPUSelectionMode::kHybrid;
}

}  // namespace quickviz