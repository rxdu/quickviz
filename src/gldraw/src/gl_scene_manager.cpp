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

#include "imview/fonts.hpp"
#include "gldraw/coordinate_system_transformer.hpp"
#include "gldraw/renderable/point_cloud.hpp"

namespace quickviz {
GlSceneManager::GlSceneManager(const std::string& name, Mode mode)
    : Panel(name), mode_(mode) {
  this->SetAutoLayout(false);
  this->SetWindowNoMenuButton();
  //   this->SetNoBackground(true);

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
}

GlSceneManager::~GlSceneManager() {
  ClearOpenGLObjects();
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
  drawable_objects_[name] = std::move(object);
}

void GlSceneManager::RemoveOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
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

void GlSceneManager::DrawOpenGLObject() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  float width = content_size.x;
  float height = content_size.y;

  if (frame_buffer_ != nullptr) {
    if (frame_buffer_->GetWidth() != width ||
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

    // render frame buffer to ImGui
    ImVec2 uv0 = ImVec2(0, 1);
    ImVec2 uv1 = ImVec2(1, 0);
    ImVec4 tint_col = ImVec4(1, 1, 1, 1);
    ImVec4 border_col = ImVec4(0, 0, 0, 0);
    ImGui::Image((void*)(intptr_t)frame_buffer_->GetTextureId(),
                 ImVec2(width, height), uv0, uv1, tint_col, border_col);
  } else {
    frame_buffer_ = std::make_unique<FrameBuffer>(width, height);
  }
}

void GlSceneManager::RenderInsideWindow() {
  // update view according to user input
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // only process mouse delta when mouse position is within the scene panel
  if (ImGui::IsMousePosValid() && io.WantCaptureMouse &&
      ImGui::IsWindowHovered()) {
    // Check for mouse buttons and update camera controller state accordingly
    int active_button = MouseButton::kNone;

    if (ImGui::IsMouseDown(MouseButton::kLeft)) {
      active_button = MouseButton::kLeft;
    } else if (ImGui::IsMouseDown(MouseButton::kMiddle)) {
      active_button = MouseButton::kMiddle;
    } else if (ImGui::IsMouseDown(MouseButton::kRight)) {
      active_button = MouseButton::kRight;
    }

    // Set the active mouse button in the camera controller
    camera_controller_->SetActiveMouseButton(active_button);

    // Process mouse movement if any button is pressed
    if (active_button != MouseButton::kNone) {
      camera_controller_->ProcessMouseMovement(io.MouseDelta.x,
                                               io.MouseDelta.y);
    }

    // track mouse wheel scroll
    camera_controller_->ProcessMouseScroll(io.MouseWheel);
  } else {
    // Reset mouse button state when mouse is outside the window
    camera_controller_->SetActiveMouseButton(MouseButton::kNone);
  }

  // get view matrices from camera
  float aspect_ratio = (frame_buffer_ == nullptr)
                           ? static_cast<float>(content_size.x) /
                                 static_cast<float>(content_size.y)
                           : frame_buffer_->GetAspectRatio();
  glm::mat4 projection =
      camera_->GetProjectionMatrix(aspect_ratio, z_near_, z_far_);
  glm::mat4 view = camera_->GetViewMatrix();
  UpdateView(projection, view);

  // Call pre-draw callback if set
  if (pre_draw_callback_) {
    pre_draw_callback_();
  }

  // finally draw the scene
  DrawOpenGLObject();

  // draw frame rate at the bottom of the scene
  if (show_rendering_info_) {
    ImGui::SetCursorPos(ImVec2(10, content_size.y - 25));
    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
    ImGui::Text("FPS: %.1f, %.3f ms/frame", ImGui::GetIO().Framerate,
                1000.0f / ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();
    ImGui::PopFont();
  }
}

void GlSceneManager::Draw() {
  Begin();

  RenderInsideWindow();

  End();
}

GlSceneManager::MouseRay GlSceneManager::GetMouseRayInWorldSpace(
    float mouse_x, float mouse_y, float window_width, float window_height) const {
  MouseRay ray;
  
  // Check if we have valid dimensions
  if (window_width <= 0 || window_height <= 0 || !camera_) {
    return ray;
  }
  
  // Convert mouse coordinates to normalized device coordinates (NDC)
  // NDC ranges from -1 to 1 in both x and y
  float x_ndc = (2.0f * mouse_x) / window_width - 1.0f;
  float y_ndc = 1.0f - (2.0f * mouse_y) / window_height; // Flip Y axis
  
  // Create ray in clip space
  glm::vec4 ray_clip(x_ndc, y_ndc, -1.0f, 1.0f);
  
  // Convert to eye space
  glm::mat4 proj_inverse = glm::inverse(projection_);
  glm::vec4 ray_eye = proj_inverse * ray_clip;
  ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0f, 0.0f);
  
  // Convert to world space
  glm::mat4 view_inverse = glm::inverse(view_);
  glm::vec4 ray_world = view_inverse * ray_eye;
  glm::vec3 ray_direction = glm::normalize(glm::vec3(ray_world));
  
  // The ray is now in world space, which is what we want
  // The coordinate transform is applied to objects when rendering,
  // so we work in the original world space for selection
  
  ray.origin = camera_->GetPosition();
  ray.direction = ray_direction;
  ray.valid = true;
  
  return ray;
}

// GPU ID-buffer picking implementation
void GlSceneManager::RenderIdBuffer() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  float width = content_size.x;
  float height = content_size.y;
  
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
  
  // Apply coordinate system transformation if enabled
  glm::mat4 transform = use_coord_transform_ ? coord_transform_ : glm::mat4(1.0f);
  
  // Render only point clouds in ID mode
  int point_cloud_count = 0;
  for (auto& obj : drawable_objects_) {
    PointCloud* point_cloud = dynamic_cast<PointCloud*>(obj.second.get());
    if (point_cloud) {
      point_cloud_count++;
      
      // Temporarily switch to ID buffer rendering mode
      PointMode original_mode = point_cloud->GetRenderMode();
      point_cloud->SetRenderMode(PointMode::kIdBuffer);
      
      // Render the point cloud with ID encoding
      point_cloud->OnDraw(projection_, view_, transform);
      
      // Restore original rendering mode
      point_cloud->SetRenderMode(original_mode);
    }
  }
  
  
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
  
  // Decode point index from RGB values
  size_t decoded = PointCloud::DecodePointId(pixel[0], pixel[1], pixel[2]);
  return decoded;
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
  // Get mouse ray for ray-casting
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  MouseRay ray = GetMouseRayInWorldSpace(screen_x, screen_y, content_size.x, content_size.y);
  
  if (!ray.valid) {
    return "";
  }
  
  // Clear previous selection
  if (!selected_object_name_.empty()) {
    auto it = drawable_objects_.find(selected_object_name_);
    if (it != drawable_objects_.end()) {
      it->second->SetHighlighted(false);
    }
  }
  
  // Find closest object that the ray intersects
  std::string closest_object_name;
  float closest_distance = std::numeric_limits<float>::max();
  
  for (const auto& [name, object] : drawable_objects_) {
    // Skip objects that don't support selection
    if (!object->SupportsSelection()) {
      continue;
    }
    
    // Get object bounding box
    auto [min_bounds, max_bounds] = object->GetBoundingBox();
    
    // Skip if bounding box is invalid (zero size)
    if (min_bounds == max_bounds) {
      continue;
    }
    
    // Apply coordinate transformation to bounds if enabled
    if (use_coord_transform_) {
      // Transform the 8 corners of the bounding box
      glm::vec3 corners[8] = {
        glm::vec3(min_bounds.x, min_bounds.y, min_bounds.z),
        glm::vec3(max_bounds.x, min_bounds.y, min_bounds.z),
        glm::vec3(min_bounds.x, max_bounds.y, min_bounds.z),
        glm::vec3(max_bounds.x, max_bounds.y, min_bounds.z),
        glm::vec3(min_bounds.x, min_bounds.y, max_bounds.z),
        glm::vec3(max_bounds.x, min_bounds.y, max_bounds.z),
        glm::vec3(min_bounds.x, max_bounds.y, max_bounds.z),
        glm::vec3(max_bounds.x, max_bounds.y, max_bounds.z)
      };
      
      // Transform all corners and find new AABB
      glm::vec3 new_min(FLT_MAX);
      glm::vec3 new_max(-FLT_MAX);
      for (int i = 0; i < 8; ++i) {
        glm::vec4 transformed = coord_transform_ * glm::vec4(corners[i], 1.0f);
        glm::vec3 point(transformed.x, transformed.y, transformed.z);
        new_min = glm::min(new_min, point);
        new_max = glm::max(new_max, point);
      }
      min_bounds = new_min;
      max_bounds = new_max;
    }
    
    
    // Simple ray-box intersection test
    // This is a basic implementation - could be improved with more sophisticated tests
    glm::vec3 inv_dir = 1.0f / ray.direction;
    glm::vec3 t_min = (min_bounds - ray.origin) * inv_dir;
    glm::vec3 t_max = (max_bounds - ray.origin) * inv_dir;
    
    glm::vec3 t1 = glm::min(t_min, t_max);
    glm::vec3 t2 = glm::max(t_min, t_max);
    
    float t_near = glm::max(glm::max(t1.x, t1.y), t1.z);
    float t_far = glm::min(glm::min(t2.x, t2.y), t2.z);
    
    // Check if ray intersects the box
    if (t_near <= t_far && t_far >= 0) {
      float distance = t_near >= 0 ? t_near : t_far;
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_object_name = name;
      }
    }
  }
  
  // Update selection
  selected_object_name_ = closest_object_name;
  
  // Highlight selected object
  if (!selected_object_name_.empty()) {
    auto it = drawable_objects_.find(selected_object_name_);
    if (it != drawable_objects_.end()) {
      it->second->SetHighlighted(true);
      object_highlights_[selected_object_name_] = true;
    }
  }
  
  // Notify callback
  if (object_selection_callback_) {
    object_selection_callback_(selected_object_name_);
  }
  
  return selected_object_name_;
}

void GlSceneManager::ClearObjectSelection() {
  // Clear highlight on current selection
  if (!selected_object_name_.empty()) {
    auto it = drawable_objects_.find(selected_object_name_);
    if (it != drawable_objects_.end()) {
      it->second->SetHighlighted(false);
    }
    object_highlights_.erase(selected_object_name_);
  }
  
  selected_object_name_.clear();
  
  // Notify callback
  if (object_selection_callback_) {
    object_selection_callback_("");
  }
}

void GlSceneManager::SetObjectHighlight(const std::string& name, bool highlighted) {
  auto it = drawable_objects_.find(name);
  if (it != drawable_objects_.end()) {
    it->second->SetHighlighted(highlighted);
    if (highlighted) {
      object_highlights_[name] = true;
    } else {
      object_highlights_.erase(name);
    }
  }
}

}  // namespace quickviz