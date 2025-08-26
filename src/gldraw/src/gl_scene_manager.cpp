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
  
  // If using coordinate transformation, we need to transform the ray
  if (use_coord_transform_) {
    glm::mat4 transform_inverse = glm::inverse(coord_transform_);
    ray_direction = glm::vec3(transform_inverse * glm::vec4(ray_direction, 0.0f));
    ray_direction = glm::normalize(ray_direction);
  }
  
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

}  // namespace quickviz