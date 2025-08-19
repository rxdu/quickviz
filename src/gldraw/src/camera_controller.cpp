/*
 * @file camera_controller.cpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "gldraw/camera_controller.hpp"

namespace quickviz {
CameraController::CameraController(Camera& camera, glm::vec3 position,
                                   float yaw, float pitch)
    : camera_(camera) {
  camera_.SetPosition(position);
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);

  orbit_distance_ = glm::length(camera_.GetPosition());

  UpdateOrbitPosition();
}

void CameraController::Reset() { camera_.Reset(); }

void CameraController::SetMode(CameraController::Mode mode) {
  if (mode == mode_) return;

  mode_ = mode;
  if (mode_ == Mode::kTopDown) {
    // Don't override the camera position, just set the pitch and yaw
    // This allows the GlSceneManager to position the camera along the Z-axis
    camera_.SetPitch(-90.0f);
    camera_.SetYaw(0.0f);

    // make sure the position is not too low
    glm::vec3 position = camera_.GetPosition();
    if (position.y < 1.0f) position.y = 1.0f;  // Set a minimum height
    camera_.SetPosition(position);

    // Reset rotation angle for top-down view
    top_down_rotation_ = 0.0f;
  }
}

void CameraController::SetActiveMouseButton(int button) {
  active_mouse_button_ = button;
}

void CameraController::SetHeight(float height) {
  glm::vec3 position = camera_.GetPosition();
  position.y = height;
  camera_.SetPosition(position);
}

glm::vec2 CameraController::GetPosition() const {
  auto pos = camera_.GetPosition();
  // Return only X and Z coordinates for 2D position
  return glm::vec2(pos.x, pos.z);
}

void CameraController::SetPosition(const glm::vec2& position) {
  glm::vec3 pos = camera_.GetPosition();
  if (mode_ == Mode::kTopDown) {
    pos.x = position.x;
    pos.z = -position.y;  // Use Y for Z in 2D view
    camera_.SetPosition(pos);
  }
  // do nothing in other modes
}

void CameraController::SetYaw(float yaw) {
  if (mode_ == Mode::kTopDown) {
    // In TopDown mode, we set the yaw directly
    top_down_rotation_ = yaw;
    camera_.SetYaw(yaw);
  } else {
    camera_.SetYaw(yaw);
  }
}

void CameraController::SetOrbitTarget(const glm::vec3& target) {
  orbit_target_ = target;
  if (mode_ == Mode::kOrbit) {
    UpdateOrbitPosition();
  }
}

void CameraController::ProcessKeyboard(
    CameraController::CameraMovement direction, float delta_time) {
  if (mode_ == Mode::kOrbit) return;
  if (mode_ == Mode::kTopDown) {
    float velocity = camera_.GetMovementSpeed() * delta_time;
    glm::vec3 position = camera_.GetPosition();

    // In TopDown mode, camera is always looking at X-Z plane from above
    if (direction == CameraMovement::kUp) position.y -= velocity;
    if (direction == CameraMovement::kDown) position.y += velocity;
    if (direction == CameraMovement::kForward) position.x -= velocity;
    if (direction == CameraMovement::kBackward) position.x += velocity;
    if (direction == CameraMovement::kLeft) position.z += velocity;
    if (direction == CameraMovement::kRight) position.z -= velocity;

    camera_.SetPosition(position);
  } else {
    camera_.ProcessKeyboard(direction, delta_time);
  }
}

void CameraController::ProcessMouseMovement(float x_offset, float y_offset) {
  switch (mode_) {
    case Mode::kFirstPerson:
    case Mode::kFreeLook:
      if (active_mouse_button_ == MouseButton::kLeft) {
        // Left mouse: standard look around
        camera_.ProcessMouseMovement(x_offset, y_offset);
      } else if (active_mouse_button_ == MouseButton::kMiddle) {
        // Middle mouse: translate perpendicular to viewing direction
        float sensitivity = 0.01f;
        
        // Get camera's right and up vectors
        glm::vec3 right = camera_.GetRight();
        glm::vec3 up = camera_.GetUp();
        
        // Calculate translation in world space
        glm::vec3 position = camera_.GetPosition();
        glm::vec3 translation = (-x_offset * right + y_offset * up) * sensitivity;
        
        camera_.SetPosition(position + translation);
      } else if (active_mouse_button_ == MouseButton::kRight) {
        // Right mouse: alternative - could be used for different behavior
        camera_.ProcessMouseMovement(x_offset, y_offset);
      }
      break;
    case Mode::kOrbit:
      if (active_mouse_button_ == MouseButton::kLeft) {
        // Left mouse: rotation around target
        camera_.ProcessMouseMovement(x_offset, y_offset);
        UpdateOrbitPosition();
      } else if (active_mouse_button_ == MouseButton::kMiddle) {
        // Middle mouse: translate the orbit target
        // Calculate movement in camera's local coordinate system
        float sensitivity = 0.01f;
        
        // Scale movement based on distance for consistent speed
        float distance_factor = orbit_distance_ / 10.0f;
        if (distance_factor < 0.1f) distance_factor = 0.1f;
        
        // Get camera's right and up vectors
        glm::vec3 right = camera_.GetRight();
        glm::vec3 up = camera_.GetUp();
        
        // Calculate translation in world space
        glm::vec3 translation = (-x_offset * right + y_offset * up) * 
                               sensitivity * distance_factor;
        
        // Update orbit target
        orbit_target_ += translation;
        UpdateOrbitPosition();
      } else if (active_mouse_button_ == MouseButton::kRight) {
        // Right mouse: could be used for alternative rotation or other function
        camera_.ProcessMouseMovement(x_offset, y_offset);
        UpdateOrbitPosition();
      }
      break;
    case Mode::kTopDown:
      // Handle mouse movement for top-down view based on mouse button
      if (active_mouse_button_ == MouseButton::kLeft) {
        // Instead of directly setting camera yaw, we'll track rotation angle
        // ourselves and apply it only when there's significant mouse movement
        float rotation_sensitivity = 0.5f;

        // Only process rotation if there's actual mouse movement
        if (std::abs(x_offset) > 0.1f) {
          // Update our own rotation variable
          top_down_rotation_ += x_offset * rotation_sensitivity;

          // Keep rotation in range [0, 360)
          while (top_down_rotation_ >= 360.0f) top_down_rotation_ -= 360.0f;
          while (top_down_rotation_ < 0.0f) top_down_rotation_ += 360.0f;

          // Set the camera yaw directly instead of using ProcessMouseMovement
          camera_.SetYaw(top_down_rotation_);
        }
      } else if (active_mouse_button_ == MouseButton::kMiddle) {
        // Translation/panning on the X-Z plane - implement true dragging
        // behavior
        float sensitivity = 0.01f;

        // Calculate movement based on camera height for consistent speed at
        // different zoom levels
        float height_factor =
            camera_.GetPosition().y / 10.0f;  // Normalize based on height
        if (height_factor < 0.1f) height_factor = 0.1f;  // Minimum factor

        // Apply rotation to the mouse movement vectors to correctly map to the
        // rotated world
        // 1. Create a rotation matrix for the current rotation angle
        float angle_rad = glm::radians(top_down_rotation_);

        // 2. Calculate rotated axes based on the current rotation
        float rot_dx =
            -y_offset * std::cos(angle_rad) - x_offset * std::sin(angle_rad);
        float rot_dz =
            -y_offset * std::sin(angle_rad) + x_offset * std::cos(angle_rad);

        // 3. Apply sensitivity and height scaling
        rot_dx *= sensitivity * height_factor;
        rot_dz *= sensitivity * height_factor;

        // 4. Update camera position
        glm::vec3 position = camera_.GetPosition();
        position.x += rot_dx;
        position.z += rot_dz;
        camera_.SetPosition(position);
      }
      break;
  }
}

void CameraController::ProcessMouseScroll(float y_offset) {
  if (mode_ == Mode::kOrbit) {
    orbit_distance_ -= y_offset * orbit_zoom_speed_;
    if (orbit_distance_ < 1.0f) orbit_distance_ = 1.0f;
    UpdateOrbitPosition();
  } else if (mode_ == Mode::kTopDown) {
    glm::vec3 position = camera_.GetPosition();
    // In TopDown mode, adjust Y position (height) with scroll
    position.y -= y_offset * topdown_zoom_speed_;
    if (position.y < 1.0f) position.y = 1.0f;  // Set a minimum height
    camera_.SetPosition(position);
  } else {
    camera_.ProcessMouseScroll(y_offset);
  }
}

void CameraController::UpdateOrbitPosition() {
  float cam_x = orbit_target_.x + orbit_distance_ *
                                      cos(glm::radians(camera_.GetYaw())) *
                                      cos(glm::radians(camera_.GetPitch()));
  float cam_y =
      orbit_target_.y + orbit_distance_ * sin(glm::radians(camera_.GetPitch()));
  float cam_z = orbit_target_.z + orbit_distance_ *
                                      sin(glm::radians(camera_.GetYaw())) *
                                      cos(glm::radians(camera_.GetPitch()));
  camera_.SetPosition(glm::vec3(cam_x, cam_y, cam_z));
  camera_.LookAt(orbit_target_);
}
}  // namespace quickviz