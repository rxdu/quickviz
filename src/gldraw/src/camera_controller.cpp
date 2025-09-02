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
    : camera_(camera), config_(CameraControllerConfig::Default()) {
  camera_.SetPosition(position);
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);

  orbit_distance_ = config_.initial_orbit_distance;

  UpdateOrbitPosition();
}

CameraController::CameraController(Camera& camera, const CameraControllerConfig& config,
                                   glm::vec3 position, float yaw, float pitch)
    : camera_(camera), config_(config) {
  camera_.SetPosition(position);
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);

  orbit_distance_ = config_.initial_orbit_distance;

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
    if (position.y < config_.min_height) position.y = config_.min_height;
    camera_.SetPosition(position);

    // Reset rotation angle for top-down view
    top_down_rotation_ = 0.0f;
  }
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
  if (!IsValidPosition(target)) {
    // Use current target if invalid position provided
    return;
  }
  orbit_target_ = target;
  if (mode_ == Mode::kOrbit) {
    UpdateOrbitPosition();
  }
}

void CameraController::SetConfig(const CameraControllerConfig& config) {
  CameraControllerConfig validated_config = config;
  validated_config.Validate();
  
  if (!validated_config.IsValid()) {
    // Log warning but use validated config anyway
    // In a production system, you might want to use a logging system here
    validated_config = CameraControllerConfig::Default();
    validated_config.Validate();
  }
  
  config_ = validated_config;
  
  // Re-validate current orbit distance with new limits
  if (orbit_distance_ < config_.min_orbit_distance) {
    orbit_distance_ = config_.min_orbit_distance;
    if (mode_ == Mode::kOrbit) {
      UpdateOrbitPosition();
    }
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


void CameraController::ProcessMouseScroll(float y_offset) {
  if (!std::isfinite(y_offset) || std::abs(y_offset) > 100.0f) {
    return;  // Ignore invalid or excessive scroll values
  }
  if (mode_ == Mode::kOrbit) {
    orbit_distance_ -= y_offset * config_.orbit_zoom_speed;
    if (orbit_distance_ < config_.min_orbit_distance) {
      orbit_distance_ = config_.min_orbit_distance;
    }
    UpdateOrbitPosition();
  } else if (mode_ == Mode::kTopDown) {
    glm::vec3 position = camera_.GetPosition();
    // In TopDown mode, adjust Y position (height) with scroll
    position.y -= y_offset * config_.topdown_zoom_speed;
    if (position.y < config_.min_height) {
      position.y = config_.min_height;
    }
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

void CameraController::ProcessOrbitMovement(float x_offset, float y_offset) {
  if (!IsValidMovement(x_offset, y_offset)) {
    return;
  }
  switch (mode_) {
    case Mode::kFirstPerson:
    case Mode::kFreeLook:
      // Standard look around movement
      camera_.ProcessMouseMovement(x_offset, y_offset);
      break;
    case Mode::kOrbit:
      // Rotation around target
      camera_.ProcessMouseMovement(x_offset, y_offset);
      UpdateOrbitPosition();
      break;
    case Mode::kTopDown:
      // Rotation in top-down view
      if (std::abs(x_offset) > config_.rotation_threshold) {
        top_down_rotation_ += x_offset * config_.orbit_rotation_sensitivity;
        
        // Keep rotation in range [0, 360)
        while (top_down_rotation_ >= 360.0f) top_down_rotation_ -= 360.0f;
        while (top_down_rotation_ < 0.0f) top_down_rotation_ += 360.0f;
        
        camera_.SetYaw(top_down_rotation_);
      }
      break;
  }
}

void CameraController::ProcessPanMovement(float x_offset, float y_offset) {
  if (!IsValidMovement(x_offset, y_offset)) {
    return;
  }
  switch (mode_) {
    case Mode::kFirstPerson:
    case Mode::kFreeLook:
      {
        // Translate perpendicular to viewing direction
        glm::vec3 right = camera_.GetRight();
        glm::vec3 up = camera_.GetUp();
        glm::vec3 position = camera_.GetPosition();
        glm::vec3 translation = (-x_offset * right + y_offset * up) * config_.pan_sensitivity;
        camera_.SetPosition(position + translation);
      }
      break;
    case Mode::kOrbit:
      {
        // Translate the orbit target
        float distance_factor = orbit_distance_ / config_.distance_scale_factor;
        if (distance_factor < 0.1f) distance_factor = 0.1f;
        
        glm::vec3 right = camera_.GetRight();
        glm::vec3 up = camera_.GetUp();
        glm::vec3 translation = (-x_offset * right + y_offset * up) * 
                               config_.pan_sensitivity * distance_factor;
        
        orbit_target_ += translation;
        UpdateOrbitPosition();
      }
      break;
    case Mode::kTopDown:
      {
        // Translation on the X-Z plane
        glm::vec3 position = camera_.GetPosition();
        float height_factor = position.y / config_.height_scale_factor;
        if (height_factor < 0.1f) height_factor = 0.1f;
        
        float angle_rad = glm::radians(top_down_rotation_);
        float rot_dx = -y_offset * std::cos(angle_rad) - x_offset * std::sin(angle_rad);
        float rot_dz = -y_offset * std::sin(angle_rad) + x_offset * std::cos(angle_rad);
        
        rot_dx *= config_.pan_sensitivity * height_factor;
        rot_dz *= config_.pan_sensitivity * height_factor;
        
        position.x += rot_dx;
        position.z += rot_dz;
        camera_.SetPosition(position);
      }
      break;
  }
}

// Static validation helper implementations
bool CameraController::IsValidMovement(float x_offset, float y_offset) {
  // Check for finite values and reasonable range
  constexpr float max_movement = 1000.0f;
  
  return std::isfinite(x_offset) && std::isfinite(y_offset) &&
         std::abs(x_offset) <= max_movement && std::abs(y_offset) <= max_movement;
}

bool CameraController::IsValidPosition(const glm::vec3& position) {
  // Check for finite values and reasonable range
  constexpr float max_position = 1e6f;  // 1 million units max
  constexpr float min_position = -1e6f;
  
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         position.x >= min_position && position.x <= max_position &&
         position.y >= min_position && position.y <= max_position &&
         position.z >= min_position && position.z <= max_position;
}

bool CameraController::IsValidDistance(float distance) {
  // Check for positive finite distance within reasonable bounds
  constexpr float max_distance = 1e6f;  // 1 million units max
  constexpr float min_distance = 1e-6f; // Very small but positive
  
  return std::isfinite(distance) && distance >= min_distance && distance <= max_distance;
}

}  // namespace quickviz