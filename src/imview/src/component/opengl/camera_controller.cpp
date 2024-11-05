/*
 * @file camera_controller.cpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/camera_controller.hpp"

namespace quickviz {
CameraController::CameraController(Camera& camera) : camera_(camera) {}

void CameraController::SetMode(CameraController::Mode mode) {
  mode_ = mode;
  if (mode == Mode::kTopDown) {
    camera_.SetPosition(glm::vec3(0.0f, top_down_height_, 0.0f));
    camera_.SetPitch(-90.0f);
    camera_.SetYaw(0.0f);
  }
}

void CameraController::ProcessKeyboard(
    CameraController::CameraMovement direction, float delta_time) {
  if (mode_ == Mode::kOrbit) return;
  if (mode_ == Mode::kTopDown) {
    float velocity = camera_.GetMovementSpeed() * delta_time;
    glm::vec3 position = camera_.GetPosition();
    // Move only along X and Z axes
    if (direction == CameraMovement::kForward) position.z -= velocity;
    if (direction == CameraMovement::kBackward) position.z += velocity;
    if (direction == CameraMovement::kLeft) position.x -= velocity;
    if (direction == CameraMovement::kRight) position.x += velocity;

    camera_.SetPosition(position);  // Update position without changing height
  }
  camera_.ProcessKeyboard(direction, delta_time);
}

void CameraController::ProcessMouseMovement(float x_offset, float y_offset) {
  switch (mode_) {
    case Mode::kFirstPerson:
    case Mode::kFreeLook:
      camera_.ProcessMouseMovement(x_offset, y_offset);
      break;
    case Mode::kOrbit:
      camera_.ProcessMouseMovement(x_offset, y_offset);
      UpdateOrbitPosition();
      break;
    case Mode::kTopDown:
      // Ignore mouse movement for top-down view
      break;
  }
}

void CameraController::ProcessMouseScroll(float y_offset) {
  if (mode_ == Mode::kOrbit) {
    orbit_distance_ -= y_offset;
    if (orbit_distance_ < 1.0f) orbit_distance_ = 1.0f;
    UpdateOrbitPosition();
  } else if (mode_ == Mode::kTopDown) {
    glm::vec3 position = camera_.GetPosition();
    position.y -= y_offset;  // Adjust height (Y position) with scroll
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