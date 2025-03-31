/*
 * @file camera.cpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "renderer/camera.hpp"

#include <cmath>

namespace quickviz {
Camera::Camera(float fov) : fov_(fov) {
  current_state_.position = glm::vec3(0.0f, 0.0f, 0.0f);
  current_state_.yaw = -90.0f;
  current_state_.pitch = 0.0f;

  UpdateCameraVectors();

  // save initial state for reset
  initial_state_.position = current_state_.position;
  initial_state_.yaw = current_state_.yaw;
  initial_state_.pitch = current_state_.pitch;
  initial_state_.front = current_state_.front;
  initial_state_.up = current_state_.up;
  initial_state_.right = current_state_.right;
}

Camera::Camera(glm::vec3 position, float yaw, float pitch, float fov)
    : fov_(fov) {
  current_state_.position = position;
  current_state_.yaw = yaw;
  current_state_.pitch = pitch;

  UpdateCameraVectors();

  // save initial state for reset
  initial_state_.position = position;
  initial_state_.yaw = yaw;
  initial_state_.pitch = pitch;
  initial_state_.front = current_state_.front;
  initial_state_.up = current_state_.up;
  initial_state_.right = current_state_.right;
}

void Camera::Reset() {
  current_state_ = initial_state_;
  UpdateCameraVectors();
}

void Camera::SetWorldUpVector(glm::vec3 up) {
  world_up_ = up;
  UpdateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix() const {
  return glm::lookAt(current_state_.position,
                     current_state_.position + current_state_.front,
                     current_state_.up);
}

glm::mat4 Camera::GetProjectionMatrix(float aspect_ratio, float z_near,
                                      float z_far) const {
  return glm::perspective(glm::radians(fov_), aspect_ratio, z_near, z_far);
}

void Camera::SetPosition(const glm::vec3& position) {
  current_state_.position = position;
}

void Camera::SetYaw(float yaw) {
  current_state_.yaw = yaw;
  UpdateCameraVectors();
}

void Camera::SetPitch(float pitch) {
  current_state_.pitch = pitch;
  UpdateCameraVectors();
}

void Camera::LookAt(const glm::vec3& target) {
  current_state_.front = glm::normalize(target - current_state_.position);
  current_state_.right =
      glm::normalize(glm::cross(current_state_.front, world_up_));
  current_state_.up =
      glm::normalize(glm::cross(current_state_.right, current_state_.front));
}

void Camera::UpdateCameraVectors() {
  glm::vec3 front;
  front.x = std::cos(glm::radians(current_state_.yaw)) *
            std::cos(glm::radians(current_state_.pitch));
  front.y = std::sin(glm::radians(current_state_.pitch));
  front.z = std::sin(glm::radians(current_state_.yaw)) *
            std::cos(glm::radians(current_state_.pitch));
  current_state_.front = glm::normalize(front);

  current_state_.right =
      glm::normalize(glm::cross(current_state_.front, world_up_));
  current_state_.up =
      glm::normalize(glm::cross(current_state_.right, current_state_.front));
}

void Camera::ProcessKeyboard(Camera::Movement direction, float dt) {
  float velocity = movement_speed_ * dt;
  if (direction == Movement::kForward)
    current_state_.position += current_state_.front * velocity;
  if (direction == Movement::kBackward)
    current_state_.position -= current_state_.front * velocity;
  if (direction == Movement::kLeft)
    current_state_.position -= current_state_.right * velocity;
  if (direction == Movement::kRight)
    current_state_.position += current_state_.right * velocity;
}

void Camera::ProcessMouseMovement(float x_offset, float y_offset,
                                  bool constrain_pitch) {
  x_offset *= mouse_sensitivity_;
  y_offset *= mouse_sensitivity_;

  current_state_.yaw += x_offset;
  current_state_.pitch += y_offset;

  if (constrain_pitch) {
    if (current_state_.pitch > pitch_max) current_state_.pitch = pitch_max;
    if (current_state_.pitch < pitch_min) current_state_.pitch = pitch_min;
  }

  UpdateCameraVectors();
}

void Camera::ProcessMouseScroll(float y_offset) {
  fov_ -= y_offset;
  if (fov_ < fov_min) fov_ = fov_min;
  if (fov_ > fov_max) fov_ = fov_max;
}
}  // namespace quickviz