/*
 * @file camera.hpp
 * @date 11/3/24
 * @brief a perspective camera class
 *
 * V_clip = P * V * M * V_local
 * P: projection matrix
 * V: view matrix
 * M: model matrix
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CAMERA_HPP
#define QUICKVIZ_CAMERA_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace quickviz {
class Camera {
  static constexpr float default_movement_speed = 2.5f;
  static constexpr float default_mouse_sensitivity = 0.05f;
  static constexpr float default_fov = 45.0f;
  static constexpr float pitch_min = -89.0f;
  static constexpr float pitch_max = 89.0f;
  static constexpr float fov_min = 1.0f;
  static constexpr float fov_max = 45.0f;

  struct State {
    // camera pose
    glm::vec3 position;
    float yaw;
    float pitch;

    // camera reference vectors
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
  };

 public:
  enum class Movement { kForward, kBackward, kLeft, kRight };

 public:
  Camera(glm::vec3 position, float yaw, float pitch, float fov = default_fov);

  void SetWorldUpVector(glm::vec3 up) { world_up_ = up; }

  void Reset();
  glm::mat4 GetViewMatrix() const;
  glm::mat4 GetProjectionMatrix(float aspect_ratio, float z_near = 0.1f,
                                float z_far = 100.0f) const;

  void ProcessKeyboard(Movement direction, float dt);
  void ProcessMouseMovement(float x_offset, float y_offset,
                            bool constrain_pitch = true);
  void ProcessMouseScroll(float y_offset);

  glm::vec3 GetPosition() const { return current_state_.position; }
  glm::vec3 GetFront() const { return current_state_.front; }

 private:
  void UpdateCameraVectors();

  // default to (0, 1, 0) unless specified otherwise
  glm::vec3 world_up_ = glm::vec3(0.0f, 1.0f, 0.0f);

  State initial_state_;
  State current_state_;

  float movement_speed_ = default_movement_speed;
  float mouse_sensitivity_ = default_mouse_sensitivity;
  float fov_ = default_fov;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CAMERA_HPP