/*
 * @file camera_controller.hpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CAMERA_CONTROLLER_HPP
#define QUICKVIZ_CAMERA_CONTROLLER_HPP

#include "imview/component/opengl/camera.hpp"

namespace quickviz {
class CameraController {
 public:
  enum class Mode { kFirstPerson, kOrbit, kTopDown, kFreeLook };
  using CameraMovement = Camera::Movement;

 public:
  CameraController(Camera& camera, glm::vec3 position = {0, 0, 0},
                   float yaw = 0, float pitch = 0);

  void Reset();
  void SetMode(Mode mode);
  void ProcessKeyboard(CameraMovement direction, float delta_time);
  void ProcessMouseMovement(float x_offset, float y_offset);
  void ProcessMouseScroll(float y_offset);

 private:
  static constexpr float initial_orbit_distance = 10.0f;
  static constexpr float initial_top_down_height = 10.0f;

  void UpdateOrbitPosition();

  Camera& camera_;
  Mode mode_ = Mode::kOrbit;
  glm::vec3 orbit_target_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float orbit_distance_ = initial_orbit_distance;
  float top_down_height_ = initial_top_down_height;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CAMERA_CONTROLLER_HPP