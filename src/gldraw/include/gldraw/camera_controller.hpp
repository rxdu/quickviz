/*
 * @file camera_controller.hpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CAMERA_CONTROLLER_HPP
#define QUICKVIZ_CAMERA_CONTROLLER_HPP

#include "gldraw/camera.hpp"
#include "imview/input/mouse.hpp"

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

  // Set the current mouse button state for camera control
  void SetActiveMouseButton(int button);
  int GetActiveMouseButton() const { return active_mouse_button_; }

  float GetHeight() const { return camera_.GetPosition().y; }
  void SetHeight(float height);

  glm::vec2 GetPosition() const;
  void SetPosition(const glm::vec2& position);

  float GetYaw() const { return camera_.GetYaw(); }
  void SetYaw(float yaw);
  
  // 3D translation support
  glm::vec3 GetOrbitTarget() const { return orbit_target_; }
  void SetOrbitTarget(const glm::vec3& target);

 private:
  static constexpr float initial_orbit_distance = 10.0f;
  static constexpr float default_orbit_zoom_speed = 2.0f;
  static constexpr float default_topdown_zoom_speed = 2.0f;

  void UpdateOrbitPosition();

  Camera& camera_;
  Mode mode_ = Mode::kOrbit;
  glm::vec3 orbit_target_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float orbit_distance_ = initial_orbit_distance;

  // For tracking mouse button states, -1 means no button pressed
  int active_mouse_button_ = MouseButton::kNone;

  // For tracking rotation in TopDown mode
  float top_down_rotation_ = 0.0f;

  // Zoom speed multipliers
  float orbit_zoom_speed_ = default_orbit_zoom_speed;
  float topdown_zoom_speed_ = default_topdown_zoom_speed;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CAMERA_CONTROLLER_HPP