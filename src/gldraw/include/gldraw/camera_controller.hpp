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
#include <stdexcept>
#include <cmath>

namespace quickviz {

/**
 * @brief Configuration parameters for camera controller behavior
 */
struct CameraControllerConfig {
  // Movement sensitivity settings
  float pan_sensitivity = 0.01f;           // Pan movement sensitivity
  float orbit_rotation_sensitivity = 0.5f; // TopDown rotation sensitivity
  
  // Distance and scaling factors
  float distance_scale_factor = 10.0f;     // For orbit pan distance scaling
  float height_scale_factor = 10.0f;       // For topdown pan height scaling
  
  // Minimum limits
  float min_orbit_distance = 1.0f;         // Minimum orbit distance
  float min_height = 1.0f;                 // Minimum camera height
  
  // Zoom speeds
  float orbit_zoom_speed = 2.0f;           // Orbit mode zoom speed
  float topdown_zoom_speed = 2.0f;         // TopDown mode zoom speed
  
  // Rotation handling
  float rotation_threshold = 0.1f;         // Minimum movement for rotation
  
  // Default initial values
  float initial_orbit_distance = 10.0f;    // Starting orbit distance
  
  /**
   * @brief Create default configuration optimized for 3D modeling
   */
  static CameraControllerConfig Default() {
    return CameraControllerConfig{};
  }
  
  /**
   * @brief Create configuration optimized for large scale scenes
   */
  static CameraControllerConfig LargeScale() {
    CameraControllerConfig config;
    config.pan_sensitivity = 0.1f;
    config.distance_scale_factor = 100.0f;
    config.height_scale_factor = 100.0f;
    config.min_orbit_distance = 10.0f;
    config.min_height = 10.0f;
    config.initial_orbit_distance = 100.0f;
    return config;
  }
  
  /**
   * @brief Create configuration optimized for precise work
   */
  static CameraControllerConfig Precision() {
    CameraControllerConfig config;
    config.pan_sensitivity = 0.001f;
    config.orbit_rotation_sensitivity = 0.1f;
    config.distance_scale_factor = 1.0f;
    config.height_scale_factor = 1.0f;
    config.orbit_zoom_speed = 0.5f;
    config.topdown_zoom_speed = 0.5f;
    return config;
  }
  
  /**
   * @brief Validate configuration parameters and clamp to safe ranges
   */
  void Validate() {
    // Clamp sensitivity values to reasonable ranges
    pan_sensitivity = std::max(0.0001f, std::min(1.0f, pan_sensitivity));
    orbit_rotation_sensitivity = std::max(0.01f, std::min(10.0f, orbit_rotation_sensitivity));
    
    // Clamp scale factors to positive values
    distance_scale_factor = std::max(0.1f, distance_scale_factor);
    height_scale_factor = std::max(0.1f, height_scale_factor);
    
    // Clamp minimum limits
    min_orbit_distance = std::max(0.01f, min_orbit_distance);
    min_height = std::max(0.01f, min_height);
    
    // Clamp zoom speeds
    orbit_zoom_speed = std::max(0.1f, std::min(20.0f, orbit_zoom_speed));
    topdown_zoom_speed = std::max(0.1f, std::min(20.0f, topdown_zoom_speed));
    
    // Clamp thresholds
    rotation_threshold = std::max(0.001f, std::min(1.0f, rotation_threshold));
    
    // Clamp initial distance
    initial_orbit_distance = std::max(min_orbit_distance, initial_orbit_distance);
  }
  
  /**
   * @brief Check if configuration is valid
   */
  bool IsValid() const {
    return pan_sensitivity > 0 &&
           orbit_rotation_sensitivity > 0 &&
           distance_scale_factor > 0 &&
           height_scale_factor > 0 &&
           min_orbit_distance > 0 &&
           min_height > 0 &&
           orbit_zoom_speed > 0 &&
           topdown_zoom_speed > 0 &&
           rotation_threshold > 0 &&
           initial_orbit_distance >= min_orbit_distance &&
           std::isfinite(pan_sensitivity) &&
           std::isfinite(orbit_rotation_sensitivity);
  }
};

class CameraController {
 public:
  enum class Mode { kFirstPerson, kOrbit, kTopDown, kFreeLook };
  using CameraMovement = Camera::Movement;

 public:
  CameraController(Camera& camera, glm::vec3 position = {0, 0, 0},
                   float yaw = 0, float pitch = 0);
  CameraController(Camera& camera, const CameraControllerConfig& config,
                   glm::vec3 position = {0, 0, 0}, float yaw = 0, float pitch = 0);

  void Reset();
  void SetMode(Mode mode);
  void ProcessKeyboard(CameraMovement direction, float delta_time);
  void ProcessMouseScroll(float y_offset);
  
  // Movement methods (decoupled from input handling)
  void ProcessOrbitMovement(float x_offset, float y_offset);
  void ProcessPanMovement(float x_offset, float y_offset);

  float GetHeight() const { return camera_.GetPosition().y; }
  void SetHeight(float height);

  glm::vec2 GetPosition() const;
  void SetPosition(const glm::vec2& position);

  float GetYaw() const { return camera_.GetYaw(); }
  void SetYaw(float yaw);
  
  // 3D translation support
  glm::vec3 GetOrbitTarget() const { return orbit_target_; }
  void SetOrbitTarget(const glm::vec3& target);
  
  // Configuration access
  void SetConfig(const CameraControllerConfig& config);
  const CameraControllerConfig& GetConfig() const { return config_; }

 private:
  void UpdateOrbitPosition();
  
  // Input validation helpers
  static bool IsValidMovement(float x_offset, float y_offset);
  static bool IsValidPosition(const glm::vec3& position);
  static bool IsValidDistance(float distance);

  Camera& camera_;
  CameraControllerConfig config_;
  Mode mode_ = Mode::kOrbit;
  glm::vec3 orbit_target_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float orbit_distance_;

  // For tracking rotation in TopDown mode
  float top_down_rotation_ = 0.0f;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CAMERA_CONTROLLER_HPP