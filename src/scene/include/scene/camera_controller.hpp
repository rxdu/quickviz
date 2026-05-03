/*
 * @file camera_controller.hpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CAMERA_CONTROLLER_HPP
#define QUICKVIZ_CAMERA_CONTROLLER_HPP

#include "scene/camera.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <memory>

namespace quickviz {

// Forward declarations
class CameraController;

/**
 * @brief Base class for camera mode-specific behavior
 */
class CameraModeStrategy {
public:
  virtual ~CameraModeStrategy() = default;
  
  // Mode-specific operations
  virtual void OnModeActivated(CameraController& controller) {}
  virtual void ProcessKeyboard(CameraController& controller, Camera::Movement direction, float delta_time) = 0;
  virtual void ProcessMouseScroll(CameraController& controller, float y_offset) = 0;
  virtual void ProcessOrbitMovement(CameraController& controller, float x_offset, float y_offset) = 0;
  virtual void ProcessPanMovement(CameraController& controller, float x_offset, float y_offset) = 0;
  
  // Position/orientation handling
  virtual void OnPositionChanged(CameraController& controller, const glm::vec3& position) {}
  virtual void OnOrientationChanged(CameraController& controller) {}
};

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
  
  // Friend classes for strategy pattern access
  friend class OrbitModeStrategy;
  friend class TopDownModeStrategy;
  friend class FreeCameraModeStrategy;

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

  // 3D Position and Orientation API (Consistent)
  glm::vec3 GetPosition3D() const { return camera_.GetPosition(); }
  void SetPosition3D(const glm::vec3& position);
  
  glm::vec3 GetOrientation() const;  // Returns (yaw, pitch, roll) in degrees
  void SetOrientation(const glm::vec3& orientation);  // (yaw, pitch, roll) in degrees
  
  float GetYaw() const { return camera_.GetYaw(); }
  void SetYaw(float yaw);
  float GetPitch() const { return camera_.GetPitch(); }
  void SetPitch(float pitch);
  
  // Legacy 2D API (for backward compatibility with TopDown mode)
  [[deprecated("Use GetPosition3D() for consistent 3D API")]]
  float GetHeight() const { return camera_.GetPosition().y; }
  [[deprecated("Use SetPosition3D() for consistent 3D API")]]  
  void SetHeight(float height);
  
  [[deprecated("Use GetPosition3D() for consistent 3D API")]]
  glm::vec2 GetPosition() const;
  [[deprecated("Use SetPosition3D() for consistent 3D API")]]
  void SetPosition(const glm::vec2& position);
  
  // 3D translation support
  glm::vec3 GetOrbitTarget() const { return orbit_target_; }
  void SetOrbitTarget(const glm::vec3& target);
  
  // Configuration access
  void SetConfig(const CameraControllerConfig& config);
  const CameraControllerConfig& GetConfig() const { return config_; }
  
  // Utility methods for common operations
  void LookAt(const glm::vec3& target, const glm::vec3& up = glm::vec3(0, 1, 0));
  void FitBounds(const glm::vec3& min_bounds, const glm::vec3& max_bounds, float padding = 1.2f);
  glm::vec3 GetViewDirection() const { return camera_.GetFront(); }
  glm::vec3 GetRightVector() const { return camera_.GetRight(); }  
  glm::vec3 GetUpVector() const { return camera_.GetUp(); }
  
  // Distance and zoom utilities
  float GetOrbitDistance() const { return orbit_distance_; }
  void SetOrbitDistance(float distance);
  
  // Additional utility methods
  void MoveToPosition(const glm::vec3& target_position, float transition_time = 1.0f);
  void OrbitToAngle(float target_yaw, float target_pitch, float transition_time = 1.0f);
  void ZoomToDistance(float target_distance, float transition_time = 1.0f);
  void ResetToDefaults();
  
  // Animation and interpolation support
  bool IsAnimating() const { return is_animating_; }
  void UpdateAnimation(float delta_time);
  void StopAnimation();
  
  // Viewport and frustum utilities
  glm::vec3 ScreenToWorld(const glm::vec2& screen_coords, const glm::vec2& viewport_size, const glm::mat4& projection, float depth = 1.0f) const;
  glm::vec2 WorldToScreen(const glm::vec3& world_pos, const glm::vec2& viewport_size, const glm::mat4& projection) const;
  bool IsPointInFrustum(const glm::vec3& point, const glm::mat4& projection, const glm::mat4& view) const;
  
  // State save/restore for bookmarks
  struct CameraState {
    glm::vec3 position;
    glm::vec3 orientation;  // yaw, pitch, roll
    glm::vec3 orbit_target;
    float orbit_distance;
    Mode mode;
  };
  CameraState SaveState() const;
  void RestoreState(const CameraState& state, float transition_time = 0.0f);

 private:
  void UpdateOrbitPosition();
  
  // Mode strategy management
  std::unique_ptr<CameraModeStrategy> CreateModeStrategy(Mode mode);
  
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
  
  // Strategy for mode-specific behavior
  std::unique_ptr<CameraModeStrategy> mode_strategy_;
  
  // Animation state
  bool is_animating_ = false;
  float animation_time_ = 0.0f;
  float animation_duration_ = 1.0f;
  CameraState animation_start_state_;
  CameraState animation_target_state_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_CAMERA_CONTROLLER_HPP