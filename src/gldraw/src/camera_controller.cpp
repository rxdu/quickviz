/*
 * @file camera_controller.cpp
 * @date 11/3/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "gldraw/camera_controller.hpp"

namespace quickviz {

// Concrete strategy implementations
class OrbitModeStrategy : public CameraModeStrategy {
public:
  void ProcessKeyboard(CameraController& controller, Camera::Movement direction, float delta_time) override {
    // Orbit mode doesn't process keyboard input for movement
  }
  
  void ProcessMouseScroll(CameraController& controller, float y_offset) override {
    auto& orbit_distance = controller.orbit_distance_;
    const auto& config = controller.config_;
    
    orbit_distance -= y_offset * config.orbit_zoom_speed;
    if (orbit_distance < config.min_orbit_distance) {
      orbit_distance = config.min_orbit_distance;
    }
    controller.UpdateOrbitPosition();
  }
  
  void ProcessOrbitMovement(CameraController& controller, float x_offset, float y_offset) override {
    controller.camera_.ProcessMouseMovement(x_offset, y_offset);
    controller.UpdateOrbitPosition();
  }
  
  void ProcessPanMovement(CameraController& controller, float x_offset, float y_offset) override {
    const auto& config = controller.config_;
    float distance_factor = controller.orbit_distance_ / config.distance_scale_factor;
    if (distance_factor < 0.1f) distance_factor = 0.1f;
    
    glm::vec3 right = controller.camera_.GetRight();
    glm::vec3 up = controller.camera_.GetUp();
    glm::vec3 translation = (-x_offset * right + y_offset * up) * 
                           config.pan_sensitivity * distance_factor;
    
    controller.orbit_target_ += translation;
    controller.UpdateOrbitPosition();
  }
  
  void OnPositionChanged(CameraController& controller, const glm::vec3& position) override {
    controller.orbit_distance_ = glm::length(position - controller.orbit_target_);
    if (controller.orbit_distance_ < controller.config_.min_orbit_distance) {
      controller.orbit_distance_ = controller.config_.min_orbit_distance;
      controller.UpdateOrbitPosition();
    }
  }
  
  void OnOrientationChanged(CameraController& controller) override {
    controller.UpdateOrbitPosition();
  }
};

class TopDownModeStrategy : public CameraModeStrategy {
public:
  void OnModeActivated(CameraController& controller) override {
    controller.camera_.SetPitch(-90.0f);
    controller.camera_.SetYaw(0.0f);
    
    glm::vec3 position = controller.camera_.GetPosition();
    if (position.y < controller.config_.min_height) position.y = controller.config_.min_height;
    controller.camera_.SetPosition(position);
    
    controller.top_down_rotation_ = 0.0f;
  }
  
  void ProcessKeyboard(CameraController& controller, Camera::Movement direction, float delta_time) override {
    float velocity = controller.camera_.GetMovementSpeed() * delta_time;
    glm::vec3 position = controller.camera_.GetPosition();

    if (direction == Camera::Movement::kUp) position.y -= velocity;
    if (direction == Camera::Movement::kDown) position.y += velocity;
    if (direction == Camera::Movement::kForward) position.x -= velocity;
    if (direction == Camera::Movement::kBackward) position.x += velocity;
    if (direction == Camera::Movement::kLeft) position.z += velocity;
    if (direction == Camera::Movement::kRight) position.z -= velocity;

    controller.camera_.SetPosition(position);
  }
  
  void ProcessMouseScroll(CameraController& controller, float y_offset) override {
    const auto& config = controller.config_;
    glm::vec3 position = controller.camera_.GetPosition();
    position.y -= y_offset * config.topdown_zoom_speed;
    if (position.y < config.min_height) {
      position.y = config.min_height;
    }
    controller.camera_.SetPosition(position);
  }
  
  void ProcessOrbitMovement(CameraController& controller, float x_offset, float y_offset) override {
    const auto& config = controller.config_;
    if (std::abs(x_offset) > config.rotation_threshold) {
      controller.top_down_rotation_ += x_offset * config.orbit_rotation_sensitivity;
      
      while (controller.top_down_rotation_ >= 360.0f) controller.top_down_rotation_ -= 360.0f;
      while (controller.top_down_rotation_ < 0.0f) controller.top_down_rotation_ += 360.0f;
      
      controller.camera_.SetYaw(controller.top_down_rotation_);
    }
  }
  
  void ProcessPanMovement(CameraController& controller, float x_offset, float y_offset) override {
    const auto& config = controller.config_;
    glm::vec3 position = controller.camera_.GetPosition();
    float height_factor = position.y / config.height_scale_factor;
    if (height_factor < 0.1f) height_factor = 0.1f;
    
    float angle_rad = glm::radians(controller.top_down_rotation_);
    float rot_dx = -y_offset * std::cos(angle_rad) - x_offset * std::sin(angle_rad);
    float rot_dz = -y_offset * std::sin(angle_rad) + x_offset * std::cos(angle_rad);
    
    rot_dx *= config.pan_sensitivity * height_factor;
    rot_dz *= config.pan_sensitivity * height_factor;
    
    position.x += rot_dx;
    position.z += rot_dz;
    controller.camera_.SetPosition(position);
  }
  
  void OnOrientationChanged(CameraController& controller) override {
    controller.top_down_rotation_ = controller.camera_.GetYaw();
  }
};

class FreeCameraModeStrategy : public CameraModeStrategy {
public:
  void ProcessKeyboard(CameraController& controller, Camera::Movement direction, float delta_time) override {
    controller.camera_.ProcessKeyboard(direction, delta_time);
  }
  
  void ProcessMouseScroll(CameraController& controller, float y_offset) override {
    controller.camera_.ProcessMouseScroll(y_offset);
  }
  
  void ProcessOrbitMovement(CameraController& controller, float x_offset, float y_offset) override {
    controller.camera_.ProcessMouseMovement(x_offset, y_offset);
  }
  
  void ProcessPanMovement(CameraController& controller, float x_offset, float y_offset) override {
    const auto& config = controller.config_;
    glm::vec3 right = controller.camera_.GetRight();
    glm::vec3 up = controller.camera_.GetUp();
    glm::vec3 position = controller.camera_.GetPosition();
    glm::vec3 translation = (-x_offset * right + y_offset * up) * config.pan_sensitivity;
    controller.camera_.SetPosition(position + translation);
  }
};

CameraController::CameraController(Camera& camera, glm::vec3 position,
                                   float yaw, float pitch)
    : camera_(camera), config_(CameraControllerConfig::Default()) {
  camera_.SetPosition(position);
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);

  orbit_distance_ = config_.initial_orbit_distance;

  mode_strategy_ = CreateModeStrategy(mode_);
  UpdateOrbitPosition();
}

CameraController::CameraController(Camera& camera, const CameraControllerConfig& config,
                                   glm::vec3 position, float yaw, float pitch)
    : camera_(camera), config_(config) {
  camera_.SetPosition(position);
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);

  orbit_distance_ = config_.initial_orbit_distance;

  mode_strategy_ = CreateModeStrategy(mode_);
  UpdateOrbitPosition();
}

void CameraController::Reset() { camera_.Reset(); }

void CameraController::SetMode(CameraController::Mode mode) {
  if (mode == mode_) return;

  mode_ = mode;
  mode_strategy_ = CreateModeStrategy(mode_);
  mode_strategy_->OnModeActivated(*this);
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

// New consistent 3D API implementations
void CameraController::SetPosition3D(const glm::vec3& position) {
  if (!IsValidPosition(position)) {
    return;
  }
  
  camera_.SetPosition(position);
  
  // Notify strategy of position change
  if (mode_strategy_) {
    mode_strategy_->OnPositionChanged(*this, position);
  }
}

glm::vec3 CameraController::GetOrientation() const {
  return glm::vec3(camera_.GetYaw(), camera_.GetPitch(), 0.0f);  // Roll is always 0
}

void CameraController::SetOrientation(const glm::vec3& orientation) {
  // Validate angles (clamp to reasonable ranges)
  float yaw = std::fmod(orientation.x, 360.0f);
  float pitch = std::clamp(orientation.y, -89.0f, 89.0f);  // Prevent gimbal lock
  // Roll ignored for now (orientation.z)
  
  camera_.SetYaw(yaw);
  camera_.SetPitch(pitch);
  
  // Notify strategy of orientation change
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
  }
}

void CameraController::SetPitch(float pitch) {
  // Validate pitch to prevent gimbal lock
  float clamped_pitch = std::clamp(pitch, -89.0f, 89.0f);
  camera_.SetPitch(clamped_pitch);
  
  // Notify strategy of orientation change
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
  }
}

void CameraController::SetOrbitTarget(const glm::vec3& target) {
  if (!IsValidPosition(target)) {
    // Use current target if invalid position provided
    return;
  }
  orbit_target_ = target;
  // Notify strategy of target change (only relevant for orbit mode)
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
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
    // Let strategy handle the update
    if (mode_strategy_) {
      mode_strategy_->OnOrientationChanged(*this);
    }
  }
}

void CameraController::ProcessKeyboard(
    CameraController::CameraMovement direction, float delta_time) {
  if (mode_strategy_) {
    mode_strategy_->ProcessKeyboard(*this, direction, delta_time);
  }
}


void CameraController::ProcessMouseScroll(float y_offset) {
  if (!std::isfinite(y_offset) || std::abs(y_offset) > 100.0f) {
    return;  // Ignore invalid or excessive scroll values
  }
  if (mode_strategy_) {
    mode_strategy_->ProcessMouseScroll(*this, y_offset);
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
  if (mode_strategy_) {
    mode_strategy_->ProcessOrbitMovement(*this, x_offset, y_offset);
  }
}

void CameraController::ProcessPanMovement(float x_offset, float y_offset) {
  if (!IsValidMovement(x_offset, y_offset)) {
    return;
  }
  if (mode_strategy_) {
    mode_strategy_->ProcessPanMovement(*this, x_offset, y_offset);
  }
}

// Utility method implementations
void CameraController::LookAt(const glm::vec3& target, const glm::vec3& up) {
  if (!IsValidPosition(target)) {
    return;
  }
  
  camera_.LookAt(target);
  
  // Update orbit parameters if relevant and notify strategy
  orbit_target_ = target;
  orbit_distance_ = glm::length(camera_.GetPosition() - target);
  if (orbit_distance_ < config_.min_orbit_distance) {
    orbit_distance_ = config_.min_orbit_distance;
  }
  
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
  }
}

void CameraController::FitBounds(const glm::vec3& min_bounds, const glm::vec3& max_bounds, float padding) {
  if (!IsValidPosition(min_bounds) || !IsValidPosition(max_bounds) || 
      padding <= 0.0f || !std::isfinite(padding)) {
    return;
  }
  
  // Calculate bounding box center and size
  glm::vec3 center = (min_bounds + max_bounds) * 0.5f;
  glm::vec3 size = max_bounds - min_bounds;
  float max_dimension = std::max(size.x, std::max(size.y, size.z));
  
  // Set orbit parameters for all modes (non-orbit modes will ignore these)
  orbit_target_ = center;
  float required_distance = max_dimension * padding;
  orbit_distance_ = std::max(required_distance, config_.min_orbit_distance);
  
  if (mode_ == Mode::kOrbit) {
    // Position camera to look at center with nice default viewing angle
    SetOrientation(glm::vec3(-45.0f, -30.0f, 0.0f));
  } else {
    // For other modes, position camera at a good viewing distance
    glm::vec3 camera_position = center + glm::vec3(max_dimension, max_dimension * 0.5f, max_dimension);
    SetPosition3D(camera_position);
    LookAt(center);
  }
}

void CameraController::SetOrbitDistance(float distance) {
  if (!IsValidDistance(distance)) {
    return;
  }
  
  orbit_distance_ = std::max(distance, config_.min_orbit_distance);
  
  // Let strategy handle the update
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
  }
}

// Additional utility method implementations
void CameraController::MoveToPosition(const glm::vec3& target_position, float transition_time) {
  if (!IsValidPosition(target_position) || transition_time < 0.0f) {
    return;
  }
  
  if (transition_time == 0.0f) {
    SetPosition3D(target_position);
    return;
  }
  
  // Set up animation
  animation_start_state_ = SaveState();
  animation_target_state_ = animation_start_state_;
  animation_target_state_.position = target_position;
  
  animation_time_ = 0.0f;
  animation_duration_ = transition_time;
  is_animating_ = true;
}

void CameraController::OrbitToAngle(float target_yaw, float target_pitch, float transition_time) {
  if (transition_time < 0.0f) return;
  
  if (transition_time == 0.0f) {
    SetOrientation(glm::vec3(target_yaw, target_pitch, 0.0f));
    return;
  }
  
  // Set up animation
  animation_start_state_ = SaveState();
  animation_target_state_ = animation_start_state_;
  animation_target_state_.orientation = glm::vec3(target_yaw, target_pitch, 0.0f);
  
  animation_time_ = 0.0f;
  animation_duration_ = transition_time;
  is_animating_ = true;
}

void CameraController::ZoomToDistance(float target_distance, float transition_time) {
  if (!IsValidDistance(target_distance) || transition_time < 0.0f) {
    return;
  }
  
  if (transition_time == 0.0f) {
    SetOrbitDistance(target_distance);
    return;
  }
  
  // Set up animation
  animation_start_state_ = SaveState();
  animation_target_state_ = animation_start_state_;
  animation_target_state_.orbit_distance = target_distance;
  
  animation_time_ = 0.0f;
  animation_duration_ = transition_time;
  is_animating_ = true;
}

void CameraController::ResetToDefaults() {
  StopAnimation();
  
  // Reset to default configuration
  CameraState default_state;
  default_state.position = glm::vec3(0.0f, 0.0f, config_.initial_orbit_distance);
  default_state.orientation = glm::vec3(0.0f, 0.0f, 0.0f);
  default_state.orbit_target = glm::vec3(0.0f, 0.0f, 0.0f);
  default_state.orbit_distance = config_.initial_orbit_distance;
  default_state.mode = Mode::kOrbit;
  
  RestoreState(default_state);
}

void CameraController::UpdateAnimation(float delta_time) {
  if (!is_animating_) return;
  
  animation_time_ += delta_time;
  float t = std::clamp(animation_time_ / animation_duration_, 0.0f, 1.0f);
  
  // Use smooth step interpolation for more natural movement
  float smooth_t = t * t * (3.0f - 2.0f * t);
  
  // Interpolate position
  glm::vec3 current_pos = glm::mix(animation_start_state_.position, 
                                  animation_target_state_.position, smooth_t);
  
  // Interpolate orientation (handling angle wrapping)
  glm::vec3 start_orient = animation_start_state_.orientation;
  glm::vec3 target_orient = animation_target_state_.orientation;
  
  // Handle yaw angle wrapping
  float yaw_diff = target_orient.x - start_orient.x;
  if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
  if (yaw_diff < -180.0f) yaw_diff += 360.0f;
  
  glm::vec3 current_orient;
  current_orient.x = start_orient.x + yaw_diff * smooth_t;
  current_orient.y = glm::mix(start_orient.y, target_orient.y, smooth_t);
  current_orient.z = glm::mix(start_orient.z, target_orient.z, smooth_t);
  
  // Interpolate orbit distance
  float current_distance = glm::mix(animation_start_state_.orbit_distance,
                                   animation_target_state_.orbit_distance, smooth_t);
  
  // Apply interpolated values
  orbit_distance_ = current_distance;
  camera_.SetPosition(current_pos);
  camera_.SetYaw(current_orient.x);
  camera_.SetPitch(current_orient.y);
  
  // Update orbit position if needed
  if (mode_strategy_) {
    mode_strategy_->OnOrientationChanged(*this);
  }
  
  // Check if animation is complete
  if (t >= 1.0f) {
    is_animating_ = false;
  }
}

void CameraController::StopAnimation() {
  is_animating_ = false;
  animation_time_ = 0.0f;
}

glm::vec3 CameraController::ScreenToWorld(const glm::vec2& screen_coords, const glm::vec2& viewport_size, const glm::mat4& projection, float depth) const {
  // Convert screen coordinates to normalized device coordinates
  glm::vec2 ndc = (screen_coords / viewport_size) * 2.0f - 1.0f;
  ndc.y = -ndc.y;  // Flip Y coordinate
  
  // Get camera view matrix
  glm::mat4 view = camera_.GetViewMatrix();
  glm::mat4 inv_view_proj = glm::inverse(projection * view);
  
  // Transform from NDC to world space
  glm::vec4 world_pos = inv_view_proj * glm::vec4(ndc.x, ndc.y, depth, 1.0f);
  if (world_pos.w != 0.0f) {
    world_pos /= world_pos.w;
  }
  
  return glm::vec3(world_pos);
}

glm::vec2 CameraController::WorldToScreen(const glm::vec3& world_pos, const glm::vec2& viewport_size, const glm::mat4& projection) const {
  // Get camera view matrix
  glm::mat4 view = camera_.GetViewMatrix();
  glm::mat4 view_proj = projection * view;
  
  // Transform world position to clip space
  glm::vec4 clip_pos = view_proj * glm::vec4(world_pos, 1.0f);
  if (clip_pos.w != 0.0f) {
    clip_pos /= clip_pos.w;
  }
  
  // Convert to screen coordinates
  glm::vec2 screen_pos;
  screen_pos.x = (clip_pos.x * 0.5f + 0.5f) * viewport_size.x;
  screen_pos.y = (-clip_pos.y * 0.5f + 0.5f) * viewport_size.y;  // Flip Y
  
  return screen_pos;
}

bool CameraController::IsPointInFrustum(const glm::vec3& point, const glm::mat4& projection, const glm::mat4& view) const {
  glm::mat4 view_proj = projection * view;
  glm::vec4 clip_pos = view_proj * glm::vec4(point, 1.0f);
  
  if (clip_pos.w <= 0.0f) return false;  // Behind near plane
  
  // Check if point is within normalized device coordinates [-1, 1]
  glm::vec3 ndc = glm::vec3(clip_pos) / clip_pos.w;
  return (ndc.x >= -1.0f && ndc.x <= 1.0f &&
          ndc.y >= -1.0f && ndc.y <= 1.0f &&
          ndc.z >= -1.0f && ndc.z <= 1.0f);
}

CameraController::CameraState CameraController::SaveState() const {
  CameraState state;
  state.position = camera_.GetPosition();
  state.orientation = GetOrientation();
  state.orbit_target = orbit_target_;
  state.orbit_distance = orbit_distance_;
  state.mode = mode_;
  return state;
}

void CameraController::RestoreState(const CameraState& state, float transition_time) {
  if (transition_time == 0.0f) {
    // Immediate restore
    StopAnimation();
    SetMode(state.mode);
    orbit_target_ = state.orbit_target;
    orbit_distance_ = state.orbit_distance;
    SetPosition3D(state.position);
    SetOrientation(state.orientation);
  } else {
    // Animated restore
    animation_start_state_ = SaveState();
    animation_target_state_ = state;
    
    animation_time_ = 0.0f;
    animation_duration_ = transition_time;
    is_animating_ = true;
  }
}

// Strategy factory implementation
std::unique_ptr<CameraModeStrategy> CameraController::CreateModeStrategy(Mode mode) {
  switch (mode) {
    case Mode::kOrbit:
      return std::make_unique<OrbitModeStrategy>();
    case Mode::kTopDown:
      return std::make_unique<TopDownModeStrategy>();
    case Mode::kFirstPerson:
    case Mode::kFreeLook:
    default:
      return std::make_unique<FreeCameraModeStrategy>();
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