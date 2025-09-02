/*
 * @file scene_input_handler.hpp
 * @date 9/1/25
 * @brief Bridge between imview input system and gldraw 3D interactions
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef GLDRAW_SCENE_INPUT_HANDLER_HPP
#define GLDRAW_SCENE_INPUT_HANDLER_HPP

#include <memory>
#include <string>

#include "imview/input/input_dispatcher.hpp"
#include "core/event/input_event.hpp"
#include "gldraw/camera.hpp"
#include "gldraw/camera_controller.hpp"
#include "gldraw/selection_manager.hpp"
#include "gldraw/camera_control_config.hpp"

namespace quickviz {

// Forward declarations
class SceneManager;

/**
 * @brief Input handler bridge for 3D scene interactions
 * 
 * Bridges imview's InputEventHandler system with gldraw's 3D-specific
 * functionality like camera control and object selection.
 */
class SceneInputHandler : public InputEventHandler {
 public:
  SceneInputHandler(SceneManager* scene_manager, int priority = 0);
  ~SceneInputHandler() = default;

  // InputEventHandler interface
  int GetPriority() const override { return priority_; }
  bool OnInputEvent(const InputEvent& event) override;
  std::string GetName() const override { return "SceneInputHandler"; }
  bool IsEnabled() const override { return enabled_; }

  // Configuration
  void SetEnabled(bool enabled) { enabled_ = enabled; }
  void SetCameraControlEnabled(bool enabled) { camera_control_enabled_ = enabled; }
  void SetSelectionEnabled(bool enabled) { selection_enabled_ = enabled; }
  
  // Camera control configuration
  void SetCameraControlConfig(const CameraControlConfig& config) { camera_config_ = config; }
  const CameraControlConfig& GetCameraControlConfig() const { return camera_config_; }

  // Viewport configuration for coordinate transformation
  void SetViewportSize(int width, int height) { 
    viewport_width_ = width; 
    viewport_height_ = height; 
  }
  
  // Get coordinate transformation utilities
  glm::vec3 ScreenToWorld(const glm::vec2& screen_pos, float depth = 0.0f) const;
  glm::vec2 WorldToScreen(const glm::vec3& world_pos) const;

 private:
  // Event handling methods
  bool HandleMouseEvent(const InputEvent& event);
  bool HandleKeyboardEvent(const InputEvent& event);
  
  // Specific interaction handling
  bool HandleCameraControl(const InputEvent& event);
  bool HandleObjectSelection(const InputEvent& event);
  
  // Mouse button mapping for camera control
  bool IsCameraControlButton(int button, const ModifierKeys& modifiers) const;
  bool IsSelectionButton(int button, const ModifierKeys& modifiers) const;

  // Member variables
  SceneManager* scene_manager_;
  int priority_;
  bool enabled_ = true;
  bool camera_control_enabled_ = true;
  bool selection_enabled_ = true;
  
  int viewport_width_ = 800;
  int viewport_height_ = 600;
  
  // Camera control configuration
  CameraControlConfig camera_config_ = CameraControlConfig::ModelingSoftware();
  
  // Track mouse state for camera control
  bool camera_active_ = false;
  int active_camera_button_ = -1;
  glm::vec2 last_mouse_pos_;
};

/**
 * @brief Factory for creating common 3D input handlers
 */
class SceneInputHandlerFactory {
 public:
  /**
   * @brief Create standard 3D scene input handler
   * @param scene_manager Scene manager to control
   * @param priority Handler priority (default: 50)
   */
  static std::shared_ptr<SceneInputHandler> CreateStandard(
    SceneManager* scene_manager, int priority = 50);

  /**
   * @brief Create camera-only input handler
   * @param scene_manager Scene manager to control
   * @param priority Handler priority (default: 40)
   */
  static std::shared_ptr<SceneInputHandler> CreateCameraOnly(
    SceneManager* scene_manager, int priority = 40);

  /**
   * @brief Create selection-only input handler
   * @param scene_manager Scene manager to control
   * @param priority Handler priority (default: 60)
   */
  static std::shared_ptr<SceneInputHandler> CreateSelectionOnly(
    SceneManager* scene_manager, int priority = 60);
  
  /**
   * @brief Create handler with FPS-style controls (left-click orbit)
   * @param scene_manager Scene manager to control
   * @param priority Handler priority (default: 50)
   */
  static std::shared_ptr<SceneInputHandler> CreateFPSStyle(
    SceneManager* scene_manager, int priority = 50);
  
  /**
   * @brief Create handler with web viewer style (left-click orbit, right-click pan)
   * @param scene_manager Scene manager to control
   * @param priority Handler priority (default: 50)
   */
  static std::shared_ptr<SceneInputHandler> CreateWebViewer(
    SceneManager* scene_manager, int priority = 50);
  
  /**
   * @brief Create handler with custom camera control configuration
   * @param scene_manager Scene manager to control
   * @param config Camera control configuration
   * @param priority Handler priority (default: 50)
   */
  static std::shared_ptr<SceneInputHandler> CreateCustom(
    SceneManager* scene_manager, const CameraControlConfig& config, int priority = 50);
};

}  // namespace quickviz

#endif  // GLDRAW_SCENE_INPUT_HANDLER_HPP