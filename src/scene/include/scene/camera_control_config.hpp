/*
 * @file camera_control_config.hpp
 * @date 9/2/25
 * @brief Configurable camera control system for 3D scene interaction
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef SCENE_CAMERA_CONTROL_CONFIG_HPP
#define SCENE_CAMERA_CONTROL_CONFIG_HPP

#include "viewer/input/input_types.hpp"
#include "core/event/input_event.hpp"

namespace quickviz {

/**
 * @brief Configuration for 3D camera controls
 * 
 * This allows applications to customize which mouse buttons and modifiers
 * are used for different camera operations (orbit, pan, zoom, etc.)
 */
struct CameraControlConfig {
  // Mouse button assignments
  MouseButton orbit_button = MouseButton::kRight;      // Camera orbit/rotation
  MouseButton pan_button = MouseButton::kMiddle;       // Camera panning
  MouseButton zoom_button = MouseButton::kNone;        // Alternative zoom (usually wheel)
  MouseButton selection_button = MouseButton::kLeft;   // Object selection
  
  // Modifier key requirements
  struct ModifierRequirement {
    bool ctrl = false;
    bool shift = false; 
    bool alt = false;
    bool super = false;
    
    // Check if current modifiers match requirement
    bool Matches(const ModifierKeys& modifiers) const {
      return modifiers.ctrl == ctrl &&
             modifiers.shift == shift &&
             modifiers.alt == alt &&
             modifiers.super == super;
    }
    
    // Check if modifiers are empty (no modifiers required)
    bool IsEmpty() const {
      return !ctrl && !shift && !alt && !super;
    }
  };
  
  ModifierRequirement orbit_modifiers;     // Modifiers for orbit (default: none)
  ModifierRequirement pan_modifiers;       // Modifiers for pan (default: none)
  ModifierRequirement zoom_modifiers;      // Modifiers for zoom (default: none)
  ModifierRequirement selection_modifiers; // Modifiers for selection (default: none)
  
  // Control behavior settings
  bool enable_orbit = true;
  bool enable_pan = true;
  bool enable_wheel_zoom = true;
  bool enable_selection = true;
  
  // Sensitivity settings
  float orbit_sensitivity = 1.0f;
  float pan_sensitivity = 1.0f;
  float zoom_sensitivity = 1.0f;
  
  /**
   * @brief Check if a button/modifier combination is for camera orbit
   */
  bool IsOrbitControl(MouseButton button, const ModifierKeys& modifiers) const {
    return enable_orbit && 
           button == orbit_button && 
           orbit_modifiers.Matches(modifiers);
  }
  
  /**
   * @brief Check if a button/modifier combination is for camera pan
   */
  bool IsPanControl(MouseButton button, const ModifierKeys& modifiers) const {
    return enable_pan && 
           button == pan_button && 
           pan_modifiers.Matches(modifiers);
  }
  
  /**
   * @brief Check if a button/modifier combination is for camera zoom
   */
  bool IsZoomControl(MouseButton button, const ModifierKeys& modifiers) const {
    return button == zoom_button && 
           zoom_modifiers.Matches(modifiers);
  }
  
  /**
   * @brief Check if a button/modifier combination is for object selection
   */
  bool IsSelectionControl(MouseButton button, const ModifierKeys& modifiers) const {
    return enable_selection && 
           button == selection_button && 
           selection_modifiers.Matches(modifiers);
  }
  
  /**
   * @brief Check if button/modifier is for any camera control (not selection)
   */
  bool IsCameraControl(MouseButton button, const ModifierKeys& modifiers) const {
    return IsOrbitControl(button, modifiers) ||
           IsPanControl(button, modifiers) ||
           IsZoomControl(button, modifiers);
  }
  
  // === PRESET CONFIGURATIONS ===
  
  /**
   * @brief 3D modeling software style (Blender, Maya)
   * - Left: Select objects
   * - Right: Orbit camera
   * - Middle: Pan camera
   * - Wheel: Zoom
   */
  static CameraControlConfig ModelingSoftware() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kRight;
    config.pan_button = MouseButton::kMiddle;
    config.selection_button = MouseButton::kLeft;
    return config;
  }
  
  /**
   * @brief FPS game style controls
   * - Left: Orbit camera (like looking around)
   * - Right: Select/interact with objects
   * - Middle: Pan camera
   * - Wheel: Zoom
   */
  static CameraControlConfig FPSStyle() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kLeft;
    config.selection_button = MouseButton::kRight;
    config.pan_button = MouseButton::kMiddle;
    return config;
  }
  
  /**
   * @brief CAD software style (SolidWorks, AutoCAD)
   * - Right: Orbit camera
   * - Middle: Pan camera  
   * - Left: Select objects
   * - Ctrl+Left: Multi-select
   * - Wheel: Zoom
   */
  static CameraControlConfig CADStyle() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kRight;
    config.pan_button = MouseButton::kMiddle;
    config.selection_button = MouseButton::kLeft;
    return config;
  }
  
  /**
   * @brief Web viewer style (simpler controls)
   * - Left: Orbit camera
   * - Right: Pan camera
   * - Wheel: Zoom
   * - Selection disabled (or Ctrl+Left)
   */
  static CameraControlConfig WebViewer() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kLeft;
    config.pan_button = MouseButton::kRight;
    config.enable_selection = false; // Or use Ctrl+Left
    return config;
  }
  
  /**
   * @brief Scientific visualization style
   * - Left: Select points/data
   * - Right: Orbit camera
   * - Middle: Pan camera
   * - Alt+Left: Box select
   * - Wheel: Zoom
   */
  static CameraControlConfig Scientific() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kRight;
    config.pan_button = MouseButton::kMiddle;
    config.selection_button = MouseButton::kLeft;
    return config;
  }
  
  /**
   * @brief Single-button mode (for tablets/touchpads)
   * - Left: Context-dependent (orbit by default)
   * - Shift+Left: Pan
   * - Ctrl+Left: Select
   * - Wheel: Zoom
   */
  static CameraControlConfig SingleButton() {
    CameraControlConfig config;
    config.orbit_button = MouseButton::kLeft;
    config.pan_button = MouseButton::kLeft;
    config.selection_button = MouseButton::kLeft;
    
    // Use modifiers to differentiate
    config.pan_modifiers.shift = true;
    config.selection_modifiers.ctrl = true;
    
    return config;
  }
};

} // namespace quickviz

#endif // SCENE_CAMERA_CONTROL_CONFIG_HPP