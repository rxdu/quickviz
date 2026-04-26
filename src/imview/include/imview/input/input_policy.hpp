/*
 * @file input_policy.hpp
 * @date 9/1/25
 * @brief Modern input control policy system for panels
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_INPUT_POLICY_HPP
#define IMVIEW_INPUT_POLICY_HPP

#include "core/event/input_event.hpp"

namespace quickviz {

/**
 * @brief Input control flags for fine-grained input handling
 */
struct InputPolicy {
  // Input type control
  bool accept_mouse = true;
  bool accept_keyboard = true; 
  bool accept_gamepad = true;
  
  // Event type control
  bool accept_mouse_clicks = true;
  bool accept_mouse_movement = true;
  bool accept_mouse_wheel = true;
  bool accept_key_presses = true;
  bool accept_gamepad_buttons = true;
  bool accept_gamepad_axes = true;
  
  // Conditional control
  bool only_when_focused = false;    // Only process input when panel has focus
  bool only_when_hovered = false;    // Only process input when mouse is over panel
  bool consume_processed_events = true;  // Consume events that are handled
  
  // ImGui integration control
  bool bypass_imgui_capture = false; // Override ImGui's input capture for mouse clicks
  
  // Priority for event processing
  int priority = 50;  // Higher values processed first
  
  /**
   * @brief Check if an event should be processed based on policy
   * @param event The input event to check
   * @param panel_focused Whether the panel currently has focus
   * @param panel_hovered Whether the mouse is over the panel
   * @return true if event should be processed
   */
  bool ShouldProcessEvent(const InputEvent& event, 
                          bool panel_focused = true, 
                          bool panel_hovered = true) const {
    // Check focus/hover conditions
    if (only_when_focused && !panel_focused) return false;
    if (only_when_hovered && !panel_hovered) return false;
    
    // Check input type
    if (event.IsMouseEvent() && !accept_mouse) return false;
    if (event.IsKeyboardEvent() && !accept_keyboard) return false;
    if (event.IsGamepadEvent() && !accept_gamepad) return false;
    
    // Check specific event types
    switch (event.GetType()) {
      case InputEventType::kMousePress:
      case InputEventType::kMouseRelease:
        return accept_mouse_clicks;
        
      case InputEventType::kMouseMove:
      case InputEventType::kMouseDrag:
        return accept_mouse_movement;
        
      case InputEventType::kMouseWheel:
        return accept_mouse_wheel;
        
      case InputEventType::kKeyPress:
      case InputEventType::kKeyRelease:
        return accept_key_presses;
        
      case InputEventType::kGamepadButtonPress:
      case InputEventType::kGamepadButtonRelease:
        return accept_gamepad_buttons;
        
      case InputEventType::kGamepadAxisMove:
        return accept_gamepad_axes;
        
      default:
        return true;  // Allow unknown event types by default
    }
  }
  
  /**
   * @brief Create policy that accepts all input
   */
  static InputPolicy AllowAll() {
    return InputPolicy{};  // Default values allow everything
  }
  
  /**
   * @brief Create policy that blocks all input
   */
  static InputPolicy BlockAll() {
    InputPolicy policy;
    policy.accept_mouse = false;
    policy.accept_keyboard = false;
    policy.accept_gamepad = false;
    return policy;
  }
  
  /**
   * @brief Create policy for mouse-only interaction
   */
  static InputPolicy MouseOnly() {
    InputPolicy policy;
    policy.accept_keyboard = false;
    policy.accept_gamepad = false;
    return policy;
  }
  
  /**
   * @brief Create policy for keyboard-only interaction
   */
  static InputPolicy KeyboardOnly() {
    InputPolicy policy;
    policy.accept_mouse = false;
    policy.accept_gamepad = false;
    return policy;
  }
  
  /**
   * @brief Create policy for gamepad-only interaction
   */
  static InputPolicy GamepadOnly() {
    InputPolicy policy;
    policy.accept_mouse = false;
    policy.accept_keyboard = false;
    return policy;
  }
  
  /**
   * @brief Create policy that only processes input when panel is focused
   */
  static InputPolicy FocusedOnly() {
    InputPolicy policy;
    policy.only_when_focused = true;
    return policy;
  }
  
  /**
   * @brief Create policy that only processes input when mouse is over panel
   */
  static InputPolicy HoveredOnly() {
    InputPolicy policy;
    policy.only_when_hovered = true;
    return policy;
  }
  
  /**
   * @brief Create policy for 3D scene interaction (bypasses ImGui capture)
   */
  static InputPolicy SceneInteraction() {
    InputPolicy policy;
    policy.bypass_imgui_capture = true;
    policy.only_when_hovered = false; // Allow interaction even when not hovering
    policy.priority = 75; // Higher priority for 3D scenes
    return policy;
  }
};

/**
 * @brief Mixin class for components that want input control
 */
class InputControlled {
public:
  virtual ~InputControlled() = default;
  
  /**
   * @brief Set the input policy for this component
   */
  void SetInputPolicy(const InputPolicy& policy) {
    input_policy_ = policy;
  }
  
  /**
   * @brief Get the current input policy
   */
  const InputPolicy& GetInputPolicy() const {
    return input_policy_;
  }
  
  /**
   * @brief Enable/disable all input processing
   */
  void SetInputEnabled(bool enabled) {
    if (enabled) {
      input_policy_ = stored_policy_;
    } else {
      stored_policy_ = input_policy_;
      input_policy_ = InputPolicy::BlockAll();
    }
  }
  
  /**
   * @brief Check if input is currently enabled
   */
  bool IsInputEnabled() const {
    return input_policy_.accept_mouse || 
           input_policy_.accept_keyboard || 
           input_policy_.accept_gamepad;
  }
  
  /**
   * @brief Enable only specific input types
   */
  void SetMouseInputEnabled(bool enabled) { input_policy_.accept_mouse = enabled; }
  void SetKeyboardInputEnabled(bool enabled) { input_policy_.accept_keyboard = enabled; }
  void SetGamepadInputEnabled(bool enabled) { input_policy_.accept_gamepad = enabled; }
  
  /**
   * @brief Check specific input type status
   */
  bool IsMouseInputEnabled() const { return input_policy_.accept_mouse; }
  bool IsKeyboardInputEnabled() const { return input_policy_.accept_keyboard; }
  bool IsGamepadInputEnabled() const { return input_policy_.accept_gamepad; }
  
protected:
  /**
   * @brief Helper to check if an event should be processed
   */
  bool ShouldProcessInput(const InputEvent& event) const {
    bool focused = IsWindowFocused();
    bool hovered = IsWindowHovered(); 
    return input_policy_.ShouldProcessEvent(event, focused, hovered);
  }
  
private:
  InputPolicy input_policy_;
  InputPolicy stored_policy_;  // Backup for enable/disable
  
  /**
   * @brief Override these to provide focus/hover detection
   */
  virtual bool IsWindowFocused() const { return true; }
  virtual bool IsWindowHovered() const { return true; }
};

}  // namespace quickviz

#endif  // IMVIEW_INPUT_POLICY_HPP