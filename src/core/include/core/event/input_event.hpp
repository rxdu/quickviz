/*
 * @file input_event.hpp
 * @date 8/30/25
 * @brief Enhanced input event system for QuickViz
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INPUT_EVENT_HPP
#define QUICKVIZ_INPUT_EVENT_HPP

#include <string>
#include <memory>
#include <chrono>
#include <glm/glm.hpp>

#include "core/event/event.hpp"

namespace quickviz {

enum class InputEventType {
  // Mouse events
  kMousePress,
  kMouseRelease,
  kMouseMove,
  kMouseDrag,
  kMouseWheel,
  
  // Keyboard events
  kKeyPress,
  kKeyRelease,
  
  // Gamepad/Joystick events
  kGamepadButtonPress,
  kGamepadButtonRelease,
  kGamepadAxisMove,
  kGamepadConnected,
  kGamepadDisconnected
};

struct ModifierKeys {
  bool ctrl : 1;
  bool shift : 1;
  bool alt : 1;
  bool super : 1;

  ModifierKeys() : ctrl(false), shift(false), alt(false), super(false) {}

  bool operator==(const ModifierKeys& other) const {
    return ctrl == other.ctrl && shift == other.shift && alt == other.alt &&
           super == other.super;
  }

  bool operator!=(const ModifierKeys& other) const { return !(*this == other); }

  bool IsEmpty() const { return !ctrl && !shift && !alt && !super; }
};

// Use existing MouseButton enum from viewer/input/mouse.hpp
// No need to redefine it here

class InputEvent : public BaseEvent {
 public:
  InputEvent(InputEventType type, int button_or_key = -1)
      : type_(type),
        button_or_key_(button_or_key),
        consumed_(false),
        user_data_(nullptr) {
    timestamp_ = GetCurrentTime();
  }

  // BaseEvent interface
  EventSource GetSource() const override {
    switch (type_) {
      case InputEventType::kMousePress:
      case InputEventType::kMouseRelease:
      case InputEventType::kMouseMove:
      case InputEventType::kMouseDrag:
        return EventSource::kMouse;
      case InputEventType::kMouseWheel:
        return EventSource::kMouseButton;
      case InputEventType::kKeyPress:
      case InputEventType::kKeyRelease:
        return EventSource::kKeyboard;
      default:
        return EventSource::kNone;
    }
  }

  std::string GetName() const override {
    switch (type_) {
      case InputEventType::kMousePress:
        return "MousePress";
      case InputEventType::kMouseRelease:
        return "MouseRelease";
      case InputEventType::kMouseMove:
        return "MouseMove";
      case InputEventType::kMouseDrag:
        return "MouseDrag";
      case InputEventType::kMouseWheel:
        return "MouseWheel";
      case InputEventType::kKeyPress:
        return "KeyPress";
      case InputEventType::kKeyRelease:
        return "KeyRelease";
      case InputEventType::kGamepadButtonPress:
        return "GamepadButtonPress";
      case InputEventType::kGamepadButtonRelease:
        return "GamepadButtonRelease";
      case InputEventType::kGamepadAxisMove:
        return "GamepadAxisMove";
      case InputEventType::kGamepadConnected:
        return "GamepadConnected";
      case InputEventType::kGamepadDisconnected:
        return "GamepadDisconnected";
      default:
        return "Unknown";
    }
  }

  // Input event specific methods
  InputEventType GetType() const { return type_; }
  int GetButtonOrKey() const { return button_or_key_; }
  int GetMouseButton() const { return button_or_key_; }
  int GetKey() const { return button_or_key_; }

  // Position and delta
  void SetScreenPosition(const glm::vec2& pos) { screen_pos_ = pos; }
  const glm::vec2& GetScreenPosition() const { return screen_pos_; }

  void SetDelta(const glm::vec2& delta) { delta_ = delta; }
  const glm::vec2& GetDelta() const { return delta_; }

  // Modifiers
  void SetModifiers(const ModifierKeys& modifiers) { modifiers_ = modifiers; }
  const ModifierKeys& GetModifiers() const { return modifiers_; }

  // Event consumption
  bool IsConsumed() const { return consumed_; }
  void Consume() { consumed_ = true; }

  // Timestamp
  float GetTimestamp() const { return timestamp_; }

  // User data
  void SetUserData(void* data) { user_data_ = data; }
  void* GetUserData() const { return user_data_; }

  // Gamepad-specific methods
  void SetGamepadId(int id) { gamepad_id_ = id; }
  int GetGamepadId() const { return gamepad_id_; }
  
  void SetGamepadName(const std::string& name) { gamepad_name_ = name; }
  const std::string& GetGamepadName() const { return gamepad_name_; }
  
  void SetAxisValue(float value) { axis_value_ = value; }
  float GetAxisValue() const { return axis_value_; }
  
  void SetAxisIndex(int index) { axis_index_ = index; }
  int GetAxisIndex() const { return axis_index_; }

  // Helper methods
  bool IsMouseEvent() const {
    return type_ == InputEventType::kMousePress ||
           type_ == InputEventType::kMouseRelease ||
           type_ == InputEventType::kMouseMove ||
           type_ == InputEventType::kMouseDrag ||
           type_ == InputEventType::kMouseWheel;
  }

  bool IsKeyboardEvent() const {
    return type_ == InputEventType::kKeyPress ||
           type_ == InputEventType::kKeyRelease;
  }
  
  bool IsGamepadEvent() const {
    return type_ == InputEventType::kGamepadButtonPress ||
           type_ == InputEventType::kGamepadButtonRelease ||
           type_ == InputEventType::kGamepadAxisMove ||
           type_ == InputEventType::kGamepadConnected ||
           type_ == InputEventType::kGamepadDisconnected;
  }

  bool HasModifier() const { return !modifiers_.IsEmpty(); }

 private:
  InputEventType type_;
  int button_or_key_;
  ModifierKeys modifiers_;
  glm::vec2 screen_pos_;
  glm::vec2 delta_;
  float timestamp_;
  bool consumed_;
  void* user_data_;
  
  // Gamepad-specific data
  int gamepad_id_ = -1;
  std::string gamepad_name_;
  float axis_value_ = 0.0f;
  int axis_index_ = -1;

  static float GetCurrentTime() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<float>(now - start).count();
  }
};

}  // namespace quickviz

#endif  // QUICKVIZ_INPUT_EVENT_HPP