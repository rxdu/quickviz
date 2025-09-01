/*
 * @file imgui_input_utils.cpp
 * @date 9/1/25
 * @brief Implementation of ImGui input utilities
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/imgui_input_utils.hpp"
#include <algorithm>

namespace quickviz {

InputEvent ImGuiInputUtils::CreateMouseEvent(InputEventType type, int button) {
  auto event = InputEvent(type, button);
  
  ImGuiIO& io = ImGui::GetIO();
  event.SetScreenPosition(glm::vec2(io.MousePos.x, io.MousePos.y));
  
  if (type == InputEventType::kMouseMove || type == InputEventType::kMouseDrag) {
    event.SetDelta(glm::vec2(io.MouseDelta.x, io.MouseDelta.y));
  }
  
  event.SetModifiers(GetCurrentModifiers());
  return event;
}

InputEvent ImGuiInputUtils::CreateKeyEvent(InputEventType type, int key) {
  auto event = InputEvent(type, key);
  event.SetModifiers(GetCurrentModifiers());
  return event;
}

ModifierKeys ImGuiInputUtils::GetCurrentModifiers() {
  ImGuiIO& io = ImGui::GetIO();
  ModifierKeys mods;
  mods.ctrl = io.KeyCtrl;
  mods.shift = io.KeyShift;
  mods.alt = io.KeyAlt;
  mods.super = io.KeySuper;
  return mods;
}

glm::vec2 ImGuiInputUtils::GetContentRelativeMousePos(const glm::vec2& content_offset) {
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 window_pos = ImGui::GetWindowPos();
  ImVec2 content_min = ImGui::GetWindowContentRegionMin();
  
  float x = io.MousePos.x - window_pos.x - content_min.x - content_offset.x;
  float y = io.MousePos.y - window_pos.y - content_min.y - content_offset.y;
  
  return glm::vec2(x, y);
}

bool ImGuiInputUtils::IsMouseOverContent() {
  return ImGui::IsWindowHovered(ImGuiHoveredFlags_None);
}

glm::vec2 ImGuiInputUtils::GetMouseDelta() {
  ImGuiIO& io = ImGui::GetIO();
  return glm::vec2(io.MouseDelta.x, io.MouseDelta.y);
}

void ImGuiInputUtils::PollMouseEvents(std::vector<InputEvent>& events) {
  if (ShouldCaptureMouseInput()) return;
  
  // Mouse clicks
  for (int button = 0; button < 3; ++button) {  // Left, Right, Middle
    if (ImGui::IsMouseClicked(button)) {
      events.push_back(CreateMouseEvent(InputEventType::kMousePress, button));
    }
    if (ImGui::IsMouseReleased(button)) {
      events.push_back(CreateMouseEvent(InputEventType::kMouseRelease, button));
    }
  }
  
  // Mouse movement
  ImGuiIO& io = ImGui::GetIO();
  if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f) {
    // Check if any mouse button is held (drag vs move)
    bool is_dragging = false;
    for (int button = 0; button < 3; ++button) {
      if (ImGui::IsMouseDown(button)) {
        is_dragging = true;
        break;
      }
    }
    
    InputEventType move_type = is_dragging ? 
      InputEventType::kMouseDrag : InputEventType::kMouseMove;
    events.push_back(CreateMouseEvent(move_type, -1));
  }
  
  // Mouse wheel
  if (io.MouseWheel != 0.0f || io.MouseWheelH != 0.0f) {
    auto event = CreateMouseEvent(InputEventType::kMouseWheel, -1);
    event.SetDelta(glm::vec2(io.MouseWheelH, io.MouseWheel));
    events.push_back(event);
  }
}

void ImGuiInputUtils::PollKeyboardEvents(std::vector<InputEvent>& events) {
  if (ShouldCaptureKeyboardInput()) return;
  
  ImGuiIO& io = ImGui::GetIO();
  
  // Check commonly used keys
  // Note: ImGui uses its own key codes, we might need conversion
  static const int common_keys[] = {
    ImGuiKey_Delete,
    ImGuiKey_Escape,
    ImGuiKey_Enter,
    ImGuiKey_Space,
    ImGuiKey_Tab,
    ImGuiKey_A, ImGuiKey_C, ImGuiKey_V, ImGuiKey_X, ImGuiKey_Z, ImGuiKey_Y
  };
  
  for (int imgui_key : common_keys) {
    if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(imgui_key))) {
      // Convert ImGui key to our key system (placeholder - needs proper mapping)
      events.push_back(CreateKeyEvent(InputEventType::kKeyPress, imgui_key));
    }
    if (ImGui::IsKeyReleased(static_cast<ImGuiKey>(imgui_key))) {
      events.push_back(CreateKeyEvent(InputEventType::kKeyRelease, imgui_key));
    }
  }
}

void ImGuiInputUtils::PollGamepadEvents(std::vector<InputEvent>& events) {
  if (ShouldCaptureGamepadInput()) return;
  
  // Check all standard gamepad keys using ImGui's unified system
  static const ImGuiKey gamepad_keys[] = {
    // Face buttons
    ImGuiKey_GamepadFaceDown,    // A/Cross
    ImGuiKey_GamepadFaceRight,   // B/Circle  
    ImGuiKey_GamepadFaceLeft,    // X/Square
    ImGuiKey_GamepadFaceUp,      // Y/Triangle
    
    // D-pad
    ImGuiKey_GamepadDpadUp,
    ImGuiKey_GamepadDpadDown,
    ImGuiKey_GamepadDpadLeft,
    ImGuiKey_GamepadDpadRight,
    
    // Shoulder buttons
    ImGuiKey_GamepadL1,          // Left bumper
    ImGuiKey_GamepadR1,          // Right bumper
    ImGuiKey_GamepadL2,          // Left trigger (analog, but treated as button)
    ImGuiKey_GamepadR2,          // Right trigger (analog, but treated as button)
    
    // Stick buttons
    ImGuiKey_GamepadL3,          // Left stick press
    ImGuiKey_GamepadR3,          // Right stick press
    
    // Menu buttons
    ImGuiKey_GamepadStart,       // Start/Menu/Options
    ImGuiKey_GamepadBack,        // Back/View/Share
    
    // Analog stick directions (treated as buttons with deadzone)
    ImGuiKey_GamepadLStickUp,
    ImGuiKey_GamepadLStickDown,
    ImGuiKey_GamepadLStickLeft,
    ImGuiKey_GamepadLStickRight,
    ImGuiKey_GamepadRStickUp,
    ImGuiKey_GamepadRStickDown,
    ImGuiKey_GamepadRStickLeft,
    ImGuiKey_GamepadRStickRight
  };
  
  // Poll button press/release events
  for (ImGuiKey key : gamepad_keys) {
    if (ImGui::IsKeyPressed(key)) {
      auto event = CreateKeyEvent(InputEventType::kGamepadButtonPress, static_cast<int>(key));
      event.SetGamepadId(0);  // GLFW_JOYSTICK_1 = 0
      events.push_back(event);
    }
    if (ImGui::IsKeyReleased(key)) {
      auto event = CreateKeyEvent(InputEventType::kGamepadButtonRelease, static_cast<int>(key));
      event.SetGamepadId(0);
      events.push_back(event);
    }
  }
  
  // Handle analog axis movements separately
  // Note: ImGui treats analog sticks as directional buttons, but we could
  // also query the raw analog values if needed for more precise control
  static const struct {
    ImGuiKey key;
    int axis_index;
  } analog_mappings[] = {
    {ImGuiKey_GamepadL2, 4},          // Left trigger
    {ImGuiKey_GamepadR2, 5},          // Right trigger
    {ImGuiKey_GamepadLStickLeft, 0},  // Left stick X-
    {ImGuiKey_GamepadLStickRight, 0}, // Left stick X+
    {ImGuiKey_GamepadLStickUp, 1},    // Left stick Y-
    {ImGuiKey_GamepadLStickDown, 1},  // Left stick Y+
    {ImGuiKey_GamepadRStickLeft, 2},  // Right stick X-
    {ImGuiKey_GamepadRStickRight, 2}, // Right stick X+
    {ImGuiKey_GamepadRStickUp, 3},    // Right stick Y-
    {ImGuiKey_GamepadRStickDown, 3}   // Right stick Y+
  };
  
  // Generate axis move events when analog values change significantly
  // This provides more granular control than just button press/release
  for (const auto& mapping : analog_mappings) {
    if (ImGui::IsKeyDown(mapping.key)) {
      // Get the analog value from ImGui's internal state
      // Note: This is a simplified approach - for full analog support,
      // we might need to access GLFW directly or extend ImGui's API
      float analog_value = 1.0f;  // ImGui normalizes to 0.0-1.0 for pressed state
      
      auto event = CreateKeyEvent(InputEventType::kGamepadAxisMove, static_cast<int>(mapping.key));
      event.SetGamepadId(0);
      event.SetAxisIndex(mapping.axis_index);
      event.SetAxisValue(analog_value);
      events.push_back(event);
    }
  }
}

void ImGuiInputUtils::PollAllEvents(std::vector<InputEvent>& events) {
  PollMouseEvents(events);
  PollKeyboardEvents(events);
  PollGamepadEvents(events);
}

bool ImGuiInputUtils::ShouldCaptureMouseInput() {
  ImGuiIO& io = ImGui::GetIO();
  return io.WantCaptureMouse;
}

bool ImGuiInputUtils::ShouldCaptureKeyboardInput() {
  ImGuiIO& io = ImGui::GetIO();
  return io.WantCaptureKeyboard;
}

bool ImGuiInputUtils::ShouldCaptureGamepadInput() {
  ImGuiIO& io = ImGui::GetIO();
  // Check if ImGui wants gamepad input (usually for UI navigation)
  // This would be true when ImGui is actively navigating UI with gamepad
  return (io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad) && 
         (io.NavActive || io.WantCaptureKeyboard);
}

// ScopedInputPoller implementation
ScopedInputPoller::ScopedInputPoller() {
  ImGuiInputUtils::PollAllEvents(events_);
}

std::vector<InputEvent> ScopedInputPoller::GetMouseEvents() const {
  std::vector<InputEvent> mouse_events;
  std::copy_if(events_.begin(), events_.end(), std::back_inserter(mouse_events),
    [](const InputEvent& event) {
      return event.IsMouseEvent();
    });
  return mouse_events;
}

std::vector<InputEvent> ScopedInputPoller::GetKeyboardEvents() const {
  std::vector<InputEvent> keyboard_events;
  std::copy_if(events_.begin(), events_.end(), std::back_inserter(keyboard_events),
    [](const InputEvent& event) {
      return event.IsKeyboardEvent();
    });
  return keyboard_events;
}

}  // namespace quickviz