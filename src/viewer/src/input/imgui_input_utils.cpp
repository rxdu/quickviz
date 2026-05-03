/*
 * @file imgui_input_utils.cpp
 * @date 9/1/25
 * @brief Implementation of ImGui input utilities
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "viewer/input/imgui_input_utils.hpp"
#include "viewer/input/gamepad_manager.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace quickviz {

InputEvent ImGuiInputUtils::CreateMouseEvent(InputEventType type, int button) {
  auto event = InputEvent(type, button);
  
  ImGuiIO& io = ImGui::GetIO();
  
  // Set global screen position (relative to GLFW window)
  event.SetScreenPosition(glm::vec2(io.MousePos.x, io.MousePos.y));
  
  // Note: Local coordinates are best calculated by the specific panel/window
  // that processes the event, as it has the correct context.
  // Panels should call SetLocalPosition() when they receive the event.
  
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
  ImGuiIO& io = ImGui::GetIO();
  
  // Check if ImGui wants to capture mouse input
  bool imgui_wants_mouse = ShouldCaptureMouseInput();
  
  // Always use manual detection for now, but we could make this configurable
  // The manual detection works reliably for both regular UI and 3D scenes
  // Track mouse button state changes manually since ImGui::IsMouseClicked() 
  // doesn't work when hovering over panels where ImGui wants capture
  static bool last_mouse_state[3] = {false, false, false};
  
  for (int button = 0; button < 3; ++button) {  // Left, Right, Middle
    bool current_state = ImGui::IsMouseDown(button);
    bool last_state = last_mouse_state[button];
    
    // Detect press (transition from up to down)
    if (current_state && !last_state) {
      events.push_back(CreateMouseEvent(InputEventType::kMousePress, button));
    }
    
    // Detect release (transition from down to up)
    if (!current_state && last_state) {
      events.push_back(CreateMouseEvent(InputEventType::kMouseRelease, button));
    }
    
    last_mouse_state[button] = current_state;
  }
  
  // Mouse wheel - Always allow wheel events 
  if (io.MouseWheel != 0.0f || io.MouseWheelH != 0.0f) {
    auto event = CreateMouseEvent(InputEventType::kMouseWheel, -1);
    event.SetDelta(glm::vec2(io.MouseWheelH, io.MouseWheel));
    events.push_back(event);
  }
  
  // Mouse movement - Always allow movement events for camera control
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
}

void ImGuiInputUtils::PollKeyboardEvents(std::vector<InputEvent>& events) {
  // NOTE: ImGui may capture keyboard input when UI elements have focus
  // This prevents application-level keyboard handling, which is usually desired
  // However, for input testing we may want to bypass this check
  
  // Check if we should respect ImGui's keyboard capture
  static bool bypass_imgui_capture = false;
  static bool capture_override_logged = false;
  
  // Allow bypassing ImGui capture for testing by checking for special key combo
  // Ctrl+Shift+K toggles bypass mode
  ImGuiIO& io = ImGui::GetIO();
  if (io.KeyCtrl && io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_K)) {
    bypass_imgui_capture = !bypass_imgui_capture;
    if (!capture_override_logged) {
      std::cout << "[INFO] Keyboard capture bypass " 
                << (bypass_imgui_capture ? "ENABLED" : "DISABLED") 
                << " (Ctrl+Shift+K to toggle)" << std::endl;
      capture_override_logged = true;
    }
  }
  
  if (ShouldCaptureKeyboardInput() && !bypass_imgui_capture) {
    // ImGui is capturing keyboard input - no need to log this constantly
    return;
  }
  
  // Check all ImGui keys systematically
  // ImGui provides a comprehensive key system, we should poll all of them
  static const ImGuiKey all_keys[] = {
    // Function keys
    ImGuiKey_F1, ImGuiKey_F2, ImGuiKey_F3, ImGuiKey_F4, ImGuiKey_F5, ImGuiKey_F6,
    ImGuiKey_F7, ImGuiKey_F8, ImGuiKey_F9, ImGuiKey_F10, ImGuiKey_F11, ImGuiKey_F12,
    
    // Number keys
    ImGuiKey_0, ImGuiKey_1, ImGuiKey_2, ImGuiKey_3, ImGuiKey_4,
    ImGuiKey_5, ImGuiKey_6, ImGuiKey_7, ImGuiKey_8, ImGuiKey_9,
    
    // Letter keys
    ImGuiKey_A, ImGuiKey_B, ImGuiKey_C, ImGuiKey_D, ImGuiKey_E, ImGuiKey_F,
    ImGuiKey_G, ImGuiKey_H, ImGuiKey_I, ImGuiKey_J, ImGuiKey_K, ImGuiKey_L,
    ImGuiKey_M, ImGuiKey_N, ImGuiKey_O, ImGuiKey_P, ImGuiKey_Q, ImGuiKey_R,
    ImGuiKey_S, ImGuiKey_T, ImGuiKey_U, ImGuiKey_V, ImGuiKey_W, ImGuiKey_X,
    ImGuiKey_Y, ImGuiKey_Z,
    
    // Special keys
    ImGuiKey_Space, ImGuiKey_Enter, ImGuiKey_Escape, ImGuiKey_Tab, ImGuiKey_Backspace,
    ImGuiKey_Delete, ImGuiKey_Insert, ImGuiKey_Home, ImGuiKey_End, 
    ImGuiKey_PageUp, ImGuiKey_PageDown,
    
    // Arrow keys
    ImGuiKey_LeftArrow, ImGuiKey_RightArrow, ImGuiKey_UpArrow, ImGuiKey_DownArrow,
    
    // Modifier keys
    ImGuiKey_LeftCtrl, ImGuiKey_RightCtrl, ImGuiKey_LeftShift, ImGuiKey_RightShift,
    ImGuiKey_LeftAlt, ImGuiKey_RightAlt, ImGuiKey_LeftSuper, ImGuiKey_RightSuper,
    
    // Punctuation and symbols (commonly used)
    ImGuiKey_Minus, ImGuiKey_Equal, ImGuiKey_LeftBracket, ImGuiKey_RightBracket,
    ImGuiKey_Backslash, ImGuiKey_Semicolon, ImGuiKey_Apostrophe, ImGuiKey_Comma,
    ImGuiKey_Period, ImGuiKey_Slash, ImGuiKey_GraveAccent,
    
    // Keypad
    ImGuiKey_Keypad0, ImGuiKey_Keypad1, ImGuiKey_Keypad2, ImGuiKey_Keypad3, ImGuiKey_Keypad4,
    ImGuiKey_Keypad5, ImGuiKey_Keypad6, ImGuiKey_Keypad7, ImGuiKey_Keypad8, ImGuiKey_Keypad9,
    ImGuiKey_KeypadDecimal, ImGuiKey_KeypadDivide, ImGuiKey_KeypadMultiply,
    ImGuiKey_KeypadSubtract, ImGuiKey_KeypadAdd, ImGuiKey_KeypadEnter, ImGuiKey_KeypadEqual
  };
  
  for (ImGuiKey imgui_key : all_keys) {
    if (ImGui::IsKeyPressed(imgui_key)) {
      auto event = CreateKeyEvent(InputEventType::kKeyPress, static_cast<int>(imgui_key));
      events.push_back(event);
    }
    if (ImGui::IsKeyReleased(imgui_key)) {
      auto event = CreateKeyEvent(InputEventType::kKeyRelease, static_cast<int>(imgui_key));
      events.push_back(event);
    }
  }
}

void ImGuiInputUtils::PollGamepadEvents(std::vector<InputEvent>& events) {
  if (ShouldCaptureGamepadInput()) return;
  
  // Use GamepadManager's centralized event polling
  // GamepadManager now handles all state tracking and event generation internally
  auto& gamepad_manager = GamepadManager::GetInstance();
  auto gamepad_events = gamepad_manager.PollEvents();
  
  // Add all gamepad events to the output vector
  events.insert(events.end(), gamepad_events.begin(), gamepad_events.end());
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