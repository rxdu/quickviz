/*
 * @file imgui_input_utils.cpp
 * @date 9/1/25
 * @brief Implementation of ImGui input utilities
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/imgui_input_utils.hpp"
#include "imview/input/gamepad_manager.hpp"
#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <iostream>

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
    // DEBUG: Show when ImGui is capturing keyboard
    static int capture_count = 0;
    if (++capture_count % 120 == 0) { // Log every 2 seconds at 60fps
      std::cout << "[DEBUG] ImGui capturing keyboard input. Press Ctrl+Shift+K to bypass for testing." << std::endl;
    }
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
  
  // ARCHITECTURAL DECISION: Use GamepadManager instead of ImGui's gamepad system
  // This creates a unified input system where all input types flow through InputEvent objects
  // while handling the different requirements of each input device appropriately.
  //
  // NOTE: Gamepad handling differs from mouse/keyboard for several reasons:
  // 1. Multiple Controller Support: GamepadManager can handle multiple gamepads 
  //    simultaneously, while ImGui assumes single gamepad for UI navigation
  // 2. Precision Analog Input: Direct GLFW access provides raw float values 
  //    for smooth camera/3D interaction, vs ImGui's button-like deadzone behavior
  // 3. Application vs UI Focus: GamepadManager is designed for application input
  //    handling (robotics, 3D navigation), while ImGui gamepad is for UI navigation
  // 4. Hardware Abstraction: GamepadManager provides connection monitoring and
  //    device enumeration that ImGui doesn't expose
  
  auto& gamepad_manager = GamepadManager::GetInstance();
  auto connected_gamepads = gamepad_manager.GetConnectedGamepads();
  
  // Track previous state for change detection (stored as static for simplicity)
  // In production code, this should be instance member or manager responsibility
  // NOTE: Using static map means state persists across multiple viewer instances
  // CRITICAL: Must be declared OUTSIDE the loop to persist across frames!
  // Bug fix: Previously declared inside loop, causing state to reset every iteration
  // which led to stuck buttons and missed release events
  static std::unordered_map<int, GamepadManager::GamepadState> previous_states;
  
  // Poll all connected gamepads (unlike mouse/keyboard which are singular)
  for (const auto& gamepad_info : connected_gamepads) {
    int gamepad_id = gamepad_info.id;
    auto current_state = gamepad_manager.GetGamepadState(gamepad_id);
    
    auto& previous_state = previous_states[gamepad_id];
    
    // Button events: Compare current vs previous button states
    // IMPORTANT: Handle button count changes properly to avoid stuck buttons
    // - First poll: previous_state is empty, need to handle all current buttons
    // - Reconnection: button count may change, need to release any orphaned buttons
    // - This fixes the issue where buttons would get stuck in pressed state
    size_t max_buttons = std::max(current_state.buttons.size(), previous_state.buttons.size());
    for (size_t i = 0; i < max_buttons; ++i) {
      bool was_pressed = (i < previous_state.buttons.size()) && (previous_state.buttons[i] != 0);
      bool is_pressed = (i < current_state.buttons.size()) && (current_state.buttons[i] != 0);
      
      if (is_pressed && !was_pressed) {
        auto event = CreateKeyEvent(InputEventType::kGamepadButtonPress, static_cast<int>(i));
        event.SetGamepadId(gamepad_id);
        events.push_back(event);
      } else if (!is_pressed && was_pressed) {
        auto event = CreateKeyEvent(InputEventType::kGamepadButtonRelease, static_cast<int>(i));
        event.SetGamepadId(gamepad_id);
        events.push_back(event);
      }
    }
    
    // Axis events: Generate continuous movement events for changed axes
    // Unlike discrete button presses, axes provide continuous float values
    const float axis_threshold = 0.01f;  // Minimum change to generate event
    size_t max_axes = std::max(current_state.axes.size(), previous_state.axes.size());
    for (size_t i = 0; i < max_axes; ++i) {
      float current_value = (i < current_state.axes.size()) ? current_state.axes[i] : 0.0f;
      float previous_value = (i < previous_state.axes.size()) ? previous_state.axes[i] : 0.0f;
      
      if (std::abs(current_value - previous_value) > axis_threshold) {
        auto event = CreateKeyEvent(InputEventType::kGamepadAxisMove, -1);
        event.SetGamepadId(gamepad_id);
        event.SetAxisIndex(static_cast<int>(i));
        event.SetAxisValue(current_value);
        events.push_back(event);
      }
    }
    
    // Hat/POV events: Digital directional pad
    size_t max_hats = std::max(current_state.hats.size(), previous_state.hats.size());
    for (size_t i = 0; i < max_hats; ++i) {
      unsigned char current_hat = (i < current_state.hats.size()) ? current_state.hats[i] : 0;
      unsigned char previous_hat = (i < previous_state.hats.size()) ? previous_state.hats[i] : 0;
      
      if (current_hat != previous_hat) {
        // Generate press events for newly pressed directions
        unsigned char pressed_directions = current_hat & (~previous_hat);
        unsigned char released_directions = previous_hat & (~current_hat);
        
        if (pressed_directions != 0) {
          auto event = CreateKeyEvent(InputEventType::kGamepadButtonPress, static_cast<int>(current_hat));
          event.SetGamepadId(gamepad_id);
          // Store hat index in a custom field if needed
          events.push_back(event);
        }
        
        if (released_directions != 0) {
          auto event = CreateKeyEvent(InputEventType::kGamepadButtonRelease, static_cast<int>(previous_hat));
          event.SetGamepadId(gamepad_id);
          events.push_back(event);
        }
      }
    }
    
    // Update previous state for next frame
    previous_state = current_state;
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