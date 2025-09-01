/*
 * @file gamepad_manager.cpp
 * @date 9/1/25
 * @brief Implementation of modern gamepad management system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/gamepad_manager.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace quickviz {

std::vector<GamepadInfo> GamepadManager::GetConnectedGamepads() {
  UpdateGamepadList();
  
  std::vector<GamepadInfo> connected;
  for (const auto& [id, info] : gamepads_) {
    if (info.connected) {
      connected.push_back(info);
    }
  }
  return connected;
}

GamepadInfo GamepadManager::GetGamepadInfo(int gamepad_id) {
  if (gamepad_id < GLFW_JOYSTICK_1 || gamepad_id > GLFW_JOYSTICK_LAST) {
    return GamepadInfo{}; // Invalid ID
  }

  UpdateGamepadList();
  
  auto it = gamepads_.find(gamepad_id);
  return (it != gamepads_.end()) ? it->second : GamepadInfo{};
}

bool GamepadManager::IsGamepadConnected(int gamepad_id) {
  // Return false if shutdown or GLFW has been terminated
  if (shutdown_ || glfwGetCurrentContext() == nullptr) {
    return false;
  }
  return glfwJoystickPresent(gamepad_id) == GLFW_TRUE;
}

std::vector<std::string> GamepadManager::GetGamepadNames() {
  auto gamepads = GetConnectedGamepads();
  std::vector<std::string> names;
  
  for (const auto& gamepad : gamepads) {
    std::string name = std::to_string(gamepad.id) + ": " + gamepad.name;
    names.push_back(name);
  }
  
  return names;
}

void GamepadManager::SetConnectionCallback(ConnectionCallback callback) {
  connection_callback_ = callback;
}

void GamepadManager::SetMonitoringEnabled(bool enabled) {
  if (enabled == monitoring_enabled_) return;
  
  monitoring_enabled_ = enabled;
  
  if (monitoring_enabled_) {
    InitializeGLFWCallback();
    UpdateGamepadList(); // Trigger initial callback for existing gamepads
    
    // Notify about currently connected gamepads
    if (connection_callback_) {
      for (const auto& [id, info] : gamepads_) {
        if (info.connected) {
          connection_callback_(info, true);
        }
      }
    }
  } else {
    glfwSetJoystickCallback(nullptr);
  }
}

GamepadManager::GamepadState GamepadManager::GetGamepadState(int gamepad_id) {
  GamepadState state;
  
  // Skip if shutdown or GLFW has been terminated
  if (shutdown_ || glfwGetCurrentContext() == nullptr) {
    return state; // Empty state
  }
  
  if (!IsGamepadConnected(gamepad_id)) {
    return state; // Empty state
  }

  int axis_count = 0;
  int button_count = 0;
  int hat_count = 0;

  const float* axes = glfwGetJoystickAxes(gamepad_id, &axis_count);
  const unsigned char* buttons = glfwGetJoystickButtons(gamepad_id, &button_count);
  const unsigned char* hats = glfwGetJoystickHats(gamepad_id, &hat_count);

  if (axes && axis_count > 0) {
    state.axes.assign(axes, axes + axis_count);
  }
  if (buttons && button_count > 0) {
    state.buttons.assign(buttons, buttons + button_count);
  }
  if (hats && hat_count > 0) {
    state.hats.assign(hats, hats + hat_count);
  }

  return state;
}

void GamepadManager::InitializeGLFWCallback() {
  glfwSetJoystickCallback(GLFWGamepadCallback);
}

void GamepadManager::UpdateGamepadList() {
  // Skip update if shutdown or GLFW has been terminated
  if (shutdown_ || glfwGetCurrentContext() == nullptr) {
    return;
  }
  
  for (int id = GLFW_JOYSTICK_1; id <= GLFW_JOYSTICK_LAST; ++id) {
    bool currently_connected = (glfwJoystickPresent(id) == GLFW_TRUE);
    
    auto it = gamepads_.find(id);
    bool was_connected = (it != gamepads_.end() && it->second.connected);
    
    if (currently_connected != was_connected) {
      // Connection status changed
      GamepadInfo info;
      info.id = id;
      info.connected = currently_connected;
      
      if (currently_connected) {
        // Get gamepad information
        const char* name = glfwGetJoystickName(id);
        info.name = name ? name : "Unknown Gamepad";
        
        // Get capability information
        int axis_count = 0, button_count = 0, hat_count = 0;
        glfwGetJoystickAxes(id, &axis_count);
        glfwGetJoystickButtons(id, &button_count);
        glfwGetJoystickHats(id, &hat_count);
        
        info.num_axes = axis_count;
        info.num_buttons = button_count;
        info.num_hats = hat_count;
        
        std::cout << "Gamepad connected: " << info.name << " (ID: " << id << ")" << std::endl;
      } else {
        // Disconnected
        if (it != gamepads_.end()) {
          std::cout << "Gamepad disconnected: " << it->second.name << " (ID: " << id << ")" << std::endl;
        }
      }
      
      gamepads_[id] = info;
      
      // Trigger callback if set
      if (connection_callback_) {
        connection_callback_(info, currently_connected);
      }
    } else if (currently_connected && it != gamepads_.end()) {
      // Update existing connected gamepad info
      it->second.connected = true;
    }
  }
  
  // Clean up disconnected gamepads
  for (auto it = gamepads_.begin(); it != gamepads_.end();) {
    if (!it->second.connected) {
      it = gamepads_.erase(it);
    } else {
      ++it;
    }
  }
}

void GamepadManager::GLFWGamepadCallback(int jid, int event) {
  GamepadManager::GetInstance().OnGamepadEvent(jid, event);
}

void GamepadManager::OnGamepadEvent(int jid, int event) {
  // Update our internal list
  UpdateGamepadList();
  
  // Clear previous state for disconnected gamepad
  if (event == GLFW_DISCONNECTED) {
    previous_states_.erase(jid);
  }
}

std::vector<InputEvent> GamepadManager::PollEvents() {
  std::vector<InputEvent> events;
  
  // Get all connected gamepads
  auto connected_gamepads = GetConnectedGamepads();
  
  // Poll each connected gamepad for state changes
  for (const auto& gamepad_info : connected_gamepads) {
    int gamepad_id = gamepad_info.id;
    auto current_state = GetGamepadState(gamepad_id);
    
    auto& previous_state = previous_states_[gamepad_id];
    
    // Generate events for state changes
    auto button_events = GenerateButtonEvents(gamepad_id, current_state, previous_state);
    auto axis_events = GenerateAxisEvents(gamepad_id, current_state, previous_state);
    auto hat_events = GenerateHatEvents(gamepad_id, current_state, previous_state);
    
    // Combine all events
    events.insert(events.end(), button_events.begin(), button_events.end());
    events.insert(events.end(), axis_events.begin(), axis_events.end());
    events.insert(events.end(), hat_events.begin(), hat_events.end());
    
    // Update previous state for next frame
    previous_state = current_state;
  }
  
  return events;
}

InputEvent GamepadManager::CreateGamepadEvent(InputEventType type, int gamepad_id, int button_or_key) {
  auto event = InputEvent(type, button_or_key);
  event.SetGamepadId(gamepad_id);
  return event;
}

std::vector<InputEvent> GamepadManager::GenerateButtonEvents(int gamepad_id,
                                                           const GamepadState& current,
                                                           const GamepadState& previous) {
  std::vector<InputEvent> events;
  
  // Handle button count changes properly to avoid stuck buttons
  size_t max_buttons = std::max(current.buttons.size(), previous.buttons.size());
  for (size_t i = 0; i < max_buttons; ++i) {
    bool was_pressed = (i < previous.buttons.size()) && (previous.buttons[i] != 0);
    bool is_pressed = (i < current.buttons.size()) && (current.buttons[i] != 0);
    
    if (is_pressed && !was_pressed) {
      auto event = CreateGamepadEvent(InputEventType::kGamepadButtonPress, gamepad_id, static_cast<int>(i));
      events.push_back(event);
    } else if (!is_pressed && was_pressed) {
      auto event = CreateGamepadEvent(InputEventType::kGamepadButtonRelease, gamepad_id, static_cast<int>(i));
      events.push_back(event);
    }
  }
  
  return events;
}

std::vector<InputEvent> GamepadManager::GenerateAxisEvents(int gamepad_id,
                                                          const GamepadState& current,
                                                          const GamepadState& previous) {
  std::vector<InputEvent> events;
  
  // Generate continuous movement events for changed axes
  const float axis_threshold = 0.01f;  // Minimum change to generate event
  size_t max_axes = std::max(current.axes.size(), previous.axes.size());
  for (size_t i = 0; i < max_axes; ++i) {
    float current_value = (i < current.axes.size()) ? current.axes[i] : 0.0f;
    float previous_value = (i < previous.axes.size()) ? previous.axes[i] : 0.0f;
    
    if (std::abs(current_value - previous_value) > axis_threshold) {
      auto event = CreateGamepadEvent(InputEventType::kGamepadAxisMove, gamepad_id, -1);
      event.SetAxisIndex(static_cast<int>(i));
      event.SetAxisValue(current_value);
      events.push_back(event);
    }
  }
  
  return events;
}

std::vector<InputEvent> GamepadManager::GenerateHatEvents(int gamepad_id,
                                                         const GamepadState& current,
                                                         const GamepadState& previous) {
  std::vector<InputEvent> events;
  
  // Hat/POV events: Digital directional pad
  size_t max_hats = std::max(current.hats.size(), previous.hats.size());
  for (size_t i = 0; i < max_hats; ++i) {
    unsigned char current_hat = (i < current.hats.size()) ? current.hats[i] : 0;
    unsigned char previous_hat = (i < previous.hats.size()) ? previous.hats[i] : 0;
    
    if (current_hat != previous_hat) {
      // Generate press events for newly pressed directions
      unsigned char pressed_directions = current_hat & (~previous_hat);
      unsigned char released_directions = previous_hat & (~current_hat);
      
      if (pressed_directions != 0) {
        auto event = CreateGamepadEvent(InputEventType::kGamepadButtonPress, gamepad_id, static_cast<int>(current_hat));
        events.push_back(event);
      }
      
      if (released_directions != 0) {
        auto event = CreateGamepadEvent(InputEventType::kGamepadButtonRelease, gamepad_id, static_cast<int>(previous_hat));
        events.push_back(event);
      }
    }
  }
  
  return events;
}

void GamepadManager::Shutdown() {
  // Set shutdown flag first to prevent new GLFW calls
  shutdown_ = true;
  
  // Disable monitoring and clear callback before GLFW termination
  if (monitoring_enabled_) {
    glfwSetJoystickCallback(nullptr);
    monitoring_enabled_ = false;
  }
  
  // Clear all state
  gamepads_.clear();
  previous_states_.clear();
  connection_callback_ = nullptr;
}

}  // namespace quickviz