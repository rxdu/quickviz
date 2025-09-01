/*
 * @file gamepad_manager.cpp
 * @date 9/1/25
 * @brief Implementation of modern gamepad management system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/gamepad_manager.hpp"
#include <iostream>

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
}

}  // namespace quickviz