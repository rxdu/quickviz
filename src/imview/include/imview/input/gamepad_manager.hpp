/*
 * @file gamepad_manager.hpp
 * @date 9/1/25
 * @brief Modern gamepad enumeration and management for unified input system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_GAMEPAD_MANAGER_HPP
#define IMVIEW_GAMEPAD_MANAGER_HPP

#include <vector>
#include <string>
#include <unordered_map>
#include <functional>

#include "core/event/input_event.hpp"
#include "GLFW/glfw3.h"

namespace quickviz {

/**
 * @brief Gamepad device information
 */
struct GamepadInfo {
  int id = -1;
  std::string name;
  bool connected = false;
  
  // Capability information
  int num_axes = 0;
  int num_buttons = 0;
  int num_hats = 0;
  
  bool operator==(const GamepadInfo& other) const {
    return id == other.id && name == other.name && connected == other.connected;
  }
  
  bool operator!=(const GamepadInfo& other) const {
    return !(*this == other);
  }
};

/**
 * @brief Modern gamepad manager for unified input system
 * 
 * This replaces the legacy joystick system with a cleaner API that
 * integrates with the new InputEvent system.
 */
class GamepadManager {
public:
  using ConnectionCallback = std::function<void(const GamepadInfo&, bool connected)>;

  static GamepadManager& GetInstance() {
    static GamepadManager instance;
    return instance;
  }

  // Delete copy/move to ensure singleton
  GamepadManager(const GamepadManager&) = delete;
  GamepadManager& operator=(const GamepadManager&) = delete;
  GamepadManager(GamepadManager&&) = delete;
  GamepadManager& operator=(GamepadManager&&) = delete;

  /**
   * @brief Enumerate all connected gamepads
   * @return Vector of connected gamepad information
   */
  std::vector<GamepadInfo> GetConnectedGamepads();

  /**
   * @brief Get information about a specific gamepad
   * @param gamepad_id GLFW gamepad ID
   * @return GamepadInfo structure, or empty info if not connected
   */
  GamepadInfo GetGamepadInfo(int gamepad_id);

  /**
   * @brief Check if a specific gamepad is connected
   * @param gamepad_id GLFW gamepad ID
   * @return true if gamepad is connected
   */
  bool IsGamepadConnected(int gamepad_id);

  /**
   * @brief Get human-readable list of connected gamepad names
   * @return Vector of strings with "ID: Name" format
   */
  std::vector<std::string> GetGamepadNames();

  /**
   * @brief Set callback for gamepad connection/disconnection events
   * @param callback Function to call when gamepad status changes
   */
  void SetConnectionCallback(ConnectionCallback callback);

  /**
   * @brief Enable/disable gamepad monitoring
   * @param enabled Whether to monitor gamepad connections
   */
  void SetMonitoringEnabled(bool enabled);

  /**
   * @brief Check if monitoring is enabled
   * @return true if monitoring gamepad connections
   */
  bool IsMonitoringEnabled() const { return monitoring_enabled_; }

  /**
   * @brief Safely shutdown the gamepad manager before GLFW termination
   * Note: This is automatically called by Window destructor. Manual calls
   * are only needed when using GLFW without the QuickViz Window class.
   */
  void Shutdown();

  /**
   * @brief Get current gamepad state for InputEvent creation
   * @param gamepad_id GLFW gamepad ID  
   * @return Current button/axis state, empty if not connected
   */
  struct GamepadState {
    std::vector<float> axes;
    std::vector<unsigned char> buttons;
    std::vector<unsigned char> hats;
  };
  GamepadState GetGamepadState(int gamepad_id);

  /**
   * @brief Poll for gamepad input events since last poll
   * @return Vector of input events from all connected gamepads
   */
  std::vector<InputEvent> PollEvents();

private:
  GamepadManager() : monitoring_enabled_(false) {
    UpdateGamepadList();
  }
  
  ~GamepadManager() {
    // Do nothing - Shutdown() is automatically called by Window destructor
    // This prevents segfaults when singleton is destroyed after program exit
  }

  void InitializeGLFWCallback();
  void UpdateGamepadList();
  static void GLFWGamepadCallback(int jid, int event);
  void OnGamepadEvent(int jid, int event);

  // Event generation helpers
  std::vector<InputEvent> GenerateButtonEvents(int gamepad_id, 
                                              const GamepadState& current, 
                                              const GamepadState& previous);
  std::vector<InputEvent> GenerateAxisEvents(int gamepad_id,
                                            const GamepadState& current,
                                            const GamepadState& previous);
  std::vector<InputEvent> GenerateHatEvents(int gamepad_id,
                                           const GamepadState& current,
                                           const GamepadState& previous);
  InputEvent CreateGamepadEvent(InputEventType type, int gamepad_id, int button_or_key = -1);

  bool monitoring_enabled_;
  std::unordered_map<int, GamepadInfo> gamepads_;
  ConnectionCallback connection_callback_;
  
  // State tracking for delta-based event generation
  std::unordered_map<int, GamepadState> previous_states_;
  
  // Flag to prevent GLFW calls after shutdown
  bool shutdown_ = false;
};

}  // namespace quickviz

#endif  // IMVIEW_GAMEPAD_MANAGER_HPP