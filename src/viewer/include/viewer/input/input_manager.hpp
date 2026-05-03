/*
 * @file input_manager.hpp
 * @date 9/1/25
 * @brief ImGui-centric input management for viewer windows
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VIEWER_INPUT_MANAGER_HPP
#define VIEWER_INPUT_MANAGER_HPP

#include <memory>
#include <string>

#include "core/event/input_event.hpp"
#include "core/event/input_mapping.hpp"
#include "viewer/input/input_dispatcher.hpp"

namespace quickviz {

/**
 * @brief Simplified input manager for ImGui-centric input handling
 * 
 * Manages input dispatching and action mapping without GLFW callback integration.
 * Input events are created from ImGui state during panel processing.
 */
class InputManager {
 public:
  InputManager() = default;
  ~InputManager() = default;

  // Non-copyable
  InputManager(const InputManager&) = delete;
  InputManager& operator=(const InputManager&) = delete;

  /**
   * @brief Get the input dispatcher
   */
  InputDispatcher& GetDispatcher() { return dispatcher_; }
  const InputDispatcher& GetDispatcher() const { return dispatcher_; }

  /**
   * @brief Get input mapping configuration
   */
  InputMapping& GetInputMapping() { return input_mapping_; }
  const InputMapping& GetInputMapping() const { return input_mapping_; }

  /**
   * @brief Register handler with priority
   */
  void RegisterHandler(std::shared_ptr<InputEventHandler> handler);

  /**
   * @brief Unregister handler by name
   */
  void UnregisterHandler(const std::string& name);

  /**
   * @brief Process events through dispatcher
   * @param events Vector of events to process
   * @return true if any event was consumed
   */
  bool ProcessEvents(const std::vector<InputEvent>& events);

  /**
   * @brief Process single event through dispatcher
   * @param event Event to process
   * @return true if event was consumed
   */
  bool ProcessEvent(const InputEvent& event);

  /**
   * @brief Check if specific action is triggered by event
   * @param action Action name to check
   * @param event Event to test against
   */
  bool IsActionTriggered(const std::string& action, const InputEvent& event) const;

  /**
   * @brief Get all actions triggered by event
   */
  std::vector<std::string> GetTriggeredActions(const InputEvent& event) const;

  /**
   * @brief Enable/disable input processing
   */
  void SetEnabled(bool enabled);
  bool IsEnabled() const;

  /**
   * @brief Get handler count for debugging
   */
  size_t GetHandlerCount() const;

 private:
  InputDispatcher dispatcher_;
  InputMapping input_mapping_;
};

}  // namespace quickviz

#endif  // VIEWER_INPUT_MANAGER_HPP