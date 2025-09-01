/*
 * @file input_manager.cpp
 * @date 9/1/25
 * @brief Implementation of simplified input manager
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/input_manager.hpp"

namespace quickviz {

void InputManager::RegisterHandler(std::shared_ptr<InputEventHandler> handler) {
  dispatcher_.RegisterHandler(handler);
}

void InputManager::UnregisterHandler(const std::string& name) {
  dispatcher_.UnregisterHandler(name);
}

bool InputManager::ProcessEvents(const std::vector<InputEvent>& events) {
  bool any_consumed = false;
  for (const auto& event : events) {
    if (dispatcher_.DispatchEvent(event)) {
      any_consumed = true;
    }
  }
  return any_consumed;
}

bool InputManager::ProcessEvent(const InputEvent& event) {
  return dispatcher_.DispatchEvent(event);
}

bool InputManager::IsActionTriggered(const std::string& action, const InputEvent& event) const {
  return input_mapping_.IsActionTriggered(action, event);
}

std::vector<std::string> InputManager::GetTriggeredActions(const InputEvent& event) const {
  return input_mapping_.GetActionsForEvent(event);
}

void InputManager::SetEnabled(bool enabled) {
  dispatcher_.SetEnabled(enabled);
}

bool InputManager::IsEnabled() const {
  return dispatcher_.IsEnabled();
}

size_t InputManager::GetHandlerCount() const {
  return dispatcher_.GetHandlerCount();
}

}  // namespace quickviz