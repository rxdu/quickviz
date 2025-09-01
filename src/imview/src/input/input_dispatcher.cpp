/*
 * @file input_dispatcher.cpp
 * @date 9/1/25
 * @brief Implementation of input event dispatcher
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/input/input_dispatcher.hpp"

#include <algorithm>

namespace quickviz {

void InputDispatcher::RegisterHandler(std::shared_ptr<InputEventHandler> handler) {
  if (!handler) return;
  
  // Check if handler with same name already exists
  auto it = std::find_if(handlers_.begin(), handlers_.end(),
    [&handler](const std::shared_ptr<InputEventHandler>& existing) {
      return existing && existing->GetName() == handler->GetName();
    });
  
  if (it != handlers_.end()) {
    // Replace existing handler
    *it = handler;
  } else {
    // Add new handler
    handlers_.push_back(handler);
  }
  
  needs_sort_ = true;
}

void InputDispatcher::UnregisterHandler(const std::string& name) {
  handlers_.erase(
    std::remove_if(handlers_.begin(), handlers_.end(),
      [&name](const std::shared_ptr<InputEventHandler>& handler) {
        return handler && handler->GetName() == name;
      }),
    handlers_.end());
}

void InputDispatcher::ClearHandlers() {
  handlers_.clear();
  needs_sort_ = false;
}

bool InputDispatcher::DispatchEvent(const InputEvent& event) {
  if (!enabled_) return false;
  
  // Clean up null handlers and sort if needed
  handlers_.erase(
    std::remove_if(handlers_.begin(), handlers_.end(),
      [](const std::shared_ptr<InputEventHandler>& handler) {
        return !handler;
      }),
    handlers_.end());
  
  if (needs_sort_) {
    SortHandlers();
    needs_sort_ = false;
  }
  
  // Dispatch to handlers in priority order
  for (auto& handler : handlers_) {
    if (handler && handler->IsEnabled() && handler->OnInputEvent(event)) {
      return true;  // Event consumed
    }
  }
  
  return false;  // Event not consumed
}

std::vector<std::string> InputDispatcher::GetHandlerNames() const {
  std::vector<std::string> names;
  for (const auto& handler : handlers_) {
    if (handler) {
      names.push_back(handler->GetName());
    }
  }
  return names;
}

void InputDispatcher::SortHandlers() {
  std::sort(handlers_.begin(), handlers_.end(),
    [](const std::shared_ptr<InputEventHandler>& a, 
       const std::shared_ptr<InputEventHandler>& b) {
      if (!a) return false;
      if (!b) return true;
      return a->GetPriority() > b->GetPriority();
    });
}


}  // namespace quickviz