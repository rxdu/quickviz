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

// InputHandlerAdapter implementation
InputHandlerAdapter::InputHandlerAdapter(InputHandler* legacy_handler, 
                                         const std::string& name,
                                         int priority)
    : legacy_handler_(legacy_handler), name_(name), priority_(priority) {
}

bool InputHandlerAdapter::OnInputEvent(const InputEvent& event) {
  if (!legacy_handler_) return false;
  
  // Convert InputEvent to legacy InputHandler calls
  switch (event.GetType()) {
    case InputEventType::kMousePress:
    case InputEventType::kMouseRelease: {
      int action = (event.GetType() == InputEventType::kMousePress) ? 1 : 0;  // GLFW press/release
      // Extract modifiers into GLFW-style mods
      int mods = 0;
      const auto& mod_keys = event.GetModifiers();
      if (mod_keys.ctrl) mods |= 0x0002;   // GLFW_MOD_CONTROL
      if (mod_keys.shift) mods |= 0x0001;  // GLFW_MOD_SHIFT
      if (mod_keys.alt) mods |= 0x0004;    // GLFW_MOD_ALT
      if (mod_keys.super) mods |= 0x0008;  // GLFW_MOD_SUPER
      
      legacy_handler_->OnMouseButton(event.GetMouseButton(), action, mods);
      return false;  // Legacy handlers don't return consumption status
    }
    
    case InputEventType::kMouseMove: {
      auto pos = event.GetScreenPosition();
      legacy_handler_->OnMouseMove(pos.x, pos.y);
      return false;
    }
    
    case InputEventType::kMouseWheel: {
      auto delta = event.GetDelta();
      legacy_handler_->OnMouseScroll(delta.x, delta.y);
      return false;
    }
    
    case InputEventType::kKeyPress:
    case InputEventType::kKeyRelease: {
      int action = (event.GetType() == InputEventType::kKeyPress) ? 1 : 0;
      int mods = 0;
      const auto& mod_keys = event.GetModifiers();
      if (mod_keys.ctrl) mods |= 0x0002;
      if (mod_keys.shift) mods |= 0x0001;
      if (mod_keys.alt) mods |= 0x0004;
      if (mod_keys.super) mods |= 0x0008;
      
      legacy_handler_->OnKeyPress(event.GetKey(), 0, action, mods);  // scancode = 0
      return false;
    }
    
    default:
      return false;
  }
}

}  // namespace quickviz