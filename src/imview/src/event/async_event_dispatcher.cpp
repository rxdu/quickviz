/*
 * @file async_event_dispatcher.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/event/async_event_dispatcher.hpp"

namespace quickviz {
AsyncEventDispatcher& AsyncEventDispatcher::GetInstance() {
  static AsyncEventDispatcher instance;
  return instance;
}

void AsyncEventDispatcher::RegisterHandler(const std::string& event_name,
                                           HandlerFunc handler) {
  std::lock_guard<std::mutex> lock(handler_mutex_);
  handlers_[event_name].push_back(handler);
}

void AsyncEventDispatcher::Dispatch(std::shared_ptr<BaseEvent> event) {
  event_queue_.Push(event);
}

void AsyncEventDispatcher::HandleEvents() {
  std::shared_ptr<BaseEvent> event;
  while (event_queue_.TryPop(event)) {
    std::vector<HandlerFunc> event_handlers;
    // lock only while accessing the handlers_ map
    {
      std::lock_guard<std::mutex> hlock(handler_mutex_);
      if (handlers_.find(event->GetName()) != handlers_.end()) {
        event_handlers =
            handlers_[event->GetName()];  // Copy the list of handlers
      }
    }
    // execute handlers outside the lock
    for (const auto& handler : event_handlers) {
      handler(event);
    }
  }
}
}  // namespace quickviz