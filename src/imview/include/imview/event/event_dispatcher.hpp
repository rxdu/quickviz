/*
 * @file event_dispatcher.hpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_EVENT_DISPATCHER_HPP
#define QUICKVIZ_EVENT_DISPATCHER_HPP

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include "imview/event/event.hpp"

namespace quickviz {
class EventDispatcher {
 public:
  using EventPtr = std::shared_ptr<BaseEvent>;
  using HandlerFunc = std::function<void(std::shared_ptr<BaseEvent>)>;

  // Get the singleton instance of EventDispatcher
  static EventDispatcher& GetInstance() {
    static EventDispatcher instance;
    return instance;
  }

  // Register an event handler for a specific event type (identified by name)
  void RegisterHandler(const std::string& event_name, HandlerFunc handler) {
    handlers_[event_name].push_back(handler);
  }

  // Dispatch an event to all registered handlers
  void Dispatch(std::shared_ptr<BaseEvent> event) const {
    if (event == nullptr) return;
    if (handlers_.find(event->GetName()) != handlers_.end()) {
      for (const auto& handler : handlers_.at(event->GetName())) {
        handler(event);
      }
    }
  }

 private:
  // Make constructor private to prevent instantiation
  EventDispatcher() = default;

  // do not allow copy or move
  EventDispatcher(const EventDispatcher&) = delete;
  EventDispatcher(EventDispatcher&&) = delete;
  EventDispatcher& operator=(const EventDispatcher&) = delete;
  EventDispatcher& operator=(EventDispatcher&&) = delete;

  std::unordered_map<std::string, std::vector<HandlerFunc>> handlers_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_DISPATCHER_HPP