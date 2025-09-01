/*
 * @file async_event_emitter.hpp
 * @date 10/8/24
 * @brief Instance-based async event emitter
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_EVENT_ASYNC_EMITTER_HPP
#define QUICKVIZ_EVENT_ASYNC_EMITTER_HPP

#include "core/event/async_event_dispatcher.hpp"

namespace quickviz {

class AsyncEventEmitter {
 public:
  explicit AsyncEventEmitter(AsyncEventDispatcher& dispatcher) 
      : dispatcher_(dispatcher) {}

  template <typename EventT, typename... Args>
  void Emit(EventSource type, const std::string& event_name, Args&&... args) {
    auto event = std::make_shared<EventT>(type, event_name, std::forward<Args>(args)...);
    dispatcher_.Dispatch(event);
  }

 private:
  AsyncEventDispatcher& dispatcher_;
};

}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_ASYNC_EMITTER_HPP
