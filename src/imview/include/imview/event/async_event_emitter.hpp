/*
 * @file event_async_emitter.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_EVENT_ASYNC_EMITTER_HPP
#define QUICKVIZ_EVENT_ASYNC_EMITTER_HPP

#include "imview/event/async_event_dispatcher.hpp"

namespace quickviz {
class AsyncEventEmitter {
 public:
  template <typename EventType, typename... Args>
  void Emit(const std::string& event_name, Args... args) {
    auto event = std::make_shared<EventType>(event_name, args...);
    AsyncEventDispatcher::GetInstance().Dispatch(event);
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_ASYNC_EMITTER_HPP
