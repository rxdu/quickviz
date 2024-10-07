/*
 * @file event_emitter.hpp
 * @date 10/7/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_EVENT_EMITTER_HPP
#define QUICKVIZ_EVENT_EMITTER_HPP

#include "imview/event/event_dispatcher.hpp"

namespace quickviz {
class EventEmitter {
 public:
  template <typename EventType, typename... Args>
  void Emit(const std::string& event_name, Args... args) {
    auto event = std::make_shared<EventType>(event_name, args...);
    EventDispatcher::GetInstance().Dispatch(event);
  }
};
}  // namespace quickviz

#endif  // QUICKVIZ_EVENT_EMITTER_HPP
