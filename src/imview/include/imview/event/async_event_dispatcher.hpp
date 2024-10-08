/*
 * @file async_event_dispatcher.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP
#define QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <string>
#include <queue>

#include "imview/event/event.hpp"

namespace quickviz {
class AsyncEventDispatcher {
 public:
  using EventPtr = std::shared_ptr<BaseEvent>;
  using HandlerFunc = std::function<void(std::shared_ptr<BaseEvent>)>;

  ~AsyncEventDispatcher();

  // public interface
  static void Initialize(size_t num_threads);
  static AsyncEventDispatcher& GetInstance();
  void RegisterHandler(const std::string& eventName, HandlerFunc handler);
  void Dispatch(std::shared_ptr<BaseEvent> event);

 private:
  AsyncEventDispatcher(size_t num_threads);

  static std::unique_ptr<AsyncEventDispatcher> instance_;

  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::unordered_map<std::string, std::vector<HandlerFunc>> handlers_;

  std::mutex queue_mutex_;
  std::mutex handler_mutex_;
  std::condition_variable condition_;
  bool stop_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP