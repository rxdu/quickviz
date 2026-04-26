/*
 * @file async_event_dispatcher.hpp
 * @date 10/8/24
 * @brief Instance-based async event dispatcher with owned worker thread
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
#include <atomic>
#include <unordered_map>
#include <vector>

#include "core/event/event.hpp"
#include "core/event/thread_safe_queue.hpp"

namespace quickviz {

class AsyncEventDispatcher {
 public:
  using HandlerToken = uint64_t;
  using HandlerFunc = std::function<bool(std::shared_ptr<BaseEvent>)>;
  
  static constexpr HandlerToken kInvalidToken = 0;

  // RAII lifecycle: constructor starts worker thread, destructor stops it
  AsyncEventDispatcher();
  ~AsyncEventDispatcher();
  
  // Non-copyable, movable
  AsyncEventDispatcher(const AsyncEventDispatcher&) = delete;
  AsyncEventDispatcher& operator=(const AsyncEventDispatcher&) = delete;
  AsyncEventDispatcher(AsyncEventDispatcher&&) noexcept;
  AsyncEventDispatcher& operator=(AsyncEventDispatcher&&) noexcept;

  // Handler management
  [[nodiscard]] HandlerToken RegisterHandler(const std::string& event_name, HandlerFunc handler);
  void UnregisterHandler(HandlerToken token);
  void ClearHandlers(const std::string& event_name = ""); // empty string = clear all

  // Event dispatch
  void Dispatch(std::shared_ptr<BaseEvent> event);
  
  // Lifecycle control
  void Start();  // Explicit start (no-op if already started)
  void Stop();   // Graceful shutdown with queue draining
  bool IsRunning() const { return running_.load(); }
  
  // Statistics for monitoring/debugging
  size_t GetQueueSize() const;
  size_t GetHandlerCount() const;
  size_t GetHandlerCount(const std::string& event_name) const;


 private:
  struct HandlerEntry {
    HandlerToken token;
    HandlerFunc handler;
    
    HandlerEntry(HandlerToken t, HandlerFunc h) : token(t), handler(std::move(h)) {}
  };

  void WorkerLoop();
  void ProcessEvent(std::shared_ptr<BaseEvent> event);
  HandlerToken GenerateToken();
  
  // Thread management
  std::unique_ptr<std::thread> worker_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> shutdown_requested_{false};
  
  // Event processing
  ThreadSafeQueue<std::shared_ptr<BaseEvent>> event_queue_;
  
  // Handler management
  mutable std::mutex handlers_mutex_;
  std::unordered_map<std::string, std::vector<HandlerEntry>> handlers_;
  std::atomic<HandlerToken> next_token_{1};
};

}  // namespace quickviz

#endif  // QUICKVIZ_ASYNC_EVENT_DISPATCHER_HPP