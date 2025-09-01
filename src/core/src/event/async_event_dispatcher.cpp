/*
 * @file async_event_dispatcher.cpp
 * @date 10/8/24
 * @brief Instance-based async event dispatcher with owned worker thread
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "core/event/async_event_dispatcher.hpp"

#include <algorithm>
#include <iostream>

namespace quickviz {

AsyncEventDispatcher::AsyncEventDispatcher() {
  Start();
}

AsyncEventDispatcher::~AsyncEventDispatcher() {
  Stop();
}

AsyncEventDispatcher::AsyncEventDispatcher(AsyncEventDispatcher&& other) noexcept
    : worker_thread_(std::move(other.worker_thread_)),
      running_(other.running_.load()),
      shutdown_requested_(other.shutdown_requested_.load()),
      event_queue_(std::move(other.event_queue_)),
      handlers_(std::move(other.handlers_)),
      next_token_(other.next_token_.load()) {
  // Mark other as moved-from
  other.running_ = false;
  other.shutdown_requested_ = true;
}

AsyncEventDispatcher& AsyncEventDispatcher::operator=(AsyncEventDispatcher&& other) noexcept {
  if (this != &other) {
    // Stop current instance
    Stop();
    
    // Move from other
    worker_thread_ = std::move(other.worker_thread_);
    running_ = other.running_.load();
    shutdown_requested_ = other.shutdown_requested_.load();
    event_queue_ = std::move(other.event_queue_);
    handlers_ = std::move(other.handlers_);
    next_token_ = other.next_token_.load();
    
    // Mark other as moved-from
    other.running_ = false;
    other.shutdown_requested_ = true;
  }
  return *this;
}

AsyncEventDispatcher::HandlerToken AsyncEventDispatcher::RegisterHandler(const std::string& event_name, HandlerFunc handler) {
  if (!handler) {
    return kInvalidToken;
  }
  
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  HandlerToken token = GenerateToken();
  handlers_[event_name].emplace_back(token, std::move(handler));
  return token;
}

void AsyncEventDispatcher::UnregisterHandler(HandlerToken token) {
  if (token == kInvalidToken) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  for (auto& [event_name, handler_list] : handlers_) {
    auto it = std::find_if(handler_list.begin(), handler_list.end(),
                          [token](const HandlerEntry& entry) {
                            return entry.token == token;
                          });
    if (it != handler_list.end()) {
      handler_list.erase(it);
      // Clean up empty event entries
      if (handler_list.empty()) {
        handlers_.erase(event_name);
      }
      break;
    }
  }
}

void AsyncEventDispatcher::ClearHandlers(const std::string& event_name) {
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  if (event_name.empty()) {
    handlers_.clear();
  } else {
    handlers_.erase(event_name);
  }
}

void AsyncEventDispatcher::Dispatch(std::shared_ptr<BaseEvent> event) {
  if (!event || !running_.load()) {
    return;
  }
  
  event_queue_.Push(std::move(event));
}

void AsyncEventDispatcher::Start() {
  if (running_.load()) {
    return; // Already running
  }
  
  shutdown_requested_ = false;
  running_ = true;
  
  worker_thread_ = std::make_unique<std::thread>(&AsyncEventDispatcher::WorkerLoop, this);
}

void AsyncEventDispatcher::Stop() {
  if (!running_.load()) {
    return; // Already stopped
  }
  
  // Signal shutdown and close the queue to unblock worker
  shutdown_requested_ = true;
  event_queue_.Close();
  
  // Wait for worker thread to finish
  if (worker_thread_ && worker_thread_->joinable()) {
    worker_thread_->join();
  }
  
  worker_thread_.reset();
  running_ = false;
}

size_t AsyncEventDispatcher::GetQueueSize() const {
  return event_queue_.Size();
}

size_t AsyncEventDispatcher::GetHandlerCount() const {
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  size_t total = 0;
  for (const auto& [event_name, handler_list] : handlers_) {
    total += handler_list.size();
  }
  return total;
}

size_t AsyncEventDispatcher::GetHandlerCount(const std::string& event_name) const {
  std::lock_guard<std::mutex> lock(handlers_mutex_);
  auto it = handlers_.find(event_name);
  return (it != handlers_.end()) ? it->second.size() : 0;
}

void AsyncEventDispatcher::WorkerLoop() {
  while (!shutdown_requested_.load()) {
    // Use non-blocking TryPop to allow graceful shutdown checking
    std::shared_ptr<BaseEvent> event;
    if (event_queue_.TryPop(event)) {
      ProcessEvent(event);
    } else {
      // No event available, sleep briefly to avoid busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  // Drain remaining events before shutdown
  std::shared_ptr<BaseEvent> event;
  while (event_queue_.TryPop(event)) {
    ProcessEvent(event);
  }
  
  running_ = false;
}

void AsyncEventDispatcher::ProcessEvent(std::shared_ptr<BaseEvent> event) {
  if (!event) {
    return;
  }
  
  // Snapshot handlers under lock to minimize lock contention
  std::vector<HandlerEntry> handlers_snapshot;
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto it = handlers_.find(event->GetName());
    if (it != handlers_.end()) {
      handlers_snapshot = it->second; // Copy handlers
    }
  }
  
  // Execute handlers outside of lock
  for (const auto& entry : handlers_snapshot) {
    try {
      bool consumed = entry.handler(event);
      if (consumed) {
        break; // Stop processing if event was consumed
      }
    } catch (const std::exception& e) {
      // Log handler exception but continue processing
      std::cerr << "AsyncEventDispatcher: Handler exception for event '" 
                << event->GetName() << "': " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "AsyncEventDispatcher: Unknown handler exception for event '" 
                << event->GetName() << "'" << std::endl;
    }
  }
}

AsyncEventDispatcher::HandlerToken AsyncEventDispatcher::GenerateToken() {
  return next_token_.fetch_add(1);
}

}  // namespace quickviz