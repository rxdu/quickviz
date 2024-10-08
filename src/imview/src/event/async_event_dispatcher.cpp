/*
 * @file async_event_dispatcher.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/event/async_event_dispatcher.hpp"

namespace quickviz {
std::unique_ptr<AsyncEventDispatcher> AsyncEventDispatcher::instance_;

void AsyncEventDispatcher::Initialize(size_t num_threads) {
  AsyncEventDispatcher* ptr = new AsyncEventDispatcher(num_threads);
  instance_.reset(ptr);
}

AsyncEventDispatcher& AsyncEventDispatcher::GetInstance() {
  if (instance_ == nullptr) {
    throw std::runtime_error("AsyncEventDispatcher has not been initialized");
  }
  return *instance_.get();
}

AsyncEventDispatcher::AsyncEventDispatcher(size_t num_threads) : stop_(false) {
  // Start the thread pool
  for (size_t i = 0; i < num_threads; ++i) {
    workers_.emplace_back([this] {
      while (true) {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(this->queue_mutex_);
          this->condition_.wait(
              lock, [this] { return this->stop_ || !this->tasks_.empty(); });
          if (this->stop_ && this->tasks_.empty()) return;

          // get the next task in the queue
          task = std::move(this->tasks_.front());
          this->tasks_.pop();
        }

        task();
      }
    });
  }
}

AsyncEventDispatcher::~AsyncEventDispatcher() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  condition_.notify_all();
  for (std::thread& worker : workers_) {
    worker.join();
  }
}

void AsyncEventDispatcher::RegisterHandler(const std::string& eventName,
                                           HandlerFunc handler) {
  std::lock_guard<std::mutex> lock(handler_mutex_);
  handlers_[eventName].push_back(handler);
}

void AsyncEventDispatcher::Dispatch(std::shared_ptr<BaseEvent> event) {
  std::lock_guard<std::mutex> hlock(handler_mutex_);
  if (handlers_.find(event->GetName()) != handlers_.end()) {
    for (const auto& handler : handlers_[event->GetName()]) {
      auto task = [handler, event] { handler(event); };
      {
        std::unique_lock<std::mutex> qlock(queue_mutex_);
        tasks_.push(task);
      }
      condition_.notify_one();
    }
  }
}
}  // namespace quickviz