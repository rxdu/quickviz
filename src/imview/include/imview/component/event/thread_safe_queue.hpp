/*
 * @file thread_safe_queue.hpp
 * @date 10/10/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_THREAD_SAFE_QUEUE_HPP
#define QUICKVIZ_THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

namespace quickviz {
template <typename T>
class ThreadSafeQueue {
 public:
  // Push data into the queue
  void Push(const T& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(data);
    condition_.notify_one();  // Notify one waiting thread
  }

  // Pop data from the queue (blocking)
  T Pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] { return !queue_.empty(); });
    T data = queue_.front();
    queue_.pop();
    return data;
  }

  // Try to pop data from the queue (non-blocking)
  bool TryPop(T& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    data = queue_.front();
    queue_.pop();
    return true;
  }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_THREAD_SAFE_QUEUE_HPP
