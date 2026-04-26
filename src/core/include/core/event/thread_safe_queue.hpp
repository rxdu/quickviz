/*
* @file thread_safe_queue.hpp
* @date 10/10/24
* @brief Thread-safe queue with shutdown protocol and modern C++ features
*
* @copyright Copyright (c) 2024 Ruixiang Du (rdu)
*/
#ifndef QUICKVIZ_THREAD_SAFE_QUEUE_HPP
#define QUICKVIZ_THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <optional>
#include <chrono>
#include <atomic>
#include <stdexcept>

namespace quickviz {
template <typename T>
class ThreadSafeQueue {
public:
 ThreadSafeQueue() = default;
 ~ThreadSafeQueue() = default;

 // do not allow copy
 ThreadSafeQueue(const ThreadSafeQueue&) = delete;
 ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

 // allow move with proper locking
 ThreadSafeQueue(ThreadSafeQueue&& other) noexcept {
   std::scoped_lock lock(mutex_, other.mutex_);
   queue_ = std::move(other.queue_);
   closed_ = other.closed_.load();
 }

 ThreadSafeQueue& operator=(ThreadSafeQueue&& other) noexcept {
   if (this != &other) {
     std::scoped_lock lock(mutex_, other.mutex_);
     queue_ = std::move(other.queue_);
     closed_ = other.closed_.load();
   }
   return *this;
 }

 // push data into the queue with perfect forwarding
 template<typename U>
 void Push(U&& data) {
   std::lock_guard<std::mutex> lock(mutex_);
   if (closed_.load()) {
     throw std::runtime_error("Cannot push to closed queue");
   }
   queue_.push(std::forward<U>(data));
   condition_.notify_one();
 }

 // convenience overload for const reference (maintains backward compatibility)
 void Push(const T& data) {
   Push<const T&>(data);
 }

 // pop data from the queue (blocking with shutdown support)
 [[nodiscard]] std::optional<T> Pop() {
   std::unique_lock<std::mutex> lock(mutex_);
   condition_.wait(lock, [this] { return !queue_.empty() || closed_.load(); });
   
   if (closed_.load() && queue_.empty()) {
     return std::nullopt;  // Graceful shutdown - no more data
   }
   
   T data = queue_.front();
   queue_.pop();
   return data;
 }

 // try to pop data from the queue (non-blocking)
 [[nodiscard]] bool TryPop(T& data) {
   std::lock_guard<std::mutex> lock(mutex_);
   if (queue_.empty()) return false;
   data = queue_.front();
   queue_.pop();
   return true;
 }

 // pop with timeout support
 template<typename Rep, typename Period>
 [[nodiscard]] std::optional<T> PopFor(const std::chrono::duration<Rep, Period>& timeout) {
   std::unique_lock<std::mutex> lock(mutex_);
   if (condition_.wait_for(lock, timeout, [this] { return !queue_.empty() || closed_.load(); })) {
     if (closed_.load() && queue_.empty()) {
       return std::nullopt;
     }
     T data = queue_.front();
     queue_.pop();
     return data;
   }
   return std::nullopt;  // Timeout
 }

 // shutdown protocol methods
 void Close() {
   std::lock_guard<std::mutex> lock(mutex_);
   closed_.store(true);
   condition_.notify_all();  // Wake up all waiting threads
 }

 [[nodiscard]] bool IsClosed() const {
   return closed_.load();
 }

 // utility methods
 [[nodiscard]] bool Empty() const {
   std::lock_guard<std::mutex> lock(mutex_);
   return queue_.empty();
 }

 [[nodiscard]] size_t Size() const {
   std::lock_guard<std::mutex> lock(mutex_);
   return queue_.size();
 }

private:
 std::queue<T> queue_;
 mutable std::mutex mutex_;
 std::condition_variable condition_;
 std::atomic<bool> closed_{false};
};
}  // namespace quickviz

#endif  // QUICKVIZ_THREAD_SAFE_QUEUE_HPP
