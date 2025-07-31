/*
 * @file double_buffer.hpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_DOUBLE_BUFFER_HPP
#define QUICKVIZ_DOUBLE_BUFFER_HPP

#include <mutex>
#include <atomic>
#include <condition_variable>

#include "core/buffer/buffer_interface.hpp"

namespace quickviz {
template <typename T>
class DoubleBuffer : public BufferInterface<T> {
 public:
  DoubleBuffer() : write_index_(0), ready_(false) {}

  std::size_t GetOccupiedSize() const override { return ready_.load() ? 1 : 0; }

  std::size_t Write(const T& data) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      buffer_[write_index_] = data;     // Write to the active write buffer
      write_index_ = 1 - write_index_;  // Swap buffer index for next write
      ready_.store(true, std::memory_order_release);
    }
    cond_var_.notify_one();  // Notify the reader that new data is ready

    return 1;
  }

  std::size_t Read(T& data) {
    std::unique_lock<std::mutex> lock(mutex_);
    // wait until data is ready
    cond_var_.wait(lock,
                   [this] { return ready_.load(std::memory_order_acquire); });

    // read from the other buffer
    int read_index = 1 - write_index_;
    data = buffer_[read_index];
    ready_.store(false, std::memory_order_release);

    return 1;
  }

  bool TryRead(T& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) {
      return false;
    }

    int readIndex = 1 - write_index_;
    data = buffer_[readIndex];
    ready_.store(false, std::memory_order_release);
    return true;
  }

  std::size_t PeekAt(T& data, std::size_t n) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (n != 0 || !ready_.load(std::memory_order_acquire)) {
      return 0;
    }
    
    int read_index = 1 - write_index_;
    data = buffer_[read_index];
    return 1;
  }

 private:
  T buffer_[2];
  int write_index_;
  std::atomic<bool> ready_{false};
  mutable std::mutex mutex_;
  std::condition_variable cond_var_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_DOUBLE_BUFFER_HPP