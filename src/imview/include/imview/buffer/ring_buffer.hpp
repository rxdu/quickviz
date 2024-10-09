/*
 * ring_buffer.hpp
 *
 * Created on: Dec 08, 2019 22:22
 * Description:
 *
 * Requirements:
 *  1. Size of buffer must be a power of 2
 *  2. Maximum buffer size is 2^(number_of_bits_of(size_t))-1
 *
 * Implementation details:
 *
 * - Initial state (empty)
 * [0][1][2][3]...[N]
 *  ^
 * R/W
 *
 * - Add one element
 * [D][1][2][3]...[N]
 *  ^  ^
 *  R  W
 *
 * - Buffer gets full when last element X is inserted
 * [X][D][D][D]...[D]
 *     ^
 *    W/R (W>R)
 *
 * - Buffer data overwritten by new element Y after getting full
 * [X][Y][D][D]...[D]
 *        ^
 *       W/R (W>R)
 *
 * To differentiate between empty and full, one slot is always left empty before
 *  the read index. This is why the buffer size is N-1.
 *
 * Reference:
 *  [1]
 * https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
 *  [2]
 * https://stackoverflow.com/questions/10527581/why-must-a-ring-buffer-size-be-a-power-of-2
 *  [3]
 * https://stackoverflow.com/questions/9718116/improving-c-circular-buffer-efficiency
 *  [4] https://www.snellman.net/blog/archive/2016-12-13-ring-buffers/
 *  [5]
 * http://www.trytoprogram.com/c-examples/c-program-to-test-if-a-number-is-a-power-of-2/
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

#include <cstdint>
#include <cstddef>

#include <mutex>
#include <array>
#include <iostream>

namespace xmotion {
template <typename T = uint8_t, std::size_t N = 1024>
class RingBuffer {
 public:
  // Init and reset of buffer
  RingBuffer(bool enable_overwrite = false)
      : enable_overwrite_(enable_overwrite) {
    // assert size is a power of 2
    static_assert(
        (N != 0) && ((N & (N - 1)) == 0),
        "Size of ring buffer has to be 2^n, where n is a positive integer");

    size_mask_ = N - 1;
    write_index_ = 0;
    read_index_ = 0;
  }

  void Reset() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    write_index_ = 0;
    read_index_ = 0;
  }

  // Buffer size information
  bool IsEmpty() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return (read_index_ == write_index_);
  }

  bool IsFull() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return ((write_index_ + 1) & size_mask_) == (read_index_ & size_mask_);
  }

  std::size_t GetFreeSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return N - 1 - ((write_index_ - read_index_) & size_mask_);
  }

  std::size_t GetOccupiedSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return (write_index_ - read_index_) & size_mask_;
  };

  std::array<T, N> GetBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_;
  }

  // Read/Write functions
  std::size_t Read(T data[], std::size_t btr) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (std::size_t i = 0; i < btr; ++i) {
      if (ReadOne(&data[i]) == 0) return i;
    }
    return btr;
  }

  std::size_t Peek(T data[], std::size_t btp) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (std::size_t i = 0; i < btp; ++i) {
      if (PeekOneAt(&data[i], i) == 0) return i;
    }
    return btp;
  }

  std::size_t Write(const T new_data[], std::size_t btw) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (std::size_t i = 0; i < btw; ++i) {
      if (WriteOne(new_data[i]) == 0) return i;
    }
    return btw;
  }

  void PrintBuffer() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::cout << "read index: " << read_index_
              << " , write index: " << write_index_ << std::endl;
    std::cout << "buffer content: " << std::endl;
    for (std::size_t i = 0; i < N; ++i)
      std::cout << "[" << i << "]"
                << " " << static_cast<int>(buffer_[i]) << std::endl;
  }

 private:
  std::array<T, N> buffer_;      // buffer memory to store data
  std::size_t size_mask_;        // for internal indexing management
  std::size_t write_index_ = 0;  // place new data to be written
  std::size_t read_index_ = 0;   // place earliest written data to be read from
  bool enable_overwrite_ = false;  // enable buffer overwrite when full

  mutable std::mutex buffer_mutex_;

  std::size_t ReadOne(T *data) {
    if (read_index_ == write_index_) return 0;  // Buffer is empty
    *data = buffer_[read_index_++ & size_mask_];
    return 1;
  }

  std::size_t PeekOneAt(T *data, size_t n) const {
    // return 0 if requested data is beyond the available range
    if (n >= (write_index_ - read_index_) & size_mask_) return 0;
    *data = buffer_[(read_index_ + n) & size_mask_];
    return 1;
  }

  std::size_t WriteOne(const T &new_data) {
    if (IsFull()) {
      // return 0 if buffer is full and overwrite is disabled
      if (!enable_overwrite_) return 0;
      // otherwise, advance the read_index to overwrite old data
      read_index_ = (read_index_ + 1) & size_mask_;
    }

    buffer_[(write_index_++) & size_mask_] = new_data;
    return 1;
  }
};
}  // namespace xmotion

#endif /* RING_BUFFER_HPP */