/*
 * @file data_stream.hpp
 * @brief Latest-only producer/consumer channel for streaming data
 *
 * `DataStream<T>` is the recommended way to bring sensor data, robot poses,
 * point clouds, or any other periodically-updated value from a background
 * thread to the render thread. It exposes a tiny stream-shaped API on top
 * of the existing `DoubleBuffer<T>` primitive:
 *
 *   - Producer thread calls `Push(value)` whenever a new sample arrives.
 *   - Render thread calls `TryPull()` once per frame and updates the
 *     renderable if a fresh value is available.
 *
 * Semantics: "latest-only, lossy." The consumer always sees the most
 * recent value; intermediate pushes between two pulls are silently
 * dropped. This is what visualization usually wants — yesterday's frame
 * is not interesting if today's is already in hand.
 *
 * The render thread never blocks. Push and Pull are thread-safe.
 *
 * For lossless delivery (e.g. accumulating every sample for a plot),
 * use `RingBuffer<T>` directly instead.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CORE_DATA_STREAM_HPP
#define QUICKVIZ_CORE_DATA_STREAM_HPP

#include <optional>
#include <utility>

#include "core/buffer/double_buffer.hpp"

namespace quickviz {

template <typename T>
class DataStream {
 public:
  DataStream() = default;
  ~DataStream() = default;

  // Non-copyable; the underlying buffer holds mutex/condvar state.
  // Movable for placement in containers if useful.
  DataStream(const DataStream&) = delete;
  DataStream& operator=(const DataStream&) = delete;

  /**
   * @brief Publish a new value. Replaces any unread previous value.
   *
   * Safe to call from any thread, including concurrently with `TryPull`.
   * The previous unread value (if any) is discarded — only the most
   * recent push is retained. This is the lossy semantics that streaming
   * visualization expects.
   */
  void Push(const T& value) { buffer_.Write(value); }
  void Push(T&& value) { buffer_.Write(std::move(value)); }

  /**
   * @brief Consume the latest value, if a fresh one is available.
   *
   * @param out Receives the latest value when this call returns true.
   *            Untouched otherwise.
   * @return true if a fresh value was available and copied into `out`;
   *         false if no new value has been pushed since the last
   *         successful pull.
   *
   * Non-blocking. Safe to call once per frame on the render thread.
   */
  bool TryPull(T& out) { return buffer_.TryRead(out); }

  /**
   * @brief Same as `TryPull(T&)` but returns the value by std::optional.
   *
   * Convenient for `if (auto v = stream.TryPull()) { ... }` patterns.
   * Slightly less efficient when T is large because it copies into
   * `optional`'s storage; for hot paths prefer the out-param overload.
   */
  std::optional<T> TryPull() {
    T value;
    if (!buffer_.TryRead(value)) return std::nullopt;
    return value;
  }

 private:
  DoubleBuffer<T> buffer_;
};

}  // namespace quickviz

#endif  // QUICKVIZ_CORE_DATA_STREAM_HPP
