/*
 * plot_buffer.hpp
 *
 * Created on: Mar 25, 2021 14:29
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef PLOT_BUFFER_HPP
#define PLOT_BUFFER_HPP

#include <cstdint>
#include <mutex>

#include "imgui.h"

namespace rdu {
class PlotBuffer {
 public:
  enum class Type { Scrolling, Rolling };
  constexpr static uint32_t default_size = 2048;

 public:
  PlotBuffer(Type type = Type::Scrolling,
             uint32_t size = PlotBuffer::default_size);

  void Clear();
  void Resize(uint32_t size);

  void AddPoint(float x, float y);
  ImVec2 operator[](uint32_t idx);

  void SetRollingSpan(float span);

 private:
  Type type_;

  std::mutex data_mutex_;
  ImVector<ImVec2> data_;

  uint32_t buffer_size_ = 0;
  uint32_t index_ = 0;
  float rolling_span_ = 10.0f;
};
}  // namespace rdu

#endif /* PLOT_BUFFER_HPP */
