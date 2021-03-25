/*
 * plot_buffer.cpp
 *
 * Created on: Mar 25, 2021 17:39
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "canvas/plot_buffer.hpp"

#include <cmath>

namespace rdu {
PlotBuffer::PlotBuffer(Type type, uint32_t size)
    : type_(type), buffer_size_(size) {
  data_.reserve(size);
}

void PlotBuffer::SetRollingSpan(float span) { rolling_span_ = span; }

void PlotBuffer::Clear() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.clear();
  data_.reserve(buffer_size_);
  index_ = 0;
}

void PlotBuffer::Resize(uint32_t size) {
  Clear();

  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.reserve(size);
  buffer_size_ = size;
}

void PlotBuffer::AddPoint(float x, float y) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (type_ == Type::Scrolling) {
    if (data_.size() < buffer_size_)
      data_.push_back(ImVec2(x, y));
    else {
      data_[index_] = ImVec2(x, y);
      index_ = (index_ + 1) % buffer_size_;
    }
  } else {
    float xmod = fmodf(x, rolling_span_);
    if (!data_.empty() && xmod < data_.back().x) Clear();
    data_.push_back(ImVec2(xmod, y));
  }
}

ImVec2 PlotBuffer::operator[](uint32_t idx) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return data_[idx];
}
}  // namespace rdu