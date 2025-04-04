/*
 * scrolling_plot_buffer.cpp
 *
 * Created on: Mar 25, 2021 17:39
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "widget/details/scrolling_plot_buffer.hpp"

#include <cmath>

namespace quickviz {
ScrollingPlotBuffer::ScrollingPlotBuffer(uint32_t size) : buffer_size_(size) {
  data_.reserve(size);
}

void ScrollingPlotBuffer::Resize(uint32_t size) {
  data_.reserve(size);
  buffer_size_ = size;
}

std::size_t ScrollingPlotBuffer::GetSize() const { return data_.size(); }

std::size_t ScrollingPlotBuffer::GetOffset() const { return offset_; }

void ScrollingPlotBuffer::Erase() {
  if (data_.size() > 0) {
    data_.shrink(0);
    offset_ = 0;
  }
}

void ScrollingPlotBuffer::AddPoint(float x, float y) {
  if (data_.size() < buffer_size_)
    data_.push_back(ImVec2(x, y));
  else {
    data_[offset_] = ImVec2(x, y);
    offset_ = (offset_ + 1) % buffer_size_;
  }
}

ImVec2 &ScrollingPlotBuffer::operator[](std::size_t index) {
  assert(index < data_.size());
  return data_[index];
}
}  // namespace quickviz