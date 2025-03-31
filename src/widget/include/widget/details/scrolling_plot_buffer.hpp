/*
 * scrolling_plot_buffer.hpp
 *
 * Created on: Mar 25, 2021 14:29
 * Description: adapted from class ScrollingBuffer from implot_demo.cpp
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef SCROLLING_PLOT_BUFFER_HPP
#define SCROLLING_PLOT_BUFFER_HPP

#include <cstdint>
#include <mutex>
#include <vector>

#include "imgui.h"

namespace quickviz {
class ScrollingPlotBuffer {
 public:
  ScrollingPlotBuffer(uint32_t size = 2048);

  void Resize(uint32_t size);
  std::size_t GetSize() const;
  std::size_t GetOffset() const;

  void Erase();
  void AddPoint(float x, float y);
  ImVector<ImVec2>& GetData() { return data_; }
  ImVec2& operator[](std::size_t index);

 private:
  ImVector<ImVec2> data_;
  uint32_t buffer_size_ = 0;
  uint32_t offset_ = 0;
};
}  // namespace quickviz

#endif /* SCROLLING_PLOT_BUFFER_HPP */
