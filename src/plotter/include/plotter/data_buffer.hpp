/*
 * data_buffer.hpp
 *
 * Created on: Dec 06, 2020 20:33
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef DATA_BUFFER_HPP
#define DATA_BUFFER_HPP

#include <cmath>

#include "implot/implot.h"

namespace rdu {
class ScrollingBuffer {
 public:
  ScrollingBuffer() {
    MaxSize = 2000;
    Offset = 0;
    Data.reserve(MaxSize);
  }
  void AddPoint(float x, float y) {
    if (Data.size() < MaxSize)
      Data.push_back(ImVec2(x, y));
    else {
      Data[Offset] = ImVec2(x, y);
      Offset = (Offset + 1) % MaxSize;
    }
  }
  void Erase() {
    if (Data.size() > 0) {
      Data.shrink(0);
      Offset = 0;
    }
  }

  int MaxSize;
  int Offset;
  ImVector<ImVec2> Data;
};
}  // namespace rdu

#endif /* DATA_BUFFER_HPP */
