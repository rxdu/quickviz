/*
 * data_plotter.hpp
 *
 * Created on: Dec 05, 2020 23:56
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef DATA_PLOTTER_HPP
#define DATA_PLOTTER_HPP

#include "implot/implot.h"
#include "canvas/im_canvas.hpp"
#include "plotter/data_buffer.hpp"

namespace rdu {
class DataPlotter : public ImCanvas {
 public:
  struct DataPoint {
    std::string line;
    float x;
    float y;
  };

 public:
  DataPlotter(uint32_t width = 960, uint32_t height = 640,
              std::string title = "Plotter")
      : ImCanvas(width, height, title){};

  void AddDataPoint(const DataPoint& dp);

 private:
  void Draw() override;
};
}  // namespace rdu

#endif /* DATA_PLOTTER_HPP */
