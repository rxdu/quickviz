/*
 * @file rt_line_plot_widget.hpp
 * @date 10/26/24
 * @brief this LinePlotWidget serves as an example of how to create a custom
 * widget that plots data in real-time
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_RT_LINE_PLOT_WIDGET_HPP
#define QUICKVIZ_RT_LINE_PLOT_WIDGET_HPP

#include <unordered_map>

#include "core/buffer/buffer_registry.hpp"

#include "imview/panel.hpp"

#include "widget/details/scrolling_plot_buffer.hpp"

namespace quickviz {
class RtLinePlotWidget : public Panel {
 public:
  struct DataPoint {
    float x;  // time
    float y;  // value
  };

 public:
  RtLinePlotWidget(const std::string& widget_name);
  ~RtLinePlotWidget() = default;

  // public methods
  void SetPlotTransparency(float alpha);
  void SetAxisLabels(const std::string& x_label, const std::string& y_label);
  void SetAxisUnits(const std::string& x_unit, const std::string& y_unit);
  void SetFixedHistory(float history);
  void SetYAxisRange(float min, float max);
  void AddLine(const std::string& line_name, const std::string& buffer_name);

  // for internal use
  void Draw() override;

 private:
  struct PlotSpecs {
    std::string name;
    ScrollingPlotBuffer internal_buffer;
    std::shared_ptr<BufferInterface<DataPoint>> input_buffer;
  };

  float alpha_ = 1.0f;
  std::string x_label_;
  std::string y_label_;
  std::string x_unit_;
  std::string y_unit_;
  float t_ = 0;
  bool fixed_history_ = true;
  float history_ = 10.0f;
  float y_min_ = 0;
  float y_max_ = 1;
  std::unordered_map<std::string, PlotSpecs> line_specs_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RT_LINE_PLOT_WIDGET_HPP