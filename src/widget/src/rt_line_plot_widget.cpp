/*
 * @file line_plot_widget.cpp
 * @date 10/26/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "widget/rt_line_plot_widget.hpp"

#include <cmath>
#include <iostream>

#include "implot/implot.h"

namespace quickviz {
RtLinePlotWidget::RtLinePlotWidget(const std::string& widget_name)
    : Panel(widget_name) {
  this->SetAutoLayout(false);
  this->SetWindowNoMenuButton();
  this->SetNoBackground(true);
}

void RtLinePlotWidget::SetPlotTransparency(float alpha) { alpha_ = alpha; }

void RtLinePlotWidget::SetAxisLabels(const std::string& x_label,
                                     const std::string& y_label) {
  x_label_ = x_label;
  y_label_ = y_label;
}

void RtLinePlotWidget::SetAxisUnits(const std::string& x_unit,
                                    const std::string& y_unit) {
  x_unit_ = x_unit;
  y_unit_ = y_unit;
}

void RtLinePlotWidget::SetFixedHistory(float history) {
  fixed_history_ = true;
  history_ = history;
}

void RtLinePlotWidget::SetYAxisRange(float min, float max) {
  y_min_ = min;
  y_max_ = max;
}

void RtLinePlotWidget::AddLine(const std::string& line_name,
                               const std::string& buffer_name) {
  line_specs_[line_name].name = line_name;
  line_specs_[line_name].internal_buffer = ScrollingPlotBuffer();
  auto& buffer_registry = BufferRegistry::GetInstance();
  line_specs_[line_name].input_buffer =
      buffer_registry.GetBuffer<DataPoint>(buffer_name);
}

void RtLinePlotWidget::Draw() {
  Begin();
  {
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    float width = contentSize.x;
    float height = contentSize.y;

    // fetch data from buffer and find the latest time
    for (auto& line : line_specs_) {
      auto& spec = line.second;
      auto size = spec.input_buffer->GetOccupiedSize();
      for (int i = 0; i < size; i++) {
        DataPoint pt;
        if (spec.input_buffer->Read(pt) != 0) {
          if (pt.x > t_) t_ = pt.x;
          spec.internal_buffer.AddPoint(pt.x, pt.y);
        }
      }
    }

    static ImPlotAxisFlags flags =
        ImPlotAxisFlags_None;  // ImPlotAxisFlags_NoTickLabels;

    auto frame_bg = ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
    auto plot_bg = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
    auto popup_bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
    frame_bg.w = alpha_;
    plot_bg.w = alpha_;
    popup_bg.w = alpha_;
    ImPlot::PushStyleColor(ImPlotCol_FrameBg, frame_bg);
    ImPlot::PushStyleColor(ImPlotCol_PlotBg, plot_bg);
    ImPlot::PushStyleColor(ImPlotCol_LegendBg, popup_bg);
    {
      float plot_height = height - ImGui::GetFrameHeight() * 1.4;
      if (fixed_history_) plot_height = height;
      if (ImPlot::BeginPlot(("##" + GetName()).c_str(),
                            ImVec2(-1, plot_height))) {
        char* x_label = nullptr;
        char* y_label = nullptr;
        if (!x_label_.empty()) x_label = &x_label_[0];
        if (!y_label_.empty()) y_label = &y_label_[0];
        ImPlot::SetupAxes(x_label, y_label, flags, flags);
        if (!x_unit_.empty())
          ImPlot::SetupAxisFormat(ImAxis_X1, ("%g " + x_unit_).c_str());
        if (!y_unit_.empty())
          ImPlot::SetupAxisFormat(ImAxis_Y1, ("%g " + y_unit_).c_str());
        ImPlot::SetupAxisLimits(ImAxis_X1, t_ - history_, t_, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, y_min_, y_max_);

        for (auto& line : line_specs_) {
          auto& spec = line.second;
          if (spec.internal_buffer.GetSize() == 0) continue;
          ImPlot::PlotLine(spec.name.c_str(), &spec.internal_buffer[0].x,
                           &spec.internal_buffer[0].y,
                           spec.internal_buffer.GetSize(), 0,
                           spec.internal_buffer.GetOffset(), 2 * sizeof(float));
        }
        ImPlot::EndPlot();
      }

      if (!fixed_history_) {
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 60);
        ImGui::SliderFloat("History", &history_, 1, 30, "%.1f s");
      }
    }
    ImPlot::PopStyleColor(3);
  }
  End();
}
}  // namespace quickviz