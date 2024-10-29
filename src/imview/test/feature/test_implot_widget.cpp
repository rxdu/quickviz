/*
 * test_cairo_widget.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

#include "imview/buffer/buffer_registry.hpp"
#include "imview/buffer/ring_buffer.hpp"

#include "imview/viewer.hpp"
#include "imview/widget/rt_line_plot_widget.hpp"
#include "scene_objects/gl_triangle_scene_object.hpp"

using namespace quickviz;

bool keep_running = true;

std::string pt_buffer_name = "data_buffer";
std::string pt_buffer_sin_name = "sin_data_buffer";

void DataGenerator() {
  auto& buffer_registry = BufferRegistry::GetInstance();
  auto pt_buffer =
      buffer_registry.GetBuffer<RtLinePlotWidget::DataPoint>(pt_buffer_name);
  auto pt_buffer_sin = buffer_registry.GetBuffer<RtLinePlotWidget::DataPoint>(
      pt_buffer_sin_name);

  static float t = 0;
  int period_ms = 1000 / 60;
  while (keep_running) {
    ImVec2 mouse = ImGui::GetMousePos();
    t += period_ms / 1000.0;

    RtLinePlotWidget::DataPoint pt;
    {
      pt.x = t;
      pt.y = mouse.x * 0.0005f;
      pt_buffer->Write(pt);
    }
    {
      pt.x = t;
      pt.y = 0.5f + 0.5f * std::sin(2 * 3.1415926 * t);
      pt_buffer_sin->Write(pt);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

int main(int argc, char* argv[]) {
  // set up buffer first
  auto& buffer_registry = BufferRegistry::GetInstance();
  std::shared_ptr<BufferInterface<RtLinePlotWidget::DataPoint>> pt_buffer =
      std::make_shared<RingBuffer<RtLinePlotWidget::DataPoint, 8>>();
  std::shared_ptr<BufferInterface<RtLinePlotWidget::DataPoint>> pt_buffer_sin =
      std::make_shared<RingBuffer<RtLinePlotWidget::DataPoint, 8>>();
  buffer_registry.AddBuffer(pt_buffer_name, pt_buffer);
  buffer_registry.AddBuffer(pt_buffer_sin_name, pt_buffer_sin);

  // set up consumer
  // note: viewer needs to be created before the data thread because this demo
  // uses ImGui::GetMousePos() to generate data
  Viewer viewer;

  // set up producer
  std::thread data_thread(DataGenerator);

  // used as background for transparency test
  auto gl_triangle = std::make_shared<GLTriangleSceneObject>();
  viewer.AddSceneObject(gl_triangle);

  auto widget = std::make_shared<RtLinePlotWidget>("line_plot");
  widget->OnResize(300, 200);
  widget->SetAxisLabels("x-axis", "y-axis");
  widget->SetAxisUnits("s", "m");
  widget->SetPlotTransparency(0.5);
  widget->SetFixedHistory(5);
  widget->SetYAxisRange(-0.5, 1.5);
  widget->AddLine("line1", pt_buffer_name);
  widget->AddLine("line2", pt_buffer_sin_name);
  viewer.AddSceneObject(widget);
  viewer.Show();

  keep_running = false;
  data_thread.join();

  return 0;
}