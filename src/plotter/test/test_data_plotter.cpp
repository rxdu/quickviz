/*
 * test_data_plotter.cpp
 *
 * Created on: Dec 06, 2020 00:06
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include <math.h>

#include "plotter/data_plotter.hpp"

using namespace rdu;

struct ScrollingBuffer {
  int MaxSize;
  int Offset;
  ImVector<ImVec2> Data;
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
};

// utility structure for realtime plot
struct RollingBuffer {
  float Span;
  ImVector<ImVec2> Data;
  RollingBuffer() {
    Span = 10.0f;
    Data.reserve(2000);
  }
  void AddPoint(float x, float y) {
    float xmod = fmodf(x, Span);
    if (!Data.empty() && xmod < Data.back().x) Data.shrink(0);
    Data.push_back(ImVec2(xmod, y));
  }
};

struct RTPlotter : public DataPlotter {
  void Draw() override {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin("Cairo Canvas", NULL,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);

    ImGui::BulletText("Move your mouse to change the data!");
    ImGui::BulletText(
        "This example assumes 60 FPS. Higher FPS requires larger buffer size.");
    static ScrollingBuffer sdata1, sdata2;
    static RollingBuffer rdata1, rdata2;
    ImVec2 mouse = ImGui::GetMousePos();
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    sdata1.AddPoint(t, mouse.x * 0.0005f);
    rdata1.AddPoint(t, mouse.x * 0.0005f);
    sdata2.AddPoint(t, mouse.y * 0.0005f);
    rdata2.AddPoint(t, mouse.y * 0.0005f);

    static float history = 10.0f;
    ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");
    rdata1.Span = history;
    rdata2.Span = history;

    static ImPlotAxisFlags rt_axis = ImPlotAxisFlags_NoTickLabels;
    ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
    if (ImPlot::BeginPlot("##Scrolling", NULL, NULL, ImVec2(-1, 150), 0,
                          rt_axis, rt_axis | ImPlotAxisFlags_LockMin)) {
      ImPlot::PlotShaded("Data 1", &sdata1.Data[0].x, &sdata1.Data[0].y,
                         sdata1.Data.size(), 0, sdata1.Offset,
                         2 * sizeof(float));
      ImPlot::PlotLine("Data 2", &sdata2.Data[0].x, &sdata2.Data[0].y,
                       sdata2.Data.size(), sdata2.Offset, 2 * sizeof(float));
      ImPlot::EndPlot();
    }
    ImPlot::SetNextPlotLimitsX(0, history, ImGuiCond_Always);
    if (ImPlot::BeginPlot("##Rolling", NULL, NULL, ImVec2(-1, 150), 0, rt_axis,
                          rt_axis)) {
      ImPlot::PlotLine("Data 1", &rdata1.Data[0].x, &rdata1.Data[0].y,
                       rdata1.Data.size(), 0, 2 * sizeof(float));
      ImPlot::PlotLine("Data 2", &rdata2.Data[0].x, &rdata2.Data[0].y,
                       rdata2.Data.size(), 0, 2 * sizeof(float));
      ImPlot::EndPlot();
    }

    ImGui::End();
  }
};

int main(int argc, const char* argv[]) {
  RTPlotter plotter;
  plotter.Show();
  return 0;
}
