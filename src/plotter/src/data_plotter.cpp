/*
 * data_plotter.cpp
 *
 * Created on: Dec 05, 2020 23:56
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#include "plotter/data_plotter.hpp"

namespace rdu {
void DataPlotter::Draw() {
  /**********************************************************************/
  // calculate window sizes
  /**********************************************************************/
  auto display_size = ImGui::GetIO().DisplaySize;

  ImVec2 setting_area_pos = ImVec2(0, 0);
  float setting_area_width = display_size[0];
  float setting_area_height = display_size[1] * 2.0 / 5.0;

  ImVec2 plot_area_pos = ImVec2(0, 19);
  float plot_area_width = display_size[0];
  float plot_area_height = display_size[1] - plot_area_pos[1];

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_ScrollbarRounding, 0);

  /**********************************************************************/
  // show settings area
  /**********************************************************************/
  static float history = 10.0f;
  static bool show_center_line, use_jetcolor_bg, centered_at_ego_vehicle,
      zoom_in_ratio, save_image;

  ImGui::SetNextWindowPos(setting_area_pos, 0, ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(setting_area_width, setting_area_height));

  ImGui::Begin("Settings");

  ImGui::PushItemWidth(setting_area_width - 58);
  ImGui::SliderFloat("History", &history, 1, 60, "%.1f s");
  ImGui::PopItemWidth();

//   ImGui::BulletText("Move your mouse to change the data!");
//   ImGui::BulletText(
//       "This example assumes 60 FPS. Higher FPS requires larger buffer size.");

  ImGui::Text("Road Map:");
  ImGui::Checkbox("Show Centerline", &show_center_line);
  ImGui::Checkbox("Jetcolor Map", &use_jetcolor_bg);

  ImGui::Text("Traffic:");
  ImGui::Checkbox("Centered at Ego Vehicle", &centered_at_ego_vehicle);
  //   ImGui::SliderFloat("Zoom-in Ratio", &zoom_in_ratio, 0.0f, 1.0f);
  ImGui::Text("Save Result:");
  static char img_name_buf[32] = "traffic_viewer";
  ImGui::InputText("", img_name_buf, IM_ARRAYSIZE(img_name_buf));
  ImGui::SameLine();
  if (ImGui::Button("Save Image")) save_image = true;
  // ImGui::SameLine();

  ImGui::Text("Application average frame rate: %.1f FPS (%.3f ms/frame)",
              ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);

  ImGui::End();

  /**********************************************************************/
  // show plotting area
  /**********************************************************************/
  ImGui::SetNextWindowPos(plot_area_pos);
  ImGui::SetNextWindowSize(ImVec2(plot_area_width, plot_area_height));

  ImGui::Begin("Realtime Plotter", NULL,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoScrollbar);

  static ScrollingBuffer sdata1, sdata2;
  ImVec2 mouse = ImGui::GetMousePos();
  static float t = 0;
  t += ImGui::GetIO().DeltaTime;
  sdata1.AddPoint(t, mouse.x * 0.0005f);
  sdata2.AddPoint(t, mouse.y * 0.0005f);

  static ImPlotAxisFlags rt_axis =
      ImPlotAxisFlags_None;  // ImPlotAxisFlags_NoTickLabels;
  ImPlot::SetNextPlotLimitsX(t - history, t, ImGuiCond_Always);
  if (ImPlot::BeginPlot("##Scrolling", NULL, NULL,
                        ImVec2(plot_area_width, plot_area_height), 0, rt_axis,
                        rt_axis | ImPlotAxisFlags_LockMin)) {
    ImPlot::PlotLine("Data 1", &sdata1.Data[0].x, &sdata1.Data[0].y,
                     sdata1.Data.size(), sdata1.Offset, 2 * sizeof(float));
    ImPlot::PlotLine("Data 2", &sdata2.Data[0].x, &sdata2.Data[0].y,
                     sdata2.Data.size(), sdata2.Offset, 2 * sizeof(float));
    ImPlot::EndPlot();
  }

  ImGui::End();
}
}  // namespace rdu