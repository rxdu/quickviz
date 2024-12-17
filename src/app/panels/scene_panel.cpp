/*
 * @file scene_panel.cpp
 * @date 11/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/scene_panel.hpp"

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "imview/component/logging/app_log_handler.hpp"

namespace quickviz {
ScenePanel::ScenePanel(const std::string& panel_name) : GlWidget(panel_name) {
  this->SetNoMove(true);
  this->SetNoResize(true);
  this->SetNoTitleBar(true);
  this->SetNoBackground(true);

  camera_ =
      std::make_unique<Camera>(glm::vec3(0.0f, 3.0f, 8.0f), -90.0f, -25.0f);

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  this->AddOpenGLObject("grid", std::move(grid));
}

void ScenePanel::Draw() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // get view matrices from camera
  float aspect_ratio =
      static_cast<float>(content_size.x) / static_cast<float>(content_size.y);
  glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio);
  glm::mat4 view = camera_->GetViewMatrix();

  if (ImGui::IsWindowHovered()) {
    AppLogHandler::GetInstance().Log(LogLevel::kInfo, "ScenePanel is hovered");
  }

  ImGuiIO& io = ImGui::GetIO();
  ImVec2 windowPos = ImGui::GetWindowPos();

  ImVec2 localMousePos =
      ImVec2(io.MousePos.x - windowPos.x, io.MousePos.y - windowPos.y);

  if (io.WantCaptureMouse) {
    if (ImGui::IsMousePosValid()) {
      AppLogHandler::GetInstance().Log(LogLevel::kInfo, "Mouse pos: (%f, %f)",
                                       io.MousePos.x, io.MousePos.y);
    } else {
      AppLogHandler::GetInstance().Log(LogLevel::kInfo, "Mouse pos: <INVALID>");
    }

    for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++)
      if (ImGui::IsMouseDown(i)) {
        ImGui::SameLine();
        // ImGui::Text("b%d (%.02f secs)", i, io.MouseDownDuration[i]);
        AppLogHandler::GetInstance().Log(LogLevel::kInfo,
                                         "Mouse button %d down", i);
      }
  }

  UpdateView(projection, view);

  GlWidget::Draw();
}
}  // namespace quickviz