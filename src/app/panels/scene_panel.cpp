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
    std::cerr << "ScenePanel is hovered" << std::endl;
    AppLogHandler::GetInstance().Log(LogLevel::kInfo, "ScenePanel is hovered");
  }

  UpdateView(projection, view);

  GlWidget::Draw();
}
}  // namespace quickviz