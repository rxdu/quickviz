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

namespace quickviz {
ScenePanel::ScenePanel(const std::string& panel_name) : GlWidget(panel_name) {
  this->SetNoMove(true);
  this->SetNoResize(true);
  this->SetNoTitleBar(true);
  this->SetNoBackground(true);

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  this->AddOpenGLObject("grid", std::move(grid));
}

void ScenePanel::Draw() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  // Orthographic projection for a top-down view
  float aspect_ratio =
      static_cast<float>(content_size.x) / static_cast<float>(content_size.y);
  //  std::cout << "aspect ratio: " << aspect_ratio << std::endl;
  glm::mat4 projection =
      glm::perspective(glm::radians(45.0f), aspect_ratio, 0.1f, 100.0f);

  // Simple view matrix looking at an angle
  glm::mat4 view = glm::lookAt(
      glm::vec3(10.0f, 10.0f, 10.0f),  // Camera positioned at an angle
      glm::vec3(0.0f, 0.0f, 0.0f),     // Looking at the origin
      glm::vec3(0.0f, 1.0f, 0.0f)      // Up vector pointing along the Y-axis
  );

  UpdateView(projection, view);

  GlWidget::Draw();
}
}  // namespace quickviz