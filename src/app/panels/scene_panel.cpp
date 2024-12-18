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

  camera_ = std::make_unique<Camera>();
  camera_controller_ = std::make_unique<CameraController>(
      *camera_, glm::vec3(0.0f, 6.0f, 8.0f), 0.0f, 25.0f);

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  this->AddOpenGLObject("grid", std::move(grid));
}

void ScenePanel::Draw() {
  Begin();

  // update view according to user input
  ImGuiIO& io = ImGui::GetIO();
  // only process mouse delta when mouse position is within the scene panel
  if (ImGui::IsMousePosValid() && io.WantCaptureMouse &&
      ImGui::IsWindowHovered()) {
    // track mouse move delta only when the mouse left button is pressed
    if (ImGui::IsMouseDown(MouseButton::kLeft)) {
      camera_controller_->ProcessMouseMovement(io.MouseDelta.x,
                                               io.MouseDelta.y);
    }

    // track mouse wheel scroll
    camera_controller_->ProcessMouseScroll(io.MouseWheel);
  }

  // get view matrices from camera
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  float aspect_ratio =
      static_cast<float>(content_size.x) / static_cast<float>(content_size.y);
  glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio);
  glm::mat4 view = camera_->GetViewMatrix();
  UpdateView(projection, view);

  // finally draw the scene
  DrawOpenGLObject();

  End();
}
}  // namespace quickviz