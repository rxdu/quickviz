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
ScenePanel::ScenePanel(const std::string& panel_name)
    : GlSceneManager(panel_name) {
  this->SetNoMove(true);
  this->SetNoResize(true);
  this->SetNoTitleBar(true);
  this->SetNoBackground(true);

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  this->AddOpenGLObject("grid", std::move(grid));
}
}  // namespace quickviz