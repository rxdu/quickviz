/**
 * @file test_gl_scene_manager.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-06
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/triangle.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene panel to manage the OpenGL objects
  auto panel = std::make_shared<GlScenePanel>("OpenGL Scene");
  panel->SetAutoLayout(true);
  panel->SetNoTitleBar(true);
  panel->SetFlexGrow(1.0f);
  panel->SetFlexShrink(0.0f);

  // now add the rendering objects to the OpenGL scene manager
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  panel->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  panel->AddOpenGLObject("grid", std::move(grid));

  // finally pass the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(panel);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}