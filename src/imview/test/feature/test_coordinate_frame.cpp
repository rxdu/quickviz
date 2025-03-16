/**
 * @file test_coordinate_frame.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief Test for the CoordinateFrame class
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <thread>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/component/opengl/renderer/grid.hpp"
#include "imview/component/opengl/renderer/coordinate_frame.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create an OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene",
                                                GlSceneManager::Mode::k3D);
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // Add a grid for reference
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a coordinate frame 
  auto coord_frame = std::make_unique<CoordinateFrame>(1.0f, false);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  // Add the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}
