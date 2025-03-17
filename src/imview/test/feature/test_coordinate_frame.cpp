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
#include <glm/gtc/quaternion.hpp>

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

  // Create an OpenGL scene manager for 3D mode
  auto gl_sm_3d = std::make_shared<GlSceneManager>("3D Mode (Z up)",
                                                GlSceneManager::Mode::k3D);
  gl_sm_3d->SetAutoLayout(true);
  gl_sm_3d->SetNoTitleBar(true);
  gl_sm_3d->SetFlexGrow(0.5f);
  gl_sm_3d->SetFlexShrink(0.0f);

  // Add a grid for reference
  auto grid_3d = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_3d->AddOpenGLObject("grid", std::move(grid_3d));

  // Add a coordinate frame in 3D mode
  auto coord_frame_3d = std::make_unique<CoordinateFrame>(3.0f, false);
  gl_sm_3d->AddOpenGLObject("coordinate_frame", std::move(coord_frame_3d));

  // Create a second OpenGL scene manager for 2D mode
  auto gl_sm_2d = std::make_shared<GlSceneManager>("2D Mode (X-Z plane)",
                                                GlSceneManager::Mode::k2D);
  gl_sm_2d->SetAutoLayout(true);
  gl_sm_2d->SetNoTitleBar(true);
  gl_sm_2d->SetFlexGrow(0.5f);
  gl_sm_2d->SetFlexShrink(0.0f);

  // Add a grid for reference
  auto grid_2d = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_2d->AddOpenGLObject("grid", std::move(grid_2d));

  // Add a coordinate frame in 2D mode
  auto coord_frame_2d = std::make_unique<CoordinateFrame>(3.0f, true);
  gl_sm_2d->AddOpenGLObject("coordinate_frame", std::move(coord_frame_2d));

  // Add additional coordinate frames with different orientations to demonstrate the transformation
  // Create a coordinate frame rotated 45 degrees around Y axis
  auto coord_frame_rotated = std::make_unique<CoordinateFrame>(2.0f, false);
  glm::quat rotation_y = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  coord_frame_rotated->SetPose(glm::vec3(5.0f, 0.0f, 5.0f), rotation_y);
  gl_sm_3d->AddOpenGLObject("coord_frame_rotated_y", std::move(coord_frame_rotated));

  // Create a coordinate frame rotated 45 degrees around X axis
  auto coord_frame_rotated_x = std::make_unique<CoordinateFrame>(2.0f, false);
  glm::quat rotation_x = glm::angleAxis(glm::radians(45.0f), glm::vec3(1.0f, 0.0f, 0.0f));
  coord_frame_rotated_x->SetPose(glm::vec3(-5.0f, 0.0f, 5.0f), rotation_x);
  gl_sm_3d->AddOpenGLObject("coord_frame_rotated_x", std::move(coord_frame_rotated_x));

  // Create a coordinate frame rotated 45 degrees around Z axis
  auto coord_frame_rotated_z = std::make_unique<CoordinateFrame>(2.0f, false);
  glm::quat rotation_z = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  coord_frame_rotated_z->SetPose(glm::vec3(0.0f, 0.0f, -5.0f), rotation_z);
  gl_sm_3d->AddOpenGLObject("coord_frame_rotated_z", std::move(coord_frame_rotated_z));

  // Add the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm_3d);
  box->AddChild(gl_sm_2d);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}
