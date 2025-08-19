/**
 * @file test_coordinate_system.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-17
 * @brief Test for the coordinate system transformation
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

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/coordinate_system_transformer.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create an OpenGL scene manager with coordinate system transformation enabled
  auto gl_sm_transformed = std::make_shared<GlSceneManager>("Standard Coordinate System (Z-up)",
                                                GlSceneManager::Mode::k3D);
  gl_sm_transformed->SetAutoLayout(true);
  gl_sm_transformed->SetNoTitleBar(true);
  gl_sm_transformed->SetFlexGrow(0.5f);
  gl_sm_transformed->SetFlexShrink(0.0f);
  gl_sm_transformed->EnableCoordinateSystemTransformation(true);

  // Add a grid for reference
  auto grid_transformed = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_transformed->AddOpenGLObject("grid", std::move(grid_transformed));

  // Add a coordinate frame
  auto coord_frame_transformed = std::make_unique<CoordinateFrame>(3.0f, false);
  gl_sm_transformed->AddOpenGLObject("coordinate_frame", std::move(coord_frame_transformed));

  // Create a second OpenGL scene manager with coordinate system transformation disabled
  auto gl_sm_native = std::make_shared<GlSceneManager>("OpenGL Native (Y-up)",
                                                GlSceneManager::Mode::k3D);
  gl_sm_native->SetAutoLayout(true);
  gl_sm_native->SetNoTitleBar(true);
  gl_sm_native->SetFlexGrow(0.5f);
  gl_sm_native->SetFlexShrink(0.0f);
  gl_sm_native->EnableCoordinateSystemTransformation(false);

  // Add a grid for reference
  auto grid_native = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_native->AddOpenGLObject("grid", std::move(grid_native));

  // Add a coordinate frame
  auto coord_frame_native = std::make_unique<CoordinateFrame>(3.0f, false);
  gl_sm_native->AddOpenGLObject("coordinate_frame", std::move(coord_frame_native));

  // Add some objects at specific positions to demonstrate the transformation
  // In the transformed scene, we place objects using standard Z-up coordinates
  auto coord_frame_z_up = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_z_up->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f)); // X-axis
  gl_sm_transformed->AddOpenGLObject("coord_frame_x", std::move(coord_frame_z_up));

  auto coord_frame_z_up2 = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_z_up2->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f)); // Y-axis
  gl_sm_transformed->AddOpenGLObject("coord_frame_y", std::move(coord_frame_z_up2));

  auto coord_frame_z_up3 = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_z_up3->SetPosition(glm::vec3(0.0f, 0.0f, 3.0f)); // Z-axis
  gl_sm_transformed->AddOpenGLObject("coord_frame_z", std::move(coord_frame_z_up3));

  // In the native scene, we place objects using OpenGL Y-up coordinates
  auto coord_frame_y_up = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_y_up->SetPosition(glm::vec3(3.0f, 0.0f, 0.0f)); // X-axis
  gl_sm_native->AddOpenGLObject("coord_frame_x", std::move(coord_frame_y_up));

  auto coord_frame_y_up2 = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_y_up2->SetPosition(glm::vec3(0.0f, 3.0f, 0.0f)); // Y-axis (up in OpenGL)
  gl_sm_native->AddOpenGLObject("coord_frame_y", std::move(coord_frame_y_up2));

  auto coord_frame_y_up3 = std::make_unique<CoordinateFrame>(1.0f, false);
  coord_frame_y_up3->SetPosition(glm::vec3(0.0f, 0.0f, 3.0f)); // Z-axis (forward in OpenGL)
  gl_sm_native->AddOpenGLObject("coord_frame_z", std::move(coord_frame_y_up3));

  // Add the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm_transformed);
  box->AddChild(gl_sm_native);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
} 