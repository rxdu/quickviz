/**
 * @file test_robot_frames.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief Test for coordinate frames with custom poses
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <iostream>
#include <vector>
#include <memory>
#include <string>

#include "imview/box.hpp"
#include "imview/viewer.hpp"

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/coordinate_frame.hpp"

using namespace quickviz;

// Simple robot link structure
struct RobotLink {
  std::string name;
  glm::vec3 position;
  glm::quat orientation;
  float frame_size;
  bool is_2d;
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  // Create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // Create a 3D OpenGL scene manager
  auto gl_sm_3d = std::make_shared<GlSceneManager>("3D Robot Frames",
                                                GlSceneManager::Mode::k3D);
  gl_sm_3d->SetAutoLayout(true);
  gl_sm_3d->SetFlexGrow(1.0f);
  gl_sm_3d->SetFlexShrink(0.0f);

  // Create a 2D OpenGL scene manager
  auto gl_sm_2d = std::make_shared<GlSceneManager>("2D Robot Frames",
                                                GlSceneManager::Mode::k2D);
  gl_sm_2d->SetAutoLayout(true);
  gl_sm_2d->SetFlexGrow(1.0f);
  gl_sm_2d->SetFlexShrink(0.0f);

  // Add a grid to the 3D scene
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_3d->AddOpenGLObject("grid", std::move(grid));

  // Add a world coordinate frame to both scenes
  auto world_frame_3d = std::make_unique<CoordinateFrame>(2.0f, false);
  gl_sm_3d->AddOpenGLObject("world_frame", std::move(world_frame_3d));

  auto world_frame_2d = std::make_unique<CoordinateFrame>(2.0f, true);
  gl_sm_2d->AddOpenGLObject("world_frame", std::move(world_frame_2d));

  // Define robot links with their poses
  std::vector<RobotLink> robot_links = {
    // 3D links
    {"base_link", glm::vec3(0.0f, 0.0f, 0.5f), 
     glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f, false},
    
    {"link1", glm::vec3(1.0f, 0.0f, 0.5f), 
     glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.8f, false},
    
    {"link2", glm::vec3(2.0f, 1.0f, 0.5f), 
     glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.6f, false},
    
    {"link3", glm::vec3(2.0f, 2.0f, 0.5f), 
     glm::angleAxis(glm::radians(135.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.4f, false},
    
    {"end_effector", glm::vec3(1.5f, 2.5f, 0.5f), 
     glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.3f, false},
    
    // 2D links (same positions but with 2D frames)
    {"base_link_2d", glm::vec3(0.0f, 0.0f, 0.0f), 
     glm::quat(1.0f, 0.0f, 0.0f, 0.0f), 1.0f, true},
    
    {"link1_2d", glm::vec3(1.0f, 0.0f, 0.0f), 
     glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.8f, true},
    
    {"link2_2d", glm::vec3(2.0f, 1.0f, 0.0f), 
     glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.6f, true},
    
    {"link3_2d", glm::vec3(2.0f, 2.0f, 0.0f), 
     glm::angleAxis(glm::radians(135.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.4f, true},
    
    {"end_effector_2d", glm::vec3(1.5f, 2.5f, 0.0f), 
     glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f)), 0.3f, true}
  };

  // Add coordinate frames for each robot link
  for (const auto& link : robot_links) {
    auto frame = std::make_unique<CoordinateFrame>(link.frame_size, link.is_2d);
    frame->SetPose(link.position, link.orientation);
    
    if (link.is_2d) {
      gl_sm_2d->AddOpenGLObject(link.name, std::move(frame));
    } else {
      gl_sm_3d->AddOpenGLObject(link.name, std::move(frame));
    }
  }

  // Add both scene managers to the box
  box->AddChild(gl_sm_3d);
  box->AddChild(gl_sm_2d);
  
  // Add the box to the viewer
  viewer.AddSceneObject(box);

  // Show the viewer
  viewer.Show();

  return 0;
} 