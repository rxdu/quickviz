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
#include <filesystem>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/component/opengl/renderer/grid.hpp"
#include "imview/component/opengl/renderer/triangle.hpp"
#include "imview/component/opengl/renderer/coordinate_frame.hpp"
#include "imview/component/opengl/renderer/canvas.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene (2D)",
                                                GlSceneManager::Mode::k2D);
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(0.5f);
  gl_sm->SetFlexShrink(0.0f);

  // now add the rendering objects to the OpenGL scene manager
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a coordinate frame in 2D mode (should show X and Z axes)
  auto coord_frame = std::make_unique<CoordinateFrame>(0.5f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  auto canvas = std::make_unique<Canvas>(100.0f, 100.0f);
  gl_sm->AddOpenGLObject("canvas", std::move(canvas));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));

    // Add a red point at the origin with a larger size
    canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);
    canvas->AddPoint(1.0f, 1.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);
    canvas->AddPoint(-1.5f, -1.5f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);

    // Add background image
    std::string image_path = "../data/fish.png";
    
    // Check if file exists and get absolute path
    fs::path abs_path = fs::absolute(image_path);
    std::cout << "Checking image path: " << abs_path.string() << std::endl;
    if (fs::exists(abs_path)) {
      std::cout << "Image file exists!" << std::endl;
    } else {
      std::cout << "Image file does not exist!" << std::endl;
      
      // Try alternative paths
      std::string alt_path1 = "data/fish.png";
      fs::path abs_alt_path1 = fs::absolute(alt_path1);
      std::cout << "Trying alternative path: " << abs_alt_path1.string() << std::endl;
      if (fs::exists(abs_alt_path1)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path1;
      }
      
      std::string alt_path2 = "fish.png";
      fs::path abs_alt_path2 = fs::absolute(alt_path2);
      std::cout << "Trying alternative path: " << abs_alt_path2.string() << std::endl;
      if (fs::exists(abs_alt_path2)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path2;
      }
    }
    
    // Add background image using a small origin offset and 1:100 resolution for debugging
    canvas->AddBackgroundImage(image_path, glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
  }

  // Create a second OpenGL scene manager for 3D mode
  auto gl_sm_3d = std::make_shared<GlSceneManager>("OpenGL Scene (3D)",
                                                   GlSceneManager::Mode::k3D);
  gl_sm_3d->SetAutoLayout(true);
  gl_sm_3d->SetNoTitleBar(true);
  gl_sm_3d->SetFlexGrow(0.5f);
  gl_sm_3d->SetFlexShrink(0.0f);

  // Add a grid for reference
  auto grid_3d =
      std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm_3d->AddOpenGLObject("grid", std::move(grid_3d));

  // Add a coordinate frame in 3D mode
  auto coord_frame_3d = std::make_unique<CoordinateFrame>(3.0f, false);
  gl_sm_3d->AddOpenGLObject("coordinate_frame", std::move(coord_frame_3d));

  auto canvas_3d = std::make_unique<Canvas>(100.0f, 100.0f);
  gl_sm_3d->AddOpenGLObject("canvas", std::move(canvas_3d));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm_3d->GetOpenGLObject("canvas"));

    // Add a red point at the origin with a larger size
    canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);
    canvas->AddPoint(1.0f, 1.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);
    canvas->AddPoint(-1.5f, -1.5f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);
    
    // Add background image to 3D canvas with a small rotation
    canvas->AddBackgroundImage("../data/fish.png", glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
  }

  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  box->AddChild(gl_sm_3d);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}