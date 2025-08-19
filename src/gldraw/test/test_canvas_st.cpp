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

#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/triangle.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"
#include "gldraw/renderable/canvas.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

// Function to test all canvas drawing functions
void TestAllCanvasFunctions(Canvas* canvas) {
  // Add reference points to verify coordinate system
  canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                   10.0f);  // White center
  canvas->AddPoint(0.0f, 2.0f, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
                   8.0f);  // Green top
  canvas->AddPoint(-4.0f, 1.0f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
                   8.0f);  // Blue left
  canvas->AddPoint(4.0f, 1.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
                   8.0f);  // Yellow right

  canvas->AddLine(-1.0f, -1.0f, -1.0f, 1.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
                  2.0f, LineType::kSolid);  // Yellow solid
  canvas->AddLine(-1.25f, -1.0f, -1.25f, 1.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
                  2.0f, LineType::kDashed);
  canvas->AddLine(-1.5f, -1.0f, -1.5f, 1.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
                  2.0f, LineType::kDotted);

  canvas->AddRectangle(-2.5, -2.5, 1.0, 1.0, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
                       true, 2.0f);  // Red filled rectangle
  canvas->AddRectangle(-2.25, -1.25, 0.5, 0.5,
                       glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), false,
                       2.0f);  // Red filled rectangle

  canvas->AddCircle(3.0f, 0.0f, 0.5f, glm::vec4(0.0f, 0.5f, 0.0f, 0.8f), true,
                    2.0f);  // Dark green filled
  canvas->AddCircle(2.0f, 0.0f, 0.25f, glm::vec4(0.0f, 0.5f, 0.0f, 0.8f), false,
                    2.0f);

  canvas->AddEllipse(-3.0f, 0.0f, 1.0f, 0.5f, 0.0f, 0.0f, 6.28f,
                     glm::vec4(0.5f, 0.5f, 0.0f, 0.8f), true,
                     2.0f);  // Olive filled
  canvas->AddEllipse(-3.0f, 1.5f, 0.5f, 0.25f, 0.0f, 0.0f, 6.28f,
                     glm::vec4(0.5f, 0.5f, 0.0f, 0.8f), false, 2.0f);

  // Add a simple test polygon
  std::vector<glm::vec2> simple_triangle = {
      {-0.5f, -1.0f},  // Bottom-left
      {0.5f, -1.0f},   // Bottom-right
      {0.0f, -0.5f}    // Top
  };
  canvas->AddPolygon(simple_triangle, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), true,
                     3.0f);
  for (auto& v : simple_triangle) {
    v += glm::vec2(0.0f, -1.0f);
  }
  canvas->AddPolygon(simple_triangle, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), false,
                     3.0f);
}

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
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(1.0f);

  // now add the rendering objects to the OpenGL scene manager
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a coordinate frame in 2D mode (should show X and Z axes)
  auto coord_frame = std::make_unique<CoordinateFrame>(0.5f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  auto canvas = std::make_unique<Canvas>();
  canvas->SetBatchingEnabled(true);
  gl_sm->AddOpenGLObject("canvas", std::move(canvas));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));

    // Add background image first so it's behind all other drawings
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
      std::cout << "Trying alternative path: " << abs_alt_path1.string()
                << std::endl;
      if (fs::exists(abs_alt_path1)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path1;
      }

      std::string alt_path2 = "fish.png";
      fs::path abs_alt_path2 = fs::absolute(alt_path2);
      std::cout << "Trying alternative path: " << abs_alt_path2.string()
                << std::endl;
      if (fs::exists(abs_alt_path2)) {
        std::cout << "Alternative image file exists!" << std::endl;
        image_path = alt_path2;
      }
    }

    // Add background image using a small origin offset and 1:100 resolution for
    // debugging
    canvas->AddBackgroundImage(image_path, glm::vec3(1.0f, 1.0f, 0.785f),
                               0.005f);

    // Test all canvas drawing functions
    TestAllCanvasFunctions(canvas);
  }

  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}