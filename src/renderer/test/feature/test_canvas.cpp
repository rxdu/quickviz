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

#include "renderer/gl_scene_manager.hpp"
#include "renderer/renderable/grid.hpp"
#include "renderer/renderable/triangle.hpp"
#include "renderer/renderable/coordinate_frame.hpp"
#include "renderer/renderable/canvas.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

// Function to test all canvas drawing functions
void TestAllCanvasFunctions(Canvas* canvas) {
  // Add some points with different colors and sizes
  canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 5.0f);  // Red
  canvas->AddPoint(1.0f, 1.0f, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), 8.0f);  // Green
  canvas->AddPoint(-1.5f, -1.5f, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), 10.0f);  // Blue
  
  // Add lines with different styles
  canvas->AddLine(2.0f, 2.0f, 3.0f, 3.0f, glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), 2.0f, LineType::kSolid);  // Yellow solid
  canvas->AddLine(-2.0f, 2.0f, -3.0f, 3.0f, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), 3.0f, LineType::kDashed);  // Magenta dashed
  canvas->AddLine(3.0f, -2.0f, 4.0f, -3.0f, glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), 4.0f, LineType::kDotted);  // Cyan dotted
  
  // Add rectangles - filled and outlined
  canvas->AddRectangle(-4.0f, -4.0f, 1.0f, 1.0f, glm::vec4(1.0f, 0.5f, 0.0f, 0.7f), true, 2.0f);  // Orange filled
  canvas->AddRectangle(3.0f, -4.0f, 1.0f, 1.0f, glm::vec4(0.5f, 0.0f, 0.5f, 0.7f), false, 2.0f);  // Purple outlined
  
  // Add circles - filled and outlined
  canvas->AddCircle(-2.0f, -2.0f, 0.7f, glm::vec4(0.0f, 0.5f, 0.0f, 0.8f), true, 2.0f);  // Dark green filled
  canvas->AddCircle(2.0f, 0.0f, 0.5f, glm::vec4(0.7f, 0.7f, 0.7f, 0.8f), false, 2.0f);  // Gray outlined
  
  // Add ellipses - filled and outlined
  canvas->AddEllipse(0.0f, 3.0f, 1.0f, 0.5f, 0.0f, 0.0f, 6.28f, 
                    glm::vec4(0.5f, 0.5f, 0.0f, 0.8f), true, 2.0f);  // Olive filled
  canvas->AddEllipse(-3.0f, 0.0f, 0.7f, 0.4f, 0.7f, 0.0f, 6.28f, 
                    glm::vec4(0.5f, 0.0f, 0.0f, 0.8f), false, 2.0f);  // Dark red outlined, rotated
  
  // Add a polygon
  std::vector<glm::vec2> star_vertices = {
    {0.0f, 5.0f},
    {1.0f, 2.0f},
    {4.0f, 2.0f},
    {2.0f, 0.0f},
    {3.0f, -3.0f},
    {0.0f, -1.0f},
    {-3.0f, -3.0f},
    {-2.0f, 0.0f},
    {-4.0f, 2.0f},
    {-1.0f, 2.0f},
  };
  
  // Scale down the star vertices
  for (auto& vertex : star_vertices) {
    vertex *= 0.3f;
  }
  
  // Move the star to a different position
  for (auto& vertex : star_vertices) {
    vertex += glm::vec2(4.0f, 3.0f);
  }
  
  canvas->AddPolygon(star_vertices, glm::vec4(0.8f, 0.8f, 0.0f, 0.9f), true, 2.0f);  // Gold filled
  
  // Add a simple test polygon that should be very visible
  std::vector<glm::vec2> test_polygon = {
    {-1.5f, -1.5f},  // Bottom left
    {-0.5f, -1.5f},  // Bottom right
    {-0.5f, -0.5f},  // Top right
    {-1.5f, -0.5f}   // Top left
  };
  canvas->AddPolygon(test_polygon, glm::vec4(1.0f, 0.0f, 1.0f, 1.0f), true, 3.0f);  // Bright magenta filled
}

int main(int argc, char* argv[]) {
  bool thread_test = false;
  
  // Check for thread test flag
  if (argc > 1 && std::string(argv[1]) == "--thread-test") {
    thread_test = true;
  }

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

  auto canvas = std::make_unique<Canvas>();
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
    
    // Test all canvas drawing functions
    TestAllCanvasFunctions(canvas);
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

  auto canvas_3d = std::make_unique<Canvas>();
  gl_sm_3d->AddOpenGLObject("canvas", std::move(canvas_3d));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm_3d->GetOpenGLObject("canvas"));
    
    // Add background image first
    canvas->AddBackgroundImage("../data/fish.png", glm::vec3(1.0f, 1.0f, 0.785f), 0.005f);
    
    // Test all canvas drawing functions in 3D view as well
    TestAllCanvasFunctions(canvas);
  }

  // If thread test is enabled, start a background thread that keeps adding shapes
  if (thread_test) {
    std::thread worker_thread([&gl_sm, &gl_sm_3d]() {
      std::cout << "Starting thread safety test..." << std::endl;
      
      auto canvas_2d = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));
      auto canvas_3d = static_cast<Canvas*>(gl_sm_3d->GetOpenGLObject("canvas"));
      
      float x = 0.0f;
      float y = 0.0f;
      int iteration = 0;
      
      // Keep adding shapes in a loop
      while (true) {
        // Clear canvases occasionally to avoid cluttering
        if (iteration % 100 == 0) {
          canvas_2d->Clear();
          canvas_3d->Clear();
          std::cout << "Cleared canvases at iteration " << iteration << std::endl;
        }
        
        // Add a variety of shapes with different parameters
        float angle = static_cast<float>(iteration) * 0.1f;
        
        // Calculate position along a spiral
        x = 5.0f * std::cos(angle) * (0.1f + 0.01f * iteration);
        y = 5.0f * std::sin(angle) * (0.1f + 0.01f * iteration);
        
        // Alternate between different shape types
        switch (iteration % 5) {
          case 0:
            canvas_2d->AddPoint(x, y, 
                              glm::vec4(std::sin(angle) * 0.5f + 0.5f, 
                                       std::cos(angle) * 0.5f + 0.5f, 
                                       0.5f, 1.0f), 
                              3.0f + std::sin(angle) * 2.0f);
            canvas_3d->AddPoint(x, y, 
                              glm::vec4(std::sin(angle) * 0.5f + 0.5f, 
                                       std::cos(angle) * 0.5f + 0.5f, 
                                       0.5f, 1.0f), 
                              3.0f + std::sin(angle) * 2.0f);
            break;
          case 1:
            canvas_2d->AddCircle(x, y, 0.3f + 0.1f * std::sin(angle), 
                               glm::vec4(0.8f, 0.2f, 0.2f, 0.7f), 
                               (iteration % 10) < 5, 2.0f);
            canvas_3d->AddCircle(x, y, 0.3f + 0.1f * std::sin(angle), 
                               glm::vec4(0.8f, 0.2f, 0.2f, 0.7f), 
                               (iteration % 10) < 5, 2.0f);
            break;
          case 2:
            canvas_2d->AddRectangle(x, y, 0.4f + 0.1f * std::sin(angle), 
                                  0.3f + 0.1f * std::cos(angle), 
                                  glm::vec4(0.2f, 0.8f, 0.2f, 0.7f), 
                                  (iteration % 10) >= 5, 2.0f);
            canvas_3d->AddRectangle(x, y, 0.4f + 0.1f * std::sin(angle), 
                                  0.3f + 0.1f * std::cos(angle), 
                                  glm::vec4(0.2f, 0.8f, 0.2f, 0.7f), 
                                  (iteration % 10) >= 5, 2.0f);
            break;
          case 3:
            canvas_2d->AddLine(x, y, x + std::cos(angle), y + std::sin(angle), 
                             glm::vec4(0.2f, 0.2f, 0.8f, 0.7f), 
                             1.0f + 0.5f * std::sin(angle), 
                             static_cast<LineType>((iteration / 20) % 3));
            canvas_3d->AddLine(x, y, x + std::cos(angle), y + std::sin(angle), 
                             glm::vec4(0.2f, 0.2f, 0.8f, 0.7f), 
                             1.0f + 0.5f * std::sin(angle), 
                             static_cast<LineType>((iteration / 20) % 3));
            break;
          case 4:
            canvas_2d->AddEllipse(x, y, 
                                0.5f + 0.1f * std::sin(angle), 
                                0.3f + 0.1f * std::cos(angle), 
                                angle * 0.5f, 0.0f, 6.28f, 
                                glm::vec4(0.8f, 0.8f, 0.2f, 0.7f), 
                                (iteration % 10) < 5, 2.0f);
            canvas_3d->AddEllipse(x, y, 
                                0.5f + 0.1f * std::sin(angle), 
                                0.3f + 0.1f * std::cos(angle), 
                                angle * 0.5f, 0.0f, 6.28f, 
                                glm::vec4(0.8f, 0.8f, 0.2f, 0.7f), 
                                (iteration % 10) < 5, 2.0f);
            break;
        }
        
        // Occasionally add a polygon
        if (iteration % 30 == 0) {
          std::vector<glm::vec2> polygon_vertices;
          int num_vertices = 3 + (iteration % 7); // 3 to 9 vertices
          
          for (int i = 0; i < num_vertices; ++i) {
            float vertex_angle = 2.0f * M_PI * i / num_vertices;
            float radius = 0.3f + 0.1f * std::sin(angle + i);
            polygon_vertices.push_back({
              x + radius * std::cos(vertex_angle), 
              y + radius * std::sin(vertex_angle)
            });
          }
          
          canvas_2d->AddPolygon(polygon_vertices, 
                              glm::vec4(0.2f, 0.8f, 0.8f, 0.7f), 
                              (iteration % 10) >= 5, 2.0f);
          canvas_3d->AddPolygon(polygon_vertices, 
                              glm::vec4(0.2f, 0.8f, 0.8f, 0.7f), 
                              (iteration % 10) >= 5, 2.0f);
        }
        
        // Sleep a little to avoid adding shapes too quickly
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        iteration++;
      }
    });
    
    // Detach the thread so it runs independently
    worker_thread.detach();
  }
  
  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  box->AddChild(gl_sm_3d);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}