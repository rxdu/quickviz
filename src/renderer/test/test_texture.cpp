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
#include "renderer/renderable/coordinate_frame.hpp"
#include "renderer/renderable/texture.hpp"

using namespace quickviz;
namespace fs = std::filesystem;

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
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(1.0f);

  std::cout << "\n=== Setting up Scene Objects ===\n" << std::endl;

  // Add a grid with smaller size and more visible color
  std::cout << "Adding grid (10m x 10m, 1m spacing)..." << std::endl;
  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a larger coordinate frame for better visibility
  std::cout << "Adding coordinate frame (2m size)..." << std::endl;
  auto coord_frame = std::make_unique<CoordinateFrame>(2.0f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  std::cout << "Creating texture object..." << std::endl;
  auto texture = std::make_unique<Texture>();
  gl_sm->AddOpenGLObject("texture", std::move(texture));

  // now let's do some drawing on the canvas
  {
    std::cout << "\n=== Loading Texture ===\n" << std::endl;
    
    auto texture = static_cast<Texture*>(gl_sm->GetOpenGLObject("texture"));
    if (!texture) {
      std::cerr << "Failed to get texture object" << std::endl;
      return -1;
    }

    // Add background image first so it's behind all other drawings
    std::string image_path = "../data/fish.png";

    // Check if file exists and get absolute path
    fs::path abs_path = fs::absolute(image_path);
    std::cout << "Image path: " << abs_path.string() << std::endl;
    if (fs::exists(abs_path)) {
      std::cout << "✓ Image file exists" << std::endl;
    } else {
      std::cerr << "✗ Image file not found at: " << abs_path.string() << std::endl;
      std::cerr << "Current working directory: " << fs::current_path() << std::endl;
      return -1;
    }
    
    std::cout << "Loading image..." << std::endl;
    auto ret = texture->LoadData(abs_path.string());
    if (!ret) {
      std::cerr << "✗ Failed to load texture" << std::endl;
      return -1;
    }
    std::cout << "✓ Image loaded successfully" << std::endl;

    // Set the origin and resolution for the texture
    std::cout << "\n=== Positioning Texture ===\n" << std::endl;
    
    // Calculate reasonable resolution based on image size
    int image_width = 1500;  // Approximate width of the fish.png in pixels
    float desired_world_size = 3.0f;  // We want the texture to be about 3 meters wide
    float resolution = desired_world_size / image_width;
    
    // Set a rotation angle for testing (45 degrees = π/4 radians)
    float rotation = M_PI / 4.0f;  // 45 degrees
    
    std::cout << "Setting texture position:" << std::endl;
    std::cout << "- Position: (1.0, 1.0) meters" << std::endl;
    std::cout << "- Rotation: " << rotation << " radians (" << (rotation * 180.0f / M_PI) << " degrees)" << std::endl;
    std::cout << "- Resolution: " << resolution << " meters/pixel" << std::endl;
    std::cout << "- Estimated world size: ~" << desired_world_size << " meters wide" << std::endl;
    
    // Set origin at (1,1) with 45-degree rotation and reasonable resolution
    texture->SetOrigin(glm::vec3(1.0f, 1.0f, rotation), resolution);
  }

  std::cout << "\n=== Setup Complete ===\n" << std::endl;
  std::cout << "You should see:" << std::endl;
  std::cout << "1. A gray grid (10m x 10m)" << std::endl;
  std::cout << "2. Coordinate axes (red = X, green = Y)" << std::endl;
  std::cout << "3. The loaded texture at the origin" << std::endl;
  std::cout << "\nIf you don't see the texture, check that:" << std::endl;
  std::cout << "- The image file exists and is valid" << std::endl;
  std::cout << "- The texture is not too small or too large (adjust resolution)" << std::endl;
  std::cout << "- The texture is within the visible area" << std::endl;

  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}