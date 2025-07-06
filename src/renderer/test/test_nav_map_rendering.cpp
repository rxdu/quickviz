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
  canvas->AddLine(-1.25f, -1.0f, -1.25f, 1.0f,
                  glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), 2.0f, LineType::kDashed);
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

void DrawRobotMarker(float x, float y, float theta, Canvas* canvas,
                     float size = 1.0f) {
  // Professional navigation marker with high-contrast colors for visibility
  const glm::vec4 outer_ring(1.0f, 0.4f, 0.0f, 1.0f);     // Bright orange outer ring
  const glm::vec4 inner_ring(0.6f, 0.6f, 0.6f, 1.0f);     // Medium gray inner ring
  const glm::vec4 circle_fill(0.1f, 0.1f, 0.1f, 1.0f);    // Very dark gray fill
  const glm::vec4 arrow_fill(1.0f, 1.0f, 1.0f, 1.0f);     // Pure white arrow
  const glm::vec4 arrow_outline(0.0f, 0.0f, 0.0f, 1.0f);  // Black outline

  // Calculate direction vector
  float cos_theta = std::cos(theta);
  float sin_theta = std::sin(theta);

  // Professional proportions - make outer ring much wider
  const float outer_radius = 1.0f * size;
  const float inner_radius = 1.0f * size;
  const float fill_radius = 0.85f * size;
  const float arrow_length = 0.6f * size;
  const float arrow_base_width = 0.7f * size;

  // Draw layered circular background for depth
  canvas->AddCircle(x, y, fill_radius, circle_fill, true);
  canvas->AddCircle(x, y, outer_radius, outer_ring, true);  // Black outer ring
  //   canvas->AddCircle(x, y, inner_radius, inner_ring, true);      // Gray
  //   inner ring

  // Calculate arrow geometry with professional notch
  float tip_x = x + arrow_length * cos_theta;
  float tip_y = y + arrow_length * sin_theta;

  // Base corners (perpendicular to direction) - wider base
  float base_offset_x = arrow_base_width * 0.5f * sin_theta;
  float base_offset_y = -arrow_base_width * 0.5f * cos_theta;

  // Position base closer to center for better proportions
  float base_center_x = x - 0.8f * arrow_length * cos_theta;
  float base_center_y = y - 0.8f * arrow_length * sin_theta;

  float base_left_x = base_center_x + base_offset_x;
  float base_left_y = base_center_y + base_offset_y;
  float base_right_x = base_center_x - base_offset_x;
  float base_right_y = base_center_y - base_offset_y;

  // Create notch at the bottom (professional navigation marker style)
  const float notch_depth = 0.2f * size;
  float notch_center_x = x - (0.4f - notch_depth) * arrow_length * cos_theta;
  float notch_center_y = y - (0.4f - notch_depth) * arrow_length * sin_theta;

  // Create the notched arrow shape (outline version)
  std::vector<glm::vec2> notched_arrow_outline = {
      {tip_x, tip_y},                    // Arrow tip
      {base_left_x, base_left_y},        // Left base corner
      {notch_center_x, notch_center_y},  // Bottom notch center
      {base_right_x, base_right_y}       // Right base corner
  };

  // Draw white arrow fill first
  canvas->AddPolygon(notched_arrow_outline, arrow_fill, true, 2.0f);
  
  // Draw black arrow outline for definition
  canvas->AddPolygon(notched_arrow_outline, arrow_outline, false, 3.0f);

  //   // Create slightly smaller notched arrow for the main fill
  //   float inner_tip_x = x + 0.9f * arrow_length * cos_theta;
  //   float inner_tip_y = y + 0.9f * arrow_length * sin_theta;

  //   float inner_base_offset_x = 0.9f * arrow_base_width * 0.5f * sin_theta;
  //   float inner_base_offset_y = -0.9f * arrow_base_width * 0.5f * cos_theta;

  //   float inner_base_center_x = x - 0.25f * arrow_length * cos_theta;
  //   float inner_base_center_y = y - 0.25f * arrow_length * sin_theta;

  //   float inner_base_left_x = inner_base_center_x + inner_base_offset_x;
  //   float inner_base_left_y = inner_base_center_y + inner_base_offset_y;
  //   float inner_base_right_x = inner_base_center_x - inner_base_offset_x;
  //   float inner_base_right_y = inner_base_center_y - inner_base_offset_y;

  //   float inner_notch_center_x = x - (0.25f - 0.9f * notch_depth) *
  //   arrow_length * cos_theta; float inner_notch_center_y = y - (0.25f - 0.9f
  //   * notch_depth) * arrow_length * sin_theta;

  //   std::vector<glm::vec2> inner_notched_arrow = {
  //       {inner_tip_x, inner_tip_y},
  //       {inner_base_left_x, inner_base_left_y},
  //       {inner_notch_center_x, inner_notch_center_y},
  //       {inner_base_right_x, inner_base_right_y}
  //   };

  //   canvas->AddPolygon(inner_notched_arrow, arrow_fill, true, 2.0f);  // Main
  //   fill
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
  //   auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f,
  //   0.5f)); gl_sm->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  // Add a coordinate frame in 2D mode (should show X and Z axes)
  auto coord_frame = std::make_unique<CoordinateFrame>(0.5f, true);
  gl_sm->AddOpenGLObject("coordinate_frame", std::move(coord_frame));

  auto canvas = std::make_unique<Canvas>();
  canvas->SetBatchingEnabled(false);
  gl_sm->AddOpenGLObject("canvas", std::move(canvas));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));

    // Add background image first so it's behind all other drawings
    std::string image_path = "../data/openstreet_map.png";

    // Check if file exists and get absolute path
    fs::path abs_path = fs::absolute(image_path);
    std::cout << "Checking image path: " << abs_path.string() << std::endl;
    if (fs::exists(abs_path)) {
      std::cout << "Image file exists!" << std::endl;
      canvas->AddBackgroundImage(image_path, glm::vec3(5.0f, -5.0f, 1.57f),
                                 0.05f);
    } else {
      std::cout << "Image file does not exist!" << std::endl;
    }

    // Test all canvas drawing functions
    // TestAllCanvasFunctions(canvas);
    DrawRobotMarker(0.0, 0.0, 0.0, canvas, 0.5f);         // Pointing right
    DrawRobotMarker(1.0, 1.0, M_PI / 4.0f, canvas, 0.5f);    // Pointing up
    DrawRobotMarker(3.0, 3.0, M_PI / 2.0f, canvas, 0.5f);    // Pointing up
    DrawRobotMarker(-3.0, 3.0, M_PI, canvas, 0.5f);       // Pointing left
    DrawRobotMarker(-2.0, 2.5, -M_PI/4.0f, canvas, 0.5f);       // Pointing left
    DrawRobotMarker(3.0, -3.0, -M_PI / 2.0f, canvas, 0.5f);  // Pointing down
  }

  // finally pass the OpenGL scene managers to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}