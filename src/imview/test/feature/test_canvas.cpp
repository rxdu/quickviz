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
#include "imview/component/opengl/renderer/grid.hpp"
#include "imview/component/opengl/renderer/triangle.hpp"
#include "imview/component/opengl/renderer/canvas.hpp"
#include "imview/component/opengl/gl_scene_manager.hpp"

using namespace quickviz;

int main(int argc, char* argv[]) {
  Viewer viewer;

  // create a box to manage size & position of the OpenGL scene
  auto box = std::make_shared<Box>("main_box");
  box->SetFlexDirection(Styling::FlexDirection::kRow);
  box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
  box->SetAlignItems(Styling::AlignItems::kStretch);

  // create a OpenGL scene manager to manage the OpenGL objects
  auto gl_sm = std::make_shared<GlSceneManager>("OpenGL Scene",
                                                GlSceneManager::Mode::k2D);
  gl_sm->SetAutoLayout(true);
  gl_sm->SetNoTitleBar(true);
  gl_sm->SetFlexGrow(1.0f);
  gl_sm->SetFlexShrink(0.0f);

  // now add the rendering objects to the OpenGL scene manager
  auto triangle = std::make_unique<Triangle>(1.0f, glm::vec3(0.0f, 0.5f, 0.5f));
  gl_sm->AddOpenGLObject("triangle", std::move(triangle));

  auto grid = std::make_unique<Grid>(10.0f, 1.0f, glm::vec3(0.7f, 0.7f, 0.7f));
  gl_sm->AddOpenGLObject("grid", std::move(grid));

  auto canvas = std::make_unique<Canvas>(100.0f, 100.0f);
  gl_sm->AddOpenGLObject("canvas", std::move(canvas));

  // now let's do some drawing on the canvas
  {
    auto canvas = static_cast<Canvas*>(gl_sm->GetOpenGLObject("canvas"));

    // Add a red point at the origin with a larger size
    canvas->AddPoint(0.0f, 0.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 20.0f);
    canvas->AddPoint(1.0f, 1.0f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 20.0f);
    canvas->AddPoint(-1.5f, -1.5f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), 20.0f);

    // // Add points in a circle pattern
    // const int num_points = 12;
    // const float radius = 5.0f;
    // for (int i = 0; i < num_points; ++i) {
    //   float angle = 2.0f * M_PI * i / num_points;
    //   float x = radius * cos(angle);
    //   float y = radius * sin(angle);

    //   // Create a rainbow effect
    //   float hue = static_cast<float>(i) / num_points;
    //   glm::vec4 color;

    //   // Simple HSV to RGB conversion
    //   if (hue < 1.0f/6.0f) {
    //     color = glm::vec4(1.0f, hue * 6.0f, 0.0f, 1.0f);
    //   } else if (hue < 2.0f/6.0f) {
    //     color = glm::vec4(1.0f - (hue - 1.0f/6.0f) * 6.0f, 1.0f, 0.0f, 1.0f);
    //   } else if (hue < 3.0f/6.0f) {
    //     color = glm::vec4(0.0f, 1.0f, (hue - 2.0f/6.0f) * 6.0f, 1.0f);
    //   } else if (hue < 4.0f/6.0f) {
    //     color = glm::vec4(0.0f, 1.0f - (hue - 3.0f/6.0f) * 6.0f, 1.0f, 1.0f);
    //   } else if (hue < 5.0f/6.0f) {
    //     color = glm::vec4((hue - 4.0f/6.0f) * 6.0f, 0.0f, 1.0f, 1.0f);
    //   } else {
    //     color = glm::vec4(1.0f, 0.0f, 1.0f - (hue - 5.0f/6.0f) * 6.0f, 1.0f);
    //   }

    //   // Add the point with larger sizes
    //   canvas->AddPoint(x, y, color, 10.0f + 10.0f * i / num_points);
    // }
  }

  // finally pass the OpenGL scene manager to the box and add it to the viewer
  box->AddChild(gl_sm);
  viewer.AddSceneObject(box);

  viewer.Show();

  return 0;
}